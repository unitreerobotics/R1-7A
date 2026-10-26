import dataclasses
import json
import math
import os
from pathlib import Path
import queue
import subprocess
import tempfile
import threading
import time
import unittest
from concurrent.futures import ThreadPoolExecutor
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
from mcap.reader import make_reader

from dual.config import PORTS,CALIBRATIONS,LEFT_BUS_SIGN,HOME,validate_pair,parse_home
from dual.controller import LeftArmBus,ArmController,R1ArmDualRealController
from dual.calibrate import offsets_from_samples
from dual.recorder import DualRecorder,paired_vectors
from dual.teleop import parse_args
from dual.replay import load_dual_episode
from robot.r1arm_config import R1ArmConfig
from robot.r1arm_sdk import R1ArmSdkState,model_to_sdk_q,sdk_to_model_q,validate_calibration_file
from robot.eef.dex1_config import load_dex1_endpoints
from robot.eef.dex1_single import Dex1Single
from scripts.capture_dex1_endpoints import stable_endpoint
from teleop.xr_snapshot import sample_xr
from teleop.control_timing import PeriodicDeadline, TimingReporter


def state(q=None):
    return R1ArmSdkState(q=model_to_sdk_q(HOME if q is None else q),dq=np.zeros(7),tau_est=np.zeros(7),
        mode=np.ones(7,dtype=np.uint8),timeout=np.zeros(7,dtype=np.uint8),error_code=np.zeros(7,dtype=np.uint32),
        ready=np.ones(7,dtype=bool),feedback_count=np.ones(7,dtype=np.uint64),
        missed_response_count=np.zeros(7,dtype=np.uint64),last_step_feedback=np.ones(7,dtype=bool))


class FakeBus:
    initial=HOME
    def __init__(self,lib=None):
        self.state=state(self.initial)
        self.is_open=False
        self.commands=[]
    def open(self,*args): self.is_open=True
    def close(self): self.is_open=False
    def stop(self): pass
    def probe_once(self): self.state=dataclasses.replace(self.state,feedback_count=self.state.feedback_count+1)
    def read_state(self): return self.state
    def command_position(self,q,kp,kd,tau=None,dq=None):
        self.commands.append((np.asarray(q).copy(),None if dq is None else np.asarray(dq).copy(),None if tau is None else np.asarray(tau).copy()))
        self.state=dataclasses.replace(self.state,q=np.asarray(q).copy(),dq=np.zeros(7),feedback_count=self.state.feedback_count+1)


class Coordinates(unittest.TestCase):
    def test_left_home_and_vector_power(self):
        bus=LeftArmBus(bus_factory=FakeBus)
        physical=HOME*np.array([1,-1,1,1,1,-1,1])
        bus.bus.state=dataclasses.replace(bus.bus.state,q=physical,dq=np.arange(7.),tau_est=np.arange(7.)+.3)
        converted=bus.read_state()
        np.testing.assert_allclose(sdk_to_model_q(converted.q),HOME)
        self.assertAlmostEqual(converted.dq@converted.tau_est,bus.bus.state.dq@bus.bus.state.tau_est)
        bus.command_position(model_to_sdk_q(HOME),60,10,tau=converted.tau_est,dq=converted.dq)
        q,dq,tau=bus.bus.commands[-1]
        np.testing.assert_allclose(q,physical)
        np.testing.assert_allclose(dq,np.arange(7.))
        np.testing.assert_allclose(tau,np.arange(7.)+.3)
    def test_pair_and_port_roles(self):
        with self.assertRaises(ValueError): validate_pair(PORTS['grip_right'],PORTS['left'],'r','l')
        with self.assertRaises(ValueError): validate_pair('same','same','r','l')
        with self.assertRaises(ValueError): validate_pair('r','l','same','same')
    def test_home_validation(self):
        np.testing.assert_allclose(parse_home('0,1.2,0,-1.2,0,0,0'),HOME)
        for text in ('0,0','0,1.2,0,-1.2,0,nan,0','0,1.2,0,-1.2,0,6.3,0'):
            with self.assertRaises(ValueError): parse_home(text)
    def test_defaults_and_calibrations(self):
        args=parse_args(['--mock-xr','--no-cameras'])
        self.assertEqual(args.left_j2_convention,'mirrored')
        self.assertEqual(args.left_j4_convention,'model')
        self.assertEqual(args.left_j6_convention,'mirrored')
        self.assertEqual(args.max_joint_acceleration,6.)
        self.assertEqual(args.trajectory_position_gain,64.)
        self.assertEqual(args.trajectory_velocity_gain,16.)
        self.assertIn('if01',args.right_port)
        for path in CALIBRATIONS.values(): self.assertEqual(validate_calibration_file(path),path.resolve())
    def test_new_offsets_use_raw_motor_samples(self):
        direction=np.array([-1,1,1,-1,-1,1,-1.])
        raw=np.tile(np.arange(7.)+.25,(25,1))
        offsets=offsets_from_samples(raw,direction,np.ones(7))
        np.testing.assert_allclose(raw[0]*direction+offsets,np.zeros(7))
        raw[-1,0]+=.5
        with self.assertRaises(ValueError): offsets_from_samples(raw,direction,np.ones(7))
    def test_nonfinite_calibration_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'bad'
            path.write_text('\n'.join(f'{i} 1 1 '+('nan' if i==2 else '0') for i in range(7)))
            with self.assertRaises(ValueError): validate_calibration_file(path)


class Grippers(unittest.TestCase):
    def test_distinct_endpoints(self):
        self.assertAlmostEqual(load_dex1_endpoints('right')[0],-4.425779819488525)
        self.assertAlmostEqual(load_dex1_endpoints('right')[1],.8784417510032654)
        self.assertAlmostEqual(load_dex1_endpoints('left')[0],-.4818770587)
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'bad.json'
            path.write_text(json.dumps({'right':{'closed':1,'open':0}}))
            with self.assertRaises(ValueError): load_dex1_endpoints('right',path)
    def test_mapping_and_nonfinite_commands(self):
        for side in ('left','right'):
            hand=Dex1Single.__new__(Dex1Single)
            hand.closed_q,hand.open_q=load_dex1_endpoints(side)
            hand.travel=hand.open_q-hand.closed_q
            hand._lock=threading.Lock()
            hand.message=SimpleNamespace(cmds=[SimpleNamespace(q=0.)])
            hand.publisher=SimpleNamespace(Write=lambda _:True)
            hand._command_count=hand._command_publish_fail_count=0
            hand._force_limited_target=lambda target:target
            for ratio,expected in ((0.,hand.open_q),(1.,hand.closed_q),(.5,(hand.open_q+hand.closed_q)/2)):
                hand.set_closed_ratio(ratio)
                self.assertAlmostEqual(hand.last_command,expected)
            with self.assertRaises(ValueError): hand.set_closed_ratio(float('nan'))
    def test_torque_latch_preserved(self):
        hand=Dex1Single.__new__(Dex1Single)
        hand._lock=threading.Lock();hand._q=2.;hand._tau=-3.;hand._tau_zero=0.
        hand._alpha=0.;hand._tau_filtered=0.;hand._latched_q=None;hand._grip_torque_limit=-1.;hand._state_received=True
        self.assertEqual(hand._force_limited_target(1.),2.)
        self.assertEqual(hand._force_limited_target(.5),2.)
        self.assertEqual(hand._force_limited_target(3.),3.)
        self.assertIsNone(hand._latched_q)
    def test_callback_feedback_normalization_and_cleanup(self):
        from robot.eef import dex1_single as module
        publishers=[]
        class Publisher:
            def __init__(self,*_): self.closed=False;publishers.append(self)
            def Init(self): pass
            def Write(self,_): return True
            def Close(self): self.closed=True
        class Subscriber:
            def __init__(self,topic,*_): self.side=topic.split('/')[-2];self.closed=False
            def Init(self,handler):
                self.handler=handler
                handler(SimpleNamespace(states=[SimpleNamespace(q=load_dex1_endpoints(self.side)[1],dq=0.,tau_est=0.)]))
            def Close(self): self.closed=True
        with patch.object(module,'ChannelFactoryInitialize'),patch.object(module,'ChannelPublisher',Publisher),patch.object(module,'ChannelSubscriber',Subscriber):
            for side in ('right','left'):
                hand=module.Dex1Single(side=side,feedback_timeout=.01)
                self.assertAlmostEqual(hand.get_record_data()['hand_state'][0],1.)
                hand.set_closed_ratio(1.)
                self.assertAlmostEqual(hand.get_record_data()['hand_cmd'][0],0.)
                self.assertAlmostEqual(hand.get_record_data()['motor_command_q'],hand.closed_q)
                self.assertFalse(hand.get_record_data()['force_latched'])
                hand.subscriber.handler(SimpleNamespace(states=[SimpleNamespace(q=hand.closed_q,dq=0.,tau_est=0.)]))
                self.assertAlmostEqual(hand.get_record_data()['hand_state'][0],0.)
                stamp=hand._state_received_time_ns
                hand.subscriber.handler(SimpleNamespace(states=[SimpleNamespace(q=float('nan'),dq=0.,tau_est=0.)]))
                self.assertEqual(hand._state_received_time_ns,stamp)
                hand.stop();self.assertTrue(hand.subscriber.closed);self.assertTrue(hand.publisher.closed)
    def test_read_only_monitor_does_not_wait_for_commands(self):
        from dual import dex1_monitor as module
        class Subscriber:
            def __init__(self,*_): self.closed=False
            def Init(self,handler): self.handler=handler
            def Close(self): self.closed=True
        with patch.object(module,'ChannelFactoryInitialize'),patch.object(module,'ChannelSubscriber',Subscriber):
            monitor=module.GripperMonitor('lo')
            subscriber=monitor.subscribers[('left','state')]
            subscriber.handler(SimpleNamespace(states=[SimpleNamespace(q=.5,dq=0.,tau_est=0.)]))
            self.assertEqual(monitor.snapshot()[('left','state')]['q'],.5)
            self.assertNotIn(('left','cmd'),monitor.snapshot())
            monitor.close();self.assertTrue(subscriber.closed)

    def test_manual_endpoint_stability(self):
        self.assertEqual(stable_endpoint([[1.,0.]]*25),1.)
        for samples in ([[1.,0.]]*4,[[1.,.1]]*25,[[float('nan'),0.]]*25,[[1+i*.01,0.] for i in range(25)]):
            with self.assertRaises(ValueError): stable_endpoint(samples)


class XR(unittest.TestCase):
    def test_pose_cached_but_buttons_refreshed(self):
        class Client:
            stamp=1;reads=0;grip=.3
            def get_frame_stamp_ns(self): return self.stamp
            def get_pose_by_name(self,n): self.reads+=1;return [0,0,1,0,0,0,1]
            def get_key_value_by_name(self,n): return self.grip
            def get_button_state_by_name(self,n): return self.grip>.5
        client=Client();first=sample_xr(client)
        client.grip=.9;second=sample_xr(client,first)
        self.assertEqual(client.reads,3)
        self.assertTrue(second.buttons['A'])
        self.assertEqual(second.keys['left_grip'],.9)
        self.assertIs(first.poses,second.poses)
    def test_cross_frame_retry_and_failure(self):
        class Client:
            def __init__(self): self.i=0
            def get_frame_stamp_ns(self): self.i+=1;return self.i
            def get_pose_by_name(self,n): return [0,0,1,0,0,0,1]
            def get_key_value_by_name(self,n): return 1.
            def get_button_state_by_name(self,n): return True
        result=sample_xr(Client())
        self.assertEqual(result.poses,{})
        self.assertEqual(result.get_key_value_by_name('left_grip'),0.)
        self.assertEqual(result.get_key_value_by_name('left_trigger'),1.)


class ArmSafety(unittest.TestCase):
    def make_arm(self,**kwargs):
        return ArmController(R1ArmConfig(control_side='left'),port='/unused',baudrate=6000000,
            calibration_path='left',mock_xr=True,motion_mode=False,rate=90,**kwargs)
    def test_home_recovery_corridor_and_large_error_rejection(self):
        arm=self.make_arm(home_on_start=True)
        try:
            q=HOME.copy();q[5]=.6661
            arm._check_feedback_range(state(q))
            self.assertTrue(arm._recovery_active)
            self.assertAlmostEqual(arm._trajectory_position_bounds[1][5],.6661)
            q[5]+=.04
            with self.assertRaises(RuntimeError): arm._check_feedback_range(state(q))
        finally: arm._on_stop()

        arm=self.make_arm(home_on_start=True)
        try:
            q=HOME.copy();q[5]=-6.15
            with self.assertRaises(RuntimeError): arm._check_feedback_range(state(q))
        finally: arm._on_stop()
    def test_reset_uses_virtual_clock_and_clears_input_history(self):
        arm=self.make_arm()
        try:
            clock=[10.]
            arm._reset_clock=lambda:clock[0]
            arm._last_ee_intent={'left':{'position':np.ones(3)}}
            arm.reset()
            self.assertAlmostEqual(arm._reset_blend_deadline,10.+arm.reset_blend_frames/90)
            self.assertEqual(arm._last_ee_intent,{})
            clock[0]=11.
            for _ in range(9): arm.step(1/90)
            self.assertFalse(arm._reset_blending)
        finally: arm._on_stop()

    def test_counter_age_does_not_stop_preflight(self):
        arm=self.make_arm()
        try:
            arm.motion_mode=True;arm.arm_bus=FakeBus()
            with patch('dual.controller.time.monotonic',return_value=100.):
                arm.preflight()
            with patch('dual.controller.time.monotonic',return_value=101.):
                arm.preflight()
            self.assertIsNotNone(arm._cycle_feedback)
        finally: arm._on_stop()
    def test_reset_uses_left_state(self):
        arm=self.make_arm()
        try:
            arm.reset();arm.step(1/90)
            self.assertIn('left',arm._trajectory_q_cmd)
            self.assertNotIn('right',arm._trajectory_q_cmd)
        finally: arm._on_stop()


class Parallel(unittest.TestCase):
    def controller(self,fail_prepare=False,fail_output=False):
        barrier=threading.Barrier(2);events=[]
        class Arm:
            motion_mode=False;enable_dex1=False
            def __init__(self,side): self.side=side
            def preflight(self): events.append('check_'+self.side)
            def prepare_step(self,dt):
                events.append('prepare_'+self.side)
                if fail_prepare and self.side=='left': raise RuntimeError('prepare failed')
            def _output_command(self):
                barrier.wait(timeout=2)
                if fail_output and self.side=='left': raise RuntimeError('I/O failed')
                events.append('sent_'+self.side)
            def control_sample(self): return {'command_time_ns':time.time_ns(),'control_time_ns':time.time_ns(),'control_seq':1,'q_purpose':np.zeros(7)}
            def _on_stop(self): events.append('stop_'+self.side)
        c=R1ArmDualRealController.__new__(R1ArmDualRealController)
        c.pool=ThreadPoolExecutor(max_workers=2);c.arms={s:Arm(s) for s in ('right','left')}
        c.native_xr=None;c.snapshot=None;c.closed=False;c.last_timing={}
        return c,events
    def test_parallel_barrier_and_isolated_snapshots(self):
        c,events=self.controller()
        try:
            c.step(.01)
            self.assertIn('sent_right',events);self.assertIn('sent_left',events)
            self.assertLess(events.index('prepare_left'),events.index('sent_right'))
            copy=c.latest_pair();copy['right']['q_purpose'][0]=99
            self.assertEqual(c.latest_pair()['right']['q_purpose'][0],0)
        finally: c.stop()
    def test_preparation_failure_neither_arm_sent(self):
        c,events=self.controller(fail_prepare=True)
        try:
            with self.assertRaises(RuntimeError): c.step(.01)
            self.assertFalse(any(e.startswith('sent') for e in events))
        finally: c.stop()
    def test_keyboard_interrupt_drains_inflight_native_work(self):
        c,events=self.controller()
        with patch('dual.controller.wait',side_effect=KeyboardInterrupt):
            with self.assertRaises(KeyboardInterrupt): c.step(.01)
        self.assertIn('sent_right',events);self.assertIn('sent_left',events)
        c.stop()
        self.assertLess(events.index('sent_right'),events.index('stop_right'))

    def test_wait_for_inflight_before_stop(self):
        c,events=self.controller(fail_output=True)
        with self.assertRaises(RuntimeError): c.step(.01)
        c.stop()
        self.assertLess(events.index('sent_right'),events.index('stop_right'))
        self.assertIn('stop_left',events)


class Recording(unittest.TestCase):
    def samples(self,sequence):
        return {side:{'control_time_ns':10**18+sequence*11111111,'command_time_ns':10**18+sequence*11111111,
            'control_seq':sequence,'q_purpose':HOME.copy(),'q_measured':HOME.copy(),
            'dex1_command_open_ratio':.5,'dex1_state_open_ratio':.5,'dex1_feedback_valid':True,
            'gripper_enabled':True,'pair_skew_ns':123} for side in ('right','left')}
    def test_high_rate_controls_and_lower_rate_frames(self):
        with tempfile.TemporaryDirectory() as directory:
            args=parse_args(['--mock-xr','--no-cameras','--data-root',directory])
            recorder=DualRecorder(args)
            recorder.toggle()
            for i in range(18): recorder.publish(self.samples(i))
            recorder.pause(True)
            recorder.publish(self.samples(18))
            recorder.pause(False)
            recorder.publish(self.samples(19))
            recorder.toggle();recorder.close()
            path=next(Path(directory).glob('*.mcap'))
            with path.open('rb') as stream:
                messages=[(channel.topic,json.loads(msg.data)) for _,channel,msg in make_reader(stream).iter_messages()]
            controls=[m for t,m in messages if t=='/r1arm/control_sample']
            frames=[m for t,m in messages if t=='/whole_body/frame']
            self.assertEqual(len(controls),19)
            self.assertLess(len(frames),len(controls))
            self.assertEqual(controls[-1]['record_segment'],1)
            metadata=next(m for t,m in messages if t=='/episode/meta')
            self.assertEqual(metadata['config']['state_dim'],16)
            self.assertEqual(len(metadata['config']['urdf_sha256']),64)
            self.assertEqual(load_dual_episode(path)[0].shape[1],16)
    def test_gripper_stale_fails_and_no_nan_vectors(self):
        samples=self.samples(0);samples['left']['dex1_feedback_valid']=False
        with self.assertRaises(RuntimeError): paired_vectors(samples,measured=True)
    def test_queue_overflow_is_explicit(self):
        recorder=DualRecorder.__new__(DualRecorder)
        recorder.error=None;recorder.queue=queue.Queue(maxsize=1)
        recorder._put(('sample',None,0))
        with self.assertRaises(RuntimeError): recorder._put(('sample',None,0))
    def test_queue_overflow_still_drains_and_closes_worker(self):
        with tempfile.TemporaryDirectory() as directory:
            args=parse_args(['--mock-xr','--no-cameras','--data-root',directory])
            args.rate=1
            recorder=DualRecorder(args)
            ready,release=threading.Event(),threading.Event()
            original=recorder.writer.start_episode
            def slow_open(metadata):
                ready.set();release.wait(timeout=3);return original(metadata)
            with patch.object(recorder.writer,'start_episode',side_effect=slow_open):
                recorder.toggle();self.assertTrue(ready.wait(timeout=1))
                for i in range(recorder.queue.maxsize): recorder.publish(self.samples(i))
                with self.assertRaises(RuntimeError): recorder.publish(self.samples(100))
                release.set()
                errors=[]
                def close():
                    try: recorder.close()
                    except RuntimeError as exc: errors.append(exc)
                thread=threading.Thread(target=close,daemon=True);thread.start();thread.join(timeout=3)
                self.assertFalse(thread.is_alive())
                self.assertEqual(len(errors),1)

    def test_writer_error_surfaces_without_close_hang(self):
        with tempfile.TemporaryDirectory() as directory:
            args=parse_args(['--mock-xr','--no-cameras','--data-root',directory])
            recorder=DualRecorder(args)
            with patch.object(recorder.writer,'start_episode',side_effect=OSError('disk failed')):
                recorder.toggle()
                recorder.thread.join(timeout=2)
            self.assertIsNotNone(recorder.error)
            with self.assertRaises(RuntimeError): recorder.close()


class ServiceLifecycle(unittest.TestCase):
    def test_id7_topic_side_and_sibling_cleanup(self):
        from dual.grippers import main
        with tempfile.TemporaryDirectory() as directory:
            r,l=Path(directory)/'right',Path(directory)/'left'
            r.touch();l.touch()
            processes=[SimpleNamespace(pid=900001,returncode=None,poll=lambda:None,wait=lambda timeout=3:0),
                       SimpleNamespace(pid=900002,returncode=1,poll=lambda:1,wait=lambda timeout=3:1)]
            def preflight(command,**kwargs): return SimpleNamespace(returncode=0 if command[0]=='ip' else 1)
            with patch.dict(os.environ,{'XDG_RUNTIME_DIR':directory}),patch('dual.grippers.subprocess.run',side_effect=preflight),patch(
                'dual.grippers.subprocess.Popen',side_effect=processes) as spawn,patch('dual.grippers.os.killpg') as kill:
                with self.assertRaises(RuntimeError): main(['--right-port',str(r),'--left-port',str(l)])
                self.assertIn('7',spawn.call_args_list[0].args[0])
                self.assertIn('right',spawn.call_args_list[0].args[0])
                self.assertIn('left',spawn.call_args_list[1].args[0])
                self.assertEqual(kill.call_count,2)
    def test_existing_service_not_killed_or_restarted(self):
        from dual.grippers import main
        with tempfile.TemporaryDirectory() as directory:
            r,l=Path(directory)/'right',Path(directory)/'left';r.touch();l.touch()
            with patch.dict(os.environ,{'XDG_RUNTIME_DIR':directory}),patch('dual.grippers.port_users',return_value=[12345]),patch('dual.grippers.subprocess.run',return_value=SimpleNamespace(returncode=0)),patch(
                'dual.grippers.subprocess.Popen') as spawn,patch('dual.grippers.os.killpg') as kill:
                with self.assertRaises(RuntimeError): main(['--right-port',str(r),'--left-port',str(l)])
                spawn.assert_not_called();kill.assert_not_called()

    def test_port_users_detects_open_file_and_alias(self):
        from dual.grippers import port_users
        with tempfile.TemporaryDirectory() as directory:
            port=Path(directory)/'port'
            alias=Path(directory)/'alias'
            port.touch();alias.symlink_to(port)
            with port.open('rb'):
                self.assertIn(os.getpid(),port_users(alias))
            self.assertNotIn(os.getpid(),port_users(alias))

    def test_single_service_needs_only_selected_port(self):
        from dual.grippers import main
        with tempfile.TemporaryDirectory() as directory:
            right=Path(directory)/'right';right.touch()
            missing=Path(directory)/'missing-left'
            process=SimpleNamespace(pid=900001,returncode=1,poll=lambda:1,wait=lambda timeout=3:1)
            with patch.dict(os.environ,{'XDG_RUNTIME_DIR':directory}), patch('dual.grippers.port_users',return_value=[]), patch('dual.grippers.subprocess.run',return_value=SimpleNamespace(returncode=0)), patch('dual.grippers.subprocess.Popen',return_value=process) as spawn, patch('dual.grippers.os.killpg'):
                with self.assertRaises(RuntimeError):
                    main(['--side','right','--right-port',str(right),'--left-port',str(missing)])
                self.assertEqual(spawn.call_count,1)
                self.assertIn('right',spawn.call_args.args[0])


class Integration(unittest.TestCase):
    def test_startup_discovers_both_hands_then_holds_ready_arm_during_slow_probe(self):
        with tempfile.TemporaryDirectory() as directory:
            ports = [Path(directory)/side for side in ('right', 'left')]
            for port in ports:
                port.touch()
            c = R1ArmDualRealController(R1ArmConfig(), right_port=str(ports[0]), left_port=str(ports[1]),
                right_calibration='r', left_calibration='l', mock_xr=True, rate=90,
                baudrate=6000000, motion_mode=False, enable_dex1=False, home_on_start=True)
            discovered = []
            test = self
            class Bus(FakeBus):
                def open(self, *args):
                    test.assertEqual(discovered, ['right', 'left'])
                    super().open(*args)
            for side, arm in c.arms.items():
                arm.motion_mode = True
                arm.bus_factory = Bus
                arm._initialize_gripper = lambda side=side: discovered.append(side) if side not in discovered else None
            left = c.arms['left']
            wait_feedback = left._wait_for_feedback
            def slow_probe():
                time.sleep(.23)
                return wait_feedback()
            left._wait_for_feedback = slow_probe
            try:
                c.start()
                right = c.arms['right']
                self.assertGreaterEqual(len(right.arm_bus.commands), 5)
                for q, dq, _ in right.arm_bus.commands:
                    np.testing.assert_allclose(q, model_to_sdk_q(HOME))
                    if dq is not None:
                        np.testing.assert_allclose(dq, 0.)
                pair = c.step(1/90)
                self.assertLess(pair['right']['command_dt_s'], .1)
            finally:
                for arm in c.arms.values():
                    if arm.arm_bus is not None:
                        arm.arm_bus.close()
                        arm.arm_bus = None
                c.stop()

    def test_startup_failure_releases_sibling_hold_loop_and_stops_both(self):
        c = R1ArmDualRealController.__new__(R1ArmDualRealController)
        c.pool = ThreadPoolExecutor(max_workers=2)
        c.closed = False
        c.native_xr = None
        holding = threading.Event()
        stopped = []
        def fail():
            if not holding.wait(timeout=2):
                raise RuntimeError('sibling did not hold')
            raise RuntimeError('fake probe failure')
        c.arms = {side: SimpleNamespace(control_side=side, rate_hz=90., xr_client=None,
            _initialize_gripper=lambda: None, _on_start=(fail if side == 'left' else lambda: None),
            _output_command=holding.set, _on_stop=lambda side=side: stopped.append(side))
            for side in ('right', 'left')}
        with self.assertRaisesRegex(RuntimeError, 'fake probe failure'):
            c.start()
        self.assertEqual(set(stopped), {'right', 'left'})
        self.assertTrue(c.closed)

    def test_independent_models_and_mock_controller(self):
        c=R1ArmDualRealController(R1ArmConfig(),right_port='right',left_port='left',right_calibration='r',left_calibration='l',
            baudrate=6000000,mock_xr=True,rate=90)
        try:
            self.assertIsNot(c.arms['right'].model,c.arms['left'].model)
            self.assertIsNot(c.arms['right'].data,c.arms['left'].data)
            pair=c.step(1/90)
            self.assertEqual(set(pair),{'right','left'})
            self.assertGreaterEqual(pair['right']['pair_skew_ns'],0)
        finally: c.stop()
    def test_parallel_replay_fake_hardware(self):
        from dual import replay
        from teleop.mcap_writer import FourCameraMcapRecorder
        from teleop.r1arm_dataset import R1ArmDatasetAdapter
        opened={}
        class Bus(FakeBus):
            def open(self,port,*args):
                self.is_open=True;opened[str(port)]=self
        class LeftPhysicalBus(Bus):
            def __init__(self,lib=None):
                super().__init__(lib)
                self.state=dataclasses.replace(self.state,q=HOME*np.array([1,-1,1,1,1,-1,1]))
        with tempfile.TemporaryDirectory() as directory:
            w=FourCameraMcapRecorder(directory,'fake replay',30,camera_order=())
            path=w.start_episode({'dual_arm':True,'state_dim':16,'action_dim':16})
            adapter=R1ArmDatasetAdapter(dual_arm=True)
            for i in range(2):
                right,left=HOME.copy(),HOME.copy();right[0]=i*.01;left[0]=-i*.01
                vector=np.r_[right,[.0245,.0245],left,[.0245,.0245]]
                states,actions=adapter.frame_data(vector,vector)
                w.add_frame((),states,actions,i,sample_time_ns=10**18+i*33333333)
            w.stop_episode()
            args=replay.parse_args(['--episode',path,'--no-enable-dex1','--hold-time','0'])
            args.motion_mode=args.no_confirm=True
            with patch.object(replay,'R1ArmSdkBus',Bus),patch.object(replay,'LeftArmBus',lambda lib: LeftArmBus(lib,bus_factory=LeftPhysicalBus)),patch.object(replay,'safe_stop',lambda bus,_:bus.close()):
                replay.run(args)
            right=opened[str(PORTS['right'])];left=opened[str(PORTS['left'])]
            self.assertFalse(right.is_open or left.is_open)
            self.assertGreater(right.state.q[1],0);self.assertLess(left.state.q[1],0)
            self.assertGreater(right.state.q[3],0);self.assertLess(left.state.q[3],0)
            self.assertAlmostEqual(left.state.q[0],-.01,places=3)

    def test_bounded_recovery_slews_without_limit_snap(self):
        with tempfile.TemporaryDirectory() as directory:
            port=Path(directory)/'port';port.touch()
            class OutsideBus(FakeBus):
                initial=HOME.copy();initial[5]=.66
            arm=ArmController(R1ArmConfig(control_side='left'),port=str(port),baudrate=6000000,calibration_path='left',
                mock_xr=True,motion_mode=False,rate=90,max_joint_speed=1.,home_on_start=True,bus_factory=OutsideBus)
            arm.motion_mode=True
            try:
                arm._on_start()
                self.assertTrue(arm._recovery_active)
                origin=time.monotonic()
                clock=[origin]
                with patch('teleop.r1arm_real_controller.time.monotonic',side_effect=lambda:clock[0]), patch(
                        'teleop.r1arm_real_controller.time.monotonic_ns',side_effect=lambda:int(clock[0]*1e9)):
                    for i in range(240):
                        clock[0]=origin+(i+1)/90
                        arm.step(1/90)
                commands=[item[0] for item in arm.arm_bus.commands]
                self.assertLessEqual(np.max(np.abs(np.diff(commands,axis=0))),1/90+1e-8)
                self.assertFalse(arm._recovery_active)
                self.assertLess(abs(sdk_to_model_q(arm.arm_bus.state.q)[5]),.05)
            finally: arm._on_stop()


class Scheduling(unittest.TestCase):
    def test_timing_report_ignores_initial_partial_cycle(self):
        reporter=TimingReporter(True)
        try:
            reporter.add({'cycle':.000001,'prepare':0.,'output':0.})
            self.assertEqual(len(reporter.samples),0)
            self.assertTrue(reporter.queue.empty())
            reporter.add({'cycle':1/90,'prepare':.001,'output':.005})
            self.assertEqual(len(reporter.samples),1)
        finally:
            reporter.close();reporter.thread.join(timeout=1)

    def test_no_catchup_burst(self):
        with patch('teleop.control_timing.time.monotonic',side_effect=[0.,.1,.1]),patch('teleop.control_timing.time.sleep') as sleep:
            clock=PeriodicDeadline(90);clock.wait()
            self.assertEqual(clock.overruns,1)
            self.assertAlmostEqual(clock.deadline,.1)
            sleep.assert_not_called()


if __name__=='__main__': unittest.main()

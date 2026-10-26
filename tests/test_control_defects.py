"""Baseline controller regression tests; no USB access."""
import dataclasses
import csv
import io
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch, Mock

import numpy as np
import pinocchio as pin

from dual.tests.test_dual import FakeBus, HOME, state
from robot.arm_model import load_robot_model, joint_indices_by_name
from robot.r1arm_config import DEFAULT_URDF_PATH, ARM_JOINTS, ARM_SAFE_LIMITS, R1ArmConfig
from teleop.ik_solver import solve_upper_arm_ik
from teleop.joint_trajectory import command_interval
from teleop.r1arm_real_controller import R1ArmRealTeleopController
from robot.r1arm_sdk import model_to_sdk_q, sdk_to_model_q
from teleop.xr_snapshot import XrSnapshot


class Dls(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model,_=load_robot_model(str(DEFAULT_URDF_PATH))
        cls.indices=[joint_indices_by_name(cls.model)[name] for name in ARM_JOINTS]

    def solve(self, **kwargs):
        model=self.model; data=model.createData(); q=pin.neutral(model)
        q[self.indices]=HOME
        if 'q' in kwargs: q=kwargs.pop('q')
        pin.forwardKinematics(model,data,q);pin.updateFramePlacements(model,data)
        target=data.oMf[model.getFrameId('dex1_base_link')].translation.copy()
        args=dict(model=model,data=data,q=q,ee_frame_name='dex1_base_link',ee_target_pos=target,
            ee_target_rot=None,elbow_frame_name=None,elbow_target_pos=None,q_indices=self.indices,
            q_reference=q.copy(),control_mode='position',iterations=1,joint_limit_weight=0.)
        args.update(kwargs)
        return solve_upper_arm_ik(**args)

    def test_no_elbow_returns_configuration_and_diagnostics(self):
        q,diag=self.solve()
        self.assertTrue(np.isfinite(q).all());self.assertIn('ee_pos_error_norm',diag)

    def test_soft_limit_uses_model_joint_name(self):
        q=pin.neutral(self.model);q[self.indices]=HOME
        idx=self.indices[0];q[idx]=ARM_SAFE_LIMITS['J1'][1]+.01
        new,_=self.solve(q=q,position_weight=0.,regularization_weight=0.,joint_limit_weight=2.)
        self.assertLess(new[idx],q[idx])

    def test_damping_ramps_relative_to_threshold(self):
        _,low=self.solve(manipulability_threshold=1000.)
        _,high=self.solve(manipulability_threshold=1e-20)
        self.assertGreater(low['damping_squared'],.49)
        self.assertAlmostEqual(high['damping_squared'],.02)




class Controller(unittest.TestCase):
    def test_original_shaper_uses_trapezoidal_integration(self):
        arm = self.arm
        arm._seed_model_from_feedback(state(HOME))
        goal = arm.q.copy()
        goal[arm.arm_q_indices[0]] = .1
        arm.q = arm._trajectory_step('right', arm.q, goal, dt=.01)
        self.assertAlmostEqual(arm.arm_q()[0], .00004)
        self.assertAlmostEqual(arm._trajectory_v_cmd['right'][0], .008)
        self.assertAlmostEqual(arm._trajectory_a_cmd['right'][0], .8)
        arm.q = arm._trajectory_step('right', arm.q, goal, dt=.01)
        self.assertAlmostEqual(arm.arm_q()[0], .0002)

    def test_startup_offset_recovers_inward_and_restores_limits(self):
        arm = self.arm
        start = np.zeros(7)
        start[1], start[3] = -.0005, .003
        arm._seed_model_from_feedback(state(start))
        goal = arm.q.copy()
        goal[arm.arm_q_indices] = HOME
        previous = start.copy()
        for _ in range(450):
            arm.q = arm._trajectory_step(arm.control_side, arm.q, goal, dt=1/90)
            current = arm.arm_q().copy()
            self.assertLessEqual(current[3], max(0., previous[3])+1e-9)
            self.assertGreaterEqual(current[1], min(0., previous[1])-1e-9)
            arm.command_sdk = model_to_sdk_q(current)
            arm._update_startup_recovery_bounds()
            previous = current
        lower, upper = arm._trajectory_position_bounds
        self.assertEqual(lower[1], 0.)
        self.assertEqual(upper[3], 0.)
        self.assertLess(np.max(np.abs(current-HOME)), .01)
        start[3] = .2
        with self.assertRaises(RuntimeError):
            arm._seed_model_from_feedback(state(start))

    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.port=Path(self.tmp.name)/'fake';self.port.touch()
        self.arm=R1ArmRealTeleopController(R1ArmConfig(),port=str(self.port),baudrate=6000000,
            calibration_path='unused',mock_xr=True,motion_mode=False,rate=90,max_joint_speed=1.)
    def tearDown(self):
        # Fake buses only; avoid an unnecessary half-second shutdown hold.
        if self.arm.arm_bus is not None:self.arm.arm_bus.close();self.arm.arm_bus=None
        self.arm._on_stop();self.tmp.cleanup()

    def enable_output(self):
        arm=self.arm;arm.motion_mode=True;arm.arm_bus=FakeBus();arm.arm_bus.is_open=True
        arm.command_sdk=model_to_sdk_q(HOME)
        arm.q[arm.arm_q_indices]=HOME
        return arm

    def test_slow_dex1_startup_does_not_age_arm_feedback_or_command_clock(self):
        arm=self.arm
        arm.motion_mode=True
        arm.enable_dex1=True
        arm.bus_factory=FakeBus
        clock=[100.]
        def connect_gripper(**kwargs):
            # DDS discovery can exceed both feedback and command-period limits.
            clock[0]+=.3
            return Mock()
        with patch('robot.eef.dex1_single.Dex1Single',side_effect=connect_gripper),patch(
                'teleop.r1arm_real_controller.time.monotonic',side_effect=lambda:clock[0]),patch(
                'teleop.r1arm_real_controller.time.monotonic_ns',side_effect=lambda:int(clock[0]*1e9)),patch(
                'teleop.r1arm_real_controller.time.sleep'):
            arm._on_start()
            clock[0]+=1/90
            arm._output_command()
        self.assertEqual(len(arm.arm_bus.commands),2)
        self.assertLess(arm.control_sample()['command_dt_s'],.02)

    def test_right_trigger_publishes_gripper_without_grip_or_arm_motion(self):
        arm=self.arm
        arm.mock_xr=False
        arm.hand_ctrl=Mock(side='right')
        arm.last_gripper_update=100.
        for i, trigger in enumerate((0.,1.,.5)):
            arm.xr_client=XrSnapshot({},i,{'right_trigger':trigger,'right_grip':0.},{})
            with patch('teleop.r1arm_controller_base.time.monotonic',return_value=101.+i):
                arm._step_grippers()
            left,right=arm.hand_ctrl.set_gripper_ratios.call_args.args
            self.assertEqual(left,0.)
            self.assertAlmostEqual(right,trigger)
        arm.xr_client=None

    def test_reset_blocks_trigger_until_open_feedback_and_release(self):
        arm=self.arm
        arm.mock_xr=False
        arm.motion_mode=True
        arm.hand_ctrl=Mock(side='right',open_q=7.)
        feedback={'motor_q':6.,'feedback_valid':True}
        arm.hand_ctrl.get_record_data.return_value=feedback
        arm.xr_client=XrSnapshot({},1,{'right_trigger':1.},{})
        arm.reset()
        with patch('teleop.r1arm_controller_base.time.monotonic',return_value=100.):
            arm._step_grippers()
        arm.hand_ctrl.set_gripper_ratios.assert_not_called()
        arm._reset_blending=False
        for i,(q,trigger,waiting) in enumerate(((6.,0.,True),(7.,1.,True),(7.,0.,False),(7.,1.,False))):
            feedback['motor_q']=q
            arm.xr_client=XrSnapshot({},i,{'right_trigger':trigger},{})
            with patch('teleop.r1arm_controller_base.time.monotonic',return_value=101.+i):
                arm._step_grippers()
            self.assertEqual('right' in arm._gripper_wait_release,waiting)
            ratio=arm.hand_ctrl.set_gripper_ratios.call_args.args[1]
            self.assertAlmostEqual(ratio,1. if i==3 else 0.)
        arm.xr_client=None

    def test_gripper_csv_records_trigger_and_feedback_each_cycle(self):
        arm=self.enable_output()
        stream=io.StringIO()
        writer=csv.DictWriter(stream,fieldnames=['time_ns','home_reset','gripper_wait_release','trigger_value','dex1_feedback_valid'])
        writer.writeheader()
        arm.gripper_diagnostic_writer=writer
        arm._trigger_samples['right']=(.75,True,1.)
        arm._output_command()
        rows=list(csv.DictReader(io.StringIO(stream.getvalue())))
        self.assertEqual(len(rows),1)
        self.assertEqual(float(rows[0]['trigger_value']),.75)
        self.assertEqual(rows[0]['gripper_wait_release'],'False')
        self.assertGreater(int(rows[0]['time_ns']),0)

    def test_original_output_limits_position_and_differences_velocity_without_rewriting_shaper(self):
        arm=self.enable_output();clock=[100.]
        with patch('teleop.r1arm_real_controller.time.monotonic',side_effect=lambda:clock[0]),patch(
                'teleop.r1arm_real_controller.time.monotonic_ns',side_effect=lambda:int(clock[0]*1e9)):
            prev=arm.command_sdk.copy();v=np.zeros(7);a=np.zeros(7)
            for i in range(150):
                dt=(1/90,.004,.02)[i%3];clock[0]+=dt
                arm._step_dt=dt
                arm.q[arm.arm_q_indices[0]]=.8 if i<75 else -.8
                purpose=arm.q.copy()
                trajectory=arm._trajectory_q_cmd.get('right')
                trajectory=None if trajectory is None else trajectory.copy()
                arm._output_command()
                sample=arm.control_sample();actual_dt=sample['command_dt_s']
                q,dq,_=arm.arm_bus.commands[-1]
                np.testing.assert_allclose((q-prev)/actual_dt,dq,atol=1e-9)
                acc=(dq-v)/actual_dt
                self.assertLessEqual(np.max(np.abs(dq)),arm.max_joint_speed+1e-8)
                np.testing.assert_array_equal(arm.q,purpose)
                if trajectory is not None:
                    np.testing.assert_array_equal(arm._trajectory_q_cmd['right'],trajectory)
                np.testing.assert_allclose(arm._solver_seeds['right'][arm.arm_q_indices],sdk_to_model_q(q))
                prev,v,a=q.copy(),dq.copy(),acc.copy()
            self.assertAlmostEqual(arm._sdk_max_delta(.003),.003)

    def test_original_independent_shaper_respects_output_lead_and_speed(self):
        arm=self.enable_output();clock=[100.];locked=[True]
        send=arm.arm_bus.command_position
        def command(*args,**kwargs):
            send(*args,**kwargs)
            if locked[0]: arm.arm_bus.state=dataclasses.replace(arm.arm_bus.state,q=model_to_sdk_q(HOME))
        arm.arm_bus.command_position=command
        with patch('teleop.r1arm_real_controller.time.monotonic',side_effect=lambda:clock[0]),patch(
                'teleop.r1arm_real_controller.time.monotonic_ns',side_effect=lambda:int(clock[0]*1e9)):
            last_q=arm.command_sdk.copy();last_v=np.zeros(7);last_a=np.zeros(7)
            for i in range(350):
                locked[0]=i<175;clock[0]+=1/90;arm._step_dt=1/90
                goal=arm.q.copy();goal[arm.arm_q_indices[0]]=.6
                arm.q=arm._trajectory_step('right',arm.q,goal)
                arm._output_command();sample=arm.control_sample();dt=sample['command_dt_s']
                q,v,_=arm.arm_bus.commands[-1];a=(v-last_v)/dt
                self.assertLessEqual(np.max(np.abs(v)),arm.max_joint_speed+1e-8)
                np.testing.assert_allclose(arm._trajectory_q_cmd['right'],arm.arm_q())
                if locked[0]:self.assertLessEqual(abs(q[0]),arm.max_command_lead+1e-8)
                last_q,last_v,last_a=q.copy(),v.copy(),a.copy()
            self.assertGreater(last_q[0],.5)

    def test_explicit_timeout_rejected_but_counter_age_is_diagnostic_only(self):
        arm=self.enable_output()
        arm.arm_bus.state=dataclasses.replace(arm.arm_bus.state,timeout=np.ones(7,dtype=np.uint8))
        with self.assertRaisesRegex(RuntimeError,'timeout'):arm._output_command()
        self.assertEqual(arm.arm_bus.commands,[])
        arm.arm_bus.state=state()
        arm._feedback_last_counts=None
        arm._feedback_stream_diagnostics(arm.arm_bus.state,1_000_000_000)
        with patch('teleop.r1arm_real_controller.time.monotonic_ns',return_value=1_100_000_000):
            arm._output_command()
        self.assertEqual(len(arm.arm_bus.commands),1)
        np.testing.assert_allclose(arm.control_sample()['feedback_age_since_counter_change_s'],.1)

    def test_stalled_send_interval_refused_without_new_command(self):
        arm=self.enable_output()
        arm._last_command_monotonic_s=100.
        with patch('teleop.r1arm_real_controller.time.monotonic',return_value=100.2):
            with self.assertRaisesRegex(RuntimeError,'interval'): arm._output_command()
        self.assertEqual(arm.arm_bus.commands,[])

    def test_missing_xr_is_invalid_but_frame_gap_keeps_anchor(self):
        arm=self.arm;arm.mock_xr=False;arm.allow_xr_fallback=False
        clock=[0];arm._xr_clock_ns=lambda:clock[0]
        pose=np.array([0.,0.,1.,0.,0.,0.,1.])
        poses={n:pose.copy() for n in ('headset','left_controller','right_controller')}
        arm.xr_client=XrSnapshot(poses,1,{'right_grip':1.},{})
        self.assertTrue(arm.read_controller_poses()[1])
        arm.track['right'].ref_ee_pos=np.ones(3)
        clock[0]=300_000_000
        arm.xr_client=XrSnapshot(poses,2,{'right_grip':1.},{})
        self.assertTrue(arm.read_controller_poses()[1])
        np.testing.assert_array_equal(arm.track['right'].ref_ee_pos,np.ones(3))
        arm.xr_client=None;arm.motion_mode=True
        self.assertFalse(arm.read_controller_poses()[1])

    def test_xr_stall_and_rollback_require_fresh_recovery(self):
        from teleop.xr_snapshot import XrSnapshot
        arm=self.arm;arm.mock_xr=False;arm.allow_xr_fallback=False
        clock=[1_000_000_000];arm._xr_clock_ns=lambda:clock[0]
        poses={'right_controller':np.array([0.,0.,1.,0.,0.,0.,1.])}
        def read(stamp):
            arm.xr_client=XrSnapshot(poses,stamp,{'right_grip':1.},{})
            return arm.read_controller_poses()
        self.assertTrue(read(clock[0])[1])
        arm.track['right'].ref_ee_pos=np.ones(3)
        clock[0]+=300_000_000
        self.assertFalse(arm.read_controller_poses()[1])
        self.assertFalse(read(clock[0])[1])
        clock[0]+=50_000_000;self.assertFalse(read(clock[0])[1])
        clock[0]+=50_000_000;self.assertTrue(read(clock[0])[1])
        self.assertIsNone(arm.track['right'].ref_ee_pos)
        self.assertEqual(arm._xr_applied_generation['right'],1)
        arm.track['right'].ref_ee_pos=np.ones(3)
        clock[0]+=10_000_000;self.assertTrue(read(clock[0])[1])
        np.testing.assert_array_equal(arm.track['right'].ref_ee_pos,np.ones(3))
        clock[0]+=10_000_000;self.assertFalse(read(1)[1])
        arm.xr_client=None

    def test_shared_reader_passes_valid_pose_without_smoothing(self):
        from teleop.xr_snapshot import XrSnapshot
        arm=self.arm
        arm.mock_xr=False
        poses={name:np.array([0.,0.,1.,0.,0.,0.,1.])
               for name in ('headset','left_controller','right_controller')}
        arm.xr_client=XrSnapshot(poses,1_000_000_000,{}, {})
        self.assertTrue(arm.read_controller_poses()[1])
        poses={name:pose.copy() for name,pose in poses.items()}
        poses['right_controller'][:3]=[.02,.01,1.]
        poses['right_controller'][3:]=[0.,0.,np.sin(.05),np.cos(.05)]
        arm.xr_client=XrSnapshot(poses,1_010_000_000,{}, {})
        result,valid=arm.read_controller_poses()
        self.assertTrue(valid)
        np.testing.assert_array_equal(result['right_controller'],poses['right_controller'])
        arm.xr_client=None


if __name__=='__main__':unittest.main()

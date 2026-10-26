"""Synthetic XR state-machine and worker tests; never initialize native SDK."""
import copy
import threading
import time
import unittest
import numpy as np

from robot.r1arm_config import R1ArmConfig
from teleop.xr_input import XrInputProcessor, XrInputWorker
from teleop.xr_snapshot import XrSnapshot


def pose(x=0., angle=0.):
    return np.array([x,0.,1.,0.,0.,np.sin(angle/2),np.cos(angle/2)])


def frame(ns, right=None, left=None, stamp=None, fields=None, grip=1.):
    poses = {}
    if right is not None: poses['right_controller']=right
    if left is not None: poses['left_controller']=left
    metadata={'received_monotonic_ns':ns, 'sequence':ns}
    if fields is not None:metadata['pose_fields']=fields
    return XrSnapshot(poses,ns if stamp is None else stamp,
        {'right_grip':grip,'left_grip':1.,'right_trigger':.5},{},metadata)


class InputProcessorTests(unittest.TestCase):
    def setUp(self):
        self.config=R1ArmConfig()
        self.processor=XrInputProcessor(self.config)
        self.start=1_000_000_000

    def test_single_right_does_not_require_headset_or_left(self):
        r=self.processor.process(frame(self.start,pose()),self.start)
        self.assertTrue(r.metadata['xr_input']['right']['ready'])
        self.assertFalse(r.metadata['xr_input']['left']['ready'])
        self.assertEqual(r.keys['right_grip'],1.)

    def test_malformed_unrelated_pose_does_not_break_right(self):
        f=frame(self.start,pose())
        f.poses.update(left_controller='bad',headset=[['bad']])
        result=self.processor.process(f,self.start)
        self.assertTrue(result.metadata['xr_input']['right']['ready'])
        self.assertFalse(result.metadata['xr_input']['left']['ready'])

    def test_repeated_frame_never_refreshes_age(self):
        f=frame(self.start,pose());self.processor.process(f,self.start)
        r=self.processor.process(f,self.start+100_000_000)
        self.assertEqual(r.metadata['xr_input']['right']['phase'],'repeat')
        r=self.processor.process(f,self.start+300_000_000)
        self.assertEqual(r.metadata['xr_input']['right']['phase'],'stale')
        self.assertNotIn('right_controller',r.poses)
        self.assertEqual(r.keys['right_grip'],0.)

    def test_global_new_packet_does_not_refresh_missing_right_field(self):
        f=frame(self.start,pose(),fields={'right_controller':dict(valid=True,
            received_monotonic_ns=self.start,source_stamp_ns=self.start)})
        self.processor.process(f,self.start)
        f.stamp+=400_000_000;f.metadata['received_monotonic_ns']+=400_000_000
        r=self.processor.process(f,self.start+400_000_000)
        self.assertFalse(r.metadata['xr_input']['right']['ready'])

    def test_new_packets_with_repeated_source_stamp_do_not_refresh_pose(self):
        f=frame(self.start,pose(),fields={'right_controller':dict(valid=True,
            received_monotonic_ns=self.start,source_stamp_ns=self.start)})
        self.processor.process(f,self.start)
        f.metadata['pose_fields']['right_controller']['received_monotonic_ns']+=300_000_000
        r=self.processor.process(f,self.start+300_000_000)
        self.assertEqual(r.metadata['xr_input']['right']['phase'],'stale')

    def test_invalid_field_stops_only_its_device(self):
        r=self.processor.process(frame(self.start,pose(),np.zeros(7)),self.start)
        self.assertIn('right_controller',r.poses)
        self.assertNotIn('left_controller',r.poses)

    def test_invalid_quaternion_stops_side(self):
        f=frame(self.start,pose());self.processor.process(f,self.start)
        r=self.processor.process(frame(self.start+10_000_000,np.zeros(7)),self.start+10_000_000)
        self.assertEqual(r.metadata['xr_input']['right']['phase'],'invalid')

    def test_stall_recovery_requires_distinct_frames_and_reanchors_once(self):
        f=frame(self.start,pose());self.processor.process(f,self.start)
        self.processor.process(f,self.start+300_000_000)
        r=self.processor.process(frame(self.start+310_000_000,pose(.1)),self.start+310_000_000)
        self.assertFalse(r.metadata['xr_input']['right']['ready'])
        self.processor.process(frame(self.start+350_000_000,pose(.11)),self.start+350_000_000)
        r=self.processor.process(frame(self.start+400_000_000,pose(.12)),self.start+400_000_000)
        self.assertTrue(r.metadata['xr_input']['right']['ready'])
        self.assertEqual(r.metadata['xr_input']['right']['generation'],1)
        r=self.processor.process(frame(self.start+410_000_000,pose(.121)),self.start+410_000_000)
        self.assertEqual(r.metadata['xr_input']['right']['generation'],1)

    def test_duplicate_recovery_frame_cannot_satisfy_stability_window(self):
        self.processor.process(frame(self.start,pose()),self.start)
        f=frame(self.start+10_000_000,pose(1.))
        self.processor.process(f,self.start+10_000_000)
        r=self.processor.process(f,self.start+150_000_000)
        self.assertFalse(r.metadata['xr_input']['right']['ready'])

    def test_ten_inconsistent_outliers_are_not_blindly_accepted(self):
        self.processor.process(frame(self.start,pose()),self.start)
        for i in range(1,15):
            ns=self.start+i*20_000_000
            r=self.processor.process(frame(ns,pose(1. if i%2 else 2.)),ns)
            self.assertFalse(r.metadata['xr_input']['right']['ready'])

    def test_jump_on_left_does_not_change_right_generation(self):
        self.processor.process(frame(self.start,pose(),pose()),self.start)
        ns=self.start+10_000_000
        r=self.processor.process(frame(ns,pose(.01),pose(1.)),ns)
        self.assertTrue(r.metadata['xr_input']['right']['ready'])
        self.assertEqual(r.metadata['xr_input']['right']['generation'],0)
        self.assertFalse(r.metadata['xr_input']['left']['ready'])

    def test_timestamp_rollback_enters_recovery(self):
        self.processor.process(frame(self.start,pose()),self.start)
        r=self.processor.process(frame(self.start+10_000_000,pose(),stamp=1),self.start+10_000_000)
        self.assertFalse(r.metadata['xr_input']['right']['ready'])

    def test_grip_hysteresis(self):
        for i,(value,expected) in enumerate([(.89,0.),(.91,1.),(.85,1.),(.79,0.),(.85,0.)]):
            ns=self.start+i*10_000_000
            r=self.processor.process(frame(ns,pose(),grip=value),ns)
            self.assertEqual(r.keys['right_grip'],expected)

    def test_jump_threshold_uses_source_dt(self):
        self.assertFalse(self.processor.plausible(pose(),pose(.15),.01))
        self.assertTrue(self.processor.plausible(pose(),pose(.15),.05))
        self.assertFalse(self.processor.plausible(pose(),pose(.5),.2))


class FakeClient:
    def __init__(self):
        self.lock=threading.Lock();self.closed=False
        ns=time.monotonic_ns()
        self.frame=dict(poses={'right_controller':pose().tolist()},source_stamp_ns=ns,
            received_monotonic_ns=ns,published_monotonic_ns=ns,sequence=1,
            keys={'right_grip':1.},buttons={'B':False},button_press_counts={'B':0})
    def get_frame_snapshot(self):
        with self.lock:return copy.deepcopy(self.frame)
    def close(self):self.closed=True


class WorkerTests(unittest.TestCase):
    def test_invalid_worker_config_closes_the_client(self):
        client=FakeClient()
        with self.assertRaises(ValueError):
            XrInputWorker(client,R1ArmConfig(xr_stale_timeout_s=-1.))
        self.assertTrue(client.closed)

    def test_missed_button_presses_are_preserved_and_worker_closes(self):
        client=FakeClient();worker=XrInputWorker(client,R1ArmConfig())
        try:
            with client.lock:
                client.frame['button_press_counts']['B']=2
                client.frame['sequence']=2
            deadline=time.monotonic()+1.
            while time.monotonic()<deadline:
                with worker.lock:ready=worker.latest and worker.latest.metadata['sequence']==2
                if ready:break
                time.sleep(.001)
            self.assertTrue(ready)
            states=[worker.get_input_snapshot().buttons['B'] for _ in range(4)]
            self.assertEqual(states,[True,False,True,False])
        finally:worker.close()
        self.assertTrue(client.closed)
        self.assertFalse(worker.thread.is_alive())

    def test_stopped_worker_slot_expires_without_native_reads(self):
        client=FakeClient();worker=XrInputWorker(client,R1ArmConfig(xr_stale_timeout_s=.01))
        try:
            deadline=time.monotonic()+1.
            while not worker.latest and time.monotonic()<deadline:time.sleep(.001)
            worker.stop_event.set();worker.thread.join(timeout=1.)
            time.sleep(.02)
            snapshot=worker.get_input_snapshot()
            self.assertNotIn('right_controller',snapshot.poses)
            self.assertEqual(snapshot.keys['right_grip'],0.)
        finally:worker.close()

    def test_worker_failure_does_not_return_previous_valid_slot(self):
        client=FakeClient();worker=XrInputWorker(client,R1ArmConfig())
        try:
            def fail():raise RuntimeError('synthetic failure')
            client.get_frame_snapshot=fail
            deadline=time.monotonic()+1.
            while not worker.error and time.monotonic()<deadline:time.sleep(.001)
            r=worker.get_input_snapshot()
            self.assertFalse(r.poses)
            self.assertIn('synthetic failure',r.metadata['xr_input_error'])
        finally:worker.close()


if __name__=='__main__':unittest.main()

"""Sample native XR once per dual cycle without mixing source frames."""
import numpy as np


class XrSnapshot:
    def __init__(self, poses, stamp, keys, buttons, metadata=None):
        self.poses, self.stamp, self.keys, self.buttons = poses, stamp, keys, buttons
        self.metadata = metadata or {}

    def get_frame_stamp_ns(self):
        return self.stamp

    def get_pose_by_name(self, name):
        return self.poses.get(name)

    def get_key_value_by_name(self, name):
        return self.keys.get(name, 0.)

    def get_button_state_by_name(self, name):
        return self.buttons.get(name, False)


def snapshot_from_native(frame):
    poses = {}
    for name, value in frame['poses'].items():
        try:
            poses[name] = np.asarray(value, dtype=float).reshape(-1).copy()
        except (TypeError, ValueError):
            continue
    return XrSnapshot(poses,
                      frame['source_stamp_ns'], dict(frame['keys']), dict(frame['buttons']),
                      {k: v for k, v in frame.items() if k not in ('poses', 'keys', 'buttons')})


def sample_xr(client, previous=None):
    processed = getattr(client, 'get_input_snapshot', None)
    if processed is not None:
        return processed()
    getter = getattr(client, 'get_frame_snapshot', None)
    frame = getter() if getter is not None else None
    if frame is not None:
        return snapshot_from_native(frame)
    for _ in range(3):
        before = client.get_frame_stamp_ns()
        if previous is not None and before is not None and before == previous.stamp:
            poses = previous.poses
        else:
            poses = {}
            for name in ('headset', 'left_controller', 'right_controller'):
                try:
                    poses[name] = np.asarray(client.get_pose_by_name(name), dtype=float).reshape(-1).copy()
                except (TypeError, ValueError, KeyError):
                    continue
        keys = {f'{s}_{k}': float(client.get_key_value_by_name(f'{s}_{k}'))
                for s in ('left', 'right') for k in ('grip', 'trigger')}
        buttons = {name: bool(client.get_button_state_by_name(name)) for name in ('A', 'B', 'X', 'Y')}
        after = client.get_frame_stamp_ns()
        if before == after:
            return XrSnapshot(poses, after, keys, buttons)
    # An invalid frame releases tracking, rather than inventing poses or retaining grip.
    stopped_keys = dict(keys)
    stopped_keys.update(left_grip=0., right_grip=0.)
    return XrSnapshot({}, after, stopped_keys, buttons)

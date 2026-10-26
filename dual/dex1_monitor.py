"""Read-only DDS monitor: no command publisher and no serial port."""
import threading
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_, MotorStates_


class GripperMonitor:
    def __init__(self, network):
        ChannelFactoryInitialize(0, network)
        self.subscribers = {}
        self.latest = {}
        self.counts = {}
        self.lock = threading.Lock()
        for side in ('right', 'left'):
            for kind, topic, message in (('cmd', 'cmd', MotorCmds_), ('state', 'state', MotorStates_)):
                key = (side,kind)
                subscriber = ChannelSubscriber(f'rt/dex1/{side}/{topic}', message)
                self.counts[key] = 0
                self.subscribers[key] = subscriber
                subscriber.Init(lambda msg, key=key: self._receive(key,msg))

    def _receive(self,key,msg):
        items = getattr(msg,'cmds' if key[1]=='cmd' else 'states',None)
        if not items:
            return
        item = items[0]
        data = {field:float(getattr(item,field,0)) for field in ('q','dq','tau','tau_est','kp','kd')}
        data['received_ns'] = time.time_ns()
        data['received_monotonic'] = time.monotonic()
        with self.lock:
            self.latest[key] = data
            self.counts[key] += 1

    def snapshot(self):
        with self.lock:
            return {key: dict(value, count=self.counts[key], age=time.monotonic()-value['received_monotonic'])
                    for key, value in self.latest.items()}

    def close(self):
        for subscriber in self.subscribers.values():
            subscriber.Close()

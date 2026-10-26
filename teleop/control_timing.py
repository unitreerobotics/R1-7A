"""Deadline scheduling and bounded background timing reports."""
import json
import os
import queue
import threading
import time
from collections import deque

import numpy as np


class PeriodicDeadline:
    def __init__(self, hz):
        if not np.isfinite(hz) or hz <= 0:
            raise ValueError('Control rate must be finite and positive')
        self.period = 1. / hz
        self.deadline = time.monotonic()
        self.overruns = 0

    def wait(self):
        self.deadline += self.period
        delay = self.deadline - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        else:
            self.overruns += 1
            self.deadline = time.monotonic()  # Never issue a backlog of future commands.


class TimingReporter:
    def __init__(self, enabled=False):
        self.enabled = enabled
        self.queue = queue.Queue(maxsize=1)
        self.samples = deque(maxlen=300)
        self.last_report = time.monotonic()
        self.started = False
        self.thread = threading.Thread(target=self._run, daemon=True, name='dual-diagnostics')
        self.thread.start()

    def add(self, timing):
        if not self.enabled:
            return
        if not self.started:
            self.started = True
            self.last_report = time.monotonic()
            return  # Startup/model loading is not a completed control interval.
        self.samples.append(dict(timing))
        if time.monotonic() - self.last_report >= 1:
            try:
                self.queue.put_nowait(list(self.samples))
            except queue.Full:
                pass
            self.last_report = time.monotonic()

    def _run(self):
        while True:
            batch = self.queue.get()
            if batch is None:
                return
            report = {key + '_p95_ms': round(float(np.percentile([s[key] for s in batch], 95)) * 1000, 3)
                      for key in batch[-1]}
            report['control_hz'] = round(1 / max(np.mean([s['cycle'] for s in batch]), 1e-6), 1)
            print('[Timing] ' + json.dumps(report), flush=True)

    def close(self):
        # A daemon reporter cannot block motor shutdown on a slow stdout pipe.
        try:
            self.queue.put_nowait(None)
        except queue.Full:
            pass


def set_affinity(text):
    if text:
        cpus = {int(item) for item in text.split(',')}
        if not cpus or not cpus.issubset(os.sched_getaffinity(0)):
            raise ValueError('CPU affinity is outside the available CPU set')
        os.sched_setaffinity(0, cpus)

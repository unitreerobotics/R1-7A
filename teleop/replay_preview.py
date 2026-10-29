"""Camera playback in an isolated process so GUI work stays off replay control."""

from __future__ import annotations

import base64
import json
import multiprocessing as mp
import os
from pathlib import Path
import queue


def _read_observations(episode: str, camera_order: tuple[str, ...]):
    from mcap.reader import make_reader

    frames = []
    with Path(episode).open('rb') as stream:
        for _, channel, message in make_reader(stream).iter_messages():
            if channel.topic != '/whole_body/frame':
                continue
            row = json.loads(message.data)
            colors = row.get('colors', {})
            if colors and any(key in colors for key in camera_order):
                frames.append((int(row.get('sample_time_ns', 0)), colors))
    frames.sort(key=lambda row: row[0])
    return frames


def _mosaic(colors, index: int, total: int, camera_order: tuple[str, ...], scale: float,
            columns: int | None = None):
    import cv2
    import numpy as np

    images = []
    for key in camera_order:
        record = colors.get(key)
        if not record or record.get('encoding') != 'jpeg':
            continue
        encoded = base64.b64decode(record['data'])
        image = cv2.imdecode(np.frombuffer(encoded, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            continue
        label = '_'.join(key.split('_')[2:]) or key
        cv2.putText(image, f'{label} | {index + 1}/{total}', (10, 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 255, 0), 2, cv2.LINE_AA)
        if scale != 1.0:
            image = cv2.resize(image, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
        images.append(image)
    if not images:
        return None
    columns = columns or (1 if len(images) == 1 else 2)
    while len(images) % columns:
        images.append(np.zeros_like(images[0]))
    return np.vstack([np.hstack(images[i:i + columns]) for i in range(0, len(images), columns)])


class _BrowserPreview:
    """Serve only the latest JPEG on localhost for VS Code Simple Browser."""
    def __init__(self):
        from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
        import threading
        self.jpeg = b''
        preview = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path.split('?')[0] == '/frame.jpg':
                    body, kind = preview.jpeg, 'image/jpeg'
                elif self.path == '/':
                    body = b'<html><head><meta name="viewport" content="width=device-width,initial-scale=1"></head><body style="margin:0;width:100vw;height:100vh;overflow:hidden;background:#111;display:flex;align-items:center;justify-content:center"><img id="frame" style="display:block;width:auto;height:auto;max-width:100vw;max-height:100vh;object-fit:contain"><script>setInterval(()=>{document.getElementById("frame").src="/frame.jpg?t="+Date.now()},33)</script></body></html>'
                    kind = 'text/html'
                else:
                    self.send_error(404)
                    return
                self.send_response(200)
                self.send_header('Content-Type', kind)
                self.send_header('Cache-Control', 'no-store')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                try:
                    self.wfile.write(body)
                except (BrokenPipeError, ConnectionResetError):
                    pass

            def log_message(self, *_):
                pass

        self.server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        print(f'[Replay] VS Code: Simple Browser: Show -> http://127.0.0.1:{self.server.server_port}', flush=True)

    def show(self, image):
        import cv2
        ok, encoded = cv2.imencode('.jpg', image)
        if ok:
            self.jpeg = encoded.tobytes()

    def close(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=1)


def _preview_worker(episode, camera_order, scale, window, requests, stopping, ready,
                    quit_requested, errors, backend):
    browser = None
    try:
        if os.environ.get('WAYLAND_DISPLAY'):
            os.environ['QT_QPA_PLATFORM'] = 'xcb'
        import cv2
        import numpy as np

        observations = _read_observations(episode, camera_order)
        stamps = np.asarray([row[0] for row in observations], dtype=np.int64)
        if not observations:
            errors.put('episode contains no decodable camera observations')
            return
        if backend == 'browser':
            browser = _BrowserPreview()
        else:
            cv2.namedWindow(window, cv2.WINDOW_NORMAL)
        ready.set()
        shown = -1
        while not stopping.is_set():
            try:
                requested = requests.get(timeout=0.01)
            except queue.Empty:
                requested = None
            if requested is not None:
                index = int(np.searchsorted(stamps, int(requested), side='right')) - 1
                index = max(0, min(index, len(observations) - 1))
                if index != shown:
                    image = _mosaic(observations[index][1], index, len(observations),
                                    camera_order, scale,
                                    columns=min(3, len(camera_order)) if backend == 'browser' else None)
                    if image is not None:
                        if browser is not None:
                            browser.show(image)
                        else:
                            cv2.imshow(window, image)
                        shown = index
            if browser is None and (cv2.waitKey(1) & 0xff) in (27, ord('q')):
                quit_requested.set()
                break
        if browser is None:
            cv2.destroyWindow(window)
    except BaseException as exc:
        try:
            errors.put(f'{type(exc).__name__}: {exc}', timeout=0.1)
        except queue.Full:
            pass
    finally:
        if browser is not None:
            browser.close()
        ready.set()


class ReplayPreview:
    """Best-effort latest-frame preview. Queue work never waits on the renderer."""

    def __init__(self, episode, camera_order, *, scale=0.75, window='R1Arm recorded observations', backend='window'):
        context = mp.get_context('spawn')
        self.requests = context.Queue(maxsize=1)
        self.stopping = context.Event()
        self.ready = context.Event()
        self.quit_requested = context.Event()
        self.errors = context.Queue(maxsize=1)
        self.last_request = None
        self.reported_error = False
        self.process = context.Process(
            target=_preview_worker,
            args=(str(Path(episode).resolve()), tuple(camera_order), float(scale), window,
                  self.requests, self.stopping, self.ready, self.quit_requested, self.errors, backend),
            name='r1arm-replay-preview', daemon=True,
        )
        self.process.start()
        self.disabled = not self.ready.wait(timeout=8.0)
        if self.disabled:
            print('[Replay] camera preview did not initialize in 8s; continuing without it')
            self.close()
        else:
            self._report_error()

    def update(self, sample_time_ns: int) -> bool:
        if self.disabled:
            return False
        if self.quit_requested.is_set():
            return True
        stamp = int(sample_time_ns)
        if stamp == self.last_request:
            return False
        self.last_request = stamp
        try:
            self.requests.put_nowait(stamp)
        except queue.Full:
            try:
                self.requests.get_nowait()
            except queue.Empty:
                pass
            try:
                self.requests.put_nowait(stamp)
            except queue.Full:
                pass
        self._report_error()
        return self.quit_requested.is_set()

    def _report_error(self):
        if self.reported_error:
            return
        try:
            error = self.errors.get_nowait()
        except queue.Empty:
            if self.process.exitcode is not None and self.process.exitcode:
                error = f'preview process exited with code {self.process.exitcode}'
            else:
                return
        print(f'[Replay] camera preview stopped: {error}')
        self.reported_error = True
        self.disabled = True

    def close(self):
        self.stopping.set()
        self.process.join(timeout=2.0)
        if self.process.is_alive():
            self.process.terminate()
            self.process.join(timeout=1.0)
        for channel in (self.requests, self.errors):
            channel.close()
            channel.join_thread()

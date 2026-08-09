#!/usr/bin/env python3
"""stdlib HTTP 서버 — 정적 페이지 + SSE 텔레메트리 + MJPEG 영상.

## 왜 stdlib 인가

컨테이너에 웹 스택이 하나도 없다(flask/fastapi/rosbridge/foxglove/
web_video_server 전부 미설치). 이 저장소는 시스템 의존성을 Dockerfile 로만
추가하도록 강제하고, Dockerfile 을 바꾸면 팀 전원이 arm64 이미지를 다시 빌드해야
한다. `http.server` + SSE + MJPEG 는 **새 의존성 0개**로 같은 일을 한다
(JPEG 인코딩만 이미 있는 cv2 를 쓴다).

## 백프레셔를 만들지 않는 구조

클라이언트별 큐를 두지 않는다. ROS 콜백은 `StateStore` 에 O(1) 로 쓰기만 하고,
각 SSE 스레드가 **자기 속도로** 스냅샷을 떠 간다. 느린 클라이언트는 자기
스레드에서만 `wfile.write` 에 막히고 프레임을 건너뛴다 — 텔레메트리에서는 그게
옳은 동작이며, 실행기는 전혀 영향받지 않는다.

## 상시 연결은 2개까지

브라우저는 오리진당 동시 연결이 6개다. SSE 1 + MJPEG 1 로 고정하고 나머지는
단발 fetch 로 처리한다. 탭을 3개 이상 열면 고갈되므로 `/api/health` 가 접속 수를
노출해 운영자가 인지하게 한다.

## 읽기 전용

**GET 만 처리한다.** POST/PUT/DELETE 는 405 다. 제어 경로는 이 파일에 존재하지
않으며, 계약이 owner 를 정해 둔 토픽(`/arm_status` 등)을 발행할 방법이 없다.
"""

import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

from .video_hub import SOURCES

#: SSE 주기. hot=모터 수치·상태 스트립, full=전체(관절/검출/텔레옵/이벤트).
HOT_INTERVAL_S = 0.2
FULL_INTERVAL_S = 1.0

#: 동시 SSE 클라이언트 상한 — 탭이 쌓여도 스레드가 무한히 늘지 않게.
MAX_SSE_CLIENTS = 8
MAX_VIDEO_CLIENTS = 4

_MJPEG_BOUNDARY = 'frameboundary'

_STATIC_TYPES = {
    '.html': 'text/html; charset=utf-8',
    '.js': 'application/javascript; charset=utf-8',
    '.css': 'text/css; charset=utf-8',
    '.svg': 'image/svg+xml',
    '.png': 'image/png',
    '.ico': 'image/x-icon',
}


class MonitorHTTPServer(ThreadingHTTPServer):
    """daemon_threads 필수 — 안 그러면 스트리밍 스레드가 종료를 막는다."""

    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, addr, handler, *, store, video_hub, web_root, logger=None):
        super().__init__(addr, handler)
        self.store = store
        self.video_hub = video_hub
        self.web_root = os.path.realpath(web_root)
        self.logger = logger
        self.sse_clients = 0
        self.video_clients = 0
        self.started_at = time.time()
        self.counter_lock = threading.Lock()


class MonitorHandler(BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.1'
    server_version = 'robot_arm_gui'

    # ------------------------------------------------------------ 로깅
    def log_message(self, fmt, *args):
        """기본 구현은 요청마다 stderr 에 찍는다 — SSE/MJPEG 때문에 시끄럽다."""
        logger = getattr(self.server, 'logger', None)
        if logger is not None:
            logger.debug(f'{self.address_string()} {fmt % args}')

    # ------------------------------------------------------------ 응답 헬퍼
    def _send_json(self, obj, status=200):
        body = json.dumps(obj, ensure_ascii=False, default=str).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def _send_text(self, text, status=200, content_type='text/plain; charset=utf-8',
                   filename=None):
        body = text.encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        if filename:
            self.send_header('Content-Disposition', f'attachment; filename="{filename}"')
        self.end_headers()
        self.wfile.write(body)

    def _begin_stream(self, content_type):
        """스트리밍 응답 시작 — Content-Length 없이 연결 종료로 끝낸다."""
        self.send_response(200)
        self.send_header('Content-Type', content_type)
        self.send_header('Cache-Control', 'no-store')
        self.send_header('Connection', 'close')
        self.end_headers()
        self.close_connection = True

    # ------------------------------------------------------------ 라우팅
    def do_GET(self):
        path = urlparse(self.path).path
        try:
            if path == '/api/state':
                return self._api_state()
            if path == '/api/health':
                return self._api_health()
            if path == '/api/traces':
                return self._send_json(self.server.store.list_traces())
            if path.startswith('/api/trace/'):
                return self._api_trace(path.rsplit('/', 1)[-1])
            if path == '/api/stream':
                return self._sse()
            if path.startswith('/video/'):
                return self._mjpeg(path.rsplit('/', 1)[-1])
            return self._static(path)
        except (BrokenPipeError, ConnectionResetError):
            # 브라우저가 탭을 닫으면 정상적으로 발생한다 — 에러가 아니다.
            self.close_connection = True

    def do_POST(self):
        # 1단계는 읽기 전용이다. 제어 경로는 존재하지 않는다.
        self._send_json({'error': '읽기 전용 모니터입니다 (제어 미탑재)'}, status=405)

    do_PUT = do_POST
    do_DELETE = do_POST

    # ------------------------------------------------------------ API
    def _api_state(self):
        self._send_json(self.server.store.snapshot(time.monotonic()))

    def _api_health(self):
        srv = self.server
        self._send_json({
            'uptime_s': round(time.time() - srv.started_at, 1),
            'threads': threading.active_count(),
            'sse_clients': srv.sse_clients,
            'video_clients': srv.video_clients,
            'video': srv.video_hub.stats(),
            'limits': {'sse': MAX_SSE_CLIENTS, 'video': MAX_VIDEO_CLIENTS},
        })

    def _api_trace(self, raw_id):
        """트립 블랙박스 — 상승 엣지 직전 링버퍼를 JSONL 로 내려준다."""
        trace_id = raw_id.split('.')[0]
        if not trace_id.isdigit():
            return self._send_json({'error': 'bad trace id'}, status=400)
        trace = self.server.store.get_trace(int(trace_id))
        if trace is None:
            return self._send_json({'error': 'not found'}, status=404)
        lines = [json.dumps({'meta': {k: v for k, v in trace.items() if k != 'samples'}},
                            ensure_ascii=False)]
        lines += [json.dumps(s, ensure_ascii=False) for s in trace['samples']]
        self._send_text('\n'.join(lines) + '\n',
                        content_type='application/x-ndjson; charset=utf-8',
                        filename=f'trace_{trace_id}.jsonl')

    # ------------------------------------------------------------ SSE
    def _sse(self):
        srv = self.server
        with srv.counter_lock:
            if srv.sse_clients >= MAX_SSE_CLIENTS:
                return self._send_json({'error': 'too many SSE clients'}, status=503)
            srv.sse_clients += 1
        try:
            self._begin_stream('text/event-stream; charset=utf-8')
            # 브라우저 EventSource 자동 재접속 간격.
            self.wfile.write(b'retry: 3000\n\n')
            self.wfile.flush()

            store = srv.store
            next_hot = 0.0
            next_full = 0.0
            while True:
                now = time.monotonic()
                if now >= next_full:
                    self._sse_send('full', store.snapshot(now))
                    next_full = now + FULL_INTERVAL_S
                    next_hot = now + HOT_INTERVAL_S
                elif now >= next_hot:
                    self._sse_send('hot', store.hot_snapshot(now))
                    next_hot = now + HOT_INTERVAL_S
                time.sleep(0.02)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            with srv.counter_lock:
                srv.sse_clients -= 1
            self.close_connection = True

    def _sse_send(self, event, payload):
        body = json.dumps(payload, ensure_ascii=False, default=str)
        self.wfile.write(f'event: {event}\ndata: {body}\n\n'.encode('utf-8'))
        self.wfile.flush()

    # ------------------------------------------------------------ MJPEG
    def _mjpeg(self, source):
        if source not in SOURCES:
            return self._send_json(
                {'error': f'unknown source (가능: {sorted(SOURCES)})'}, status=404)

        srv = self.server
        with srv.counter_lock:
            if srv.video_clients >= MAX_VIDEO_CLIENTS:
                return self._send_json({'error': 'too many video clients'}, status=503)
            srv.video_clients += 1

        hub = srv.video_hub
        hub.acquire(source)
        try:
            self._begin_stream(
                f'multipart/x-mixed-replace; boundary={_MJPEG_BOUNDARY}')
            seq = 0
            idle = 0
            while True:
                seq, jpeg = hub.wait_frame(source, seq, timeout=1.0)
                if jpeg is None:
                    # 구독이 살아나기까지(노드 2Hz 조정 타이머) 잠깐 걸린다.
                    # 너무 오래 비면 클라이언트가 끊긴 것으로 보고 정리한다.
                    idle += 1
                    if idle > 30:
                        break
                    continue
                idle = 0
                head = (f'--{_MJPEG_BOUNDARY}\r\n'
                        f'Content-Type: image/jpeg\r\n'
                        f'Content-Length: {len(jpeg)}\r\n\r\n').encode('ascii')
                self.wfile.write(head)
                self.wfile.write(jpeg)
                self.wfile.write(b'\r\n')
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            hub.release(source)
            with srv.counter_lock:
                srv.video_clients -= 1
            self.close_connection = True

    # ------------------------------------------------------------ 정적 파일
    def _static(self, path):
        rel = 'index.html' if path in ('/', '') else path.lstrip('/')
        target = os.path.realpath(os.path.join(self.server.web_root, rel))
        # 경로 탈출 방어 — web_root 밖은 절대 서빙하지 않는다.
        if not target.startswith(self.server.web_root + os.sep):
            return self._send_json({'error': 'forbidden'}, status=403)
        if not os.path.isfile(target):
            return self._send_json({'error': 'not found', 'path': rel}, status=404)

        ctype = _STATIC_TYPES.get(os.path.splitext(target)[1], 'application/octet-stream')
        with open(target, 'rb') as fh:
            body = fh.read()
        self.send_response(200)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)


def serve_forever_in_thread(*, store, video_hub, web_root, bind='127.0.0.1',
                            port=8088, logger=None):
    """서버를 daemon 스레드에서 띄우고 (server, thread) 를 돌려준다.

    메인 스레드는 `rclpy.spin()` 이 가져간다 — 블록하는 쪽(HTTP)을 스레드로
    빼는 게 더 단순하고, `spin_once(0.0)` 바쁜 대기의 CPU 낭비도 없다.
    (`vision_test_node` 가 반대로 한 건 cv2.imshow 가 메인 스레드를 요구해서다.)
    """
    server = MonitorHTTPServer((bind, port), MonitorHandler, store=store,
                               video_hub=video_hub, web_root=web_root, logger=logger)
    thread = threading.Thread(target=server.serve_forever, name='monitor-http',
                              daemon=True)
    thread.start()
    return server, thread

# Copyright 2026 Meraj Safari
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
A stand-in for abb-rws-daemon, for the tests.

Python standard library only: an HTTP server on 127.0.0.1 with the WebSocket upgrade,
text frames, ping, pong and close written by hand. It lives in the test folder and is
never installed. Import it as a top-level module (``import fake_daemon``).

It applies the daemon's own rules: the Host and Origin checks, the access token on
every route but health, the stream subprotocol, and a consumer that sends a data frame
is closed with 1008. Every answer is built with the ``frames`` builders, and
``test_fake_daemon.py`` validates each of them against the vendored schema.

Routes: ``GET /v0/health``, ``GET /v0/controllers``, ``GET /v0/controllers/<id>``,
``GET /v0/controllers/<id>/io/signals``, ``POST /v0/controllers/<id>/motion/watch``,
``POST /v0/controllers/<id>/io/watch`` and ``GET /v0/stream`` (WebSocket).

A short tour::

    with fake_daemon.FakeDaemon([frames.controller('cell-1')]) as fake:
        ...                                   # a client connects to fake.url
        fake.push(frames.delta_frame('cell-1', {'motion': frames.motion_group()}))
        fake.close_stream(1001)
        assert fake.requests[0].path == '/v0/health'

The fake keeps its own controller state from the frames it was given (``apply``,
``push``), which is what a new consumer's snapshot and the controller list show.
"""

import base64
import collections
import copy
import datetime
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import re
import select
import socket
import struct
import threading
import time
from urllib.parse import unquote, urlsplit

import frames

# Synthetic tokens: they open nothing but this fake.
TOKEN = 'synthetic-token-for-tests-0123456789abcdef'
OTHER_TOKEN = 'another-synthetic-token-fedcba9876543210'

TOKEN_HEADER = 'x-abb-rws-token'
TOKEN_PROTOCOL_PREFIX = 'abb-rws-token.'
STREAM_PROTOCOL = 'abb-rws.v0'
NEW_LEASE = 'new'
LOCAL_HOSTS = frozenset({'127.0.0.1', 'localhost', '::1'})
IO_WATCH_MAX = 24
IO_WATCH_LEASE_MAX = 16
MAX_CONSUMERS = 32

CLOSE_GOING_AWAY = 1001
CLOSE_POLICY = 1008
CLOSE_REPLAY_RESTART = 1012
CLOSE_TRY_LATER = 1013

ROUTES = ('health', 'controllers', 'controller', 'signals', 'motion', 'io', 'stream')

_WS_GUID = '258EAFA5-E914-47DA-95CA-C5AB0DC85B11'
_OP_TEXT, _OP_CLOSE, _OP_PING, _OP_PONG = 0x1, 0x8, 0x9, 0xA
# What _decide answers when the request becomes a stream consumer.
_UPGRADE = object()
_LEASE_ID = re.compile(r'[A-Za-z0-9_-]{1,64}')
_SIGNAL_NAME = re.compile(r'(?:[A-Za-z0-9_]+/[A-Za-z0-9_]+/)?[A-Za-z0-9_]+')
_CONTROLLER_PATH = re.compile(r'/v0/controllers/([^/]+)(/motion/watch|/io/watch|/io/signals)?')
_NOT_A_SIGNAL = ('not an I/O signal name (letters, digits and underscores, optionally as '
                 'network/device/signal)')
_NO_SUCH_SIGNAL = 'no such signal on this controller'
_LEASE_CAP = f'lease cap reached (at most {IO_WATCH_LEASE_MAX} signals per consumer lease)'
_CONTROLLER_CAP = (f'controller cap reached (at most {IO_WATCH_MAX} signals watched on this '
                   'controller, all consumers together)')


def utc_now():
    """Return the system time as the daemon writes times (UTC, milliseconds, Z)."""
    moment = datetime.datetime.now(datetime.timezone.utc)
    return moment.strftime('%Y-%m-%dT%H:%M:%S.') + f'{moment.microsecond // 1000:03d}Z'


def wait_until(predicate, timeout=1.5, step=0.005):
    """Poll predicate until it is true; return whether it became true in time."""
    deadline = time.monotonic() + timeout
    while True:
        if predicate():
            return True
        if time.monotonic() >= deadline:
            return False
        time.sleep(step)


def error_body(code, text):
    """Return an ErrorBody for a refusal that ``frames.error_answer`` has no entry for."""
    return {'error': text, 'code': code}


class Request:
    """
    One request the fake received.

    headers has lower-case names. The token is never kept: the token header's value is
    replaced by ``matched`` or ``not matched``, which is also what ``token`` holds (None
    when the request carried no token). body is the parsed JSON, the text when it is
    not JSON, or None. status is what the fake answered: None while it has not.
    """

    def __init__(self, method, path, headers, body, token):
        self.method = method
        self.path = path
        self.headers = headers
        self.body = body
        self.token = token
        self.status = None

    def __repr__(self):
        return f'Request({self.method} {self.path}, token {self.token}, status {self.status})'


def _accept_key(key):
    digest = hashlib.sha1((key + _WS_GUID).encode('ascii')).digest()
    return base64.b64encode(digest).decode('ascii')


def _frame(opcode, payload=b''):
    """Return one unfragmented, unmasked WebSocket frame (what a server sends)."""
    size = len(payload)
    if size < 126:
        head = bytes([0x80 | opcode, size])
    elif size < 65536:
        head = bytes([0x80 | opcode, 126]) + struct.pack('!H', size)
    else:
        head = bytes([0x80 | opcode, 127]) + struct.pack('!Q', size)
    return head + payload


def _take_frames(buffer):
    """Remove every complete frame from the front of buffer; return (opcode, payload)s."""
    taken = []
    while len(buffer) >= 2:
        size = buffer[1] & 0x7F
        offset = 2
        if size == 126:
            if len(buffer) < 4:
                break
            size = struct.unpack('!H', bytes(buffer[2:4]))[0]
            offset = 4
        elif size == 127:
            if len(buffer) < 10:
                break
            size = struct.unpack('!Q', bytes(buffer[2:10]))[0]
            offset = 10
        mask = None
        if buffer[1] & 0x80:
            if len(buffer) < offset + 4:
                break
            mask = bytes(buffer[offset:offset + 4])
            offset += 4
        if len(buffer) < offset + size:
            break
        payload = bytes(buffer[offset:offset + size])
        if mask is not None:
            payload = bytes(value ^ mask[index % 4] for index, value in enumerate(payload))
        taken.append((buffer[0] & 0x0F, payload))
        del buffer[:offset + size]
    return taken


def _hostname_of(value):
    """Return the host name of a Host header, without port and brackets, lower case."""
    text = value.strip()
    bracket = re.match(r'\[([^\]]+)\]', text)
    if bracket:
        return bracket.group(1).lower()
    if text.count(':') > 1:
        return text.lower()
    return text.split(':')[0].lower()


class _Consumer:
    """One attached stream consumer."""

    def __init__(self, sock):
        self.sock = sock
        self.lock = threading.Lock()
        self.alive = True
        self.close_sent = False
        self.close_seen = False

    def send(self, data):
        with self.lock:
            if not self.alive:
                return
            try:
                self.sock.sendall(data)
            except OSError:
                self.alive = False

    def close(self, code, reason=''):
        if self.close_sent:
            return
        self.close_sent = True
        self.send(_frame(_OP_CLOSE, struct.pack('!H', code) + reason.encode('utf-8')))


class _Server(ThreadingHTTPServer):
    daemon_threads = True
    block_on_close = False
    allow_reuse_address = True
    fake = None

    def handle_error(self, request, client_address):
        # A consumer that hangs up in the middle of a request is not news.
        pass


class _Handler(BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.1'
    server_version = 'fake-daemon'
    sys_version = ''
    # Unbuffered reads: nothing behind the request head is taken from the socket, so
    # the WebSocket frames that follow an upgrade are read from the socket itself.
    rbufsize = 0

    def setup(self):
        super().setup()
        self.server.fake._track(self.connection, True)

    def finish(self):
        self.server.fake._track(self.connection, False)
        try:
            super().finish()
        except OSError:
            pass

    def log_message(self, *args):
        pass

    def _serve(self):
        self.server.fake._serve(self)

    do_GET = do_POST = do_PUT = do_DELETE = do_HEAD = do_OPTIONS = do_PATCH = _serve


class FakeDaemon:
    """
    A daemon stand-in on ``127.0.0.1``; ``start()`` it or use it as a context manager.

    controllers is a list of ControllerSnapshot dicts (``frames.controller()``): the
    state the fake starts with.

    Settings, all plain attributes that may be changed at any time:

    - ``token``: the access token the fake expects.
    - ``mode`` (``live`` or ``replay``), ``replay`` (a ``frames.replay_health()``; None
      gives the default of the mode), ``version``: what health answers.
    - ``schema_version``: what a snapshot and the controller list name.
    - ``lease_ms``, ``target_hz``: what a watch grant answers.
    - ``allowed_hosts``: the Host names that are accepted (default: the loopback names;
      an empty set refuses even ``127.0.0.1``). ``allowed_origins``: Origin values
      accepted besides the fake's own.
    - ``known_signals``: ``{controller id: set of signal names}``; a controller without
      an entry accepts every well-formed name.
    - ``max_consumers``: how many stream consumers may attach (default 32).
    - ``attach_frames``: when not None, the frames a new consumer gets instead of the
      snapshot.
    - ``upgrade_headers``: extra headers of the stream's 101 answer.
    - ``now``: a callable returning the daemon's time as ISO text (``observedAt``).
    - ``on_controller_list``: when set, called with the fake before every controller
      list answer is built (a test advances stamps there).

    What it saw: ``requests``, ``consumers``, ``attached``, ``consumer_closes``,
    ``data_frames``, ``pongs``, ``leases``.
    """

    def __init__(self, controllers=None, *, token=TOKEN, mode='live', replay=None,
                 lease_ms=frames.LEASE_MS, target_hz=frames.MOTION_HZ, port=0):
        self.token = token
        self.mode = mode
        self.replay = replay
        self.version = '0.1.0'
        self.schema_version = frames.SCHEMA_VERSION
        self.lease_ms = lease_ms
        self.target_hz = target_hz
        self.allowed_hosts = set(LOCAL_HOSTS)
        self.allowed_origins = set()
        self.known_signals = {}
        self.max_consumers = MAX_CONSUMERS
        self.attach_frames = None
        self.upgrade_headers = {}
        self.now = utc_now
        self.on_controller_list = None
        self.started_at = frames.T0
        # What the fake saw.
        self.attached = 0             # consumers that ever attached
        self.consumer_closes = []     # close codes consumers sent; None: gone without one
        self.data_frames = 0          # data frames consumers sent (each is closed with 1008)
        self.pongs = 0                # pongs consumers sent
        self._lock = threading.RLock()
        self._state = {}
        self._requests = []
        self._leases = {}
        self._minted = 0
        self._scripts = {route: collections.deque() for route in ROUTES}
        self._redirect_to = None
        self._redirect_routes = ()
        self._consumers = []
        self._connections = set()
        self._pending = []
        self._thawed = threading.Event()
        self._thawed.set()
        self._stopping = threading.Event()
        self._port = port
        self._server = None
        self._thread = None
        self._started_mono = time.monotonic()
        for snapshot in controllers or []:
            self._state[snapshot['id']] = copy.deepcopy(snapshot)

    # ----- life cycle ---------------------------------------------------------------

    def start(self):
        """
        Start listening; after ``stop()``, on the same port again.

        A fake that starts again is like a daemon that was restarted: it keeps its
        controllers and settings, knows no lease any more and is not frozen.
        """
        with self._lock:
            if self._server is not None:
                raise RuntimeError('the fake daemon is running')
            server = _Server(('127.0.0.1', self._port), _Handler)
            server.fake = self
            self._port = server.server_address[1]
            self._stopping = threading.Event()
            self._thawed.set()
            self._pending = []
            self._leases = {}
            self._started_mono = time.monotonic()
            self.started_at = self.now()
            self._server = server
            self._thread = threading.Thread(
                target=server.serve_forever, kwargs={'poll_interval': 0.01},
                name='fake-daemon', daemon=True)
            self._thread.start()
        return self

    def stop(self):
        """
        Stop like a daemon that was killed: the port closes, every connection is cut.

        Consumers get no close frame; call ``close_stream(1001)`` first for a daemon
        that stops in good order.
        """
        with self._lock:
            server, thread = self._server, self._thread
            if server is None:
                return
            self._server = None
            self._thread = None
            self._stopping.set()
        server.shutdown()
        server.server_close()
        with self._lock:
            connections = list(self._connections)
            self._consumers = []
        for sock in connections:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
        thread.join(1.0)
        wait_until(lambda: not self._connections, timeout=1.0)

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc_info):
        self.stop()

    @property
    def port(self):
        """Return the port the fake listens on (known once it was started)."""
        return self._port

    @property
    def url(self):
        """Return the fake's address as a daemon_url."""
        return f'http://127.0.0.1:{self._port}'

    @property
    def ws_url(self):
        """Return the address of the fake's stream."""
        return f'ws://127.0.0.1:{self._port}/v0/stream'

    # ----- state --------------------------------------------------------------------

    @property
    def controllers(self):
        """Return a copy of the fake's controllers, in the order they were added."""
        with self._lock:
            return copy.deepcopy(list(self._state.values()))

    def set_controllers(self, controllers):
        """Replace the fake's controllers without telling any consumer."""
        with self._lock:
            self._state = {snapshot['id']: copy.deepcopy(snapshot) for snapshot in controllers}

    def update(self, controller_id, **groups):
        """
        Replace groups (or ``name``) of one controller without telling any consumer.

        This is a daemon whose reads moved a stamp and found no news: the controller
        list shows the change, the stream sends nothing.
        """
        with self._lock:
            self._state[controller_id].update(copy.deepcopy(groups))

    def apply(self, frame):
        """Apply a stream frame to the fake's own state; nothing is sent."""
        with self._lock:
            kind = frame.get('type')
            if kind == 'snapshot':
                self._state = {snapshot['id']: copy.deepcopy(snapshot)
                               for snapshot in frame['controllers']}
            elif kind == 'delta':
                snapshot = self._state.setdefault(frame['id'], {'id': frame['id']})
                snapshot['name'] = frame['name']
                snapshot.update(copy.deepcopy(frame['groups']))
            elif kind == 'removed':
                self._state.pop(frame['id'], None)

    # ----- test controls ------------------------------------------------------------

    def push(self, frame):
        """Apply a frame to the fake's state and send it to every consumer."""
        with self._lock:
            self.apply(frame)
            self._send_all(_frame(_OP_TEXT, json.dumps(frame).encode('utf-8')))

    def play(self, frames, speed=1.0):
        """
        Push frames at their times; return the thread that does it (join it to wait).

        frames is an iterable of ``(seconds from the start, frame)``, the shape
        ``recording.read_frames`` gives. speed 2.0 plays twice as fast. Playing ends
        early when the fake stops.
        """
        if not speed > 0:
            raise ValueError('speed must be above zero')
        stopping = self._stopping

        def run():
            started = time.monotonic()
            for offset_s, frame in frames:
                delay = started + offset_s / speed - time.monotonic()
                if (delay > 0 and stopping.wait(delay)) or stopping.is_set():
                    return
                self.push(frame)

        thread = threading.Thread(target=run, name='fake-daemon-play', daemon=True)
        thread.start()
        return thread

    def send_text(self, text):
        """Send a text frame as it is to every consumer (for frames that are not JSON)."""
        with self._lock:
            self._send_all(_frame(_OP_TEXT, text.encode('utf-8')))

    def send_binary(self, data):
        """Send a binary frame to every consumer (the daemon never does)."""
        with self._lock:
            self._send_all(_frame(0x2, bytes(data)))

    def ping(self):
        """Send a ping to every consumer, as the daemon does every 30 s."""
        with self._lock:
            self._send_all(_frame(_OP_PING, b'fake'))

    def close_stream(self, code=CLOSE_GOING_AWAY, reason=''):
        """Close every consumer with a close frame: 1001, 1012, 1013, 1008, ..."""
        with self._lock:
            if not self._thawed.is_set():
                self._pending.append((code, reason))
                return
            for consumer in list(self._consumers):
                consumer.close(code, reason)

    def freeze(self):
        """
        Stop answering: sockets stay open, nothing is answered, nothing is sent.

        Requests that arrive are logged and wait; frames pushed meanwhile wait too.
        """
        self._thawed.clear()

    def unfreeze(self):
        """Answer again: what waited is answered and sent, in order."""
        with self._lock:
            pending, self._pending = self._pending, []
            for item in pending:
                for consumer in list(self._consumers):
                    if isinstance(item, tuple):
                        consumer.close(*item)
                    else:
                        consumer.send(item)
            self._thawed.set()

    @property
    def frozen(self):
        """Return whether the fake is frozen."""
        return not self._thawed.is_set()

    def redirect(self, to_url, routes=('health', 'stream')):
        """
        Answer the named routes with 302 to another address; None switches it off.

        to_url is ``http://<host>:<port>``; the Location is that address with the
        request's path (as ``ws://`` for the stream). A redirecting listener checks
        nothing: no Host, no Origin, no token.
        """
        unknown = set(routes) - set(ROUTES)
        if unknown:
            raise ValueError(f'unknown routes: {sorted(unknown)}')
        with self._lock:
            self._redirect_to = to_url
            self._redirect_routes = tuple(routes) if to_url else ()

    def script(self, route, *answers, controller_id=None):
        """
        Answer the next requests of a route with the given answers, one each.

        route is one of ROUTES; an answer is ``(status, headers, body)`` as
        ``frames.error_answer`` returns it (body: a dict, bytes, or None). For
        ``stream`` it is what the upgrade request gets instead of the stream. The Host,
        Origin and token checks still come first. With controller_id only requests for
        that controller take an answer.
        """
        if route not in ROUTES:
            raise ValueError(f'unknown route: {route}')
        with self._lock:
            for answer in answers:
                self._scripts[route].append((controller_id, answer))

    # ----- what the fake saw --------------------------------------------------------

    @property
    def requests(self):
        """Return the requests received so far, oldest first."""
        with self._lock:
            return list(self._requests)

    def requests_to(self, path, method=None):
        """Return the requests whose path (without query) is path, oldest first."""
        return [entry for entry in self.requests
                if urlsplit(entry.path).path == path and method in (None, entry.method)]

    def clear_requests(self):
        """Forget the requests received so far."""
        with self._lock:
            self._requests = []

    @property
    def consumers(self):
        """Return how many stream consumers are attached now."""
        with self._lock:
            return len(self._consumers)

    @property
    def leases(self):
        """
        Return the leases granted since the last start.

        ``{(controller id, 'motion' | 'io'): {lease id: signals}}``; the id is empty
        for the shared lease, and signals is None for a motion lease.
        """
        with self._lock:
            return copy.deepcopy(self._leases)

    def io_watched(self, controller_id):
        """Return the signals watched on a controller, every lease's together."""
        with self._lock:
            held = self._leases.get((controller_id, 'io'), {})
            names = []
            for signals in held.values():
                names.extend(name for name in signals if name not in names)
            return names

    # ----- serving ------------------------------------------------------------------

    def _track(self, sock, add):
        with self._lock:
            if add:
                self._connections.add(sock)
            else:
                self._connections.discard(sock)

    def _send_all(self, data):
        if not self._thawed.is_set():
            self._pending.append(data)
            return
        for consumer in list(self._consumers):
            consumer.send(data)

    def _log(self, handler, raw):
        headers = {}
        for name, value in handler.headers.items():
            name = name.lower()
            headers[name] = f'{headers[name]}, {value}' if name in headers else value
        token = None
        if TOKEN_HEADER in headers:
            token = 'matched' if headers[TOKEN_HEADER] == self.token else 'not matched'
            headers[TOKEN_HEADER] = token
        offered = [entry.strip() for entry in headers.get('sec-websocket-protocol', '').split(',')
                   if entry.strip()]
        for index, entry in enumerate(offered):
            if entry.startswith(TOKEN_PROTOCOL_PREFIX):
                matched = entry[len(TOKEN_PROTOCOL_PREFIX):] == self.token
                offered[index] = TOKEN_PROTOCOL_PREFIX + ('matched' if matched else 'not matched')
                if token is None:
                    token = 'matched' if matched else 'not matched'
        if offered:
            headers['sec-websocket-protocol'] = ', '.join(offered)
        body = None
        if raw:
            text = raw.decode('utf-8', 'replace')
            try:
                body = json.loads(text)
            except ValueError:
                body = text
        entry = Request(handler.command, handler.path, headers, body, token)
        with self._lock:
            self._requests.append(entry)
        return entry

    def _serve(self, handler):
        handler.close_connection = True
        declared = handler.headers.get('Content-Length', '')
        raw = b''
        if declared.isdigit():
            while len(raw) < int(declared):
                chunk = handler.rfile.read(int(declared) - len(raw))
                if not chunk:
                    break
                raw += chunk
        entry = self._log(handler, raw)
        stopping = self._stopping
        while not self._thawed.wait(0.01):
            if stopping.is_set():
                return
        if stopping.is_set() or handler.server is not self._server:
            return
        upgrade = handler.headers.get('Upgrade', '').strip().lower() == 'websocket'
        route, controller_id = self._route(handler.command, urlsplit(handler.path).path)
        status, headers, body = self._decide(handler, entry, route, controller_id, upgrade)
        if status is _UPGRADE:
            self._upgrade(handler, entry)
            return
        if isinstance(body, bytes):
            payload = body
        else:
            payload = b'' if body is None else json.dumps(body).encode('utf-8')
        entry.status = status
        try:
            handler.send_response(status)
            for name, value in headers.items():
                handler.send_header(name, value)
            if body is not None and 'Content-Type' not in headers:
                handler.send_header('Content-Type', 'application/json; charset=utf-8')
            handler.send_header('Content-Length', str(len(payload)))
            handler.send_header('Connection', 'close')
            handler.end_headers()
            handler.wfile.write(payload)
        except OSError:
            pass

    @staticmethod
    def _route(method, path):
        if method == 'GET' and path == '/v0/health':
            return 'health', None
        if method == 'GET' and path == '/v0/controllers':
            return 'controllers', None
        if method == 'GET' and path == '/v0/stream':
            return 'stream', None
        match = _CONTROLLER_PATH.fullmatch(path)
        if match:
            suffix = match.group(2)
            names = {(None, 'GET'): 'controller', ('/io/signals', 'GET'): 'signals',
                     ('/motion/watch', 'POST'): 'motion', ('/io/watch', 'POST'): 'io'}
            if (suffix, method) in names:
                return names[(suffix, method)], unquote(match.group(1))
        return None, None

    def _decide(self, handler, entry, route, controller_id, upgrade):
        """Return (status, headers, body) for a request; status _UPGRADE opens the stream."""
        with self._lock:
            if route in self._redirect_routes:
                location = self._redirect_to.rstrip('/') + handler.path
                if route == 'stream':
                    location = 'ws' + location[len('http'):]
                return 302, {'Location': location}, None
        host = handler.headers.get('Host')
        if not host or _hostname_of(host) not in self.allowed_hosts:
            return frames.error_answer('HOST_NOT_ALLOWED')
        origin = handler.headers.get('Origin')
        own = {f'http://127.0.0.1:{self._port}', f'http://localhost:{self._port}'}
        if origin and origin not in own | set(self.allowed_origins):
            return 403, {}, error_body('ORIGIN_NOT_ALLOWED', 'origin not allowed')
        if route != 'health':
            presented = handler.headers.get(TOKEN_HEADER)
            offered = [name.strip() for name in
                       handler.headers.get('Sec-WebSocket-Protocol', '').split(',')
                       if name.strip()] if upgrade else []
            by_protocol = [name[len(TOKEN_PROTOCOL_PREFIX):] for name in offered
                           if name.startswith(TOKEN_PROTOCOL_PREFIX)]
            if not presented and by_protocol:
                presented = by_protocol[0]
            if not presented:
                return frames.error_answer('TOKEN_MISSING')
            if presented != self.token:
                return frames.error_answer('TOKEN_INVALID')
            if by_protocol and STREAM_PROTOCOL not in offered:
                return 400, {}, error_body(
                    'BAD_REQUEST', 'a token offered as a subprotocol needs the subprotocol '
                    f'{STREAM_PROTOCOL} offered next to it')
        with self._lock:
            script = self._scripts.get(route)
            if script and script[0][0] in (None, controller_id):
                return script.popleft()[1]
            if route == 'health':
                return 200, {}, self._health()
            if route == 'controllers':
                if self.on_controller_list is not None:
                    self.on_controller_list(self)
                return 200, {}, frames.controller_list(
                    copy.deepcopy(list(self._state.values())), observed_at=self.now(),
                    schema_version=self.schema_version)
            if route == 'stream' and upgrade and handler.headers.get('Sec-WebSocket-Key'):
                return _UPGRADE, {}, None
            if route in ('controller', 'signals', 'motion', 'io'):
                return self._controller_route(route, controller_id, entry.body)
        return 404, {}, error_body('NO_ROUTE', f'no route: {handler.command} {handler.path}')

    def _health(self):
        uptime_s = int(time.monotonic() - self._started_mono)
        if self.replay is None:
            return frames.health(mode=self.mode, version=self.version,
                                 started_at=self.started_at, uptime_s=uptime_s)
        return frames.health(mode=self.mode, version=self.version, started_at=self.started_at,
                             uptime_s=uptime_s, replay=copy.deepcopy(self.replay))

    def _controller_route(self, route, controller_id, body):
        bad = 400, {}, error_body('BAD_REQUEST', 'the request body does not fit the route')
        lease_id = None
        signals = []
        if route == 'motion':
            if not isinstance(body, dict):
                body = {}
            if set(body) - {'leaseId'}:
                return bad
            lease_id = body.get('leaseId')
        elif route == 'io':
            if not isinstance(body, dict) or set(body) - {'signals', 'leaseId'}:
                return bad
            signals = body.get('signals')
            if (not isinstance(signals, list) or len(signals) > 256
                    or not all(isinstance(name, str) and 0 < len(name) <= 160
                               for name in signals)):
                return bad
            lease_id = body.get('leaseId')
        if lease_id is not None and not (isinstance(lease_id, str)
                                         and _LEASE_ID.fullmatch(lease_id)):
            return bad
        snapshot = self._state.get(controller_id)
        if snapshot is None:
            return frames.error_answer('NO_CONTROLLER', controller_id=controller_id)
        if route == 'controller':
            return 200, {}, copy.deepcopy(snapshot)
        check = (snapshot.get('link') or {}).get('identityCheck') or {}
        if check.get('status') == 'mismatch':
            return frames.error_answer('IDENTITY_MISMATCH', controller_id=controller_id)
        if route == 'signals':
            watched = (snapshot.get('io') or {}).get('signals') or []
            return 200, {}, frames.io_signal_table(watched)
        key = ''
        if lease_id == NEW_LEASE:
            self._minted += 1
            key = f'lease-{self._minted}'
        elif lease_id is not None:
            key = lease_id
        held = self._leases.setdefault((controller_id, route), {})
        if route == 'motion':
            held[key] = None
            return 200, {}, frames.motion_watch_answer(
                lease_ms=self.lease_ms, target_hz=self.target_hz, lease_id=key or None)
        accepted, rejected = self._grant(controller_id, signals, key, held)
        held[key] = accepted
        return 200, {}, frames.io_watch_answer(
            accepted, rejected, lease_ms=self.lease_ms, lease_id=key or None)

    def _grant(self, controller_id, requested, key, held):
        """Decide which signals a lease may hold, by the daemon's own rules."""
        known = self.known_signals.get(controller_id)
        union = set()
        for other, names in held.items():
            if other != key:
                union.update(names)
        seen = set()
        accepted = []
        rejected = []
        for raw in requested:
            name = raw.strip()
            if not name or name in seen:
                continue
            seen.add(name)
            if not _SIGNAL_NAME.fullmatch(name):
                rejected.append((name, _NOT_A_SIGNAL))
            elif known is not None and name.rsplit('/', 1)[-1] not in known:
                rejected.append((name, _NO_SUCH_SIGNAL))
            elif key and len(accepted) >= IO_WATCH_LEASE_MAX:
                rejected.append((name, _LEASE_CAP))
            elif name not in union and len(union) >= IO_WATCH_MAX:
                rejected.append((name, _CONTROLLER_CAP))
            else:
                accepted.append(name)
                union.add(name)
        return accepted, rejected

    # ----- the stream ---------------------------------------------------------------

    def _upgrade(self, handler, entry):
        offered = [name.strip() for name in
                   handler.headers.get('Sec-WebSocket-Protocol', '').split(',') if name.strip()]
        lines = ['HTTP/1.1 101 Switching Protocols', 'Upgrade: websocket', 'Connection: Upgrade',
                 'Sec-WebSocket-Accept: ' + _accept_key(handler.headers['Sec-WebSocket-Key'])]
        if STREAM_PROTOCOL in offered:
            # Never the token's subprotocol: that would send the token back.
            lines.append(f'Sec-WebSocket-Protocol: {STREAM_PROTOCOL}')
        lines.extend(f'{name}: {value}' for name, value in self.upgrade_headers.items())
        sock = handler.connection
        consumer = _Consumer(sock)
        entry.status = 101
        try:
            sock.settimeout(2.0)
            sock.sendall(('\r\n'.join(lines) + '\r\n\r\n').encode('ascii'))
        except OSError:
            return
        with self._lock:
            if len(self._consumers) >= self.max_consumers:
                consumer.close(CLOSE_TRY_LATER, 'too many stream consumers; retry later')
            else:
                self._consumers.append(consumer)
                self.attached += 1
                first = self.attach_frames
                if first is None:
                    first = [frames.snapshot_frame(
                        copy.deepcopy(list(self._state.values())), observed_at=self.now(),
                        schema_version=self.schema_version)]
                for frame in first:
                    consumer.send(_frame(_OP_TEXT, json.dumps(frame).encode('utf-8')))
        try:
            self._read_consumer(consumer)
        finally:
            with self._lock:
                if consumer in self._consumers:
                    self._consumers.remove(consumer)
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass

    def _read_consumer(self, consumer):
        """Read what a consumer sends until the connection is over."""
        sock = consumer.sock
        stopping = self._stopping
        buffer = bytearray()
        give_up_at = None
        while not stopping.is_set() and consumer.alive:
            if not self._thawed.is_set():
                time.sleep(0.005)
                continue
            if consumer.close_sent:
                # The consumer answers a close with its own; it gets half a second.
                give_up_at = give_up_at or time.monotonic() + 0.5
                if time.monotonic() > give_up_at:
                    return
            try:
                readable = select.select([sock], [], [], 0.01)[0]
                data = sock.recv(65536) if readable else None
            except (OSError, ValueError):
                return
            if data is None:
                continue
            if not data:
                if not consumer.close_seen:
                    with self._lock:
                        self.consumer_closes.append(None)
                return
            buffer += data
            for opcode, payload in _take_frames(buffer):
                if opcode == _OP_CLOSE:
                    code = struct.unpack('!H', payload[:2])[0] if len(payload) >= 2 else None
                    consumer.close_seen = True
                    with self._lock:
                        self.consumer_closes.append(code)
                    consumer.close(1000 if code is None else code)
                    return
                if opcode == _OP_PING:
                    consumer.send(_frame(_OP_PONG, payload))
                elif opcode == _OP_PONG:
                    with self._lock:
                        self.pongs += 1
                else:
                    # The stream accepts no messages.
                    with self._lock:
                        self.data_frames += 1
                    consumer.close(CLOSE_POLICY, 'the stream accepts no messages')


class Bystander:
    """
    A listener nobody should speak to: it records every connection and every byte.

    A test points a redirect at it and then checks that ``connections`` is 0 and
    ``received`` is empty. Whatever does reach it is answered with 404 and closed.
    """

    def __init__(self):
        self.connections = 0
        self.received = b''
        self._lock = threading.Lock()
        self._listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._listener.bind(('127.0.0.1', 0))
        self._listener.listen(8)
        self._listener.settimeout(0.02)
        self.port = self._listener.getsockname()[1]
        self.url = f'http://127.0.0.1:{self.port}'
        self._stopping = threading.Event()
        self._thread = threading.Thread(target=self._run, name='bystander', daemon=True)
        self._thread.start()

    def _run(self):
        while not self._stopping.is_set():
            try:
                connection, _ = self._listener.accept()
            except OSError:
                continue
            with self._lock:
                self.connections += 1
            data = b''
            try:
                connection.settimeout(0.3)
                while b'\r\n\r\n' not in data:
                    chunk = connection.recv(4096)
                    if not chunk:
                        break
                    data += chunk
                connection.sendall(b'HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\n'
                                   b'Connection: close\r\n\r\n')
            except OSError:
                pass
            finally:
                with self._lock:
                    self.received += data
                connection.close()

    def stop(self):
        """Stop listening."""
        self._stopping.set()
        self._thread.join(1.0)
        self._listener.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc_info):
        self.stop()

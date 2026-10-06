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
Client of abb-rws-daemon: the stream, the watch leases and the periodic reads.

Transport and timing only: it keeps no state of the controllers. A ``DaemonClient``
runs two threads. The stream thread connects (health first, then the token file, then
the WebSocket stream), hands every frame on and reconnects with a backoff. The lease
thread works only while the stream is connected and has delivered its snapshot: it holds
the motion and I/O watch leases, asks for the controller list at the refresh rate and
reads health every 5 s.

Everything the client learns is handed to ``sink`` as one of the event classes below,
from the client's own threads. After ``stop()`` has returned no event follows.

The access token goes to the configured daemon address and nowhere else: no redirect is
followed on any route, no proxy is used, and the token is in no log line, no event and
no error text.

Reasons of a ``Disconnected`` event (``REASONS``): ``unreachable``, ``host-refused``,
``not-a-daemon``, ``replay-refused``, ``token-file``, ``token-refused``,
``origin-refused``, ``handshake-failed``, ``protocol``, ``silent``, ``daemon-stopping``,
``replay-restart``, ``too-many-consumers``, ``policy``, ``closed``, ``refresh-failed``,
``mode-changed`` and ``stream-stalled``. The last one is never the client's own finding:
the client keeps no state of the controllers, so whoever does tells it that the stream
no longer delivers what the controller list shows (``end_stalled_stream``).

Codes of a ``LeaseResult`` that is not ok: the daemon's own code when it gave one
(``NO_CONTROLLER``, ``IDENTITY_MISMATCH``, ``LEASE_LIMIT``, ...), otherwise ``NETWORK``
(no answer), ``BAD_ANSWER`` (an answer that is not a lease) or ``HTTP_<status>``.
"""

import dataclasses
import http.client
import json
import logging
import os
import re
import socket
import struct
import threading
import time
from typing import Callable, Iterable, Optional
from urllib.parse import quote, urlsplit

from abb_rws_bridge.backoff import Backoff
from abb_rws_bridge.token_file import default_token_path, read_token, TokenFileError
import websocket

TOKEN_HEADER = 'x-abb-rws-token'
STREAM_PROTOCOL = 'abb-rws.v0'
NEW_LEASE = 'new'

HEALTH_PATH = '/v0/health'
CONTROLLERS_PATH = '/v0/controllers'
STREAM_PATH = '/v0/stream'

# No answer larger than this is read.
MAX_ANSWER_BYTES = 16 * 1024 * 1024
# The longest text a Disconnected event carries.
DETAIL_MAX = 200

# Times measured on the client's clock, in seconds.
SILENCE_S = 75.0                  # the daemon pings every 30 s; this long without a frame ends it
HEALTH_EVERY_S = 5.0              # health is read this often, connected or refused for replay
STABLE_AFTER_S = 10.0             # a connection that lasted this long resets the backoff
REPLAY_RESTART_S = 0.2            # wait after a replay that starts over (close 1012)
TOO_MANY_CONSUMERS_MIN_S = 5.0    # the shortest wait after close 1013
SLOW_RETRY_S = 15.0               # identity mismatch, and a refusal that names no time
FAILED_RETRY_S = 1.0              # a lease request that got no answer, or a 5xx
RETRY_AFTER_MAX_S = 300.0         # the longest Retry-After that is believed
MIN_RENEW_S = 0.2                 # a lease is never renewed more often than this
LEASE_MS_MAX = 3600000            # a grant that names a longer lease is no grant
# Real time, in seconds.
RELEASE_BUDGET_S = 1.0            # all the time stop() spends on freeing I/O signals

REASONS = (
    'unreachable', 'host-refused', 'not-a-daemon', 'replay-refused', 'token-file',
    'token-refused', 'origin-refused', 'handshake-failed', 'protocol', 'silent',
    'daemon-stopping', 'replay-restart', 'too-many-consumers', 'policy', 'closed',
    'refresh-failed', 'mode-changed', 'stream-stalled',
)

# One controller the client works for: its id, whether to hold a motion watch, and the
# I/O signals to watch.
Target = tuple[str, bool, tuple[str, ...]]

_LOG = logging.getLogger(__name__)

_CLOSE_REASONS = {
    1001: 'daemon-stopping',
    1008: 'policy',
    1012: 'replay-restart',
    1013: 'too-many-consumers',
}
_NOT_PRINTABLE = re.compile(r'[^\x20-\x7e]')
_ERROR_CODE = re.compile(r'[A-Z][A-Z0-9_]{0,47}')
_LEASE_ID = re.compile(r'[A-Za-z0-9_-]{1,64}')
_MODE = re.compile(r'[a-z][a-z0-9-]{0,15}')
_SECONDS = re.compile(r'[0-9]{1,6}')


@dataclasses.dataclass(frozen=True)
class Arrival:
    """When something arrived here: the system clock (ns) and the monotonic clock (s)."""

    wall_ns: int
    mono: float


@dataclasses.dataclass(frozen=True)
class HealthRead:
    """A successful GET /v0/health; health is the daemon's answer."""

    health: dict


@dataclasses.dataclass(frozen=True)
class Connected:
    """
    The stream is open. health is the answer read just before it.

    The next event of this connection is its snapshot (a FrameReceived) or, when no
    snapshot comes, its Disconnected. Refreshed and LeaseResult follow the snapshot.
    """

    health: dict


@dataclasses.dataclass(frozen=True)
class FrameReceived:
    """One stream frame, parsed; the first of a connection is its snapshot."""

    frame: dict
    at: Arrival


@dataclasses.dataclass(frozen=True)
class Refreshed:
    """An answer of GET /v0/controllers."""

    answer: dict
    at: Arrival


@dataclasses.dataclass(frozen=True)
class Disconnected:
    """
    The connection ended, or an attempt to connect failed.

    reason is one of REASONS. detail is at most 200 printable ASCII characters and
    never an answer's body or the token. The next attempt starts in retry_in_s seconds.
    """

    reason: str
    detail: str
    retry_in_s: float


@dataclasses.dataclass(frozen=True)
class LeaseResult:
    """
    What the daemon answered to one watch request.

    kind is ``motion`` or ``io``. When ok, code is empty and answer is the daemon's
    grant (for ``io`` with ``accepted`` and ``rejected``); otherwise answer is None.
    """

    controller_id: str
    kind: str
    ok: bool
    code: str
    answer: Optional[dict]


@dataclasses.dataclass(frozen=True)
class ClientSettings:
    """
    What the client needs to know.

    daemon_url is ``http://<host>:<port>``. token_file is the token file's path; when
    empty the daemon's default path is used. refresh_timeout_s is the timeout of a
    controller list request, http_timeout_s that of every other request and of the
    stream's handshake.
    """

    daemon_url: str
    token_file: str
    allow_replay: bool
    refresh_hz: float
    refresh_timeout_s: float
    reconnect_min_s: float
    reconnect_max_s: float
    http_timeout_s: float


@dataclasses.dataclass(frozen=True)
class _Timing:
    """Waits in real time. The tests shorten them; nothing else changes them."""

    tick_s: float = 0.5           # the lease thread's tick, and the longest single sleep
    stream_poll_s: float = 0.25   # how long one read of the stream blocks
    close_wait_s: float = 0.5     # how long the daemon's close frame is waited for


@dataclasses.dataclass(frozen=True)
class _End:
    """Why a connection or an attempt ended."""

    reason: str
    detail: str
    stable: bool = False


@dataclasses.dataclass(frozen=True)
class _Answer:
    """An HTTP answer: status None means there was none, and error says why."""

    status: Optional[int]
    headers: dict
    data: object
    error: str


class _AnswerTooLarge(Exception):
    pass


class _Session:
    """One open stream connection, as the two threads share it."""

    __slots__ = ('token', 'mode', 'ready', 'end', '_ending')

    def __init__(self, token: str, mode: str) -> None:
        self.token = token
        self.mode = mode
        # Set by the stream thread once the snapshot was handed on: only then does the
        # lease thread work for this connection.
        self.ready = False
        # Set once, by the lease thread or through end_stalled_stream(): the stream
        # thread then closes and reports it. The first reason stays.
        self.end: Optional[_End] = None
        self._ending = threading.Lock()

    def finish(self, reason: str, detail: str) -> None:
        with self._ending:
            if self.end is None:
                self.end = _end(reason, detail, self.token)


class _Watch:
    """One watch lease on one controller, as the lease thread tracks it."""

    __slots__ = ('due', 'signals', 'holding', 'parked_after')

    def __init__(self) -> None:
        self.due = float('-inf')      # client clock time of the next request
        self.signals: tuple = ()      # io: what the last request asked for
        self.holding = False          # io: the last grant holds signals
        self.parked_after = None      # 404: wait for a frame newer than this one


class _Book:
    """What the lease thread remembers about one connection."""

    __slots__ = ('refresh_due', 'health_due', 'refresh_failures', 'watches', 'held_until')

    def __init__(self, now: float) -> None:
        self.refresh_due = now
        self.health_due = now + HEALTH_EVERY_S
        self.refresh_failures = 0
        self.watches: dict = {}       # (controller id, kind) -> _Watch
        self.held_until: dict = {}    # controller id -> no watch request before this time


class _Connection(http.client.HTTPConnection):
    """An HTTP connection whose socket is opened by the client's own rule."""

    def __init__(self, host: str, port: int, timeout: float, opener) -> None:
        super().__init__(host, port, timeout=timeout)
        self._opener = opener

    def connect(self) -> None:
        self.sock = self._opener(self.timeout)


def _clean(text, secret: str = '') -> str:
    """Return text as a detail: the secret cut out, printable ASCII, at most 200."""
    text = str(text)
    if secret:
        text = text.replace(secret, '')
    return _NOT_PRINTABLE.sub('?', text)[:DETAIL_MAX]


def _end(reason: str, detail: str, secret: str = '') -> _End:
    return _End(reason, _clean(detail, secret))


def _exception_text(error: BaseException) -> str:
    """
    Say what went wrong, from the exception's class and the system's own words.

    An exception's message can quote what a peer sent, so it is never used as it is.
    """
    if isinstance(error, (TimeoutError, websocket.WebSocketTimeoutException)):
        return 'timed out'
    name = type(error).__name__
    if isinstance(error, OSError) and error.strerror:
        return f'{name}: {error.strerror}'
    return name


def _error_code(data) -> str:
    """Return the code of a daemon error body, or nothing when it has no usable one."""
    code = data.get('code') if isinstance(data, dict) else None
    if isinstance(code, str) and _ERROR_CODE.fullmatch(code):
        return code
    return ''


def _status_text(status, data=None) -> str:
    code = _error_code(data)
    return f'HTTP {status} {code}' if code else f'HTTP {status}'


def _not_json(name: str):
    raise ValueError(f'{name} is not JSON')


def _json(raw: bytes):
    """Return raw parsed as JSON, or None when it is not JSON (NaN and Infinity are not)."""
    try:
        return json.loads(raw.decode('utf-8'), parse_constant=_not_json)
    except (ValueError, UnicodeDecodeError, RecursionError):
        return None


def _health_of(answer: _Answer) -> Optional[dict]:
    """Return the health answer when it is one: status ok and a mode."""
    data = answer.data
    if answer.status != 200 or not isinstance(data, dict) or data.get('status') != 'ok':
        return None
    mode = data.get('mode')
    if not isinstance(mode, str) or not mode:
        return None
    return data


def _mode_text(mode: str) -> str:
    return mode if _MODE.fullmatch(mode) else 'another mode'


def _retry_after_s(headers: dict) -> float:
    """Return the Retry-After of an answer in seconds; 15 s when it names none."""
    value = str(headers.get('retry-after', '')).strip()
    if not _SECONDS.fullmatch(value):
        return SLOW_RETRY_S
    return min(RETRY_AFTER_MAX_S, max(1.0, float(value)))


def _read_limited(response) -> bytes:
    declared = (response.getheader('Content-Length') or '').strip()
    if declared.isdigit() and int(declared) > MAX_ANSWER_BYTES:
        raise _AnswerTooLarge()
    chunks = []
    total = 0
    while True:
        chunk = response.read1(65536)
        if not chunk:
            break
        total += len(chunk)
        if total > MAX_ANSWER_BYTES:
            raise _AnswerTooLarge()
        chunks.append(chunk)
    return b''.join(chunks)


def _forget_cookies() -> None:
    """
    Empty the WebSocket library's cookie jar.

    The library keeps cookies a handshake answer set, for the whole process, and sends
    them with the next handshake. The bridge sends no cookies.
    """
    jar = getattr(getattr(websocket, '_handshake', None), 'CookieJar', None)
    cookies = getattr(jar, 'jar', None)
    if isinstance(cookies, dict):
        cookies.clear()


def _endpoint(daemon_url: str) -> tuple:
    """Return (host, port) of ``http://<host>:<port>``; ValueError for anything else."""
    problem = 'daemon_url must be http://<host>:<port>'
    try:
        parts = urlsplit(daemon_url)
        port = parts.port
    except (ValueError, AttributeError, TypeError):
        raise ValueError(problem) from None
    if (parts.scheme != 'http' or not parts.hostname or port is None
            or parts.username is not None or parts.password is not None
            or parts.path not in ('', '/') or parts.query or parts.fragment):
        raise ValueError(problem)
    return parts.hostname, port


class DaemonClient:
    """
    The connection to one daemon.

    sink is called with every event, from the client's two threads. targets is called
    from the lease thread and returns the controllers to work for, as Target tuples; it
    must be safe to call from another thread. clock is the monotonic clock every wait
    is measured on, wall_ns the system clock for arrival times, and rng the random
    source of the backoff (see Backoff).
    """

    def __init__(self, settings: ClientSettings, sink: Callable[[object], None],
                 targets: Callable[[], Iterable[Target]], *,
                 clock: Callable[[], float] = time.monotonic,
                 wall_ns: Callable[[], int] = time.time_ns, rng=None) -> None:
        self._settings = settings
        self._host, self._port = _endpoint(settings.daemon_url)
        netloc = f'[{self._host}]' if ':' in self._host else self._host
        self._ws_url = f'ws://{netloc}:{self._port}{STREAM_PATH}'
        if not settings.refresh_hz > 0:
            raise ValueError('refresh_hz must be above zero')
        for name in ('refresh_timeout_s', 'http_timeout_s'):
            if not getattr(settings, name) > 0:
                raise ValueError(f'{name} must be above zero')
        self._refresh_period_s = 1.0 / settings.refresh_hz
        self._backoff = Backoff(settings.reconnect_min_s, settings.reconnect_max_s, rng)
        self._sink = sink
        self._targets = targets
        self._clock = clock
        self._wall_ns = wall_ns
        self._timing = _Timing()
        self._stop = threading.Event()
        # Set to end the lease thread's wait early: a new connection, or stop.
        self._wake = threading.Event()
        self._lock = threading.Lock()
        # Held while the current connection changes and while the lease thread hands an
        # event on, so that nothing of a connection follows its Disconnected.
        self._order = threading.Lock()
        self._session: Optional[_Session] = None
        # The one lease id of this process; empty until the daemon has granted one.
        self._lease_id = ''
        # Which stream frame last named a controller: a watch the daemon answered with
        # 404 waits for a newer one.
        self._frame_seq = 0
        self._named: dict = {}
        # Sockets in use by requests, so that stop() can end a request that hangs.
        self._sockets: dict = {'connect': set(), 'lease': set()}
        self._closed_groups: set = set()
        self._stream_thread: Optional[threading.Thread] = None
        self._lease_thread: Optional[threading.Thread] = None

    # ----- life cycle ---------------------------------------------------------------

    def start(self) -> None:
        """Start the two threads. A client is started once."""
        with self._lock:
            if self._stream_thread is not None:
                raise RuntimeError('the client was started before')
            if self._stop.is_set():
                raise RuntimeError('the client was stopped')
            self._stream_thread = threading.Thread(
                target=self._run_stream, name='abb-rws-stream', daemon=True)
            self._lease_thread = threading.Thread(
                target=self._run_leases, name='abb-rws-leases', daemon=True)
        self._stream_thread.start()
        self._lease_thread.start()

    def stop(self, timeout_s: float = 3.0) -> None:
        """
        Stop, and return within timeout_s seconds whatever the daemon does.

        Frees the watched I/O signals (one request per controller with an empty list,
        one second for all of them), closes the stream with code 1000 and ends both
        threads. A motion lease cannot be given back: it runs out by itself.
        """
        deadline = time.monotonic() + max(0.0, timeout_s)
        self._stop.set()
        self._wake.set()
        # An attempt to connect is of no use any more: end what it waits for.
        self._abort('connect')
        own = threading.current_thread()
        lease, stream = (thread if thread is not own else None
                         for thread in (self._lease_thread, self._stream_thread))
        if lease is not None:
            # The lease thread finishes its request and frees the signals. A daemon
            # that does not answer gets no more than the budget.
            grace = min(RELEASE_BUDGET_S + 0.25, 0.6 * max(0.0, deadline - time.monotonic()))
            lease.join(grace)
            if lease.is_alive():
                self._abort('lease')
        if stream is not None:
            stream.join(max(0.0, deadline - time.monotonic()))
        if lease is not None:
            lease.join(max(0.0, deadline - time.monotonic()))

    def end_stalled_stream(self, detail: str = '') -> bool:
        """
        End the current connection because its stream has stalled; safe from any thread.

        The client cannot see a stalled stream by itself: the socket is open, the
        daemon answers every request, and only the frames no longer arrive. Whoever
        keeps the state of the controllers sees it (a controller list keeps showing
        what no frame delivered) and calls this. The stream is then closed, a
        ``Disconnected`` with the reason ``stream-stalled`` follows, and the client
        connects again as after any other loss.

        Returns False when there is no connection to end: nothing happens then.
        detail goes into the event, cut and cleaned like every other detail.
        """
        with self._order:
            session = self._session
        if session is None or self._stop.is_set():
            return False
        session.finish('stream-stalled', detail or 'the stream no longer delivers frames')
        return True

    # ----- shared helpers -----------------------------------------------------------

    def _emit(self, event) -> None:
        if self._stop.is_set():
            return
        try:
            self._sink(event)
        except Exception as error:  # the sink is the node's; its failure is not ours
            _LOG.error('the event sink failed on %s: %s',
                       type(event).__name__, type(error).__name__)

    def _emit_for(self, session: _Session, event) -> None:
        """Hand on an event of the lease thread, unless its connection is over."""
        with self._order:
            if self._session is session:
                self._emit(event)

    def _sleep(self, seconds: float) -> None:
        """Wait on the client's clock, in short real waits, until then or until stop."""
        deadline = self._clock() + seconds
        while not self._stop.is_set():
            remaining = deadline - self._clock()
            if remaining <= 0:
                return
            self._stop.wait(min(remaining, self._timing.tick_s))

    def _token_path(self) -> str:
        if self._settings.token_file:
            return self._settings.token_file
        return default_token_path(os.environ, os.path.expanduser('~'))

    def _abort(self, group: str) -> None:
        """End every request of a group: what blocks on its socket fails at once."""
        with self._lock:
            self._closed_groups.add(group)
            sockets = list(self._sockets[group])
        for sock in sockets:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass

    def _open_socket(self, timeout: float, group: str) -> socket.socket:
        """
        Open a TCP connection to the daemon.

        Every address of the host is tried, and when all fail the first error is the
        one raised: the last one misleads (a name with an IPv4 and an IPv6 address ends
        on "network unreachable" although the real answer was "connection refused").
        """
        try:
            sock = socket.create_connection((self._host, self._port), timeout, all_errors=True)
        except ExceptionGroup as errors:
            raise errors.exceptions[0] from None
        with self._lock:
            refused = group in self._closed_groups
            if not refused:
                self._sockets[group].add(sock)
        if refused:
            sock.close()
            raise ConnectionAbortedError('the client is stopping')
        try:
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except OSError:
            pass
        return sock

    def _untrack(self, group: str, sock) -> None:
        with self._lock:
            self._sockets[group].discard(sock)

    def _request(self, method: str, path: str, *, token: Optional[str], timeout: float,
                 group: str, body=None) -> _Answer:
        """
        Send one HTTP request to the daemon and read the answer.

        The Host header is the one http.client derives from the address; there is no
        Origin and no cookie, and the token header only when a token is given. A
        redirect is an answer like any other: it is never followed.
        """
        headers = {'Accept': 'application/json', 'Connection': 'close'}
        if token is not None:
            headers[TOKEN_HEADER] = token
        payload = None
        if body is not None:
            payload = json.dumps(body, separators=(',', ':')).encode('utf-8')
            headers['Content-Type'] = 'application/json'
        opened = []

        def opener(connect_timeout):
            sock = self._open_socket(connect_timeout, group)
            opened.append(sock)
            return sock

        connection = _Connection(self._host, self._port, timeout, opener)
        try:
            connection.request(method, path, body=payload, headers=headers)
            response = connection.getresponse()
            try:
                status = response.status
                answer_headers = {name.lower(): value for name, value in response.getheaders()}
                raw = _read_limited(response)
            finally:
                response.close()
        except _AnswerTooLarge:
            return _Answer(None, {}, None, 'the answer is larger than 16 MiB')
        except (OSError, http.client.HTTPException) as error:
            return _Answer(None, {}, None, _exception_text(error))
        finally:
            connection.close()
            for sock in opened:
                self._untrack(group, sock)
                sock.close()
        return _Answer(status, answer_headers, _json(raw) if raw else None, '')

    # ----- stream thread ------------------------------------------------------------

    def _run_stream(self) -> None:
        while not self._stop.is_set():
            try:
                end = self._attempt()
            except Exception as error:  # a bug here must not end the bridge silently
                _LOG.error('the connect attempt failed on %s', type(error).__name__)
                end = _end('closed', f'bridge error: {type(error).__name__}')
            if end is None or self._stop.is_set():
                return
            if end.stable:
                self._backoff.reset()
            if end.reason == 'replay-restart':
                wait_s = REPLAY_RESTART_S
            elif end.reason == 'replay-refused':
                wait_s = HEALTH_EVERY_S
            elif end.reason == 'too-many-consumers':
                wait_s = max(TOO_MANY_CONSUMERS_MIN_S, self._backoff.next())
            else:
                wait_s = self._backoff.next()
            if end.reason in ('policy', 'origin-refused'):
                _LOG.error('the daemon refused the bridge as a consumer (%s, %s): a bridge bug',
                           end.reason, end.detail)
            _LOG.info('not connected to the daemon: %s (%s); next attempt in %.1f s',
                      end.reason, end.detail, wait_s)
            self._emit(Disconnected(end.reason, end.detail, wait_s))
            self._sleep(wait_s)

    def _attempt(self) -> Optional[_End]:
        """Run one connection from its health check to its end; None when stopping."""
        settings = self._settings
        # 1. Health, without the token: it tells "no daemon" from "no token" and names
        #    the mode. The token is neither read nor sent before this has succeeded.
        answer = self._request('GET', HEALTH_PATH, token=None,
                               timeout=settings.http_timeout_s, group='connect')
        if self._stop.is_set():
            return None
        if answer.status is None:
            return _end('unreachable', answer.error)
        if answer.status == 421:
            return _end('host-refused', 'health answered ' + _status_text(421, answer.data))
        health = _health_of(answer)
        if health is None:
            return _end('not-a-daemon',
                        f'health answered HTTP {answer.status}, not a health answer')
        self._emit(HealthRead(health))
        mode = health['mode']
        # 2. A recording is not a robot: without allow_replay no stream is opened.
        if mode == 'replay' and not settings.allow_replay:
            return _end('replay-refused', 'the daemon replays a recording')
        # 3. The token file, read on every attempt so that a new token is picked up.
        try:
            token = read_token(self._token_path())
        except TokenFileError as error:
            return _end('token-file', str(error))
        # 4. The stream.
        stream, end = self._open_stream(token)
        if stream is None:
            return end
        # 5. Connected: frames until the connection ends.
        session = _Session(token, mode)
        with self._lock:
            self._frame_seq = 0
            self._named = {}
        started = self._clock()
        _LOG.debug('connected to the daemon stream (mode %s)', _mode_text(mode))
        try:
            # Connected goes out before the lease thread can see the connection.
            self._emit(Connected(health))
            with self._order:
                self._session = session
            end, snapshot = self._read_stream(stream, session)
        finally:
            with self._order:
                self._session = None
            try:
                stream.shutdown()
            except Exception:  # the socket is gone either way
                pass
        if end is None:
            return None
        stable = snapshot and self._clock() - started >= STABLE_AFTER_S
        return dataclasses.replace(end, stable=stable)

    def _open_stream(self, token: str) -> tuple:
        """Open the stream; return (stream, None) or (None, why not)."""
        timeout = self._settings.http_timeout_s
        try:
            sock = self._open_socket(timeout, 'connect')
        except OSError as error:
            return None, _end('unreachable', 'stream: ' + _exception_text(error))
        try:
            _forget_cookies()
            # redirect_limit=0: the library would otherwise repeat the handshake, token
            # header included, at whatever address a redirect names. The socket is
            # ours, so no proxy setting of the environment applies.
            stream = websocket.create_connection(
                self._ws_url, timeout=timeout, header=[f'{TOKEN_HEADER}: {token}'],
                subprotocols=[STREAM_PROTOCOL], suppress_origin=True, http_no_proxy=['*'],
                redirect_limit=0, socket=sock)
        except websocket.WebSocketBadStatusException as error:
            sock.close()
            return None, self._refused_handshake(error.status_code, error.resp_body, token)
        except (websocket.WebSocketTimeoutException,
                websocket.WebSocketConnectionClosedException, OSError) as error:
            sock.close()
            return None, _end('unreachable', 'stream handshake: ' + _exception_text(error))
        except Exception as error:  # an answer that is no WebSocket handshake at all
            sock.close()
            return None, _end('handshake-failed', 'stream handshake: ' + type(error).__name__)
        finally:
            self._untrack('connect', sock)
        # With redirect_limit=0 the library hands a 3xx answer back as if it were
        # connected, so the status is checked here.
        status = stream.status
        if status != 101:
            stream.shutdown()
            return None, _end('not-a-daemon', f'stream handshake answered HTTP {status}', token)
        return stream, None

    @staticmethod
    def _refused_handshake(status, body, token: str) -> _End:
        data = _json(bytes(body)) if isinstance(body, (bytes, bytearray)) else None
        text = 'stream handshake answered ' + _status_text(status, data)
        if status == 401:
            return _end('token-refused', text, token)
        if status == 421:
            return _end('host-refused', text, token)
        if status == 403:
            return _end('origin-refused', text, token)
        if isinstance(status, int) and 300 <= status < 400:
            return _end('not-a-daemon', text, token)
        return _end('handshake-failed', text, token)

    def _read_stream(self, stream, session: _Session) -> tuple:
        """
        Hand frames on until the connection ends.

        Returns (why it ended, whether the snapshot arrived); the reason is None when
        the client is stopping. The bridge sends no data frame: only the library's
        pongs and a close frame ever leave.
        """
        stream.settimeout(self._timing.stream_poll_s)
        last = self._clock()
        snapshot = False
        while True:
            if self._stop.is_set():
                self._close(stream)
                return None, snapshot
            if session.end is not None:
                self._close(stream)
                return session.end, snapshot
            try:
                opcode, frame = stream.recv_data_frame(control_frame=True)
            except websocket.WebSocketTimeoutException:
                if self._clock() - last >= SILENCE_S:
                    self._close(stream)
                    return _end('silent', f'no frame for {SILENCE_S:.0f} s'), snapshot
                continue
            except (websocket.WebSocketConnectionClosedException, OSError) as error:
                return _end('closed', 'connection lost: ' + _exception_text(error)), snapshot
            except websocket.WebSocketException as error:
                self._close(stream, 1002)
                return _end('protocol', type(error).__name__), snapshot
            last = self._clock()
            at = Arrival(self._wall_ns(), last)
            if opcode == websocket.ABNF.OPCODE_CLOSE:
                return self._closed_by_daemon(frame.data), snapshot
            if opcode in (websocket.ABNF.OPCODE_PING, websocket.ABNF.OPCODE_PONG):
                continue
            message = _json(frame.data) if opcode == websocket.ABNF.OPCODE_TEXT else None
            if not isinstance(message, dict):
                self._close(stream, 1002)
                return _end('protocol', 'a frame that is not a JSON object'), snapshot
            if not snapshot and message.get('type') != 'snapshot':
                self._close(stream, 1002)
                return _end('protocol', 'the first frame was not a snapshot'), snapshot
            self._note(message)
            self._emit(FrameReceived(message, at))
            if not snapshot:
                # The snapshot is out: the lease thread may take the connection up.
                snapshot = True
                session.ready = True
                self._wake.set()

    @staticmethod
    def _closed_by_daemon(data) -> _End:
        code = 1005
        if isinstance(data, (bytes, bytearray)) and len(data) >= 2:
            code = struct.unpack('!H', bytes(data[:2]))[0]
        return _end(_CLOSE_REASONS.get(code, 'closed'), f'close code {code}')

    def _close(self, stream, status: int = 1000) -> None:
        try:
            stream.close(status=status, timeout=self._timing.close_wait_s)
        except Exception:  # closing a dead connection may fail in any way
            pass

    def _note(self, message: dict) -> None:
        """Remember which controllers a frame names (for watches that wait on one)."""
        kind = message.get('type')
        named = []
        if kind == 'snapshot':
            controllers = message.get('controllers')
            if isinstance(controllers, list):
                named = [entry.get('id') for entry in controllers if isinstance(entry, dict)]
        elif kind == 'delta':
            named = [message.get('id')]
        with self._lock:
            self._frame_seq += 1
            for controller_id in named:
                if isinstance(controller_id, str):
                    self._named[controller_id] = self._frame_seq

    # ----- lease thread -------------------------------------------------------------

    def _run_leases(self) -> None:
        session: Optional[_Session] = None
        book: Optional[_Book] = None
        while not self._stop.is_set():
            current = self._session
            if current is not None and not current.ready:
                current = None
            if current is not session:
                # A new connection: every lease, the refresh and health start over.
                session = current
                book = _Book(self._clock()) if session is not None else None
            pause_s = self._timing.tick_s
            if session is not None and session.end is None:
                try:
                    self._work(session, book)
                except Exception as error:  # a bug here must not end the leases silently
                    _LOG.error('the lease thread failed on %s', type(error).__name__)
                    session.finish('closed', f'bridge error: {type(error).__name__}')
                # A refresh rate above the tick's is kept: wake up when the next is due.
                pause_s = min(pause_s, max(0.001, book.refresh_due - self._clock()))
            self._wake.wait(pause_s)
            self._wake.clear()
        if session is not None and session.end is None:
            self._release(session)

    def _over(self, session: _Session) -> bool:
        return self._stop.is_set() or session.end is not None

    def _work(self, session: _Session, book: _Book) -> None:
        if self._clock() >= book.refresh_due:
            book.refresh_due = self._clock() + self._refresh_period_s
            self._refresh(session, book)
        # After a failed refresh nothing else is asked until one has answered: the
        # daemon may be gone, and another request would only wait for its timeout.
        if self._over(session) or book.refresh_failures:
            return
        if self._clock() >= book.health_due:
            book.health_due = self._clock() + HEALTH_EVERY_S
            self._health(session)
            if self._over(session):
                return
        self._leases(session, book)

    def _fatal(self, session: _Session, answer: _Answer, what: str) -> bool:
        """End the connection on an answer that says this is not our daemon any more."""
        status = answer.status
        if status == 401:
            reason = 'token-refused'
        elif status == 421:
            reason = 'host-refused'
        elif status is not None and 300 <= status < 400:
            reason = 'not-a-daemon'
        else:
            return False
        session.finish(reason, f'{what} answered ' + _status_text(status, answer.data))
        return True

    def _refresh(self, session: _Session, book: _Book) -> None:
        answer = self._request('GET', CONTROLLERS_PATH, token=session.token,
                               timeout=self._settings.refresh_timeout_s, group='lease')
        at = Arrival(self._wall_ns(), self._clock())
        if self._stop.is_set() or self._fatal(session, answer, 'the controller list'):
            return
        data = answer.data
        if (answer.status == 200 and isinstance(data, dict)
                and isinstance(data.get('controllers'), list)):
            book.refresh_failures = 0
            self._emit_for(session, Refreshed(data, at))
            return
        book.refresh_failures += 1
        if answer.status is None:
            why = answer.error
        elif answer.status == 200:
            why = 'an answer that is not a controller list'
        else:
            why = _status_text(answer.status, data)
        _LOG.debug('controller list request failed (%s)', _clean(why, session.token))
        if book.refresh_failures >= 2:
            session.finish('refresh-failed',
                           f'two controller list requests in a row failed: {why}')

    def _health(self, session: _Session) -> None:
        answer = self._request('GET', HEALTH_PATH, token=None,
                               timeout=self._settings.http_timeout_s, group='lease')
        if self._stop.is_set() or self._fatal(session, answer, 'health'):
            return
        health = _health_of(answer)
        if health is None:
            # Whether the daemon still answers is the refresh's question, not this one's.
            return
        self._emit_for(session, HealthRead(health))
        if health['mode'] != session.mode:
            session.finish('mode-changed', 'the daemon changed from '
                           f"{_mode_text(session.mode)} to {_mode_text(health['mode'])}")

    def _wanted(self) -> dict:
        """Return {controller id: (watch motion, io signals)} as targets() has it now."""
        try:
            listed = tuple(self._targets())
        except Exception as error:  # targets is the node's; its failure is not ours
            _LOG.error('targets() failed: %s', type(error).__name__)
            return {}
        wanted = {}
        for entry in listed:
            try:
                controller_id, watch_motion, signals = entry
                wanted[str(controller_id)] = (bool(watch_motion), tuple(signals))
            except (TypeError, ValueError):
                continue
        return wanted

    def _leases(self, session: _Session, book: _Book) -> None:
        wanted = self._wanted()
        for key in list(book.watches):
            controller_id, kind = key
            motion, signals = wanted.get(controller_id, (False, ()))
            if (kind == 'motion' and motion) or (kind == 'io' and signals):
                continue
            watch = book.watches.pop(key)
            if kind == 'io' and watch.holding:
                # No longer asked for: free the signals at once instead of in 15 s.
                self._post_watch(session, controller_id, 'io', ())
                if self._over(session):
                    return
        for controller_id, (motion, signals) in wanted.items():
            if motion:
                self._renew(session, book, controller_id, 'motion', ())
            if signals and not self._over(session):
                self._renew(session, book, controller_id, 'io', signals)
            if self._over(session):
                return

    def _renew(self, session: _Session, book: _Book, controller_id: str, kind: str,
               signals: tuple) -> None:
        watch = book.watches.setdefault((controller_id, kind), _Watch())
        now = self._clock()
        if kind == 'io' and watch.signals != signals:
            # Another set: the request replaces the lease's set, so it goes out now.
            watch.signals = signals
            watch.due = now
        with self._lock:
            seq = self._frame_seq
            named = self._named.get(controller_id, -1)
        if watch.parked_after is not None:
            if named <= watch.parked_after:
                return
            watch.parked_after = None
            watch.due = now
        if now < watch.due or now < book.held_until.get(controller_id, now):
            return
        answer = self._post_watch(session, controller_id, kind, signals)
        if self._stop.is_set() or self._fatal(session, answer, f'the {kind} watch'):
            return
        now = self._clock()
        data = answer.data
        status = answer.status
        granted = self._granted(status, data)
        if granted:
            self._lease_id = data['leaseId']
            watch.due = now + max(MIN_RENEW_S, data['leaseMs'] / 3000.0)
            if kind == 'io':
                watch.holding = bool(data.get('accepted'))
            self._emit_for(session, LeaseResult(controller_id, kind, True, '', data))
            return
        code = _error_code(data)
        if status is None:
            code = 'NETWORK'
        elif status == 200:
            code = 'BAD_ANSWER'
        elif not code:
            code = f'HTTP_{status}'
        if status == 404 and code == 'NO_CONTROLLER':
            # The daemon does not serve it (yet): wait for the stream to name it.
            watch.parked_after = seq
        elif status == 429:
            watch.due = now + _retry_after_s(answer.headers)
        elif status is None or status >= 500:
            # Tried again at the next tick, but no controller is asked more than once a
            # second, whichever of its two watches is due.
            watch.due = now
            book.held_until[controller_id] = now + FAILED_RETRY_S
        else:
            # 409 IDENTITY_MISMATCH (the link is on another controller) and any other
            # refusal: report it and ask again later, never anything else.
            watch.due = now + SLOW_RETRY_S
        _LOG.debug('%s watch on a controller refused (%s)', kind, code)
        self._emit_for(session, LeaseResult(controller_id, kind, False, code, None))

    @staticmethod
    def _granted(status, data) -> bool:
        if status != 200 or not isinstance(data, dict):
            return False
        lease_ms = data.get('leaseMs')
        if isinstance(lease_ms, bool) or not isinstance(lease_ms, (int, float)):
            return False
        if not 0 < lease_ms <= LEASE_MS_MAX:
            return False
        lease_id = data.get('leaseId')
        if not isinstance(lease_id, str) or lease_id == NEW_LEASE:
            return False
        return _LEASE_ID.fullmatch(lease_id) is not None

    def _post_watch(self, session: _Session, controller_id: str, kind: str, signals: tuple,
                    timeout: Optional[float] = None) -> _Answer:
        """Ask for a watch lease, or renew it, under the one lease id of this process."""
        body = {'leaseId': self._lease_id or NEW_LEASE}
        if kind == 'io':
            body = {'signals': list(signals), 'leaseId': body['leaseId']}
        path = f"{CONTROLLERS_PATH}/{quote(controller_id, safe='')}/{kind}/watch"
        if timeout is None:
            timeout = self._settings.http_timeout_s
        return self._request('POST', path, token=session.token, timeout=timeout,
                             group='lease', body=body)

    def _release(self, session: _Session) -> None:
        """Free the watched I/O signals at once: an empty set for every controller."""
        if not self._lease_id:
            return
        deadline = time.monotonic() + RELEASE_BUDGET_S
        for controller_id, (_, signals) in self._wanted().items():
            if not signals:
                continue
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            self._post_watch(session, controller_id, 'io', (),
                             timeout=min(remaining, self._settings.http_timeout_s))

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
The daemon client against the stand-in: connecting, frames, leases, timing, stopping.

Unless a test says otherwise the client runs on a clock the test moves by hand, so
nothing that depends on time happens before the test advances it, and no test waits
for real time to pass.
"""

import dataclasses
import http.client
import inspect
import logging
import random
import re
import threading
import time

from abb_rws_bridge import backoff
from abb_rws_bridge import daemon_client
from abb_rws_bridge import token_file
from abb_rws_bridge.daemon_client import (
    Arrival, ClientSettings, Connected, DaemonClient, Disconnected, FrameReceived, HealthRead,
    LeaseResult, Refreshed)
import fake_daemon
import frames
import pytest
import websocket

TOKEN = fake_daemon.TOKEN
OTHER_TOKEN = fake_daemon.OTHER_TOKEN
wait_until = fake_daemon.wait_until

# Real-time waits of a client under test: short, so that a test is over in milliseconds.
FAST = daemon_client._Timing(tick_s=0.005, stream_poll_s=0.005, close_wait_s=0.2)
WALL_NS = 1_700_000_000_000_000_000
MARKER = 'BODY-MARKER'
BUSY = (503, {}, fake_daemon.error_body('CONTROLLER_ERROR', 'synthetic'))
MISMATCH = '00000000-0000-4000-8000-000000000002'


class Events:
    """The sink of a client under test: keeps every event and lets a test wait for one."""

    def __init__(self):
        self._items = []
        self._changed = threading.Condition()

    def __call__(self, event):
        with self._changed:
            self._items.append(event)
            self._changed.notify_all()

    def of(self, kind=object):
        """Return the events of a kind so far, oldest first."""
        with self._changed:
            return [event for event in self._items if isinstance(event, kind)]

    def mark(self):
        """Return how many events there are: what ``after`` takes."""
        with self._changed:
            return len(self._items)

    def wait(self, kind, match=None, *, after=0, timeout=1.5):
        """Return the first event of a kind (that matches) at or after an index."""
        deadline = time.monotonic() + timeout
        with self._changed:
            while True:
                for event in self._items[after:]:
                    if isinstance(event, kind) and (match is None or match(event)):
                        return event
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    seen = [type(event).__name__ + getattr(event, 'reason', '')
                            for event in self._items[after:]]
                    raise AssertionError(f'no {kind.__name__} within {timeout} s; saw {seen}')
                self._changed.wait(remaining)


class ManualClock:
    """A monotonic clock that only a test moves."""

    def __init__(self, now=1000.0):
        self._now = now
        self._lock = threading.Lock()

    def __call__(self):
        with self._lock:
            return self._now

    def advance(self, seconds):
        with self._lock:
            self._now += seconds


class FixedRng:
    """A random source without randomness: every backoff factor is 1."""

    def uniform(self, low, high):
        return 1.0


class Bench:
    """A fake daemon, a token file, an event sink and the clients a test makes."""

    def __init__(self, tmp_path):
        self.fake = fake_daemon.FakeDaemon([frames.controller('cell-1')]).start()
        self.token_file = tmp_path / 'token'
        self.token_file.write_text(self.fake.token + '\n', encoding='ascii')
        self.events = Events()
        self.clock = ManualClock()
        self.targets = ()
        self.clients = []

    def settings(self, **changes):
        base = ClientSettings(
            daemon_url=self.fake.url, token_file=str(self.token_file), allow_replay=False,
            refresh_hz=2.0, refresh_timeout_s=1.0, reconnect_min_s=0.5, reconnect_max_s=15.0,
            http_timeout_s=1.0)
        return dataclasses.replace(base, **changes)

    def client(self, *, clock=None, rng=None, start=True, sink=None, targets=None, **changes):
        made = DaemonClient(
            self.settings(**changes), self.events if sink is None else sink,
            (lambda: self.targets) if targets is None else targets,
            clock=self.clock if clock is None else clock, wall_ns=lambda: WALL_NS,
            rng=FixedRng() if rng is None else rng)
        made._timing = FAST
        self.clients.append(made)
        if start:
            made.start()
        return made

    def connected(self, **options):
        """
        Start a client; wait for its snapshot and its first controller list.

        The first controller list shows that the lease thread has taken the connection
        up: from then on a test may move the clock and count on what is due.
        """
        made = self.client(**options)
        self.events.wait(FrameReceived)
        self.events.wait(Refreshed)
        return made

    def reconnect(self, ended):
        """Move the clock past a Disconnected's wait; return the index to wait from."""
        mark = self.events.mark()
        self.clock.advance(ended.retry_in_s)
        return mark

    def posts(self, kind=None, controller_id='cell-1'):
        """Return the watch requests the fake received (of one kind, for one controller)."""
        return [entry for entry in self.fake.requests
                if entry.method == 'POST' and f'/{controller_id}/' in entry.path
                and (kind is None or entry.path.endswith(f'/{kind}/watch'))]

    def close(self):
        for made in self.clients:
            made.stop(1.0)
        self.fake.stop()


@pytest.fixture
def bench(tmp_path):
    made = Bench(tmp_path)
    yield made
    made.close()


def steady(read, pause=0.05):
    """Return whether a value stays the same over a short real pause."""
    before = read()
    time.sleep(pause)
    return read() == before


def stopped(client):
    return not client._stream_thread.is_alive() and not client._lease_thread.is_alive()


# ----- connecting -------------------------------------------------------------------

def test_health_comes_first_and_the_token_only_after_it(bench):
    bench.connected()
    log = bench.fake.requests
    assert [(entry.method, entry.path, entry.token) for entry in log[:2]] == [
        ('GET', '/v0/health', None), ('GET', '/v0/stream', 'matched')]
    assert 'x-abb-rws-token' not in log[0].headers
    assert log[1].headers['sec-websocket-protocol'] == 'abb-rws.v0'
    events = bench.events.of()
    # The snapshot is the first thing a connection hands on, before any controller list.
    assert [type(event) for event in events[:4]] == [
        HealthRead, Connected, FrameReceived, Refreshed]
    health = events[0].health
    assert health['status'] == 'ok' and health['mode'] == 'live'
    assert frames.schema_errors('HealthAnswer', health) == []
    assert events[1].health == health
    received = bench.events.of(FrameReceived)[0]
    assert received.frame['type'] == 'snapshot'
    assert [entry['id'] for entry in received.frame['controllers']] == ['cell-1']
    assert received.at == Arrival(wall_ns=WALL_NS, mono=1000.0)
    assert frames.schema_errors('StreamFrame', received.frame) == []


def test_frames_are_handed_on_in_order_after_the_snapshot(bench):
    bench.connected()
    deltas = [frames.delta_frame('cell-1', {'motion': frames.motion_group(as_of=frames.iso(n))},
                                 observed_at=frames.iso(n)) for n in range(1, 6)]
    bench.clock.advance(3.0)
    for delta in deltas:
        bench.fake.push(delta)
    last = bench.events.wait(FrameReceived, lambda event: event.frame == deltas[-1])
    received = [event.frame for event in bench.events.of(FrameReceived)]
    assert received[0]['type'] == 'snapshot' and received[1:] == deltas
    assert last.at == Arrival(wall_ns=WALL_NS, mono=1003.0)
    # A frame type the bridge does not know is handed on too: the state ignores it.
    bench.fake.push({'type': 'something-new', 'observedAt': frames.iso(9), 'more': [1]})
    bench.events.wait(FrameReceived, lambda event: event.frame.get('type') == 'something-new')
    removed = frames.removed_frame('cell-1')
    bench.fake.push(removed)
    bench.events.wait(FrameReceived, lambda event: event.frame == removed)
    assert not bench.events.of(Disconnected)


def test_requests_carry_no_origin_no_cookie_and_the_host_of_the_url(bench, monkeypatch):
    # A dead proxy in the environment changes nothing: no request goes through one.
    for name in ('http_proxy', 'HTTP_PROXY', 'https_proxy', 'HTTPS_PROXY', 'all_proxy',
                 'ALL_PROXY', 'ws_proxy', 'WS_PROXY'):
        monkeypatch.setenv(name, 'http://127.0.0.1:9')
    monkeypatch.setenv('no_proxy', '')
    monkeypatch.setenv('NO_PROXY', '')
    bench.targets = (('cell-1', True, ('do_gripper',)),)
    bench.connected()
    bench.events.wait(LeaseResult, lambda event: event.kind == 'io')
    bench.clock.advance(5.0)
    assert wait_until(lambda: len(bench.fake.requests_to('/v0/health')) == 2)
    bench.fake.ping()
    assert wait_until(lambda: bench.fake.pongs == 1)
    log = bench.fake.requests
    assert {entry.path for entry in log} == {
        '/v0/health', '/v0/stream', '/v0/controllers', '/v0/controllers/cell-1/motion/watch',
        '/v0/controllers/cell-1/io/watch'}
    for entry in log:
        assert entry.headers['host'] == f'127.0.0.1:{bench.fake.port}', entry
        assert not {'origin', 'cookie', 'proxy-authorization', 'authorization',
                    'referer'} & set(entry.headers), entry
        assert entry.status in (200, 101)
        assert entry.token == (None if entry.path == '/v0/health' else 'matched')
    posts = [entry for entry in log if entry.method == 'POST']
    assert posts and all(entry.headers['content-type'] == 'application/json' for entry in posts)
    # The bridge never sends a data frame: the daemon would close it for one.
    assert bench.fake.data_frames == 0 and not bench.events.of(Disconnected)


def test_the_host_header_follows_the_name_in_the_url(bench):
    bench.connected(daemon_url=f'http://localhost:{bench.fake.port}/')
    bench.events.wait(Refreshed)
    assert len(bench.fake.requests) >= 3
    assert all(entry.headers['host'] == f'localhost:{bench.fake.port}'
               for entry in bench.fake.requests)


def test_no_cookie_is_ever_sent_back(bench):
    bench.fake.upgrade_headers = {'Set-Cookie': 'crumb=1; Domain=127.0.0.1; Path=/'}
    try:
        # Left alone, the library keeps the cookie and sends it with its next handshake.
        for _ in range(2):
            websocket.create_connection(
                bench.fake.ws_url, timeout=2.0, header=[f'x-abb-rws-token: {TOKEN}'],
                subprotocols=['abb-rws.v0'], suppress_origin=True,
                http_no_proxy=['*']).shutdown()
        assert bench.fake.requests[-1].headers.get('cookie') == 'crumb=1'
        bench.fake.clear_requests()
        bench.connected()
        mark = bench.events.mark()
        bench.fake.close_stream(1012)
        mark = bench.reconnect(bench.events.wait(Disconnected, after=mark))
        bench.events.wait(Connected, after=mark)
        upgrades = bench.fake.requests_to('/v0/stream')
        assert len(upgrades) == 2
        assert all('cookie' not in entry.headers for entry in upgrades)
    finally:
        daemon_client._forget_cookies()


def test_a_daemon_that_is_not_there_is_unreachable(bench):
    bench.fake.stop()
    bench.client()
    ended = bench.events.wait(Disconnected)
    assert ended == Disconnected('unreachable', 'ConnectionRefusedError: Connection refused', 0.5)
    assert bench.events.of() == [ended] and bench.fake.requests == []


HEALTH_FAILURES = {
    'host refused': (frames.error_answer('HOST_NOT_ALLOWED'), 'host-refused',
                     'health answered HTTP 421 HOST_NOT_ALLOWED'),
    'server error': ((500, {}, None), 'not-a-daemon',
                     'health answered HTTP 500, not a health answer'),
    'no such route': ((404, {}, fake_daemon.error_body('NO_ROUTE', MARKER)), 'not-a-daemon',
                      'health answered HTTP 404, not a health answer'),
    'moved': ((301, {'Location': 'http://127.0.0.1:9/v0/health'}, None), 'not-a-daemon',
              'health answered HTTP 301, not a health answer'),
    'no mode': ((200, {}, {'status': 'ok'}), 'not-a-daemon',
                'health answered HTTP 200, not a health answer'),
    'empty mode': ((200, {}, {'status': 'ok', 'mode': ''}), 'not-a-daemon',
                   'health answered HTTP 200, not a health answer'),
    'not ok': ((200, {}, {'status': 'starting', 'mode': 'live'}), 'not-a-daemon',
               'health answered HTTP 200, not a health answer'),
    'a list': ((200, {}, [1, 2]), 'not-a-daemon',
               'health answered HTTP 200, not a health answer'),
    'a web page': ((200, {'Content-Type': 'text/html'}, b'<html>' + MARKER.encode() + b'</html>'),
                   'not-a-daemon', 'health answered HTTP 200, not a health answer'),
}


@pytest.mark.parametrize('label', sorted(HEALTH_FAILURES))
def test_no_token_is_read_or_sent_before_health_has_succeeded(bench, monkeypatch, label):
    answer, reason, detail = HEALTH_FAILURES[label]
    reads = []
    real = daemon_client.read_token
    monkeypatch.setattr(daemon_client, 'read_token', lambda path: reads.append(path) or real(path))
    bench.fake.script('health', answer)
    bench.client()
    ended = bench.events.wait(Disconnected)
    assert ended == Disconnected(reason, detail, 0.5)
    assert reads == []
    assert [(entry.path, entry.token) for entry in bench.fake.requests] == [('/v0/health', None)]
    assert bench.events.of() == [ended]
    # The next attempt finds a daemon: only now is the file read and the token sent.
    mark = bench.reconnect(ended)
    bench.events.wait(Connected, after=mark)
    assert reads == [str(bench.token_file)]
    assert bench.fake.requests[-1].token == 'matched'


def test_a_refused_host_sends_no_token(bench):
    bench.fake.allowed_hosts = set()
    bench.client()
    ended = bench.events.wait(Disconnected)
    assert (ended.reason, ended.detail) == (
        'host-refused', 'health answered HTTP 421 HOST_NOT_ALLOWED')
    mark = bench.reconnect(ended)
    bench.events.wait(Disconnected, after=mark)
    assert len(bench.fake.requests) == 2
    assert all(entry.token is None and entry.path == '/v0/health'
               for entry in bench.fake.requests)


HANDSHAKES = {
    'another token': (frames.error_answer('TOKEN_INVALID'), 'token-refused',
                      'HTTP 401 TOKEN_INVALID'),
    'no token': (frames.error_answer('TOKEN_MISSING'), 'token-refused', 'HTTP 401 TOKEN_MISSING'),
    'host': (frames.error_answer('HOST_NOT_ALLOWED'), 'host-refused',
             'HTTP 421 HOST_NOT_ALLOWED'),
    'origin': ((403, {}, fake_daemon.error_body('ORIGIN_NOT_ALLOWED', MARKER)),
               'origin-refused', 'HTTP 403 ORIGIN_NOT_ALLOWED'),
    'not modified': ((304, {}, None), 'not-a-daemon', 'HTTP 304'),
    'found': ((302, {'Location': 'ws://127.0.0.1:9/v0/stream'}, None), 'not-a-daemon',
              'HTTP 302'),
    'permanent': ((308, {'Location': 'ws://127.0.0.1:9/v0/stream'}, None), 'not-a-daemon',
                  'HTTP 308'),
    'bad request': ((400, {}, fake_daemon.error_body('BAD_REQUEST', MARKER)),
                    'handshake-failed', 'HTTP 400 BAD_REQUEST'),
    'server error': ((500, {}, None), 'handshake-failed', 'HTTP 500'),
    'plain answer': ((200, {}, {'status': 'ok', 'mode': 'live'}), 'handshake-failed', 'HTTP 200'),
}


@pytest.mark.parametrize('label', sorted(HANDSHAKES))
def test_each_refused_handshake_has_its_reason(bench, label):
    answer, reason, text = HANDSHAKES[label]
    bench.fake.script('stream', answer)
    bench.client()
    ended = bench.events.wait(Disconnected)
    assert ended == Disconnected(reason, 'stream handshake answered ' + text, 0.5)
    assert [type(event) for event in bench.events.of()] == [HealthRead, Disconnected]
    assert bench.fake.consumers == 0


def test_a_handshake_answer_that_is_no_handshake_fails(bench):
    # 101 without the accept key of the handshake.
    bench.fake.script('stream', (101, {'Upgrade': 'websocket', 'Connection': 'Upgrade'}, None))
    bench.client()
    ended = bench.events.wait(Disconnected)
    assert ended.reason == 'handshake-failed'
    assert ended.detail == 'stream handshake: WebSocketException'


def test_a_wrong_token_in_the_file_is_refused_by_the_daemon(bench):
    bench.token_file.write_text(OTHER_TOKEN + '\n', encoding='ascii')
    bench.client()
    ended = bench.events.wait(Disconnected)
    assert (ended.reason, ended.detail) == (
        'token-refused', 'stream handshake answered HTTP 401 TOKEN_INVALID')
    assert bench.fake.requests_to('/v0/stream')[0].token == 'not matched'


def test_a_token_file_problem_names_the_path_and_the_file_is_tried_again(bench):
    bench.token_file.unlink()
    bench.client()
    ended = bench.events.wait(Disconnected)
    assert ended.reason == 'token-file'
    assert ended.detail == f'token file {bench.token_file} does not exist ' \
                           '(the daemon makes it when it starts)'
    garbage = 'not-a-token-but-some-other-text'
    bench.token_file.write_text(garbage, encoding='ascii')
    mark = bench.reconnect(ended)
    ended = bench.events.wait(Disconnected, after=mark)
    assert ended.reason == 'token-file' and str(bench.token_file) in ended.detail
    assert 'does not hold an access token' in ended.detail
    assert garbage not in ended.detail and str(len(garbage)) not in ended.detail.replace(
        str(bench.token_file), '').replace('32 to 256', '')
    # Health was asked each time; nothing else was, and nothing carried a token.
    assert [(entry.path, entry.token) for entry in bench.fake.requests] == [
        ('/v0/health', None), ('/v0/health', None)]
    bench.token_file.write_text('\ufeff' + TOKEN + '\r\n', encoding='utf-8')
    mark = bench.reconnect(ended)
    bench.events.wait(Connected, after=mark)
    assert bench.fake.requests[-1].token == 'matched'


def test_without_a_configured_path_the_default_token_file_is_used(bench, monkeypatch, tmp_path):
    monkeypatch.setenv('ABB_RWS_DAEMON_TOKEN_FILE', str(bench.token_file))
    bench.connected(token_file='')
    assert not bench.events.of(Disconnected)
    # Without the variable: the daemon's own place under the config home.
    monkeypatch.delenv('ABB_RWS_DAEMON_TOKEN_FILE')
    monkeypatch.setenv('XDG_CONFIG_HOME', str(tmp_path / 'xdg'))
    bench.client(token_file='')
    ended = bench.events.wait(Disconnected)
    expected = tmp_path / 'xdg' / 'abb-rws-daemon' / 'token'
    assert ended.reason == 'token-file' and f'token file {expected} ' in ended.detail


def test_the_first_error_of_all_addresses_is_the_one_reported(bench, monkeypatch):
    calls = []

    def refuse(address, timeout=None, source_address=None, *, all_errors=False):
        calls.append((address, timeout, all_errors))
        raise ExceptionGroup('create_connection failed', [
            ConnectionRefusedError(111, 'Connection refused'),
            OSError(101, 'Network is unreachable')])

    monkeypatch.setattr(daemon_client.socket, 'create_connection', refuse)
    bench.client(http_timeout_s=0.7)
    ended = bench.events.wait(Disconnected)
    assert (ended.reason, ended.detail) == (
        'unreachable', 'ConnectionRefusedError: Connection refused')
    assert calls == [(('127.0.0.1', bench.fake.port), 0.7, True)]


# ----- the stream -------------------------------------------------------------------

CLOSES = [(1001, 'daemon-stopping', 0.5), (1012, 'replay-restart', 0.2),
          (1013, 'too-many-consumers', 5.0), (1008, 'policy', 0.5), (1000, 'closed', 0.5),
          (1011, 'closed', 0.5)]


@pytest.mark.parametrize('code, reason, retry_in_s', CLOSES)
def test_each_close_code_has_its_reason(bench, code, reason, retry_in_s):
    bench.connected()
    mark = bench.events.mark()
    bench.fake.close_stream(code, MARKER)
    ended = bench.events.wait(Disconnected, after=mark)
    assert ended == Disconnected(reason, f'close code {code}', retry_in_s)
    # The library answered the close; the bridge sent nothing else.
    assert wait_until(lambda: bench.fake.consumer_closes == [1000])
    assert bench.fake.data_frames == 0
    mark = bench.reconnect(ended)
    bench.events.wait(FrameReceived, after=mark)


def test_a_replay_that_starts_over_takes_nothing_from_the_backoff(bench):
    bench.connected()
    for _ in range(3):
        mark = bench.events.mark()
        bench.fake.close_stream(1012)
        ended = bench.events.wait(Disconnected, after=mark)
        assert (ended.reason, ended.retry_in_s) == ('replay-restart', 0.2)
        mark = bench.reconnect(ended)
        bench.events.wait(FrameReceived, after=mark)
    mark = bench.events.mark()
    bench.fake.close_stream(1001)
    assert bench.events.wait(Disconnected, after=mark).retry_in_s == 0.5


def test_a_full_daemon_is_tried_again_after_five_seconds_at_least(bench):
    bench.fake.max_consumers = 0
    bench.client(reconnect_min_s=4.0, reconnect_max_s=20.0)
    waits = []
    mark = 0
    for _ in range(3):
        ended = bench.events.wait(Disconnected, after=mark)
        assert (ended.reason, ended.detail) == ('too-many-consumers', 'close code 1013')
        waits.append(ended.retry_in_s)
        mark = bench.reconnect(ended)
    assert waits == [5.0, 8.0, 16.0]
    assert not bench.events.of(FrameReceived) and len(bench.events.of(Connected)) == 3


def test_a_daemon_that_dies_is_closed(bench):
    bench.connected()
    mark = bench.events.mark()
    bench.fake.stop()
    ended = bench.events.wait(Disconnected, after=mark)
    assert ended.reason == 'closed' and ended.detail.startswith('connection lost: ')
    assert ended.retry_in_s == 0.5
    mark = bench.reconnect(ended)
    again = bench.events.wait(Disconnected, after=mark)
    assert (again.reason, again.retry_in_s) == ('unreachable', 1.0)
    bench.fake.start()
    mark = bench.reconnect(again)
    bench.events.wait(FrameReceived, after=mark)


def test_a_delta_before_the_snapshot_ends_the_connection(bench):
    bench.fake.attach_frames = [frames.delta_frame('cell-1', {'panel': frames.panel_group()})]
    bench.client()
    ended = bench.events.wait(Disconnected)
    assert (ended.reason, ended.detail) == ('protocol', 'the first frame was not a snapshot')
    assert [type(event) for event in bench.events.of()] == [HealthRead, Connected, Disconnected]
    assert bench.fake.data_frames == 0


@pytest.mark.parametrize('sent', ['text', 'list', 'not a number', 'binary'])
def test_a_frame_that_is_no_json_object_ends_the_connection(bench, sent):
    bench.connected()
    mark = bench.events.mark()
    if sent == 'text':
        bench.fake.send_text(MARKER + ' is not json')
    elif sent == 'list':
        bench.fake.send_text('[1, 2, 3]')
    elif sent == 'not a number':
        bench.fake.send_text('{"type": "delta", "id": "cell-1", "value": NaN}')
    else:
        bench.fake.send_binary(b'\x00\x01')
    ended = bench.events.wait(Disconnected, after=mark)
    assert (ended.reason, ended.detail) == ('protocol', 'a frame that is not a JSON object')
    assert bench.fake.data_frames == 0


def test_75_s_without_a_frame_end_the_connection_and_a_ping_counts_as_one(bench):
    bench.connected()
    bench.clock.advance(74.0)
    assert wait_until(lambda: len(bench.events.of(HealthRead)) == 2)
    time.sleep(0.03)
    assert not bench.events.of(Disconnected)
    # A ping is activity: the 75 s start again.
    bench.fake.ping()
    assert wait_until(lambda: bench.fake.pongs == 1)
    time.sleep(0.02)
    bench.clock.advance(74.0)
    assert steady(lambda: len(bench.events.of(Disconnected)))
    assert not bench.events.of(Disconnected)
    bench.clock.advance(1.5)
    ended = bench.events.wait(Disconnected)
    assert (ended.reason, ended.detail) == ('silent', 'no frame for 75 s')
    assert wait_until(lambda: bench.fake.consumer_closes == [1000])
    assert bench.fake.data_frames == 0


def test_the_silence_rule_on_a_clock_that_runs_fast(bench):
    # 150 times as fast: the 75 s are half a second.
    bench.connected(clock=lambda: time.monotonic() * 150.0)
    for _ in range(12):
        bench.fake.ping()
        time.sleep(0.05)
    assert not bench.events.of(Disconnected)
    started = time.monotonic()
    ended = bench.events.wait(Disconnected)
    assert ended.reason == 'silent'
    assert 0.3 < time.monotonic() - started < 1.2


# ----- backoff ----------------------------------------------------------------------

def test_the_backoff_sequence_follows_the_injected_clock(bench):
    bench.fake.stop()
    bench.client()
    mark = 0
    for expected in (0.5, 1.0, 2.0, 4.0, 8.0, 15.0, 15.0):
        ended = bench.events.wait(Disconnected, after=mark)
        mark = bench.events.mark()
        assert (ended.reason, ended.retry_in_s) == ('unreachable', expected)
        # Nothing happens before the clock says so.
        bench.clock.advance(expected - 0.01)
        assert steady(bench.events.mark, 0.03)
        bench.clock.advance(0.02)
    assert len(bench.events.of(Disconnected)) >= 7


def test_the_backoff_uses_the_injected_random_source(bench):
    bench.fake.stop()
    bench.client(rng=random.Random(11))
    twin = random.Random(11)
    mark = 0
    for base in (0.5, 1.0, 2.0, 4.0):
        ended = bench.events.wait(Disconnected, after=mark)
        assert ended.retry_in_s == pytest.approx(base * twin.uniform(0.8, 1.2))
        mark = bench.reconnect(ended)


def test_a_connection_that_lasted_resets_the_backoff_and_a_short_one_does_not(bench):
    bench.fake.stop()
    bench.client()
    first = bench.events.wait(Disconnected)
    mark = bench.reconnect(first)
    second = bench.events.wait(Disconnected, after=mark)
    assert (first.retry_in_s, second.retry_in_s) == (0.5, 1.0)
    bench.fake.start()
    mark = bench.reconnect(second)
    bench.events.wait(FrameReceived, after=mark)
    # Under ten seconds: the backoff goes on where it was.
    bench.clock.advance(9.5)
    mark = bench.events.mark()
    bench.fake.close_stream(1001)
    short = bench.events.wait(Disconnected, after=mark)
    assert short.retry_in_s == 2.0
    mark = bench.reconnect(short)
    bench.events.wait(FrameReceived, after=mark)
    # Ten seconds and more, with its snapshot: start again from the minimum.
    bench.clock.advance(10.5)
    mark = bench.events.mark()
    bench.fake.close_stream(1001)
    assert bench.events.wait(Disconnected, after=mark).retry_in_s == 0.5


def test_a_long_connection_without_a_snapshot_does_not_reset_the_backoff(bench):
    bench.fake.attach_frames = []
    bench.fake.stop()
    bench.client()
    first = bench.events.wait(Disconnected)
    bench.fake.start()
    mark = bench.reconnect(first)
    bench.events.wait(Connected, after=mark)
    bench.clock.advance(30.0)
    mark = bench.events.mark()
    bench.fake.close_stream(1001)
    assert bench.events.wait(Disconnected, after=mark).retry_in_s == 1.0


# ----- leases -----------------------------------------------------------------------

def test_one_lease_id_is_opened_with_new_and_renewed_in_time(bench):
    signals = ['do_gripper', 'Local/DRV_1/do_valve', 'bad name']
    bench.targets = (('cell-1', True, tuple(signals)),)
    bench.connected()
    watched = bench.events.wait(LeaseResult, lambda event: event.kind == 'io')
    motion = bench.events.wait(LeaseResult, lambda event: event.kind == 'motion')
    assert motion == LeaseResult('cell-1', 'motion', True, '', {
        'leaseMs': 15000, 'targetHz': 8.0, 'leaseId': 'lease-1'})
    assert frames.schema_errors('MotionWatch', motion.answer) == []
    assert frames.schema_errors('IoWatch', watched.answer) == []
    assert (watched.controller_id, watched.ok, watched.code) == ('cell-1', True, '')
    assert watched.answer['accepted'] == signals[:2] and watched.answer['leaseId'] == 'lease-1'
    assert [entry['name'] for entry in watched.answer['rejected']] == ['bad name']
    # The first request opens the lease; everything after it carries the id it got.
    assert [(entry.path.split('/')[-2], entry.body) for entry in bench.posts()] == [
        ('motion', {'leaseId': 'new'}), ('io', {'signals': signals, 'leaseId': 'lease-1'})]
    # Not renewed before a third of the 15 s has passed ...
    bench.clock.advance(4.9)
    assert steady(lambda: len(bench.posts())) and len(bench.posts()) == 2
    # ... and then at once: long before half of it.
    bench.clock.advance(0.2)
    assert wait_until(lambda: len(bench.posts()) == 4)
    assert [entry.body for entry in bench.posts()[2:]] == [
        {'leaseId': 'lease-1'}, {'signals': signals, 'leaseId': 'lease-1'}]
    assert wait_until(lambda: len(bench.events.of(LeaseResult)) == 4)
    assert all(event.ok for event in bench.events.of(LeaseResult))
    assert bench.fake.io_watched('cell-1') == signals[:2]


def test_the_renewal_follows_the_lease_the_daemon_grants(bench):
    bench.fake.lease_ms = 3000
    bench.targets = (('cell-1', True, ()),)
    bench.connected()
    bench.events.wait(LeaseResult)
    bench.clock.advance(0.9)
    assert steady(lambda: len(bench.posts())) and len(bench.posts()) == 1
    bench.clock.advance(0.2)
    assert wait_until(lambda: len(bench.posts()) == 2)


def test_the_same_lease_id_after_a_reconnect_and_after_a_daemon_restart(bench):
    bench.targets = (('cell-1', True, ('do_gripper',)),)
    bench.connected()
    bench.events.wait(LeaseResult, lambda event: event.kind == 'io')
    expected = [{'leaseId': 'lease-1'}, {'signals': ['do_gripper'], 'leaseId': 'lease-1'}]
    # A reconnect: every lease is asked for again, under the same id.
    bench.fake.clear_requests()
    mark = bench.events.mark()
    bench.fake.close_stream(1012)
    mark = bench.reconnect(bench.events.wait(Disconnected, after=mark))
    bench.events.wait(LeaseResult, lambda event: event.kind == 'io', after=mark)
    assert [entry.body for entry in bench.posts()] == expected
    # A daemon that was restarted knows no lease: it opens ours again under our id.
    bench.fake.clear_requests()
    mark = bench.events.mark()
    bench.fake.stop()
    ended = bench.events.wait(Disconnected, after=mark)
    bench.fake.start()
    assert bench.fake.leases == {}
    mark = bench.reconnect(ended)
    bench.events.wait(LeaseResult, lambda event: event.kind == 'io' and event.ok, after=mark)
    assert [entry.body for entry in bench.posts()] == expected
    assert bench.fake.leases == {('cell-1', 'motion'): {'lease-1': None},
                                 ('cell-1', 'io'): {'lease-1': ['do_gripper']}}


def test_one_id_serves_every_controller(bench):
    bench.fake.set_controllers([frames.controller('cell-1'), frames.controller('cell-2')])
    bench.targets = (('cell-1', True, ()), ('cell-2', True, ('do_a',)))
    bench.connected()
    bench.events.wait(LeaseResult, lambda event: event.controller_id == 'cell-2'
                      and event.kind == 'io')
    bodies = [entry.body for entry in bench.fake.requests if entry.method == 'POST']
    assert bodies == [{'leaseId': 'new'}, {'leaseId': 'lease-1'},
                      {'signals': ['do_a'], 'leaseId': 'lease-1'}]
    assert {event.answer['leaseId'] for event in bench.events.of(LeaseResult)} == {'lease-1'}


def test_a_lease_limit_waits_for_retry_after(bench):
    bench.fake.script('motion', frames.error_answer('LEASE_LIMIT', retry_after_s=7))
    bench.targets = (('cell-1', True, ()),)
    bench.connected()
    assert bench.events.wait(LeaseResult) == LeaseResult(
        'cell-1', 'motion', False, 'LEASE_LIMIT', None)
    bench.clock.advance(6.9)
    assert steady(lambda: len(bench.posts())) and len(bench.posts()) == 1
    bench.clock.advance(0.2)
    assert bench.events.wait(LeaseResult, lambda event: event.ok).answer['leaseId'] == 'lease-1'
    assert [entry.body for entry in bench.posts()] == [{'leaseId': 'new'}, {'leaseId': 'new'}]


def test_retry_after_is_read_as_whole_seconds_within_bounds():
    read = daemon_client._retry_after_s
    assert read({'retry-after': '7'}) == 7.0
    assert read({'retry-after': ' 30 '}) == 30.0
    assert read({'retry-after': '0'}) == 1.0
    assert read({'retry-after': '99999'}) == 300.0
    for unusable in ({}, {'retry-after': 'soon'}, {'retry-after': '-5'}, {'retry-after': '1.5'},
                     {'retry-after': 'Wed, 21 Oct 2026 07:28:00 GMT'}, {'retry-after': ''}):
        assert read(unusable) == 15.0


def test_a_controller_the_daemon_does_not_serve_waits_for_the_stream(bench):
    bench.targets = (('cell-1', True, ()), ('cell-2', True, ('do_a',)))
    bench.connected()
    bench.events.wait(LeaseResult, lambda event: event.controller_id == 'cell-2'
                      and event.kind == 'io')
    missing = [event for event in bench.events.of(LeaseResult) if event.controller_id == 'cell-2']
    assert missing == [LeaseResult('cell-2', 'motion', False, 'NO_CONTROLLER', None),
                       LeaseResult('cell-2', 'io', False, 'NO_CONTROLLER', None)]
    assert len(bench.posts(controller_id='cell-2')) == 2
    # However long it takes, and whatever else the stream says: it is not asked again.
    mark = bench.events.mark()
    bench.clock.advance(60.0)
    bench.fake.push(frames.delta_frame('cell-1', {'panel': frames.panel_group(op_mode='MANR')}))
    bench.events.wait(LeaseResult, lambda event: event.controller_id == 'cell-1', after=mark)
    assert steady(lambda: len(bench.posts(controller_id='cell-2')))
    assert len(bench.posts(controller_id='cell-2')) == 2
    # The stream names it: asked at once, without any time passing.
    added = frames.controller('cell-2')
    bench.fake.push(frames.delta_frame('cell-2', frames.groups_of(added)))
    granted = bench.events.wait(LeaseResult, lambda event: event.controller_id == 'cell-2'
                                and event.kind == 'io' and event.ok, after=mark)
    assert granted.answer['accepted'] == ['do_a']
    assert len(bench.posts(controller_id='cell-2')) == 4


def test_an_identity_mismatch_is_reported_and_asked_again_every_15_s(bench):
    wrong = frames.controller('cell-1', link=frames.link_group(
        identity_status='mismatch', actual=MISMATCH))
    bench.fake.set_controllers([wrong])
    bench.targets = (('cell-1', True, ('do_a',)),)
    bench.connected()
    bench.events.wait(LeaseResult, lambda event: event.kind == 'io')
    assert bench.events.of(LeaseResult) == [
        LeaseResult('cell-1', 'motion', False, 'IDENTITY_MISMATCH', None),
        LeaseResult('cell-1', 'io', False, 'IDENTITY_MISMATCH', None)]
    bench.clock.advance(14.9)
    assert steady(lambda: len(bench.posts())) and len(bench.posts()) == 2
    bench.clock.advance(0.2)
    assert wait_until(lambda: len(bench.posts()) == 4)
    # Reported, asked again, and never anything else: no other route, no other body.
    assert {entry.path for entry in bench.fake.requests} == {
        '/v0/health', '/v0/stream', '/v0/controllers', '/v0/controllers/cell-1/motion/watch',
        '/v0/controllers/cell-1/io/watch'}
    assert all(entry.body['leaseId'] == 'new' for entry in bench.posts())
    assert not bench.events.of(Disconnected)


def test_a_failed_lease_request_is_tried_again_no_more_than_once_a_second(bench):
    bench.fake.script(
        'motion', BUSY, (500, {}, None), (200, {}, b'no json'),
        (200, {}, {'leaseMs': 15000, 'targetHz': 8.0}),
        (200, {}, {'leaseMs': 0, 'targetHz': 8.0, 'leaseId': 'lease-9'}),
        (200, {}, b'{"leaseMs": Infinity, "targetHz": 8.0, "leaseId": "lease-9"}'),
        (200, {}, {'leaseMs': 15000, 'targetHz': 8.0, 'leaseId': 'new'}))
    bench.targets = (('cell-1', True, ()),)
    made = bench.client(start=False)
    real = made._request
    failed = []

    def flaky(method, path, **options):
        if method == 'POST' and not failed:
            failed.append(path)
            return daemon_client._Answer(None, {}, None, 'timed out')
        return real(method, path, **options)

    made._request = flaky
    made.start()
    codes = []
    mark = 0
    for wait_s in (1.0, 1.0, 1.0, 15.0, 15.0, 15.0, 15.0, 15.0):
        result = bench.events.wait(LeaseResult, after=mark)
        mark = bench.events.mark()
        codes.append(result.code)
        assert not result.ok and result.answer is None
        # (A ping now and then: the clock moves on by more than the 75 s of silence.)
        bench.fake.ping()
        sent = len(bench.posts())
        bench.clock.advance(wait_s - 0.1)
        assert steady(lambda: len(bench.posts()), 0.03) and len(bench.posts()) == sent
        bench.clock.advance(0.2)
    assert codes == ['NETWORK', 'CONTROLLER_ERROR', 'HTTP_500'] + ['BAD_ANSWER'] * 5
    assert bench.events.wait(LeaseResult, after=mark).ok
    assert all(entry.body == {'leaseId': 'new'} for entry in bench.posts())
    assert not bench.events.of(Disconnected)


def test_one_controller_is_not_asked_twice_in_the_second_after_a_failure(bench):
    bench.fake.script('motion', BUSY)
    bench.targets = (('cell-1', True, ('do_a',)),)
    bench.connected()
    failed = bench.events.wait(LeaseResult)
    assert (failed.kind, failed.ok, failed.code) == ('motion', False, 'CONTROLLER_ERROR')
    # Its other watch is due as well, and waits with it.
    bench.clock.advance(0.9)
    assert steady(lambda: len(bench.posts())) and len(bench.posts()) == 1
    bench.clock.advance(0.2)
    assert wait_until(lambda: len(bench.events.of(LeaseResult)) == 3)
    assert [(event.kind, event.ok) for event in bench.events.of(LeaseResult)] == [
        ('motion', False), ('motion', True), ('io', True)]


def test_a_changed_signal_set_is_sent_at_once_and_a_dropped_one_is_freed(bench):
    path = '/v0/controllers/cell-1/io/watch'

    def bodies():
        return [entry.body for entry in bench.fake.requests_to(path)]

    bench.targets = (('cell-1', False, ('do_a', 'do_b')),)
    bench.connected()
    bench.events.wait(LeaseResult, lambda event: event.kind == 'io')
    bench.targets = (('cell-1', False, ('do_b', 'do_c')),)
    assert wait_until(lambda: len(bodies()) == 2)
    assert bodies()[1] == {'signals': ['do_b', 'do_c'], 'leaseId': 'lease-1'}
    # No signal any more: one request with an empty list frees them at once.
    bench.targets = (('cell-1', False, ()),)
    assert wait_until(lambda: len(bodies()) == 3)
    assert bodies()[2] == {'signals': [], 'leaseId': 'lease-1'}
    assert steady(lambda: len(bodies())) and bench.fake.io_watched('cell-1') == []
    # The same when the controller is no target at all any more.
    bench.targets = (('cell-1', False, ('do_a',)),)
    assert wait_until(lambda: len(bodies()) == 4)
    bench.targets = ()
    assert wait_until(lambda: len(bodies()) == 5)
    assert bodies()[4] == {'signals': [], 'leaseId': 'lease-1'}
    assert steady(lambda: len(bodies()))
    # Without watch_motion the daemon is never asked to sample the joints.
    assert bench.posts('motion') == []


# ----- refresh and health -----------------------------------------------------------

def test_the_controller_list_is_asked_at_the_refresh_rate(bench):
    bench.connected(refresh_hz=4.0)
    first = bench.events.wait(Refreshed)
    assert frames.schema_errors('ControllerList', first.answer) == []
    assert first.at == Arrival(wall_ns=WALL_NS, mono=1000.0)
    for count in (2, 3, 4):
        bench.clock.advance(0.24)
        assert steady(lambda: len(bench.events.of(Refreshed)), 0.03)
        bench.clock.advance(0.02)
        assert wait_until(lambda: len(bench.events.of(Refreshed)) == count)
    assert bench.events.of(Refreshed)[-1].at.mono == pytest.approx(1000.78)
    assert all(entry.token == 'matched' for entry in bench.fake.requests_to('/v0/controllers'))


def test_a_refresh_rate_above_the_tick_is_kept_and_a_new_connection_is_taken_up_at_once(bench):
    # The real clock and the real tick of half a second, at ten refreshes a second.
    stamps = []

    def sink(event):
        stamps.append((time.monotonic(), event))

    made = bench.client(start=False, sink=sink, clock=time.monotonic, refresh_hz=10.0)
    made._timing = dataclasses.replace(daemon_client._Timing(), stream_poll_s=0.01)
    assert made._timing.tick_s == 0.5
    made.start()
    time.sleep(0.65)
    made.stop()
    connected = [at for at, event in stamps if isinstance(event, Connected)]
    refreshed = [at for at, event in stamps if isinstance(event, Refreshed)]
    # Bound to the tick there would be two in this time, and the first after half a second.
    assert len(connected) == 1 and 4 <= len(refreshed) <= 8
    assert refreshed[0] - connected[0] < 0.25
    gaps = [later - earlier for earlier, later in zip(refreshed, refreshed[1:])]
    assert min(gaps) > 0.05


def test_two_failed_refreshes_in_a_row_end_the_connection_and_one_does_not(bench):
    def answered():
        return len([entry for entry in bench.fake.requests_to('/v0/controllers')
                    if entry.status is not None])

    def one_refresh():
        before = answered()
        bench.clock.advance(0.5)
        assert wait_until(lambda: answered() == before + 1)

    bench.connected()
    bench.events.wait(Refreshed)
    good = 1
    # One failure of each kind, each followed by an answer: the count starts over.
    for failure in (BUSY, (200, {}, b'no json'), (200, {}, {'controllers': 'none'}),
                    (200, {}, [1]), (404, {}, fake_daemon.error_body('NO_ROUTE', MARKER))):
        bench.fake.script('controllers', failure)
        one_refresh()
        one_refresh()
        good += 1
        assert wait_until(lambda: len(bench.events.of(Refreshed)) == good)
        assert not bench.events.of(Disconnected)
    assert len(bench.events.of(Refreshed)) == 6
    # Two in a row.
    bench.fake.script('controllers', BUSY, (500, {}, None))
    one_refresh()
    one_refresh()
    ended = bench.events.wait(Disconnected)
    assert ended.reason == 'refresh-failed'
    assert ended.detail == 'two controller list requests in a row failed: HTTP 500'
    assert len(bench.events.of(Refreshed)) == 6
    assert wait_until(lambda: bench.fake.consumer_closes == [1000])


def test_after_a_failed_refresh_nothing_else_is_asked_until_one_answers(bench):
    bench.targets = (('cell-1', True, ()),)
    bench.connected()
    bench.events.wait(LeaseResult)
    bench.events.wait(Refreshed)
    bench.fake.script('controllers', BUSY)
    bench.fake.clear_requests()
    # The lease and health are due as well, and wait.
    bench.clock.advance(6.0)
    assert wait_until(lambda: len(bench.fake.requests) == 1)
    assert steady(lambda: len(bench.fake.requests))
    assert [entry.path for entry in bench.fake.requests] == ['/v0/controllers']
    bench.clock.advance(0.5)
    assert wait_until(lambda: {entry.path for entry in bench.fake.requests} == {
        '/v0/controllers', '/v0/health', '/v0/controllers/cell-1/motion/watch'})


def test_a_frozen_daemon_is_given_up_after_two_refresh_timeouts(bench):
    bench.connected(refresh_timeout_s=0.15)
    bench.events.wait(Refreshed)
    mark = bench.events.mark()
    bench.fake.freeze()
    started = time.monotonic()
    for count in (2, 3):
        bench.clock.advance(0.5)
        assert wait_until(lambda: len(bench.fake.requests_to('/v0/controllers')) == count)
    ended = bench.events.wait(Disconnected, after=mark)
    assert ended.reason == 'refresh-failed'
    assert ended.detail == 'two controller list requests in a row failed: timed out'
    assert time.monotonic() - started < 1.5
    bench.fake.unfreeze()
    mark = bench.reconnect(ended)
    bench.events.wait(FrameReceived, after=mark)


@pytest.mark.parametrize('answer, reason, text', [
    (frames.error_answer('TOKEN_INVALID'), 'token-refused', 'HTTP 401 TOKEN_INVALID'),
    (frames.error_answer('HOST_NOT_ALLOWED'), 'host-refused', 'HTTP 421 HOST_NOT_ALLOWED'),
    ((307, {'Location': 'http://127.0.0.1:9/v0/controllers'}, None), 'not-a-daemon', 'HTTP 307'),
])
def test_a_refresh_that_is_refused_ends_the_connection_at_once(bench, answer, reason, text):
    bench.fake.script('controllers', answer)
    bench.client()
    ended = bench.events.wait(Disconnected)
    assert ended == Disconnected(reason, 'the controller list answered ' + text, 0.5)
    assert not bench.events.of(Refreshed) and bench.events.of(Connected)


@pytest.mark.parametrize('kind', ['motion', 'io'])
@pytest.mark.parametrize('answer, reason, text', [
    (frames.error_answer('TOKEN_MISSING'), 'token-refused', 'HTTP 401 TOKEN_MISSING'),
    (frames.error_answer('HOST_NOT_ALLOWED'), 'host-refused', 'HTTP 421 HOST_NOT_ALLOWED'),
    ((302, {'Location': 'http://127.0.0.1:9/'}, None), 'not-a-daemon', 'HTTP 302'),
])
def test_a_watch_that_is_refused_for_the_token_or_host_ends_the_connection(
        bench, kind, answer, reason, text):
    bench.fake.script(kind, answer)
    bench.targets = (('cell-1', kind == 'motion', ('do_a',) if kind == 'io' else ()),)
    bench.client()
    ended = bench.events.wait(Disconnected)
    assert ended == Disconnected(reason, f'the {kind} watch answered {text}', 0.5)
    assert not bench.events.of(LeaseResult)


def test_a_refused_token_ends_the_connection_and_the_file_is_read_again(bench):
    bench.connected()
    bench.events.wait(Refreshed)
    # The daemon has another token now (it was started again with a new token file).
    bench.fake.token = OTHER_TOKEN
    mark = bench.events.mark()
    bench.clock.advance(0.5)
    ended = bench.events.wait(Disconnected, after=mark)
    assert (ended.reason, ended.detail) == (
        'token-refused', 'the controller list answered HTTP 401 TOKEN_INVALID')
    # The file still holds the old one: refused again, this time at the handshake.
    mark = bench.reconnect(ended)
    ended = bench.events.wait(Disconnected, after=mark)
    assert (ended.reason, ended.detail) == (
        'token-refused', 'stream handshake answered HTTP 401 TOKEN_INVALID')
    # The new token is picked up from the file without a restart of the bridge.
    bench.token_file.write_text(OTHER_TOKEN + '\r\n', encoding='ascii')
    mark = bench.reconnect(ended)
    bench.events.wait(FrameReceived, after=mark)
    assert [entry.token for entry in bench.fake.requests_to('/v0/stream')] == [
        'matched', 'not matched', 'matched']


def test_health_is_read_every_5_s_and_another_mode_ends_the_connection(bench):
    bench.connected()
    assert len(bench.events.of(HealthRead)) == 1
    bench.clock.advance(4.9)
    assert steady(lambda: len(bench.fake.requests_to('/v0/health')))
    bench.clock.advance(0.2)
    assert wait_until(lambda: len(bench.events.of(HealthRead)) == 2)
    assert not bench.events.of(Disconnected)
    assert all(entry.token is None for entry in bench.fake.requests_to('/v0/health'))
    # The daemon now replays a recording.
    bench.fake.mode = 'replay'
    mark = bench.events.mark()
    bench.clock.advance(5.0)
    ended = bench.events.wait(Disconnected, after=mark)
    assert (ended.reason, ended.detail) == (
        'mode-changed', 'the daemon changed from live to replay')
    assert bench.events.of(HealthRead)[2].health['mode'] == 'replay'
    # The next attempt decides afresh: a recording is refused, no stream is opened and
    # health is read again every 5 s.
    streams = len(bench.fake.requests_to('/v0/stream'))
    for _ in range(3):
        mark = bench.reconnect(ended)
        ended = bench.events.wait(Disconnected, after=mark)
        assert ended == Disconnected('replay-refused', 'the daemon replays a recording', 5.0)
    assert len(bench.fake.requests_to('/v0/stream')) == streams
    assert len(bench.events.of(Connected)) == 1
    bench.fake.mode = 'live'
    mark = bench.reconnect(ended)
    bench.events.wait(FrameReceived, after=mark)


def test_the_node_may_end_a_connection_whose_stream_stalled(bench):
    # The client cannot see a stalled stream (the socket is open and every request is
    # answered); whoever keeps the state tells it. The connection ends with its own
    # reason, the fake sees a close, and the client connects again as after any loss.
    made = bench.connected()
    mark = bench.events.mark()
    assert made.end_stalled_stream('no frame for what the controller list has shown') is True
    ended = bench.events.wait(Disconnected, after=mark)
    assert ended.reason == 'stream-stalled'
    assert ended.detail == 'no frame for what the controller list has shown'
    assert ended.retry_in_s == 0.5
    assert wait_until(lambda: bench.fake.consumer_closes == [1000])
    # The first reason stays: a second call, and anything the lease thread finds, changes nothing.
    assert made.end_stalled_stream('again') is False
    assert bench.events.of(Disconnected) == [ended]
    mark = bench.reconnect(ended)
    bench.events.wait(FrameReceived, after=mark)
    assert len(bench.events.of(Connected)) == 2
    # Without a detail the event still says what happened; after stop nothing is ended.
    assert made.end_stalled_stream() is True
    ended = bench.events.wait(Disconnected, after=mark)
    assert ended.reason == 'stream-stalled' and ended.detail
    assert all(' ' <= character <= '~' for character in ended.detail)
    made.stop(1.0)
    assert made.end_stalled_stream('after stop') is False


def test_a_stall_reported_while_nothing_is_connected_ends_nothing(bench):
    bench.fake.allowed_hosts = set()
    made = bench.client()
    bench.events.wait(Disconnected)
    assert made.end_stalled_stream('nothing to end') is False
    assert all(event.reason == 'host-refused' for event in bench.events.of(Disconnected))


def test_a_recording_is_connected_to_only_when_it_is_allowed(bench):
    bench.fake.mode = 'replay'
    bench.connected(allow_replay=True)
    connected = bench.events.of(Connected)[0]
    assert connected.health['mode'] == 'replay'
    assert connected.health['replay']['state'] == 'playing'
    # The other way round ends it too: what is published as a replay is a replay.
    bench.fake.mode = 'live'
    bench.clock.advance(5.0)
    ended = bench.events.wait(Disconnected)
    assert (ended.reason, ended.detail) == (
        'mode-changed', 'the daemon changed from replay to live')


def test_a_health_read_that_fails_is_left_to_the_refresh(bench):
    def answered():
        return len([entry for entry in bench.fake.requests_to('/v0/health')
                    if entry.status is not None])

    bench.connected()
    bench.fake.script('health', BUSY, (200, {}, b'no json'))
    for count in (2, 3, 4):
        bench.clock.advance(5.0)
        assert wait_until(lambda: answered() == count)
    # Two reads that failed, then one that answered: nothing ended, nothing was made up.
    assert wait_until(lambda: len(bench.events.of(HealthRead)) == 2)
    assert not bench.events.of(Disconnected)
    assert [entry.status for entry in bench.fake.requests_to('/v0/health')] == [
        200, 503, 200, 200]


# ----- redirects --------------------------------------------------------------------

def test_a_redirected_upgrade_is_not_followed_and_the_other_listener_hears_nothing(bench):
    with fake_daemon.Bystander() as other:
        bench.fake.redirect(other.url, routes=('stream',))
        bench.client()
        ended = bench.events.wait(Disconnected)
        assert ended == Disconnected('not-a-daemon', 'stream handshake answered HTTP 302', 0.5)
        mark = bench.reconnect(ended)
        assert bench.events.wait(Disconnected, after=mark).reason == 'not-a-daemon'
        time.sleep(0.05)
        assert other.connections == 0 and other.received == b''
    # The token went to the configured address, twice, and nowhere else.
    assert [entry.token for entry in bench.fake.requests_to('/v0/stream')] == ['matched'] * 2
    assert [entry.status for entry in bench.fake.requests_to('/v0/stream')] == [302, 302]
    assert not bench.events.of(Connected) and bench.fake.consumers == 0


def test_a_redirected_health_check_is_not_followed_and_no_token_is_read(bench, monkeypatch):
    reads = []
    real = daemon_client.read_token
    monkeypatch.setattr(daemon_client, 'read_token', lambda path: reads.append(path) or real(path))
    with fake_daemon.Bystander() as other:
        bench.fake.redirect(other.url)
        bench.client()
        ended = bench.events.wait(Disconnected)
        assert ended == Disconnected(
            'not-a-daemon', 'health answered HTTP 302, not a health answer', 0.5)
        time.sleep(0.05)
        assert other.connections == 0 and other.received == b''
    assert reads == [] and bench.events.of() == [ended]
    assert [(entry.path, entry.token) for entry in bench.fake.requests] == [('/v0/health', None)]


@pytest.mark.parametrize('route, what', [
    ('controllers', 'the controller list'), ('motion', 'the motion watch'),
    ('io', 'the io watch'), ('health', 'health')])
def test_a_redirect_on_any_route_ends_the_connection_and_is_not_followed(bench, route, what):
    bench.targets = (('cell-1', True, ('do_a',)),)
    with fake_daemon.Bystander() as other:
        bench.connected()
        bench.events.wait(LeaseResult, lambda event: event.kind == 'io')
        bench.fake.redirect(other.url, routes=(route,))
        bench.clock.advance(5.0)
        ended = bench.events.wait(Disconnected)
        assert ended == Disconnected('not-a-daemon', f'{what} answered HTTP 302', 0.5)
        time.sleep(0.05)
        assert other.connections == 0 and other.received == b''


# ----- stop -------------------------------------------------------------------------

def test_stop_frees_the_signals_and_closes_the_stream_with_1000(bench):
    bench.fake.set_controllers([frames.controller(f'cell-{n}') for n in (1, 2, 3)])
    bench.targets = (('cell-1', True, ('do_gripper',)), ('cell-2', False, ('do_a', 'do_b')),
                     ('cell-3', True, ()))
    made = bench.connected()
    assert wait_until(lambda: len(bench.events.of(LeaseResult)) == 4)
    assert all(event.ok for event in bench.events.of(LeaseResult))
    assert bench.fake.io_watched('cell-1') == ['do_gripper']
    assert bench.fake.io_watched('cell-2') == ['do_a', 'do_b']
    bench.fake.clear_requests()
    events = bench.events.mark()
    started = time.monotonic()
    made.stop()
    assert time.monotonic() - started < 1.0
    assert stopped(made)
    assert wait_until(lambda: bench.fake.consumer_closes == [1000] and bench.fake.consumers == 0)
    # One request with an empty list for every controller with signals, nothing else.
    assert sorted((entry.path, entry.body) for entry in bench.fake.requests) == [
        ('/v0/controllers/cell-1/io/watch', {'signals': [], 'leaseId': 'lease-1'}),
        ('/v0/controllers/cell-2/io/watch', {'signals': [], 'leaseId': 'lease-1'})]
    assert bench.fake.io_watched('cell-1') == [] and bench.fake.io_watched('cell-2') == []
    # A motion lease has no release route: it runs out by itself.
    assert bench.fake.leases[('cell-1', 'motion')] == {'lease-1': None}
    assert bench.fake.data_frames == 0
    # Nothing follows a stop, not even a Disconnected.
    time.sleep(0.05)
    assert bench.events.mark() == events
    made.stop()
    with pytest.raises(RuntimeError):
        made.start()


def test_stop_returns_at_once_before_start_and_between_attempts(bench):
    idle = bench.client(start=False)
    started = time.monotonic()
    idle.stop()
    assert time.monotonic() - started < 0.2
    with pytest.raises(RuntimeError):
        idle.start()
    bench.fake.stop()
    waiting = bench.client()
    bench.events.wait(Disconnected)
    with pytest.raises(RuntimeError):
        waiting.start()
    started = time.monotonic()
    waiting.stop()
    assert time.monotonic() - started < 0.5 and stopped(waiting)
    events = bench.events.mark()
    bench.clock.advance(60.0)
    time.sleep(0.05)
    assert bench.events.mark() == events


def test_stop_returns_within_its_timeout_when_a_connected_daemon_hangs(bench):
    bench.targets = (('cell-1', True, ('do_gripper',)),)
    made = bench.connected(refresh_timeout_s=30.0, http_timeout_s=30.0)
    bench.events.wait(LeaseResult, lambda event: event.kind == 'io')
    bench.fake.freeze()
    bench.clock.advance(0.5)
    # A refresh is on its way and will get no answer.
    assert wait_until(lambda: bench.fake.requests[-1].status is None)
    started = time.monotonic()
    made.stop(1.0)
    took = time.monotonic() - started
    assert took < 1.2 and stopped(made)
    assert not bench.events.of(Disconnected)
    bench.fake.unfreeze()


def test_stop_returns_when_the_daemon_hangs_during_the_connect(bench):
    bench.fake.freeze()
    made = bench.client(http_timeout_s=30.0)
    assert wait_until(lambda: len(bench.fake.requests) == 1)
    started = time.monotonic()
    made.stop(1.0)
    assert time.monotonic() - started < 0.5 and stopped(made)
    assert bench.events.of() == []
    bench.fake.unfreeze()


def test_stop_while_replay_is_refused_and_while_the_stream_has_no_snapshot(bench):
    bench.fake.mode = 'replay'
    refused = bench.client()
    bench.events.wait(Disconnected, lambda event: event.reason == 'replay-refused')
    started = time.monotonic()
    refused.stop()
    assert time.monotonic() - started < 0.5 and stopped(refused)
    bench.fake.mode = 'live'
    bench.fake.attach_frames = []
    quiet = bench.client()
    bench.events.wait(Connected)
    started = time.monotonic()
    quiet.stop()
    assert time.monotonic() - started < 0.5 and stopped(quiet)
    assert wait_until(lambda: bench.fake.consumer_closes == [1000])


# ----- what may be said -------------------------------------------------------------

class Collected(logging.Handler):
    """A log handler that keeps every record it is given."""

    def __init__(self):
        super().__init__(logging.DEBUG)
        self.records = []

    def emit(self, record):
        self.records.append(record)


@pytest.fixture
def debug_logs():
    """
    Collect what every logger says at DEBUG, the WebSocket library's included.

    The handler sits on each logger itself, so it does not matter whether a logger
    hands its records on to the root logger (under the ROS test plugins it does not).
    """
    handler = Collected()
    loggers = [logging.root]
    loggers += [logger for logger in logging.root.manager.loggerDict.values()
                if isinstance(logger, logging.Logger)]
    for name in ('websocket', 'abb_rws_bridge', 'abb_rws_bridge.daemon_client'):
        if logging.getLogger(name) not in loggers:
            loggers.append(logging.getLogger(name))
    levels = [logger.level for logger in loggers]
    for logger in loggers:
        logger.addHandler(handler)
        logger.setLevel(logging.DEBUG)
    try:
        yield handler
    finally:
        for logger, level in zip(loggers, levels):
            logger.removeHandler(handler)
            logger.setLevel(level)


def test_the_token_is_in_no_log_line_at_debug(bench, debug_logs, capfd):
    assert logging.getLogger('websocket').isEnabledFor(logging.DEBUG)
    assert daemon_client._LOG.isEnabledFor(logging.DEBUG)
    bench.targets = (('cell-1', True, ('do_gripper', 'bad name')), ('cell-2', True, ()))
    made = bench.connected()
    bench.events.wait(LeaseResult, lambda event: event.controller_id == 'cell-2')
    bench.fake.push(frames.delta_frame('cell-1', {'panel': frames.panel_group()}))
    bench.fake.ping()
    assert wait_until(lambda: bench.fake.pongs == 1)
    # A token that is refused, at a request and at the handshake; a new one from the file.
    bench.fake.token = OTHER_TOKEN
    mark = bench.events.mark()
    bench.clock.advance(5.0)
    ended = bench.events.wait(Disconnected, after=mark)
    mark = bench.reconnect(ended)
    ended = bench.events.wait(Disconnected, after=mark)
    bench.token_file.write_text('no token at all', encoding='ascii')
    mark = bench.reconnect(ended)
    ended = bench.events.wait(Disconnected, after=mark)
    bench.token_file.write_text(OTHER_TOKEN, encoding='ascii')
    mark = bench.reconnect(ended)
    bench.events.wait(LeaseResult, after=mark)
    # A failed refresh, a close the daemon should never send, a redirect, a stop.
    bench.fake.script('controllers', BUSY)
    bench.clock.advance(0.5)
    mark = bench.events.mark()
    bench.fake.close_stream(1008, 'the stream accepts no messages')
    ended = bench.events.wait(Disconnected, after=mark)
    bench.fake.redirect('http://127.0.0.1:9', routes=('stream',))
    mark = bench.reconnect(ended)
    ended = bench.events.wait(Disconnected, after=mark)
    bench.fake.redirect(None)
    mark = bench.reconnect(ended)
    bench.events.wait(FrameReceived, after=mark)
    made.stop()
    captured = capfd.readouterr()
    records = list(debug_logs.records)
    text = captured.out + captured.err + ' | '.join(
        f'{record.name} {record.levelname} {record.getMessage()} {record.args!r}'
        for record in records)
    ours = [record for record in records if record.name == 'abb_rws_bridge.daemon_client']
    # The check is not vacuous: the client did log, at every level it uses.
    assert {record.levelno for record in ours} >= {logging.DEBUG, logging.INFO, logging.ERROR}
    assert 'token-refused' in text and 'policy' in text and 'token-file' in text
    for secret in (TOKEN, OTHER_TOKEN, TOKEN[:16], OTHER_TOKEN[:16]):
        assert secret not in text
        assert secret not in repr(bench.events.of())
    # The two switches that would print request headers were never touched.
    assert not websocket.isEnabledForTrace()
    assert http.client.HTTPConnection.debuglevel == 0
    assert daemon_client._Connection.debuglevel == 0


def test_the_module_never_switches_on_a_trace_and_never_sets_a_host_or_an_origin():
    source = inspect.getsource(daemon_client)
    assert 'enableTrace' not in source
    assert 'set_debuglevel' not in source and 'debuglevel' not in source
    # No Host and no Origin is ever set by hand, in a header or as a library option.
    assert not re.search(r"""['"]\s*(host|origin)\s*['":]""", source, re.I)
    assert not re.search(r'\b(host|origin)\s*=', source)
    # The stream is opened without an Origin, without following a redirect, and on a
    # socket the client opened itself (every address tried, the first error reported).
    assert 'redirect_limit=0' in source and 'suppress_origin=True' in source
    assert 'all_errors=True' in source and 'socket=sock' in source


def test_details_are_short_printable_and_never_an_answer_body(bench, tmp_path):
    noisy = {'error': (MARKER + ' \u00fc\n') * 100, 'code': MARKER.lower() * 30}
    long_code = {'error': MARKER, 'code': 'A' * 60}
    weird_mode = {'status': 'ok', 'mode': MARKER * 40, 'version': MARKER}
    deep = tmp_path / ('d' * 120) / ('e' * 120)
    deep.mkdir(parents=True)
    # A path too long for a detail: cut at 200.
    made = bench.client(token_file=str(deep / 'token'))
    seen = [bench.events.wait(Disconnected)]
    assert seen[0].reason == 'token-file' and len(seen[0].detail) == 200
    made.stop()

    def attempt(route, *answers):
        """Script the answers and start the next attempt; return where to wait from."""
        bench.fake.script(route, *answers)
        mark = bench.events.mark()
        if len(seen) == 1:
            bench.client()
        else:
            bench.clock.advance(seen[-1].retry_in_s)
        return mark

    # Answers whose bodies are long, not ASCII, and carry a code that is no code.
    for route, answer in (('health', (421, {}, noisy)), ('health', (500, {}, noisy)),
                          ('stream', (401, {}, noisy)), ('stream', (403, {}, long_code)),
                          ('stream', (418, {}, MARKER.encode() * 500)),
                          ('controllers', (401, {}, noisy))):
        mark = attempt(route, answer)
        seen.append(bench.events.wait(Disconnected, after=mark))
    mark = attempt('controllers', (503, {}, noisy), (503, {}, long_code))
    assert wait_until(lambda: bench.fake.requests_to('/v0/controllers')[-1].status == 503)
    bench.clock.advance(0.5)
    seen.append(bench.events.wait(Disconnected, after=mark))
    # A daemon whose mode is no mode name: connected to, and then the mode "changes".
    mark = attempt('health', (200, {}, weird_mode))
    assert bench.events.wait(Connected, after=mark).health['mode'] == MARKER * 40
    bench.events.wait(Refreshed, after=mark)
    bench.clock.advance(5.0)
    seen.append(bench.events.wait(Disconnected, after=mark))
    assert [event.reason for event in seen] == [
        'token-file', 'host-refused', 'not-a-daemon', 'token-refused', 'origin-refused',
        'handshake-failed', 'token-refused', 'refresh-failed', 'mode-changed']
    for event in seen:
        assert len(event.detail) <= 200, event
        assert re.fullmatch(r'[\x20-\x7e]*', event.detail), event
        assert MARKER not in event.detail and MARKER.lower() not in event.detail, event
        assert 'AAAA' not in event.detail and TOKEN not in event.detail, event
    assert [event.detail for event in seen[1:]] == [
        'health answered HTTP 421',
        'health answered HTTP 500, not a health answer',
        'stream handshake answered HTTP 401',
        'stream handshake answered HTTP 403',
        'stream handshake answered HTTP 418',
        'the controller list answered HTTP 401',
        'two controller list requests in a row failed: HTTP 503',
        'the daemon changed from another mode to live']


def test_text_for_a_detail_is_cleaned():
    clean = daemon_client._clean
    assert clean('a\x00b\u00e9c\nd\te') == 'a?b?c?d?e'
    assert clean('x' * 500) == 'x' * 200
    assert clean(f'before {TOKEN} after {TOKEN}', TOKEN) == 'before  after '
    assert clean(404) == '404' and clean('') == ''
    ended = daemon_client._end('closed', f'{TOKEN}\n' * 20, TOKEN)
    assert ended == daemon_client._End('closed', '?' * 20, False)


def test_an_exception_is_named_by_its_class_and_the_systems_own_words():
    text = daemon_client._exception_text
    assert text(TimeoutError('timed out')) == 'timed out'
    assert text(websocket.WebSocketTimeoutException(MARKER)) == 'timed out'
    assert text(ConnectionRefusedError(111, 'Connection refused')) == \
        'ConnectionRefusedError: Connection refused'
    assert text(OSError(MARKER)) == 'OSError'
    # What a peer sent can be quoted in an exception's message: never passed on.
    assert text(http.client.BadStatusLine(MARKER)) == 'BadStatusLine'
    assert text(websocket.WebSocketPayloadException('cannot decode: ' + MARKER)) == \
        'WebSocketPayloadException'
    refused = websocket.WebSocketBadStatusException(
        f'Handshake status 401 -+-+- {MARKER}', 401, None, {'x': MARKER}, MARKER.encode())
    assert text(refused) == 'WebSocketBadStatusException'
    assert text(ValueError(MARKER)) == 'ValueError'


def test_only_a_code_that_looks_like_one_is_taken_from_an_error_body():
    code = daemon_client._error_code
    assert code({'error': 'x', 'code': 'TOKEN_INVALID'}) == 'TOKEN_INVALID'
    assert code({'code': 'A' * 48}) == 'A' * 48
    for unusable in ({'code': 'A' * 49}, {'code': 'lower'}, {'code': 'WITH SPACE'},
                     {'code': 'NEW\nLINE'}, {'code': 5}, {'code': ''}, {}, None, [], 'TEXT'):
        assert code(unusable) == ''


def test_an_answer_over_the_size_limit_is_refused(bench, monkeypatch):
    monkeypatch.setattr(daemon_client, 'MAX_ANSWER_BYTES', 3000)
    bench.fake.set_controllers([frames.edge_controller('edge-1'), frames.controller('cell-1')])
    bench.client()
    bench.events.wait(FrameReceived)
    assert wait_until(lambda: len(bench.fake.requests_to('/v0/controllers')) == 1)
    bench.clock.advance(0.5)
    ended = bench.events.wait(Disconnected)
    assert ended.reason == 'refresh-failed'
    assert ended.detail.endswith('the answer is larger than 16 MiB')
    assert not bench.events.of(Refreshed)


def test_an_answer_is_read_no_further_than_the_limit(monkeypatch):
    class Response:

        def __init__(self, chunks, declared=None):
            self.chunks = list(chunks)
            self.declared = declared
            self.reads = 0

        def getheader(self, name):
            assert name == 'Content-Length'
            return self.declared

        def read1(self, size):
            self.reads += 1
            return self.chunks.pop(0) if self.chunks else b''

    monkeypatch.setattr(daemon_client, 'MAX_ANSWER_BYTES', 10)
    read = daemon_client._read_limited
    assert read(Response([b'12345', b'67890'], '10')) == b'1234567890'
    assert read(Response([])) == b''
    declared = Response([b'x' * 11], '11')
    with pytest.raises(daemon_client._AnswerTooLarge):
        read(declared)
    assert declared.reads == 0
    endless = Response([b'12345'] * 100, 'what')
    with pytest.raises(daemon_client._AnswerTooLarge):
        read(endless)
    assert endless.reads == 3


# ----- robustness -------------------------------------------------------------------

def test_a_failing_sink_or_targets_does_not_stop_the_client(bench, debug_logs):
    seen = []

    def sink(event):
        seen.append(event)
        if isinstance(event, (HealthRead, Refreshed, FrameReceived)):
            raise RuntimeError('the sink failed')

    def targets():
        raise RuntimeError('no targets today')

    def count(kind):
        return len([event for event in list(seen) if isinstance(event, kind)])

    bench.client(sink=sink, targets=targets)
    assert wait_until(lambda: count(FrameReceived) == 1 and count(Refreshed) == 1)
    bench.fake.push(frames.delta_frame('cell-1', {'panel': frames.panel_group()}))
    assert wait_until(lambda: count(FrameReceived) == 2)
    bench.clock.advance(0.5)
    assert wait_until(lambda: count(Refreshed) == 2)
    assert count(Disconnected) == 0 and bench.posts() == []
    logged = {record.getMessage() for record in debug_logs.records
              if record.levelno == logging.ERROR}
    assert logged == {'the event sink failed on HealthRead: RuntimeError',
                      'the event sink failed on FrameReceived: RuntimeError',
                      'the event sink failed on Refreshed: RuntimeError',
                      'targets() failed: RuntimeError'}


def test_targets_that_are_no_targets_are_passed_over(bench):
    bench.targets = (('cell-1', True), 'cell-1', None, ('cell-1', True, ('do_a',)))
    bench.connected()
    bench.events.wait(LeaseResult, lambda event: event.kind == 'io')
    assert len(bench.posts()) == 2 and not bench.events.of(Disconnected)


def test_nothing_of_a_connection_follows_its_disconnected(bench):
    bench.targets = (('cell-1', True, ('do_a',)),)
    # A fast clock: a refresh and a renewal at nearly every tick of the lease thread.
    bench.client(clock=lambda: time.monotonic() * 400.0)
    mark = 0
    for _ in range(15):
        bench.events.wait(LeaseResult, lambda event: event.kind == 'io', after=mark)
        time.sleep(0.01)
        mark = bench.events.mark()
        bench.fake.close_stream(1012)
        bench.events.wait(Disconnected, after=mark)
        mark = bench.events.mark()
    connected = False
    kinds = set()
    for event in bench.events.of():
        if isinstance(event, Connected):
            assert not connected
            connected = True
            snapshot = False
        elif isinstance(event, FrameReceived) and not snapshot:
            # The first thing a connection hands on is its snapshot.
            assert connected and event.frame['type'] == 'snapshot'
            snapshot = True
            kinds.add(FrameReceived)
        elif isinstance(event, Disconnected):
            connected = False
        elif not isinstance(event, HealthRead):
            # (A health read also belongs to an attempt, before its Connected.)
            kinds.add(type(event))
            assert connected and snapshot, f'{type(event).__name__} outside a connection'
    assert kinds == {FrameReceived, Refreshed, LeaseResult}
    assert len(bench.events.of(Connected)) >= 15


# ----- the contract -----------------------------------------------------------------

def test_the_names_other_modules_code_against():
    def names(kind):
        return [field.name for field in dataclasses.fields(kind)]

    assert names(Arrival) == ['wall_ns', 'mono']
    assert names(HealthRead) == ['health'] and names(Connected) == ['health']
    assert names(FrameReceived) == ['frame', 'at'] and names(Refreshed) == ['answer', 'at']
    assert names(Disconnected) == ['reason', 'detail', 'retry_in_s']
    assert names(LeaseResult) == ['controller_id', 'kind', 'ok', 'code', 'answer']
    assert names(ClientSettings) == [
        'daemon_url', 'token_file', 'allow_replay', 'refresh_hz', 'refresh_timeout_s',
        'reconnect_min_s', 'reconnect_max_s', 'http_timeout_s']
    for kind in (Arrival, HealthRead, Connected, FrameReceived, Refreshed, Disconnected,
                 LeaseResult, ClientSettings):
        assert kind.__dataclass_params__.frozen
    assert daemon_client.Target == tuple[str, bool, tuple[str, ...]]
    made = inspect.signature(DaemonClient.__init__).parameters
    assert list(made) == ['self', 'settings', 'sink', 'targets', 'clock', 'wall_ns', 'rng']
    assert [made[name].kind for name in ('clock', 'wall_ns', 'rng')] == [
        inspect.Parameter.KEYWORD_ONLY] * 3
    assert made['clock'].default is time.monotonic and made['wall_ns'].default is time.time_ns
    assert made['rng'].default is None
    assert list(inspect.signature(DaemonClient.start).parameters) == ['self']
    stop = inspect.signature(DaemonClient.stop).parameters
    assert list(stop) == ['self', 'timeout_s'] and stop['timeout_s'].default == 3.0
    assert set(daemon_client._CLOSE_REASONS.values()) < set(daemon_client.REASONS)
    assert len(set(daemon_client.REASONS)) == len(daemon_client.REASONS) == 18
    assert 'stream-stalled' in daemon_client.REASONS
    ending = inspect.signature(DaemonClient.end_stalled_stream).parameters
    assert list(ending) == ['self', 'detail'] and ending['detail'].default == ''


def test_the_client_modules_import_no_ros_and_no_other_module_of_the_package():
    allowed = {'abb_rws_bridge.backoff', 'abb_rws_bridge.token_file'}
    for module in (daemon_client, backoff, token_file):
        source = inspect.getsource(module)
        assert 'rclpy' not in source
        imported = set(re.findall(r'^(?:from|import)\s+(abb_rws_bridge[\w.]*)', source, re.M))
        assert imported <= allowed, module.__name__
    assert not re.findall(r'^(?:from|import)\s+abb_rws_bridge', inspect.getsource(backoff), re.M)


@pytest.mark.parametrize('url', [
    'https://127.0.0.1:7810', 'http://127.0.0.1', 'http://user:secret@127.0.0.1:7810',
    'http://user@127.0.0.1:7810', 'http://127.0.0.1:7810/path', 'http://127.0.0.1:7810/?q=1',
    'http://127.0.0.1:7810/#part', 'ws://127.0.0.1:7810', '127.0.0.1:7810', '', 'http://:7810',
    'http://127.0.0.1:port', 'http://127.0.0.1:99999', None,
])
def test_a_daemon_url_that_is_not_plain_http_host_and_port_is_refused(bench, url):
    with pytest.raises(ValueError) as caught:
        DaemonClient(bench.settings(daemon_url=url), bench.events, tuple)
    assert 'http://<host>:<port>' in str(caught.value) and 'secret' not in str(caught.value)


def test_the_stream_address_follows_the_daemon_url(bench):
    for url, host, port, stream in [
            ('http://127.0.0.1:7810', '127.0.0.1', 7810, 'ws://127.0.0.1:7810/v0/stream'),
            ('http://[::1]:7811/', '::1', 7811, 'ws://[::1]:7811/v0/stream'),
            ('http://host.docker.internal:7830', 'host.docker.internal', 7830,
             'ws://host.docker.internal:7830/v0/stream')]:
        made = DaemonClient(bench.settings(daemon_url=url), bench.events, tuple)
        assert (made._host, made._port, made._ws_url) == (host, port, stream)


@pytest.mark.parametrize('changes', [
    {'refresh_hz': 0.0}, {'refresh_hz': -1.0}, {'refresh_hz': float('nan')},
    {'refresh_timeout_s': 0.0}, {'http_timeout_s': 0.0}, {'reconnect_min_s': 0.0},
    {'reconnect_min_s': 20.0, 'reconnect_max_s': 15.0},
])
def test_settings_that_make_no_sense_are_refused(bench, changes):
    with pytest.raises(ValueError):
        DaemonClient(bench.settings(**changes), bench.events, tuple)

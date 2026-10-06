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
The stand-in obeys the daemon's rules, and every answer it gives fits the schema.

Every HTTP answer read through ``ask`` and every stream frame read through
``next_frame`` is validated against the vendored schema on the way, so no test here
can look at an answer that the schema would refuse.
"""

import http.client
import json
import re
import socket
import struct
import time

import fake_daemon
import frames
import pytest
import websocket

SCHEMA = frames.load_schema()
TOKEN = fake_daemon.TOKEN
OTHER_TOKEN = fake_daemon.OTHER_TOKEN
FakeDaemon = fake_daemon.FakeDaemon
wait_until = fake_daemon.wait_until

# Which schema definition a 200 answer of each route must fit.
ANSWERS = {
    'health': 'HealthAnswer',
    'controllers': 'ControllerList',
    'controller': 'ControllerSnapshot',
    'motion': 'MotionWatch',
    'io': 'IoWatch',
}
validated = set()


def fits(definition, data):
    """Check data against a definition of the vendored schema and return it."""
    errors = frames.schema_errors(definition, data, SCHEMA)
    assert errors == [], f'{definition}: {errors}'
    validated.add(definition)
    return data


def ask(fake, method, path, *, token=TOKEN, body=None, headers=None, timeout=2.0, check=True):
    """
    Send one request; return (status, headers, parsed body), the body validated.

    check=False is for an answer a test scripted itself and that is no daemon answer.
    """
    send = dict(headers or {})
    connection = http.client.HTTPConnection('127.0.0.1', fake.port, timeout=timeout)
    try:
        connection.putrequest(method, path, skip_host=any(name.lower() == 'host' for name in send))
        if token is not None:
            send['x-abb-rws-token'] = token
        payload = None
        if body is not None:
            payload = body if isinstance(body, bytes) else json.dumps(body).encode('utf-8')
            send['Content-Type'] = 'application/json'
            send['Content-Length'] = str(len(payload))
        for name, value in send.items():
            connection.putheader(name, value)
        connection.endheaders(payload)
        response = connection.getresponse()
        raw = response.read()
        answer_headers = {name.lower(): value for name, value in response.getheaders()}
        status = response.status
    finally:
        connection.close()
    data = json.loads(raw) if raw else None
    route = FakeDaemon._route(method, path.split('?')[0])[0]
    if not check:
        pass
    elif status == 200 and route in ANSWERS:
        fits(ANSWERS[route], data)
    elif status >= 400:
        fits('ErrorBody', data)
        assert answer_headers['content-type'].startswith('application/json')
    return status, answer_headers, data


def consume(fake, *, token=TOKEN, timeout=2.0, **options):
    """Open the stream as a consumer would, with the WebSocket library."""
    options.setdefault('subprotocols', ['abb-rws.v0'])
    options.setdefault('suppress_origin', True)
    header = [] if token is None else [f'x-abb-rws-token: {token}']
    return websocket.create_connection(fake.ws_url, timeout=timeout, header=header,
                                       http_no_proxy=['*'], **options)


def next_frame(stream):
    """Read one stream frame; it must fit the schema."""
    return fits('StreamFrame', json.loads(stream.recv()))


def next_control(stream):
    """Read one frame of any kind; return (opcode, payload)."""
    opcode, frame = stream.recv_data_frame(control_frame=True)
    return opcode, frame.data


def refused(fake, **options):
    """Try to open the stream; return (status, parsed body) of the refusal."""
    with pytest.raises(websocket.WebSocketBadStatusException) as caught:
        consume(fake, **options)
    body = json.loads(caught.value.resp_body)
    fits('ErrorBody', body)
    return caught.value.status_code, body


@pytest.fixture
def fake():
    started = FakeDaemon([frames.controller('cell-1'), frames.edge_controller('edge-1')])
    with started:
        yield started


@pytest.fixture
def streams():
    opened = []
    yield opened
    for stream in opened:
        stream.shutdown()


def test_health_needs_no_token_and_says_the_mode(fake):
    status, headers, body = ask(fake, 'GET', '/v0/health', token=None)
    assert status == 200 and headers['content-type'].startswith('application/json')
    assert body['status'] == 'ok' and body['mode'] == 'live' and body['replay'] is None
    assert body['version'] == '0.1.0' and body['startedAt'] == fake.started_at
    fake.mode = 'replay'
    fake.version = '9.9.9'
    body = ask(fake, 'GET', '/v0/health', token=None)[2]
    assert body['mode'] == 'replay' and body['replay']['state'] == 'playing'
    assert body['version'] == '9.9.9'
    fake.replay = frames.replay_health(state='paused', position_ms=1500, loop=True)
    body = ask(fake, 'GET', '/v0/health', token=OTHER_TOKEN)[2]
    assert body['replay']['state'] == 'paused' and body['replay']['positionMs'] == 1500


@pytest.mark.parametrize('method, path, body', [
    ('GET', '/v0/controllers', None),
    ('GET', '/v0/controllers/cell-1', None),
    ('GET', '/v0/controllers/cell-1/io/signals', None),
    ('POST', '/v0/controllers/cell-1/motion/watch', {'leaseId': 'new'}),
    ('POST', '/v0/controllers/cell-1/io/watch', {'signals': ['do_gripper']}),
    ('GET', '/v0/stream', None),
    ('GET', '/v0/no/such/route', None),
    ('DELETE', '/v0/controllers/cell-1', None),
])
def test_every_route_but_health_needs_the_token(fake, method, path, body):
    status, _, answer = ask(fake, method, path, token=None, body=body)
    assert (status, answer['code']) == (401, 'TOKEN_MISSING')
    status, _, answer = ask(fake, method, path, token=OTHER_TOKEN, body=body)
    assert (status, answer['code']) == (401, 'TOKEN_INVALID')
    assert fake.leases == {}


def test_a_path_that_is_no_route_is_404_with_the_token(fake):
    for method, path in (('GET', '/v0/no/such/route'), ('GET', '/v0/stream'),
                         ('DELETE', '/v0/controllers/cell-1'), ('POST', '/v0/health')):
        status, _, answer = ask(fake, method, path)
        assert (status, answer['code']) == (404, 'NO_ROUTE'), (method, path)


def test_the_host_rule(fake):
    port = fake.port

    def health(host):
        return ask(fake, 'GET', '/v0/health', token=None, headers={'Host': host})

    assert health(f'127.0.0.1:{port}')[0] == 200
    assert health(f'LOCALHOST:{port}')[0] == 200
    assert health(f'[::1]:{port}')[0] == 200
    assert health('localhost')[0] == 200
    for host in ('evil.example', f'evil.example:{port}', f'host.docker.internal:{port}',
                 f'127.0.0.1.evil.example:{port}'):
        status, _, answer = health(host)
        assert (status, answer['code']) == (421, 'HOST_NOT_ALLOWED'), host
    # A configurable list of further names, as the daemon's own option gives it.
    fake.allowed_hosts.add('host.docker.internal')
    assert health(f'host.docker.internal:{port}')[0] == 200
    assert health(f'evil.example:{port}')[0] == 421
    # And configurable so that even the loopback address is refused.
    fake.allowed_hosts = set()
    status, _, answer = health(f'127.0.0.1:{port}')
    assert (status, answer['code']) == (421, 'HOST_NOT_ALLOWED')
    assert ask(fake, 'GET', '/v0/controllers')[0] == 421


def test_the_origin_rule(fake):
    port = fake.port

    def listed(origin):
        return ask(fake, 'GET', '/v0/controllers', headers={'Origin': origin})

    assert listed(f'http://127.0.0.1:{port}')[0] == 200
    assert listed(f'http://localhost:{port}')[0] == 200
    for origin in ('http://evil.example', 'http://127.0.0.1:5173', 'null',
                   f'https://127.0.0.1:{port}'):
        status, _, answer = listed(origin)
        assert (status, answer['code']) == (403, 'ORIGIN_NOT_ALLOWED'), origin
    status, _, answer = ask(fake, 'GET', '/v0/health', token=None,
                            headers={'Origin': 'http://evil.example'})
    assert (status, answer['code']) == (403, 'ORIGIN_NOT_ALLOWED')
    fake.allowed_origins.add('http://127.0.0.1:5173')
    assert listed('http://127.0.0.1:5173')[0] == 200


def test_the_controller_list_shows_the_state_and_the_time(fake):
    fake.now = lambda: frames.iso(42)
    status, _, body = ask(fake, 'GET', '/v0/controllers')
    assert status == 200
    assert body['schemaVersion'] == frames.SCHEMA_VERSION and body['observedAt'] == frames.iso(42)
    assert body['controllers'] == [frames.controller('cell-1'), frames.edge_controller('edge-1')]
    assert fake.controllers == body['controllers']
    fake.schema_version = '0.13'
    assert ask(fake, 'GET', '/v0/controllers?x=1')[2]['schemaVersion'] == '0.13'


def test_a_hook_runs_before_every_controller_list_answer(fake):
    calls = []

    def advance(daemon):
        calls.append(daemon)
        motion = frames.motion_group(as_of=frames.iso(len(calls)))
        daemon.update('cell-1', motion=motion)

    fake.on_controller_list = advance
    first = ask(fake, 'GET', '/v0/controllers')[2]['controllers'][0]
    second = ask(fake, 'GET', '/v0/controllers')[2]['controllers'][0]
    assert calls == [fake, fake]
    assert (first['motion']['asOf'], second['motion']['asOf']) == (frames.iso(1), frames.iso(2))


def test_one_controller_and_its_signal_table(fake):
    status, _, body = ask(fake, 'GET', '/v0/controllers/cell-1')
    assert status == 200 and body == frames.controller('cell-1')
    assert ask(fake, 'GET', '/v0/controllers/edge%2D1')[2] == frames.edge_controller('edge-1')
    status, _, body = ask(fake, 'GET', '/v0/controllers/cell-7')
    assert (status, body['code']) == (404, 'NO_CONTROLLER') and 'cell-7' in body['error']
    status, _, table = ask(fake, 'GET', '/v0/controllers/edge-1/io/signals')
    assert status == 200 and set(table) == {'signals'}
    assert table == frames.io_signal_table(frames.EDGE_CONTROLLER['io']['signals'])
    assert all(set(row) == {'name', 'type', 'value'} for row in table['signals'])
    assert ask(fake, 'GET', '/v0/controllers/cell-7/io/signals')[0] == 404


def test_motion_watch_leases(fake):
    path = '/v0/controllers/cell-1/motion/watch'
    status, _, body = ask(fake, 'POST', path, body={'leaseId': 'new'})
    assert status == 200
    assert body == {'leaseMs': 15000, 'targetHz': 8.0, 'leaseId': frames.LEASE_ID}
    # Another "new" is another lease; a known id renews its own.
    assert ask(fake, 'POST', path, body={'leaseId': 'new'})[2]['leaseId'] == 'lease-2'
    assert ask(fake, 'POST', path, body={'leaseId': 'lease-1'})[2]['leaseId'] == 'lease-1'
    # An id the daemon does not know opens that lease again under the same id.
    assert ask(fake, 'POST', path, body={'leaseId': 'kept_by-Me'})[2]['leaseId'] == 'kept_by-Me'
    # Without an id (or without a body) it is the shared lease, which has no id.
    assert 'leaseId' not in ask(fake, 'POST', path)[2]
    assert 'leaseId' not in ask(fake, 'POST', path, body={})[2]
    assert set(fake.leases[('cell-1', 'motion')]) == {'lease-1', 'lease-2', 'kept_by-Me', ''}
    # The same id serves another controller.
    other = '/v0/controllers/edge-1/motion/watch'
    assert ask(fake, 'POST', other, body={'leaseId': 'lease-1'})[2]['leaseId'] == 'lease-1'
    fake.lease_ms = 6000
    fake.target_hz = 2.0
    assert ask(fake, 'POST', path, body={'leaseId': 'lease-1'})[2] == {
        'leaseMs': 6000, 'targetHz': 2.0, 'leaseId': 'lease-1'}


@pytest.mark.parametrize('kind, body', [
    ('motion', {'leaseId': 'has space'}),
    ('motion', {'leaseId': 5}),
    ('motion', {'leaseId': ''}),
    ('motion', {'leaseId': 'x' * 65}),
    ('motion', {'other': 1}),
    ('io', {}),
    ('io', {'leaseId': 'new'}),
    ('io', {'signals': 'do_gripper'}),
    ('io', {'signals': [1]}),
    ('io', {'signals': ['']}),
    ('io', {'signals': ['x' * 161]}),
    ('io', {'signals': ['do_a'], 'leaseId': 'no/slash'}),
    ('io', {'signals': ['do_a'], 'more': True}),
    ('io', ['do_a']),
])
def test_a_watch_body_that_does_not_fit_is_400(fake, kind, body):
    status, _, answer = ask(fake, 'POST', f'/v0/controllers/cell-1/{kind}/watch', body=body)
    assert (status, answer['code']) == (400, 'BAD_REQUEST')
    assert fake.leases == {}


@pytest.mark.parametrize('kind, body', [
    ('motion', {'leaseId': 'new'}), ('io', {'signals': ['do_a'], 'leaseId': 'new'})])
def test_a_watch_is_refused_for_no_controller_and_for_the_wrong_one(fake, kind, body):
    status, _, answer = ask(fake, 'POST', f'/v0/controllers/cell-7/{kind}/watch', body=body)
    assert (status, answer['code']) == (404, 'NO_CONTROLLER')
    wrong = frames.controller('cell-9', link=frames.link_group(
        identity_status='mismatch', actual='00000000-0000-4000-8000-000000000002'))
    fake.push(frames.delta_frame('cell-9', frames.groups_of(wrong)))
    status, _, answer = ask(fake, 'POST', f'/v0/controllers/cell-9/{kind}/watch', body=body)
    assert (status, answer['code']) == (409, 'IDENTITY_MISMATCH')
    assert ask(fake, 'GET', '/v0/controllers/cell-9/io/signals')[0] == 409
    assert ask(fake, 'GET', '/v0/controllers/cell-9')[0] == 200
    assert fake.leases == {}


def test_io_watch_grants_by_name(fake):
    path = '/v0/controllers/cell-1/io/watch'
    fake.known_signals['cell-1'] = {'do_gripper', 'di_ready', 'go_program'}
    asked = ['do_gripper', ' di_ready ', 'do_gripper', 'bad name', 'do_unknown',
             'Local/DRV_1/go_program', '   ', 'a/b']
    status, _, body = ask(fake, 'POST', path, body={'signals': asked, 'leaseId': 'new'})
    assert status == 200 and body['leaseId'] == 'lease-1' and body['leaseMs'] == 15000
    assert body['accepted'] == ['do_gripper', 'di_ready', 'Local/DRV_1/go_program']
    rejected = {entry['name']: entry['reason'] for entry in body['rejected']}
    assert set(rejected) == {'bad name', 'do_unknown', 'a/b'}
    assert rejected['bad name'].startswith('not an I/O signal name')
    assert rejected['a/b'].startswith('not an I/O signal name')
    assert rejected['do_unknown'] == 'no such signal on this controller'
    assert fake.io_watched('cell-1') == ['do_gripper', 'di_ready', 'Local/DRV_1/go_program']
    # A request replaces the lease's set; an empty list leaves it holding none.
    body = ask(fake, 'POST', path, body={'signals': ['di_ready'], 'leaseId': 'lease-1'})[2]
    assert body['accepted'] == ['di_ready'] and body['rejected'] == []
    assert fake.io_watched('cell-1') == ['di_ready']
    body = ask(fake, 'POST', path, body={'signals': [], 'leaseId': 'lease-1'})[2]
    assert body == {'leaseMs': 15000, 'accepted': [], 'rejected': [], 'leaseId': 'lease-1'}
    assert fake.io_watched('cell-1') == []
    assert fake.leases[('cell-1', 'io')] == {'lease-1': []}
    # A controller without a known table accepts every well-formed name.
    body = ask(fake, 'POST', '/v0/controllers/edge-1/io/watch',
               body={'signals': ['anything_goes']})[2]
    assert body['accepted'] == ['anything_goes'] and 'leaseId' not in body


def test_io_watch_caps(fake):
    path = '/v0/controllers/cell-1/io/watch'
    first = [f'sig_{index}' for index in range(20)]
    body = ask(fake, 'POST', path, body={'signals': first, 'leaseId': 'new'})[2]
    assert body['accepted'] == first[:16]
    assert [entry['name'] for entry in body['rejected']] == first[16:]
    assert all('lease cap' in entry['reason'] for entry in body['rejected'])
    # A second lease: the controller holds 24 in all, and a signal another lease
    # already watches does not count against that.
    second = ['sig_0'] + [f'more_{index}' for index in range(12)]
    body = ask(fake, 'POST', path, body={'signals': second, 'leaseId': 'new'})[2]
    assert body['leaseId'] == 'lease-2'
    assert body['accepted'] == ['sig_0'] + [f'more_{index}' for index in range(8)]
    assert [entry['name'] for entry in body['rejected']] == [f'more_{i}' for i in range(8, 12)]
    assert all('controller cap' in entry['reason'] for entry in body['rejected'])
    assert len(fake.io_watched('cell-1')) == 24
    # The shared lease has no cap of its own.
    shared = [f'any_{index}' for index in range(20)]
    body = ask(fake, 'POST', '/v0/controllers/edge-1/io/watch', body={'signals': shared})[2]
    assert body['accepted'] == shared and body['rejected'] == []


def test_scripted_answers_are_given_once_each(fake):
    path = '/v0/controllers/cell-1/motion/watch'
    fake.script('motion', frames.error_answer('LEASE_LIMIT', retry_after_s=7))
    # The checks still come first: a request without the token takes no answer.
    assert ask(fake, 'POST', path, token=None, body={'leaseId': 'new'})[0] == 401
    status, headers, body = ask(fake, 'POST', path, body={'leaseId': 'new'})
    assert (status, body['code'], headers['retry-after']) == (429, 'LEASE_LIMIT', '7')
    assert ask(fake, 'POST', path, body={'leaseId': 'new'})[0] == 200
    # For one controller only.
    busy = (503, {}, fake_daemon.error_body('CONTROLLER_ERROR', 'synthetic'))
    fake.script('io', busy, busy, controller_id='edge-1')
    watch = {'signals': ['do_a']}
    assert ask(fake, 'POST', '/v0/controllers/cell-1/io/watch', body=watch)[0] == 200
    assert ask(fake, 'POST', '/v0/controllers/edge-1/io/watch', body=watch)[0] == 503
    assert ask(fake, 'POST', '/v0/controllers/edge-1/io/watch', body=watch)[0] == 503
    assert ask(fake, 'POST', '/v0/controllers/edge-1/io/watch', body=watch)[0] == 200
    fake.script('health', (500, {'Content-Type': 'text/plain'}, None),
                (200, {}, b'[1, 2'))
    assert ask(fake, 'GET', '/v0/health', token=None, check=False)[:3:2] == (500, None)
    with pytest.raises(ValueError):
        ask(fake, 'GET', '/v0/health', token=None, check=False)
    assert ask(fake, 'GET', '/v0/health', token=None)[0] == 200
    with pytest.raises(ValueError):
        fake.script('nothing', busy)


def test_the_stream_sends_a_snapshot_and_then_what_is_pushed(fake, streams):
    fake.now = lambda: frames.iso(5)
    stream = consume(fake)
    streams.append(stream)
    assert stream.status == 101 and stream.subprotocol == 'abb-rws.v0'
    snapshot = next_frame(stream)
    assert snapshot == frames.snapshot_frame(
        [frames.controller('cell-1'), frames.edge_controller('edge-1')],
        observed_at=frames.iso(5))
    assert wait_until(lambda: fake.consumers == 1) and fake.attached == 1
    motion = frames.motion_group([frames.motion_unit(joints=[frames.joint('J6', 5.0)])],
                                 as_of=frames.iso(6))
    delta = frames.delta_frame('cell-1', {'motion': motion}, observed_at=frames.iso(6))
    fake.push(delta)
    assert next_frame(stream) == delta
    assert fake.controllers[0]['motion'] == motion
    added = frames.controller('cell-2', name='Second')
    fake.push(frames.delta_frame('cell-2', frames.groups_of(added), name='Second'))
    assert next_frame(stream)['id'] == 'cell-2'
    assert [entry['id'] for entry in fake.controllers] == ['cell-1', 'edge-1', 'cell-2']
    assert fake.controllers[2] == added
    fake.push(frames.removed_frame('edge-1'))
    assert next_frame(stream) == frames.removed_frame('edge-1')
    assert [entry['id'] for entry in fake.controllers] == ['cell-1', 'cell-2']
    # A second consumer gets the state as it is now.
    second = consume(fake)
    streams.append(second)
    assert [entry['id'] for entry in next_frame(second)['controllers']] == ['cell-1', 'cell-2']
    fake.push(frames.snapshot_frame([frames.controller('only')]))
    assert next_frame(stream)['type'] == 'snapshot' and next_frame(second)['type'] == 'snapshot'
    assert [entry['id'] for entry in fake.controllers] == ['only']
    assert fake.consumers == 2 and fake.attached == 2


def test_state_can_change_without_a_frame(fake, streams):
    stream = consume(fake, timeout=0.15)
    streams.append(stream)
    next_frame(stream)
    fake.update('cell-1', motion=frames.motion_group(as_of=frames.iso(9)), name='Renamed')
    fake.apply(frames.removed_frame('edge-1'))
    with pytest.raises(websocket.WebSocketTimeoutException):
        stream.recv()
    listed = ask(fake, 'GET', '/v0/controllers')[2]['controllers']
    assert [entry['id'] for entry in listed] == ['cell-1']
    assert listed[0]['motion']['asOf'] == frames.iso(9) and listed[0]['name'] == 'Renamed'
    fake.set_controllers([frames.controller('a'), frames.controller('b')])
    assert [entry['id'] for entry in fake.controllers] == ['a', 'b']
    with pytest.raises(websocket.WebSocketTimeoutException):
        stream.recv()


def test_the_stream_is_refused_like_any_other_route(fake, streams):
    assert refused(fake, token=None) == (401, frames.error_answer('TOKEN_MISSING')[2])
    assert refused(fake, token=OTHER_TOKEN) == (401, frames.error_answer('TOKEN_INVALID')[2])
    status, body = refused(fake, host='evil.example')
    assert (status, body['code']) == (421, 'HOST_NOT_ALLOWED')
    status, body = refused(fake, suppress_origin=False, origin='http://evil.example')
    assert (status, body['code']) == (403, 'ORIGIN_NOT_ALLOWED')
    # A token offered as a subprotocol is taken, and never selected.
    by_protocol = consume(fake, token=None,
                          subprotocols=['abb-rws.v0', 'abb-rws-token.' + TOKEN])
    streams.append(by_protocol)
    assert by_protocol.status == 101 and by_protocol.subprotocol == 'abb-rws.v0'
    assert next_frame(by_protocol)['type'] == 'snapshot'
    status, body = refused(fake, token=None, subprotocols=['abb-rws-token.' + TOKEN])
    assert (status, body['code']) == (400, 'BAD_REQUEST')
    status, body = refused(fake, token=None,
                           subprotocols=['abb-rws.v0', 'abb-rws-token.' + OTHER_TOKEN])
    assert (status, body['code']) == (401, 'TOKEN_INVALID')
    # The browser's own origin passes.
    own = consume(fake, suppress_origin=False)
    streams.append(own)
    assert own.status == 101
    # No subprotocol offered: none selected.
    plain = websocket.create_connection(
        fake.ws_url, timeout=2.0, header=[f'x-abb-rws-token: {TOKEN}'], suppress_origin=True,
        http_no_proxy=['*'])
    streams.append(plain)
    assert plain.status == 101 and 'sec-websocket-protocol' not in plain.headers
    assert fake.attached == 3


def test_a_consumer_that_sends_a_data_frame_is_closed_with_1008(fake, streams):
    stream = consume(fake)
    streams.append(stream)
    next_frame(stream)
    stream.send('hello')
    opcode, payload = next_control(stream)
    assert opcode == websocket.ABNF.OPCODE_CLOSE
    assert struct.unpack('!H', payload[:2])[0] == 1008
    assert wait_until(lambda: fake.consumers == 0)
    assert fake.data_frames == 1


def test_pings_and_pongs(fake, streams):
    stream = consume(fake)
    streams.append(stream)
    next_frame(stream)
    fake.ping()
    opcode, _ = next_control(stream)
    assert opcode == websocket.ABNF.OPCODE_PING
    assert wait_until(lambda: fake.pongs == 1)
    stream.ping('are you there')
    assert next_control(stream) == (websocket.ABNF.OPCODE_PONG, b'are you there')
    assert fake.data_frames == 0 and fake.consumers == 1


@pytest.mark.parametrize('code', [1000, 1001, 1008, 1011, 1012, 1013])
def test_close_stream_closes_every_consumer_with_the_code(fake, streams, code):
    first, second = consume(fake), consume(fake)
    streams.extend([first, second])
    next_frame(first)
    next_frame(second)
    fake.close_stream(code, 'a reason')
    for stream in (first, second):
        opcode, payload = next_control(stream)
        assert opcode == websocket.ABNF.OPCODE_CLOSE
        assert struct.unpack('!H', payload[:2])[0] == code and payload[2:] == b'a reason'
    assert wait_until(lambda: fake.consumers == 0)
    # The library answers a close with its own, code 1000.
    assert wait_until(lambda: fake.consumer_closes == [1000, 1000])
    # The fake itself goes on: a new consumer attaches.
    third = consume(fake)
    streams.append(third)
    assert next_frame(third)['type'] == 'snapshot'


def test_a_consumer_too_many_is_closed_with_1013(fake, streams):
    fake.max_consumers = 1
    first = consume(fake)
    streams.append(first)
    next_frame(first)
    second = consume(fake)
    streams.append(second)
    assert second.status == 101
    opcode, payload = next_control(second)
    assert opcode == websocket.ABNF.OPCODE_CLOSE and struct.unpack('!H', payload[:2])[0] == 1013
    assert fake.consumers == 1 and fake.attached == 1


def test_a_consumer_that_closes_is_answered_and_recorded(fake, streams):
    stream = consume(fake)
    streams.append(stream)
    next_frame(stream)
    stream.close(status=1000, timeout=1.0)
    assert wait_until(lambda: fake.consumer_closes == [1000] and fake.consumers == 0)
    gone = consume(fake)
    next_frame(gone)
    gone.shutdown()
    assert wait_until(lambda: fake.consumer_closes == [1000, None] and fake.consumers == 0)


def test_what_a_new_consumer_gets_can_be_replaced(fake, streams):
    fake.attach_frames = [frames.delta_frame('cell-1', {'panel': frames.panel_group()})]
    fake.upgrade_headers = {'X-Extra': 'yes'}
    stream = consume(fake)
    streams.append(stream)
    assert stream.headers['x-extra'] == 'yes'
    assert next_frame(stream)['type'] == 'delta'
    fake.attach_frames = []
    quiet = consume(fake, timeout=0.15)
    streams.append(quiet)
    with pytest.raises(websocket.WebSocketTimeoutException):
        quiet.recv()
    fake.send_text('not json')
    assert quiet.recv() == 'not json'
    fake.send_binary(b'\x00\x01')
    assert next_control(quiet) == (websocket.ABNF.OPCODE_BINARY, b'\x00\x01')


def test_a_frozen_fake_answers_nothing_and_keeps_its_sockets_open(fake, streams):
    stream = consume(fake)
    streams.append(stream)
    next_frame(stream)
    fake.freeze()
    assert fake.frozen
    waiting = socket.create_connection(('127.0.0.1', fake.port), timeout=0.15)
    try:
        waiting.sendall(b'GET /v0/health HTTP/1.1\r\nHost: 127.0.0.1\r\n\r\n')
        # Not answered and not hung up on: the read times out.
        with pytest.raises(socket.timeout):
            waiting.recv(1)
        logged = fake.requests_to('/v0/health')
        assert len(logged) == 1 and logged[0].status is None
        # Nothing is sent to a consumer either, not even an answer to its ping.
        delta = frames.delta_frame('cell-1', {'panel': frames.panel_group(op_mode='MANR')})
        fake.push(delta)
        fake.ping()
        stream.ping('x')
        stream.settimeout(0.15)
        with pytest.raises(websocket.WebSocketTimeoutException):
            next_control(stream)
        fake.unfreeze()
        assert not fake.frozen
        # What waited is answered and sent, in order.
        waiting.settimeout(1.0)
        assert waiting.recv(65536).startswith(b'HTTP/1.1 200')
        assert logged[0].status == 200
        stream.settimeout(1.0)
        assert next_frame(stream) == delta
        seen = {next_control(stream)[0], next_control(stream)[0]}
        assert seen == {websocket.ABNF.OPCODE_PING, websocket.ABNF.OPCODE_PONG}
    finally:
        waiting.close()
    assert ask(fake, 'GET', '/v0/health', token=None)[0] == 200


def test_a_frozen_fake_can_be_stopped(fake):
    fake.freeze()
    waiting = socket.create_connection(('127.0.0.1', fake.port), timeout=1.0)
    try:
        waiting.sendall(b'GET /v0/health HTTP/1.1\r\nHost: 127.0.0.1\r\n\r\n')
        assert wait_until(lambda: len(fake.requests) == 1)
        started = time.monotonic()
        fake.stop()
        assert time.monotonic() - started < 1.0
        # Hung up on without an answer.
        assert waiting.recv(1) == b''
    finally:
        waiting.close()
    assert fake.requests[0].status is None


def test_redirects_name_the_other_address_and_ask_for_nothing(fake, streams):
    with fake_daemon.Bystander() as other:
        fake.redirect(other.url)
        status, headers, body = ask(fake, 'GET', '/v0/health', token=None)
        assert (status, headers['location'], body) == (302, other.url + '/v0/health', None)
        # The library hands a 3xx back as if it were connected when it may not follow it.
        stream = consume(fake, redirect_limit=0)
        streams.append(stream)
        assert stream.status == 302
        assert stream.headers['location'] == f'ws://127.0.0.1:{other.port}/v0/stream'
        # No check comes before a redirect: not the Host, not the token.
        foreign = {'Host': 'evil.example'}
        assert ask(fake, 'GET', '/v0/health', token=None, headers=foreign)[0] == 302
        # Routes that were not named answer as before.
        assert ask(fake, 'GET', '/v0/controllers')[0] == 200
        fake.redirect(other.url, routes=('controllers', 'motion'))
        assert ask(fake, 'GET', '/v0/health', token=None)[0] == 200
        status, headers, _ = ask(fake, 'GET', '/v0/controllers')
        assert (status, headers['location']) == (302, other.url + '/v0/controllers')
        status, headers, _ = ask(fake, 'POST', '/v0/controllers/cell-1/motion/watch',
                                 body={'leaseId': 'new'})
        assert status == 302 and headers['location'].startswith(other.url)
        fake.redirect(None)
        assert ask(fake, 'GET', '/v0/controllers')[0] == 200
        with pytest.raises(ValueError):
            fake.redirect(other.url, routes=('nothing',))
        assert other.connections == 0 and other.received == b''


def test_the_bystander_records_what_a_followed_redirect_sends(fake):
    # The check that "the second listener received nothing" is not vacuous: left to
    # its defaults, the library follows the redirect and repeats its headers there.
    with fake_daemon.Bystander() as other:
        fake.redirect(other.url, routes=('stream',))
        with pytest.raises(websocket.WebSocketBadStatusException) as caught:
            websocket.create_connection(
                fake.ws_url, timeout=2.0, header=['x-marker: followed'], suppress_origin=True,
                http_no_proxy=['*'])
        assert caught.value.status_code == 404
        assert wait_until(lambda: other.connections == 1 and b'\r\n\r\n' in other.received)
        assert other.received.startswith(b'GET /v0/stream HTTP/1.1\r\n')
        assert b'x-marker: followed' in other.received


def test_stop_cuts_everything_and_start_listens_on_the_same_port(fake, streams):
    port = fake.port
    stream = consume(fake)
    streams.append(stream)
    next_frame(stream)
    path = '/v0/controllers/cell-1/motion/watch'
    assert ask(fake, 'POST', path, body={'leaseId': 'new'})[2]['leaseId'] == 'lease-1'
    fake.stop()
    # A killed daemon sends no close frame.
    with pytest.raises((websocket.WebSocketConnectionClosedException, OSError)):
        stream.recv()
    with pytest.raises(ConnectionRefusedError):
        socket.create_connection(('127.0.0.1', port), timeout=1.0)
    assert fake.consumers == 0
    fake.stop()
    assert fake.start() is fake and fake.port == port and fake.url == f'http://127.0.0.1:{port}'
    with pytest.raises(RuntimeError):
        fake.start()
    # It kept its controllers and forgot its leases; a known id is taken as it is.
    assert [entry['id'] for entry in ask(fake, 'GET', '/v0/controllers')[2]['controllers']] == [
        'cell-1', 'edge-1']
    assert fake.leases == {}
    assert ask(fake, 'POST', path, body={'leaseId': 'lease-1'})[2]['leaseId'] == 'lease-1'
    assert ask(fake, 'POST', path, body={'leaseId': 'new'})[2]['leaseId'] == 'lease-2'
    again = consume(fake)
    streams.append(again)
    assert next_frame(again)['type'] == 'snapshot'


def test_the_request_log_never_holds_a_token(fake, streams):
    ask(fake, 'GET', '/v0/health?probe=1', token=None)
    ask(fake, 'GET', '/v0/controllers', token=TOKEN, headers={'X-Other': 'Kept'})
    ask(fake, 'GET', '/v0/controllers', token=OTHER_TOKEN)
    ask(fake, 'POST', '/v0/controllers/cell-1/io/watch',
        body={'signals': ['do_gripper'], 'leaseId': 'new'})
    ask(fake, 'POST', '/v0/controllers/cell-1/motion/watch', body=b'this is not json')
    streams.append(consume(fake, token=None,
                           subprotocols=['abb-rws.v0', 'abb-rws-token.' + TOKEN]))
    with pytest.raises(websocket.WebSocketBadStatusException):
        consume(fake, token=None, subprotocols=['abb-rws.v0', 'abb-rws-token.' + OTHER_TOKEN])
    log = fake.requests
    assert [(entry.method, entry.path, entry.token, entry.status) for entry in log] == [
        ('GET', '/v0/health?probe=1', None, 200),
        ('GET', '/v0/controllers', 'matched', 200),
        ('GET', '/v0/controllers', 'not matched', 401),
        ('POST', '/v0/controllers/cell-1/io/watch', 'matched', 200),
        ('POST', '/v0/controllers/cell-1/motion/watch', 'matched', 200),
        ('GET', '/v0/stream', 'matched', 101),
        ('GET', '/v0/stream', 'not matched', 401),
    ]
    assert 'x-abb-rws-token' not in log[0].headers
    assert log[1].headers['x-abb-rws-token'] == 'matched' and log[1].headers['x-other'] == 'Kept'
    assert log[2].headers['x-abb-rws-token'] == 'not matched'
    assert log[1].headers['host'] == f'127.0.0.1:{fake.port}' and log[1].body is None
    assert log[3].body == {'signals': ['do_gripper'], 'leaseId': 'new'}
    assert log[4].body == 'this is not json'
    assert log[5].headers['sec-websocket-protocol'] == 'abb-rws.v0, abb-rws-token.matched'
    assert log[6].headers['sec-websocket-protocol'] == 'abb-rws.v0, abb-rws-token.not matched'
    everything = ' '.join(repr(entry) + repr(vars(entry)) for entry in log)
    assert TOKEN not in everything and OTHER_TOKEN not in everything
    assert all(all(name == name.lower() for name in entry.headers) for entry in log)
    assert [entry.path for entry in fake.requests_to('/v0/health')] == ['/v0/health?probe=1']
    assert len(fake.requests_to('/v0/controllers', 'GET')) == 2
    assert fake.requests_to('/v0/controllers', 'POST') == []
    fake.clear_requests()
    assert fake.requests == []


def test_play_pushes_frames_at_their_times(fake, streams):
    stream = consume(fake)
    streams.append(stream)
    next_frame(stream)
    timed = [
        (0.0, frames.delta_frame('cell-1', {'panel': frames.panel_group(op_mode='MANR')})),
        (0.06, frames.delta_frame('cell-1', {'panel': frames.panel_group(op_mode='AUTO')})),
        (0.12, frames.removed_frame('edge-1')),
    ]
    started = time.monotonic()
    player = fake.play(iter(timed))
    arrivals = []
    for _, frame in timed:
        assert next_frame(stream) == frame
        arrivals.append(time.monotonic() - started)
    player.join(1.0)
    assert not player.is_alive()
    assert arrivals[1] >= 0.05 and arrivals[2] >= 0.11 and arrivals[2] < 1.0
    assert [entry['id'] for entry in fake.controllers] == ['cell-1']
    # A hundred times as fast: four seconds of frames in a few hundredths.
    fake.set_controllers([frames.controller('cell-1'), frames.edge_controller('edge-1')])
    slow = [(2.0 * index, frame) for index, (_, frame) in enumerate(timed)]
    started = time.monotonic()
    player = fake.play(slow, speed=100.0)
    for _, frame in slow:
        assert next_frame(stream) == frame
    assert time.monotonic() - started < 1.0
    player.join(1.0)
    assert not player.is_alive()
    with pytest.raises(ValueError):
        fake.play(timed, speed=0)


def test_playing_ends_when_the_fake_stops(fake):
    player = fake.play([(0.0, frames.removed_frame('edge-1')),
                        (30.0, frames.removed_frame('cell-1'))])
    assert wait_until(lambda: len(fake.controllers) == 1)
    fake.stop()
    player.join(1.0)
    assert not player.is_alive()
    assert [entry['id'] for entry in fake.controllers] == ['cell-1']


def test_the_fake_starts_empty_and_works_as_a_context_manager():
    fake = FakeDaemon(token=OTHER_TOKEN, mode='replay', lease_ms=3000, target_hz=1.0)
    assert fake.port == 0 and fake.controllers == []
    fake.now = lambda: frames.iso(1)
    with fake as started:
        assert started is fake and fake.port != 0 and fake.started_at == frames.iso(1)
        assert ask(fake, 'GET', '/v0/controllers', token=OTHER_TOKEN)[2]['controllers'] == []
        assert ask(fake, 'GET', '/v0/controllers', token=TOKEN)[0] == 401
        assert ask(fake, 'GET', '/v0/health', token=None)[2]['mode'] == 'replay'
        stream = consume(fake, token=OTHER_TOKEN)
        try:
            assert next_frame(stream) == frames.snapshot_frame([], observed_at=frames.iso(1))
        finally:
            stream.shutdown()
        fake.push(frames.delta_frame('cell-1', frames.groups_of(frames.controller('cell-1'))))
        body = ask(fake, 'POST', '/v0/controllers/cell-1/motion/watch', token=OTHER_TOKEN,
                   body={'leaseId': 'new'})[2]
        assert body == {'leaseMs': 3000, 'targetHz': 1.0, 'leaseId': 'lease-1'}
    with pytest.raises(ConnectionRefusedError):
        socket.create_connection(('127.0.0.1', fake.port), timeout=1.0)


def test_times_are_written_as_the_daemon_writes_them():
    assert re.fullmatch(r'\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}\.\d{3}Z', fake_daemon.utc_now())
    with FakeDaemon() as fake:
        assert re.fullmatch(r'20\d\d-.*Z', fake.started_at)
        assert ask(fake, 'GET', '/v0/controllers')[2]['observedAt'] >= fake.started_at


def test_frames_are_written_and_read_as_the_protocol_says():
    for size in (0, 1, 125, 126, 65535, 65536, 70000):
        payload = bytes(index % 251 for index in range(size))
        wire = fake_daemon._frame(0x1, payload)
        # What the fake writes, the library reads.
        expected_head = 2 if size < 126 else 4 if size < 65536 else 10
        assert len(wire) == expected_head + size and wire[0] == 0x81
        buffer = bytearray(wire)
        assert fake_daemon._take_frames(buffer) == [(0x1, payload)] and not buffer
        # What the library writes (masked, as a client must), the fake reads.
        masked = websocket.ABNF.create_frame(payload, websocket.ABNF.OPCODE_BINARY).format()
        buffer = bytearray(masked + masked[:1])
        assert fake_daemon._take_frames(buffer) == [(0x2, payload)]
        assert bytes(buffer) == masked[:1]
    # Half a frame is left where it is.
    close = websocket.ABNF.create_frame(struct.pack('!H', 1000), websocket.ABNF.OPCODE_CLOSE)
    wire = close.format()
    for cut in range(len(wire)):
        buffer = bytearray(wire[:cut])
        assert fake_daemon._take_frames(buffer) == [] and bytes(buffer) == wire[:cut]
    buffer = bytearray(wire + fake_daemon._frame(0x9, b'p'))
    assert fake_daemon._take_frames(buffer) == [(0x8, struct.pack('!H', 1000)), (0x9, b'p')]


def test_wait_until_reports_whether_it_became_true():
    assert wait_until(lambda: True, timeout=0)
    started = time.monotonic()
    assert not wait_until(lambda: False, timeout=0.05)
    assert 0.04 <= time.monotonic() - started < 0.5
    calls = []
    assert wait_until(lambda: calls.append(1) or len(calls) >= 3, timeout=1.0)


def test_error_bodies_of_every_kind_fit_the_schema(fake):
    for code in frames.ERROR_CODES:
        fits('ErrorBody', frames.error_answer(code)[2])
    fits('ErrorBody', fake_daemon.error_body('NO_ROUTE', 'no route: GET /x'))
    assert fake_daemon.error_body('A', 'b') == {'error': 'b', 'code': 'A'}


def test_this_file_validated_every_kind_of_answer_the_bridge_reads(fake, streams):
    # The definitions the bridge reads. Every test above went through the same ask()
    # and next_frame(), so each of their answers was validated the same way.
    ask(fake, 'GET', '/v0/health', token=None)
    ask(fake, 'GET', '/v0/controllers')
    ask(fake, 'GET', '/v0/controllers/cell-1')
    ask(fake, 'POST', '/v0/controllers/cell-1/motion/watch', body={'leaseId': 'new'})
    ask(fake, 'POST', '/v0/controllers/cell-1/io/watch', body={'signals': ['do_a']})
    ask(fake, 'GET', '/v0/controllers', token=None)
    stream = consume(fake)
    streams.append(stream)
    next_frame(stream)
    assert validated >= {'HealthAnswer', 'ControllerList', 'ControllerSnapshot', 'MotionWatch',
                         'IoWatch', 'ErrorBody', 'StreamFrame'}

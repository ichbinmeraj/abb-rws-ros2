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

import copy
import dataclasses

from abb_rws_bridge import health
from abb_rws_bridge import joint_state
from abb_rws_bridge import names
from abb_rws_bridge.health import ERROR, OK, STALE, WARN
import frames
import pytest

NODE = 'abb_rws_bridge'
NAMER = names.JointNamer('{unit}_{joint}', {})
DEFAULT = object()
REASONS = (
    'unreachable', 'host-refused', 'not-a-daemon', 'token-file', 'token-refused',
    'replay-refused', 'refresh-failed', 'mode-changed', 'origin-refused', 'handshake-failed',
    'protocol', 'silent', 'daemon-stopping', 'replay-restart', 'too-many-consumers', 'policy',
    'closed', 'stream-stalled',
)


def daemon_view(**changes):
    view = health.DaemonView(
        node_name=NODE, host_port='127.0.0.1:7810', connected=True, reason='', detail='',
        retry_in_s=None, mode='live', allow_replay=False, replay=None,
        daemon_version='0.1.0', daemon_schema='0.12', messages_schema='0.12',
        definitions_sha256='0123456789abcdef' * 4,
        connected_for_s=12.5, reconnects=0, frames=40, contact_age_s=0.1, stale_after_s=2.0,
        clock_offset_ms=3.0, stamp_source='daemon', stamps_held_back=0,
        stamps_held_back_recently=False)
    return dataclasses.replace(view, **changes)


def controller_view(controller=DEFAULT, **changes):
    # controller: a snapshot, or None for a controller the daemon does not serve.
    if controller is DEFAULT:
        controller = frames.controller()
    controller_id = 'cell-1'
    joints = None
    if controller is not None:
        controller_id = controller['id']
        joints = joint_state.build(controller_id, 'cell_1', controller.get('motion'), NAMER)
    view = health.ControllerView(
        node_name=NODE, controller_id=controller_id, controller=controller,
        daemon_live=True, name_problem=None, conversion_problem=None, joints=joints,
        motion_age_s=0.1, stale_after_s=2.0, leases={'motion': (True, '')}, io_accepted=(),
        io_rejected=(), io_unpublishable=())
    return dataclasses.replace(view, **changes)


def values_of(status):
    keys = [key for key, _ in status.values]
    assert len(set(keys)) == len(keys)
    assert all(isinstance(key, str) and isinstance(value, str) for key, value in status.values)
    return dict(status.values)


def test_the_levels_are_those_of_diagnostic_status():
    assert (health.OK, health.WARN, health.ERROR, health.STALE) == (0, 1, 2, 3)


def test_the_contract_of_the_views():
    assert [field.name for field in dataclasses.fields(health.Status)] == [
        'name', 'hardware_id', 'level', 'message', 'values']
    assert [field.name for field in dataclasses.fields(health.DaemonView)] == [
        'node_name', 'host_port', 'connected', 'reason', 'detail', 'retry_in_s', 'mode',
        'allow_replay', 'replay', 'daemon_version', 'daemon_schema', 'messages_schema',
        'definitions_sha256', 'connected_for_s', 'reconnects', 'frames', 'contact_age_s',
        'stale_after_s', 'clock_offset_ms', 'stamp_source', 'stamps_held_back',
        'stamps_held_back_recently', 'stream_behind_s']
    assert [field.name for field in dataclasses.fields(health.ControllerView)] == [
        'node_name', 'controller_id', 'controller', 'daemon_live', 'name_problem',
        'conversion_problem', 'joints', 'motion_age_s', 'stale_after_s', 'leases',
        'io_accepted', 'io_rejected', 'io_unpublishable', 'watch_motion']
    # The one field with a default: a view made without it is of a bridge that watches.
    assert health.ControllerView.__dataclass_fields__['watch_motion'].default is True
    with pytest.raises(dataclasses.FrozenInstanceError):
        health.daemon_status(daemon_view()).level = OK


# Current or not.

def test_a_healthy_controller_is_current():
    assert health.controller_current(frames.controller()) == (True, '')
    polled = frames.controller(link=frames.link_group(quality='polling'))
    assert health.controller_current(polled) == (True, '')
    unverified = frames.controller(link=frames.link_group(identity_status='unverified'))
    assert health.controller_current(unverified) == (True, '')
    assert health.controller_current(frames.EDGE_CONTROLLER) == (True, '')


@pytest.mark.parametrize('quality', ['reconnecting', 'stale', 'disconnected'])
def test_a_link_that_is_neither_live_nor_polling_is_not_current(quality):
    link = frames.link_group(connected=True, quality=quality, quality_reason='no answer')
    current, why = health.controller_current(frames.controller(link=link))
    assert current is False
    assert quality in why and 'no answer' in why


def test_a_link_that_is_not_connected_is_not_current():
    link = frames.link_group(connected=False, quality='disconnected',
                             quality_reason='connection refused')
    current, why = health.controller_current(frames.controller(link=link))
    assert current is False
    assert 'not connected' in why and 'connection refused' in why
    # connected false wins even over a quality that reads well
    odd = frames.link_group(connected=False, quality='live')
    assert health.controller_current(frames.controller(link=odd))[0] is False


def test_an_identity_mismatch_is_not_current():
    link = frames.link_group(identity_status='mismatch', actual='another-system')
    current, why = health.controller_current(frames.controller(link=link))
    assert current is False
    assert 'identity mismatch' in why


@pytest.mark.parametrize('controller', [None, {}, {'link': None}, {'link': {}}, 'cell-1', 5])
def test_something_that_is_not_a_served_controller_is_not_current(controller):
    current, why = health.controller_current(controller)
    assert current is False
    assert why
    assert health.joints_current(controller, 0.0, 2.0)[0] is False
    assert health.signal_current(controller, frames.signal(), True)[0] is False


def test_a_link_without_an_identity_check_is_not_current():
    controller = frames.controller()
    del controller['link']['identityCheck']
    assert health.controller_current(controller)[0] is False


def test_joints_are_current_while_confirmed():
    controller = frames.controller()
    assert health.joints_current(controller, 0.0, 2.0) == (True, '')
    assert health.joints_current(controller, 1.999, 2.0) == (True, '')
    current, why = health.joints_current(controller, 2.0, 2.0)
    assert current is False
    assert 'not confirmed for 2.0 s' in why
    assert health.joints_current(controller, 30.0, 2.0)[0] is False
    assert health.joints_current(controller, float('nan'), 2.0)[0] is False
    assert health.joints_current(controller, None, 2.0) == (False, 'joints not confirmed yet')


def test_joints_are_not_current_without_a_watch_or_a_read():
    not_watched = frames.controller(motion=frames.motion_group(watching=False))
    assert health.joints_current(not_watched, 0.0, 2.0) == (False, 'joints not watched')
    kept = frames.controller(motion=frames.motion_group(
        [frames.motion_unit()], watching=False, as_of=frames.T0))
    assert health.joints_current(kept, 0.0, 2.0) == (False, 'joints not watched')
    unread = frames.controller(motion=frames.motion_group(as_of=None))
    assert health.joints_current(unread, 0.0, 2.0) == (False, 'joints not read yet')


def test_joints_are_not_current_when_the_controller_is_not():
    link = frames.link_group(connected=False, quality='reconnecting')
    current, why = health.joints_current(frames.controller(link=link), 0.0, 2.0)
    assert current is False
    assert 'not connected' in why
    mismatch = frames.controller(link=frames.link_group(identity_status='mismatch'))
    assert health.joints_current(mismatch, 0.0, 2.0)[0] is False


def test_a_signal_is_current_while_the_daemon_serves_it():
    signal = frames.signal('do_gripper', 'DO', '1')
    controller = frames.controller(io=frames.io_group([signal]))
    assert health.signal_current(controller, signal, True) == (True, '')
    assert health.signal_current(controller, 'do_gripper', True) == (True, '')
    polled = frames.signal('ao_speed', 'AO', '1.5', via='poll')
    controller = frames.controller(io=frames.io_group([polled]))
    assert health.signal_current(controller, polled, True) == (True, '')


def test_there_is_no_age_limit_on_the_time_of_a_signal():
    # A subscribed signal's asOf moves only when the signal changes.
    old = frames.signal('do_gripper', 'DO', '1', as_of='2020-01-01T00:00:00.000Z')
    controller = frames.controller(io=frames.io_group([old], as_of=frames.iso(3600)))
    assert health.signal_current(controller, old, True) == (True, '')
    assert 'age_s' not in health.signal_current.__code__.co_varnames


def test_a_signal_is_not_current_for_each_reason():
    signal = frames.signal('do_gripper', 'DO', '1')
    watched = frames.controller(io=frames.io_group([signal]))
    assert health.signal_current(watched, signal, False) == (
        False, 'not granted to the lease of the bridge')
    assert health.signal_current(watched, signal, 1)[0] is False

    lapsed = frames.controller(io=frames.io_group([signal], watching=False))
    assert health.signal_current(lapsed, signal, True) == (False, 'I/O not watched')

    unserved = frames.signal('do_gripper', 'DO', '1', via=None)
    controller = frames.controller(io=frames.io_group([unserved]))
    assert health.signal_current(controller, unserved, True) == (
        False, 'not served by the watch yet')

    failed = frames.signal('do_missing', None, None, via='poll', error=frames.group_error())
    controller = frames.controller(io=frames.io_group([failed]))
    current, why = health.signal_current(controller, failed, True)
    assert current is False
    assert 'RESOURCE_NOT_FOUND' in why

    assert health.signal_current(watched, 'do_other', True) == (
        False, 'not in the watch list of the daemon')
    assert health.signal_current(watched, None, True)[0] is False

    down = frames.controller(link=frames.link_group(connected=False, quality='stale'),
                             io=frames.io_group([signal]))
    current, why = health.signal_current(down, signal, True)
    assert current is False
    assert 'not connected' in why


def test_the_signals_of_the_edge_controller():
    edge = frames.edge_controller()
    current = {signal['name']: health.signal_current(edge, signal, True)[0]
               for signal in edge['io']['signals']}
    assert current == {
        'di_ready': True, 'do_gripper': True, 'ai_pressure': True, 'ao_speed': True,
        'gi_code': True, 'Local/DRV_1/go_program': True, 'do_missing': False}


def test_judging_changes_nothing():
    edge = frames.edge_controller()
    before = copy.deepcopy(edge)
    health.controller_current(edge)
    health.joints_current(edge, 0.1, 2.0)
    for signal in edge['io']['signals']:
        health.signal_current(edge, signal, True)
    health.controller_status(controller_view(edge))
    assert edge == before


# Schema and clock.

@pytest.mark.parametrize('daemon_schema, level, said', [
    ('0.12', OK, '0.12'),
    (None, OK, 'not known yet'),
    ('0.13', WARN, 'newer'),
    ('0.99', WARN, 'newer'),
    ('0.11', ERROR, 'schema too old'),
    ('0.0', ERROR, 'schema too old'),
    ('1.0', ERROR, 'major'),
    ('1.12', ERROR, 'major'),
    ('2.5', ERROR, 'major'),
    ('', ERROR, 'not a version'),
    ('0.12.1', ERROR, 'not a version'),
    ('v0.12', ERROR, 'not a version'),
    ('twelve', ERROR, 'not a version'),
    ('0.x', ERROR, 'not a version'),
    (12, ERROR, 'not a version'),
])
def test_schema_verdict(daemon_schema, level, said):
    verdict = health.schema_verdict(daemon_schema, '0.12')
    assert verdict[0] == level
    assert said in verdict[1]


def test_schema_minors_are_numbers_not_text():
    assert health.schema_verdict('0.9', '0.12')[0] == ERROR
    assert health.schema_verdict('0.100', '0.12')[0] == WARN
    assert health.schema_verdict('0.12', '0.9')[0] == WARN
    assert health.schema_verdict('0.12', 'unknown')[0] == ERROR


@pytest.mark.parametrize('stale_after_s, limit', [
    (2.0, 0.5), (0.5, 0.125), (1.0, 0.25), (4.0, 1.0), (8.0, 1.0), (60.0, 1.0),
])
def test_clock_offset_limit(stale_after_s, limit):
    assert health.clock_offset_limit_s(stale_after_s) == limit


# The daemon status.

def test_daemon_ok():
    status = health.daemon_status(daemon_view())
    assert status.name == 'abb_rws_bridge: daemon'
    assert status.hardware_id == 'abb-rws-daemon'
    assert status.level == OK
    assert status.message == 'live'
    values = values_of(status)
    assert values['daemon'] == '127.0.0.1:7810'
    assert values['connected'] == 'true'
    assert values['daemon version'] == '0.1.0'
    assert values['mode'] == 'live'
    assert values['daemon schema'] == '0.12'
    assert values['messages schema'] == '0.12'
    assert values['definitions sha256'] == '0123456789ab'
    assert values['connected for (s)'] == '12.500'
    assert values['reconnects'] == '0'
    assert values['frames received'] == '40'
    assert values['last contact age (s)'] == '0.100'
    assert values['clock offset (ms)'] == '3'
    assert values['stamp source'] == 'daemon'
    assert values['stamps held back'] == '0'
    assert values['stream behind for (s)'] == 'not behind'
    assert not any(key.startswith('replay') for key in values)


def test_the_status_name_carries_the_real_node_name():
    assert health.daemon_status(daemon_view(node_name='cell_bridge')).name == (
        'cell_bridge: daemon')
    view = controller_view(node_name='cell_bridge')
    assert health.controller_status(view).name == 'cell_bridge: controller cell-1'


def test_a_controller_called_daemon_does_not_take_the_name_of_the_daemon_status():
    controller = frames.controller('daemon')
    status = health.controller_status(controller_view(controller))
    assert status.name == 'abb_rws_bridge: controller daemon'
    assert status.hardware_id == 'daemon'
    assert status.name != health.daemon_status(daemon_view()).name


@pytest.mark.parametrize('reason', REASONS)
def test_daemon_not_connected_is_an_error_with_the_reason(reason):
    view = daemon_view(connected=False, reason=reason, detail='status 503', retry_in_s=2.0,
                       connected_for_s=None, contact_age_s=None, mode=None,
                       host_port='host.docker.internal:7810')
    status = health.daemon_status(view)
    assert status.level == ERROR
    assert status.message.startswith(f'{reason}: ')
    assert '(status 503)' in status.message
    assert status.message.endswith('; next try in 2.0 s')
    values = values_of(status)
    assert values['connected'] == 'false'
    assert values['reason'] == reason
    assert values['detail'] == 'status 503'
    assert values['next try in (s)'] == '2.000'
    assert values['connected for (s)'] == 'unknown'
    assert values['mode'] == 'unknown'


@pytest.mark.parametrize('reason, said', [
    ('unreachable', 'start abb-rws-daemon'),
    ('host-refused',
     'start abb-rws-daemon with --allowed-hosts host.docker.internal, or use 127.0.0.1'),
    ('not-a-daemon', 'is not abb-rws-daemon'),
    ('token-file', 'token_file'),
    ('token-refused', 'something else holds the port'),
    ('replay-refused', 'set allow_replay to publish a recording under replay/'),
    ('refresh-failed', 'stopped answering'),
    ('mode-changed', 'live and replay'),
    ('protocol', 'no snapshot first, or a frame that cannot be read'),
    ('stream-stalled', 'stopped delivering what the controller list of the daemon shows'),
    ('host-refused', 'one that does not know it ignores it without a word'),
])
def test_what_to_do_about_each_reason(reason, said):
    view = daemon_view(connected=False, reason=reason, host_port='host.docker.internal:7810')
    assert said in health.daemon_status(view).message


def test_host_refused_names_the_host_of_the_url():
    for host_port, host in (('my-pc.example:7830', 'my-pc.example'),
                            ('host.containers.internal:7810', 'host.containers.internal')):
        view = daemon_view(connected=False, reason='host-refused', host_port=host_port)
        assert f'--allowed-hosts {host}, or use 127.0.0.1' in health.daemon_status(view).message
    for host_port in ('127.0.0.1:7810', 'localhost:7810', '[::1]:7810'):
        view = daemon_view(connected=False, reason='host-refused', host_port=host_port)
        message = health.daemon_status(view).message
        assert message.startswith('host-refused: ')
        assert '--allowed-hosts' not in message


@pytest.mark.parametrize('host_port', [
    '192.0.2.5:7810', '[2001:db8::1]:7810', '2130706433:7810', '0x7f.1:7810',
    '0x7f000001:7810', 'cell.example.0x:7810', 'robot.42:7810',
])
def test_host_refused_for_an_address_does_not_propose_an_option_the_daemon_refuses(host_port):
    # The daemon takes no address, and no name that reads as one, as an extra host: the
    # hint names the rule instead of an option value that would stop the daemon.
    view = daemon_view(connected=False, reason='host-refused', host_port=host_port)
    message = health.daemon_status(view).message
    assert message.startswith('host-refused: ')
    assert 'takes DNS host names only' in message
    assert 'forwarder' in message and 'use 127.0.0.1' in message
    assert '--allowed-hosts ' + host_port.split(']')[0].lstrip('[').rsplit(':', 1)[0] not in (
        message)


@pytest.mark.parametrize('host, address', [
    ('192.0.2.5', True), ('2001:db8::1', True), ('::1', True), ('2130706433', True),
    ('0x7f.1', True), ('0x7f000001', True), ('cell.0x', True), ('robot.42', True),
    ('robot.42.', True), ('host.docker.internal', False), ('my-pc.example', False),
    ('x0', False), ('cell-1', False), ('', False), (None, False),
])
def test_reads_as_address(host, address):
    assert health.reads_as_address(host) is address


def test_daemon_not_connected_without_a_known_reason():
    status = health.daemon_status(daemon_view(connected=False, reason='', retry_in_s=None))
    assert status.level == ERROR
    assert status.message.startswith('not connected')
    assert values_of(status)['reason'] == 'none'
    status = health.daemon_status(daemon_view(connected=False, reason='something-new'))
    assert status.level == ERROR
    assert status.message == 'not connected: something-new'


def test_the_detail_is_kept_short_and_printable():
    view = daemon_view(connected=False, reason='unreachable',
                       detail='first line\nsecond\x00 line \u00e9 ' + 'x' * 500)
    status = health.daemon_status(view)
    assert all(' ' <= character <= '~' for character in status.message)
    assert len(values_of(status)['detail']) <= 200
    assert 'x' * 201 not in status.message


def test_daemon_stale_when_it_stops_answering():
    status = health.daemon_status(daemon_view(contact_age_s=2.0))
    assert status.level == STALE
    assert status.message == 'no answer from the daemon for 2.0 s'
    assert health.daemon_status(daemon_view(contact_age_s=1.99)).level == OK
    late = health.daemon_status(daemon_view(contact_age_s=12.34))
    assert late.level == STALE
    assert late.message == 'no answer from the daemon for 12.3 s'
    waiting = health.daemon_status(daemon_view(contact_age_s=None))
    assert waiting.level == STALE
    assert 'first snapshot' in waiting.message


def test_daemon_stale_when_the_stream_stalled():
    stalled = health.daemon_status(daemon_view(stream_behind_s=2.0))
    assert stalled.level == STALE
    assert stalled.message == (
        'the stream has not delivered what the controller list shows for 2.0 s; reconnecting')
    assert values_of(stalled)['stream behind for (s)'] == '2.0'
    assert health.daemon_status(daemon_view(stream_behind_s=1.99)).level == OK
    assert values_of(health.daemon_status(daemon_view(stream_behind_s=0.4)))[
        'stream behind for (s)'] == '0.4'
    assert health.daemon_status(daemon_view(stream_behind_s=0.5, stale_after_s=0.5)).level == (
        STALE)
    # No answer at all comes first; and a daemon that is not connected owes nothing.
    both = health.daemon_status(daemon_view(stream_behind_s=5.0, contact_age_s=3.0))
    assert both.message.startswith('no answer from the daemon')
    gone = health.daemon_status(daemon_view(connected=False, reason='stream-stalled',
                                            stream_behind_s=None))
    assert gone.level == ERROR and gone.message.startswith('stream-stalled: ')


@pytest.mark.parametrize('behind_s, stale_after_s, stalled', [
    (None, 2.0, False), (0.0, 2.0, False), (1.99, 2.0, False), (2.0, 2.0, True),
    (60.0, 2.0, True), (0.49, 0.5, False), (0.5, 0.5, True),
])
def test_stream_stalled(behind_s, stale_after_s, stalled):
    assert health.stream_stalled(behind_s, stale_after_s) is stalled


def test_daemon_schema_levels():
    newer = health.daemon_status(daemon_view(daemon_schema='0.13'))
    assert newer.level == WARN
    assert 'newer' in newer.message
    older = health.daemon_status(daemon_view(daemon_schema='0.11'))
    assert older.level == ERROR
    assert older.message.startswith('schema too old')
    # too old beats everything else that is wrong while connected
    assert health.daemon_status(
        daemon_view(daemon_schema='0.11', contact_age_s=99.0)).level == ERROR


def test_daemon_warns_about_stamps_held_back_only_while_the_count_grows():
    growing = health.daemon_status(
        daemon_view(stamps_held_back=3, stamps_held_back_recently=True))
    assert growing.level == WARN
    assert growing.message.startswith('3 stamps held back')
    assert values_of(growing)['stamps held back'] == '3'
    one = health.daemon_status(daemon_view(stamps_held_back=1, stamps_held_back_recently=True))
    assert one.message.startswith('1 stamp held back')
    settled = health.daemon_status(
        daemon_view(stamps_held_back=3, stamps_held_back_recently=False))
    assert settled.level == OK
    assert values_of(settled)['stamps held back'] == '3'


@pytest.mark.parametrize('offset_ms, stale_after_s, level', [
    (0.0, 2.0, OK), (500.0, 2.0, OK), (-500.0, 2.0, OK), (501.0, 2.0, WARN),
    (-501.0, 2.0, WARN), (126.0, 0.5, WARN), (125.0, 0.5, OK), (999.0, 60.0, OK),
    (1001.0, 60.0, WARN), (-86400000.0, 2.0, WARN), (None, 2.0, OK),
])
def test_daemon_warns_about_the_clock_offset_when_stamps_are_the_daemons(
        offset_ms, stale_after_s, level):
    view = daemon_view(clock_offset_ms=offset_ms, stale_after_s=stale_after_s,
                       stamp_source='daemon')
    status = health.daemon_status(view)
    assert status.level == level
    if level == WARN:
        assert 'clock' in status.message
        assert f'{abs(offset_ms):.0f} ms' in status.message
    by_bridge = health.daemon_status(dataclasses.replace(view, stamp_source='bridge'))
    assert by_bridge.level == OK
    assert values_of(by_bridge)['stamp source'] == 'bridge'


def test_daemon_replay_published_is_a_warning_that_starts_with_replay():
    replay = frames.replay_health(position_ms=1500, speed=2.0, time_mode='recorded',
                                  recorded_at='2026-01-01T08:00:00.000Z')
    status = health.daemon_status(daemon_view(mode='replay', allow_replay=True, replay=replay))
    assert status.level == WARN
    assert status.message == 'REPLAY: recorded 2026-01-01T08:00:00.000Z'
    values = values_of(status)
    assert values['mode'] == 'replay'
    assert values['replay state'] == 'playing'
    assert values['replay position (ms)'] == '1500'
    assert values['replay speed'] == '2.0'
    assert values['replay time mode'] == 'recorded'
    assert values['recorded at'] == '2026-01-01T08:00:00.000Z'

    more = health.daemon_status(daemon_view(
        mode='replay', allow_replay=True, replay=replay, daemon_schema='0.13',
        clock_offset_ms=-9000000.0))
    assert more.level == WARN
    assert more.message.startswith('REPLAY: recorded 2026-01-01T08:00:00.000Z; ')
    assert 'newer' in more.message and 'clock' in more.message

    unknown = health.daemon_status(daemon_view(mode='replay', allow_replay=True, replay=None))
    assert unknown.level == WARN
    assert unknown.message.startswith('REPLAY: recorded ')


def test_daemon_replay_without_leave_is_an_error():
    refused = health.daemon_status(daemon_view(
        connected=False, reason='replay-refused', mode='replay',
        replay=frames.replay_health()))
    assert refused.level == ERROR
    assert 'set allow_replay' in refused.message
    assert values_of(refused)['replay state'] == 'playing'
    # Never OK or WARN while a recording is served and not allowed.
    odd = health.daemon_status(daemon_view(mode='replay', allow_replay=False,
                                           replay=frames.replay_health()))
    assert odd.level == ERROR
    assert odd.message.startswith('replay-refused: ')


@pytest.mark.parametrize('mode', [None, '', 'simulation', 'LIVE'])
def test_daemon_with_a_mode_the_bridge_does_not_know_is_an_error(mode):
    status = health.daemon_status(daemon_view(mode=mode))
    assert status.level == ERROR
    assert 'mode' in status.message


def test_daemon_stale_in_replay_does_not_claim_a_replay_is_published():
    status = health.daemon_status(daemon_view(
        mode='replay', allow_replay=True, replay=frames.replay_health(), contact_age_s=5.0))
    assert status.level == STALE
    assert not status.message.startswith('REPLAY')


# The controller status.

def test_controller_ok():
    status = health.controller_status(controller_view())
    assert status.name == 'abb_rws_bridge: controller cell-1'
    assert status.hardware_id == 'cell-1'
    assert status.level == OK
    assert status.message == 'current'
    values = values_of(status)
    assert values == {
        'link quality': 'live',
        'link quality reason': 'events streaming',
        'identity check': 'confirmed',
        'controller state': 'motoron',
        'operating mode': 'AUTO',
        'motion watching': 'true',
        'motion asOf': frames.T0,
        'motion asOf age (s)': '0.100',
        'target rate (Hz)': '8.0',
        'achieved rate (Hz)': '7.9',
        'joints published': '6',
        'joints left out': 'none',
        'motion lease': 'granted',
        'io lease': 'not asked',
        'signals accepted': 'none',
        'signals refused': 'none',
        'signals not publishable': 'none',
    }


def test_controller_polling_is_a_warning():
    polled = frames.controller(link=frames.link_group(
        quality='polling', quality_reason='subscriptions refused'))
    status = health.controller_status(controller_view(polled))
    assert status.level == WARN
    assert status.message == 'link polling: subscriptions refused'


def test_controller_joints_left_out_is_a_warning():
    status = health.controller_status(controller_view(frames.edge_controller('cell-1')))
    assert status.level == WARN
    assert status.message == (
        'link polling: polled; 3 joints left out of joint_states: ROB_1/J3: no-value, '
        'ROB_1/J4: unit-unknown, STN_1/J1: unit-error')
    values = values_of(status)
    assert values['joints published'] == '5'
    assert values['joints left out'] == (
        'ROB_1/J3: no-value, ROB_1/J4: unit-unknown, STN_1/J1: unit-error')
    assert values['controller state'] == 'unknown'
    assert values['operating mode'] == 'MANR'
    assert values['achieved rate (Hz)'] == 'unknown'
    assert values['identity check'] == 'unverified'


def test_controller_with_no_joint_left_warns_and_publishes_none():
    unit = frames.motion_unit('ROB_1', [frames.joint('J1', None), frames.joint('J2', 1.0, None)])
    controller = frames.controller(motion=frames.motion_group([unit]))
    view = controller_view(controller)
    assert view.joints.problem == joint_state.NO_JOINT
    status = health.controller_status(view)
    assert status.level == WARN
    assert status.message.startswith('2 joints left out of joint_states: ')
    assert values_of(status)['joints published'] == '0'

    no_units = frames.controller(motion=frames.motion_group([], as_of=frames.T0))
    status = health.controller_status(controller_view(no_units))
    assert status.level == OK
    assert values_of(status)['joints published'] == '0'


def test_controller_signals_refused_or_not_publishable_is_a_warning():
    view = controller_view(
        leases={'motion': (True, ''), 'io': (True, '')},
        io_accepted=('do_gripper', 'ai_pressure'),
        io_rejected=(('do_nothing', 'not in the signal table of the controller'),),
        io_unpublishable=(('ai_pressure', 'the value "x" of an AI is not a finite number'),))
    status = health.controller_status(view)
    assert status.level == WARN
    assert status.message == (
        '1 signal refused by the daemon: do_nothing: not in the signal table of the '
        'controller; 1 signal not publishable: ai_pressure: the value "x" of an AI is not a '
        'finite number')
    values = values_of(status)
    assert values['signals accepted'] == 'do_gripper, ai_pressure'
    assert values['signals refused'] == 'do_nothing: not in the signal table of the controller'
    assert values['signals not publishable'] == (
        'ai_pressure: the value "x" of an AI is not a finite number')
    assert values['io lease'] == 'granted'

    only_refused = health.controller_status(controller_view(io_rejected=(('a', 'r'),)))
    assert only_refused.level == WARN
    only_unpublishable = health.controller_status(
        controller_view(io_unpublishable=(('a', 'no value yet'), ('b', 'no value yet'))))
    assert only_unpublishable.level == WARN
    assert only_unpublishable.message.startswith('2 signals not publishable: ')


@pytest.mark.parametrize('kind', ['motion', 'io'])
def test_controller_lease_refused_is_a_warning(kind):
    view = controller_view(leases={'motion': (True, ''), kind: (False, 'LEASE_LIMIT')})
    status = health.controller_status(view)
    assert status.level == WARN
    assert status.message == f'{kind} lease refused: LEASE_LIMIT'
    assert values_of(status)[f'{kind} lease'] == 'refused: LEASE_LIMIT'
    no_code = health.controller_status(controller_view(leases={kind: (False, '')}))
    assert no_code.message == f'{kind} lease refused: unknown'


def test_controller_link_not_connected_is_an_error():
    link = frames.link_group(connected=False, quality='reconnecting',
                             quality_reason='connection refused',
                             reconnect={'failures': 2, 'nextAttemptAt': frames.iso(5)})
    status = health.controller_status(controller_view(frames.controller(link=link)))
    assert status.level == ERROR
    assert status.message == 'link not connected (reconnecting: connection refused)'
    assert values_of(status)['link quality'] == 'reconnecting'
    assert values_of(status)['joints published'] == '0'


def test_controller_identity_mismatch_is_an_error():
    link = frames.link_group(identity_status='mismatch', actual='another')
    status = health.controller_status(controller_view(frames.controller(link=link)))
    assert status.level == ERROR
    assert status.message.startswith('identity mismatch')
    assert values_of(status)['identity check'] == 'mismatch'
    assert values_of(status)['joints published'] == '0'


def test_controller_not_served_by_the_daemon_is_an_error():
    view = controller_view(None, controller_id='cell-9', motion_age_s=None, leases={})
    assert view.controller is None and view.joints is None
    status = health.controller_status(view)
    assert status.name == 'abb_rws_bridge: controller cell-9'
    assert status.hardware_id == 'cell-9'
    assert status.level == ERROR
    assert status.message == 'not served by the daemon'
    values = values_of(status)
    assert values['link quality'] == 'unknown'
    assert values['motion asOf age (s)'] == 'unknown'
    assert values['joints published'] == '0'
    assert values['motion lease'] == 'not asked'


def test_controller_name_collision_is_an_error():
    _, problems = names.controller_tokens(['cell-1', 'cell_1'])
    view = controller_view(None, controller_id='cell_1', name_problem=problems['cell_1'])
    status = health.controller_status(view)
    assert status.level == ERROR
    assert status.message == (
        'not bridged: its topic name "cell_1" is already taken by controller "cell-1"')
    # Also while the daemon is not live: the name is wrong whatever the daemon does.
    assert health.controller_status(
        dataclasses.replace(view, daemon_live=False)).level == ERROR


def test_controller_snapshot_that_does_not_fit_is_an_error():
    view = controller_view(conversion_problem='io.signals: 30 items, at most 24 fit')
    status = health.controller_status(view)
    assert status.level == ERROR
    assert status.message == (
        'the snapshot does not fit the messages: io.signals: 30 items, at most 24 fit')


def test_controller_signal_that_changed_its_type_is_an_error():
    why = f'{health.TYPE_CHANGED}: published as bool, now reported as float64'
    view = controller_view(io_unpublishable=(('do_gripper', why), ('ai_x', 'no value yet')))
    status = health.controller_status(view)
    assert status.level == ERROR
    assert status.message == (
        f'1 signal changed type and stopped: do_gripper: {why}; '
        '1 signal not publishable: ai_x: no value yet')
    assert values_of(status)['signals not publishable'] == (
        f'do_gripper: {why}; ai_x: no value yet')
    assert health.TYPE_CHANGED == 'type changed'
    capital = controller_view(io_unpublishable=(('do_gripper', 'Type changed to AO'),))
    assert health.controller_status(capital).level == ERROR


def test_controller_duplicate_joint_names_is_an_error():
    controller = frames.controller()
    joints = joint_state.build('cell-1', '', controller['motion'], names.JointNamer('j', {}))
    status = health.controller_status(controller_view(controller, joints=joints))
    assert status.level == ERROR
    assert status.message.startswith('joint_states stopped: the joint name "j" is given to ')
    assert values_of(status)['joints published'] == '0'


def test_controller_joints_not_confirmed_is_stale():
    status = health.controller_status(controller_view(motion_age_s=2.0))
    assert status.level == STALE
    assert status.message == 'joints not confirmed for 2.0 s'
    assert values_of(status)['joints published'] == '0'
    assert values_of(status)['motion asOf age (s)'] == '2.000'
    never = health.controller_status(controller_view(motion_age_s=None))
    assert never.level == STALE
    assert never.message == 'joints not confirmed yet'


def test_controller_joints_not_watched_is_stale():
    controller = frames.controller(motion=frames.motion_group(watching=False))
    view = controller_view(controller, leases={})
    assert view.joints is None
    status = health.controller_status(view)
    assert status.level == STALE
    assert status.message == 'joints not watched'
    values = values_of(status)
    assert values['motion watching'] == 'false'
    assert values['motion asOf'] == 'unknown'
    assert values['target rate (Hz)'] == 'unknown'


def test_controller_joints_not_watched_is_what_was_asked_for_without_watch_motion():
    controller = frames.controller(motion=frames.motion_group(watching=False))
    view = controller_view(controller, leases={}, watch_motion=False)
    status = health.controller_status(view)
    assert status.level == OK
    assert status.message == 'current; joints not watched (watch_motion is false)'
    assert values_of(status)['joints published'] == '0'
    assert values_of(status)['motion lease'] == 'not asked'
    # A warning keeps its level and the note stays readable behind it.
    polled = frames.controller(link=frames.link_group(quality='polling'),
                               motion=frames.motion_group(watching=False))
    status = health.controller_status(controller_view(polled, leases={}, watch_motion=False))
    assert status.level == WARN
    assert status.message.startswith('link polling: ')
    assert status.message.endswith('; joints not watched (watch_motion is false)')


def test_without_watch_motion_joints_that_somebody_else_watches_are_judged_as_always():
    # Another consumer of the daemon watches: the joints are published while confirmed
    # and stale when they are not, whatever watch_motion says.
    view = controller_view(leases={}, watch_motion=False)
    assert health.controller_status(view).level == OK
    assert health.controller_status(view).message == 'current'
    late = dataclasses.replace(view, motion_age_s=2.0)
    assert health.controller_status(late).level == STALE
    assert health.controller_status(late).message == 'joints not confirmed for 2.0 s'
    # And with watch_motion a controller nobody watches is not what was asked for.
    idle = frames.controller(motion=frames.motion_group(watching=False))
    assert health.controller_status(controller_view(idle, watch_motion=True)).level == STALE


def test_controller_is_stale_while_the_daemon_is_not_live():
    for view in (controller_view(daemon_live=False),
                 controller_view(None, daemon_live=False)):
        status = health.controller_status(view)
        assert status.level == STALE
        assert status.message.startswith('no current data: the daemon is not live')
        assert values_of(status)['joints published'] == '0'


def test_controller_error_comes_before_stale_and_warning():
    polled = frames.controller(link=frames.link_group(quality='polling', quality_reason='slow'))
    view = controller_view(polled, motion_age_s=9.0, conversion_problem='too many events',
                           io_rejected=(('x', 'unknown signal'),))
    status = health.controller_status(view)
    assert status.level == ERROR
    assert status.message == (
        'the snapshot does not fit the messages: too many events; '
        'joints not confirmed for 9.0 s; link polling: slow; '
        '1 signal refused by the daemon: x: unknown signal')
    stale = health.controller_status(dataclasses.replace(view, conversion_problem=None))
    assert stale.level == STALE
    warn = health.controller_status(
        dataclasses.replace(view, conversion_problem=None, motion_age_s=0.1))
    assert warn.level == WARN


# Replay on every status, and what no status holds.

def test_replay_marked():
    replay = frames.replay_health(recorded_at='2026-01-01T08:00:00.000Z')
    ok = health.controller_status(controller_view())
    marked = health.replay_marked(ok, replay)
    assert marked.level == WARN
    assert marked.message == 'REPLAY: recorded 2026-01-01T08:00:00.000Z; current'
    assert (marked.name, marked.hardware_id, marked.values) == (
        ok.name, ok.hardware_id, ok.values)
    for view, level in ((controller_view(motion_age_s=9.0), STALE),
                        (controller_view(conversion_problem='x'), ERROR),
                        (controller_view(io_rejected=(('a', 'b'),)), WARN)):
        marked = health.replay_marked(health.controller_status(view), replay)
        assert marked.level == level
        assert marked.message.startswith('REPLAY: recorded 2026-01-01T08:00:00.000Z; ')
    assert health.replay_marked(ok, None).message.startswith('REPLAY: recorded ')


def test_replay_marked_does_not_mark_twice():
    replay = frames.replay_health(recorded_at='2026-01-01T08:00:00.000Z')
    once = health.replay_marked(health.controller_status(controller_view()), replay)
    assert health.replay_marked(once, replay) == once
    daemon = health.daemon_status(daemon_view(mode='replay', allow_replay=True, replay=replay))
    assert health.replay_marked(daemon, replay) == daemon
    assert daemon.message.count('REPLAY') == 1


def test_no_view_has_a_field_for_a_secret():
    fields = {field.name for view in (health.DaemonView, health.ControllerView, health.Status)
              for field in dataclasses.fields(view)}
    assert not {name for name in fields if 'token' in name or 'password' in name}


def test_every_value_is_text_at_every_level():
    statuses = [
        health.daemon_status(daemon_view()),
        health.daemon_status(daemon_view(connected=False, reason='unreachable')),
        health.daemon_status(daemon_view(
            mode='replay', allow_replay=True, replay=frames.replay_health())),
        health.daemon_status(daemon_view(
            daemon_version=None, daemon_schema=None, connected_for_s=None,
            contact_age_s=None, clock_offset_ms=None)),
        health.controller_status(controller_view()),
        health.controller_status(controller_view(frames.edge_controller())),
        health.controller_status(controller_view(None)),
        health.controller_status(controller_view({'id': 'odd'})),
    ]
    for status in statuses:
        assert status.level in (OK, WARN, ERROR, STALE)
        assert isinstance(status.message, str) and status.message
        assert isinstance(status.values, tuple)
        values_of(status)

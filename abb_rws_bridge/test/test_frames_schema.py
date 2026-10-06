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

"""Every builder of frames.py gives data that is valid against the vendored schema."""

import copy

import frames
import pytest

SCHEMA = frames.load_schema()


def _full_delta():
    snapshot = frames.controller('cell-2', name='Second cell')
    return frames.delta_frame('cell-2', frames.groups_of(snapshot), name='Second cell')


VALID = {
    'joint': ('MotionJoint', frames.joint()),
    'joint, millimetres': ('MotionJoint', frames.joint('E1', 12.5, 'mm', 0.0, 1000.0)),
    'joint, unit null': ('MotionJoint', frames.joint('J2', 1.0, None, None, None)),
    'joint, value null': ('MotionJoint', frames.joint('J3', None)),
    'pose': ('MotionPose', frames.tcp_pose()),
    'pose, named frame': ('MotionPose', frames.tcp_pose(
        tool='tool0', wobj='wobj0', coord='Wobj')),
    'group error': ('GroupError', frames.group_error()),
    'group error, controller and refusal': ('GroupError', frames.group_error(
        'REFUSED', 'refused', controller=(403, None, None), refusal={'kind': 'grant'})),
    'motion unit': ('MotionUnitInfo', frames.motion_unit()),
    'motion unit, pose and error': ('MotionUnitInfo', frames.motion_unit(
        'STN_1', [frames.joint('E1', 3.0, 'mm')], kind=None, type_raw=None, task=None,
        pose=frames.tcp_pose(), error=frames.group_error())),
    'signal': ('IoWatchedSignal', frames.signal()),
    'signal, not read': ('IoWatchedSignal', frames.signal(
        'do_x', None, None, via=None, as_of=None, error=frames.group_error())),
    'identity': ('IdentityGroup', frames.identity_group()),
    'identity, nulls': ('IdentityGroup', frames.identity_group(
        controller_name=None, controller_id=None, controller_type=None, virtual=None, mac=None,
        system_name=None, sys_id=None, start_time=None, options=(),
        version=frames.robot_ware(None, None, None, None))),
    'link': ('LinkGroup', frames.link_group()),
    'link, down': ('LinkGroup', frames.link_group(
        connected=False, quality='disconnected', quality_reason='not connected',
        subscriptions=False, reconnect={'failures': 2, 'nextAttemptAt': frames.iso(5)})),
    'link, mismatch': ('LinkGroup', frames.link_group(
        identity_status='mismatch', actual='00000000-0000-4000-8000-000000000002')),
    'panel': ('PanelGroup', frames.panel_group()),
    'rapid': ('RapidGroup', frames.rapid_group()),
    'motion': ('MotionGroup', frames.motion_group()),
    'motion, not watched': ('MotionGroup', frames.motion_group(watching=False)),
    'motion, lease ran out': ('MotionGroup', frames.motion_group(
        [frames.motion_unit()], watching=False)),
    'io, none': ('IoGroup', frames.io_group()),
    'io, one of each type': ('IoGroup', frames.io_group(
        [frames.signal(f'sig_{kind.lower()}', kind, '1') for kind in frames.SIGNAL_TYPES])),
    'io, lease ran out': ('IoGroup', frames.io_group([frames.signal()], watching=False)),
    'event': ('ControllerEvent', frames.event(args=[('string', 'x')])),
    'events': ('EventsGroup', frames.events_group([frames.event(1), frames.event(2)])),
    'controller': ('ControllerSnapshot', frames.controller()),
    'controller, identity null': ('ControllerSnapshot', frames.controller(identity=None)),
    'controller, every group given': ('ControllerSnapshot', frames.controller(
        'cell-9', name='Cell 9', identity=frames.identity_group(), link=frames.link_group(),
        panel=frames.panel_group(), rapid=frames.rapid_group([]),
        motion=frames.motion_group(watching=False), io=frames.io_group([frames.signal()]),
        events=frames.events_group([frames.event()]))),
    'edge controller': ('ControllerSnapshot', frames.EDGE_CONTROLLER),
    'snapshot frame': ('StreamFrame', frames.snapshot_frame()),
    'snapshot frame, empty fleet': ('StreamFrame', frames.snapshot_frame([])),
    'snapshot frame, two controllers': ('StreamFrame', frames.snapshot_frame(
        [frames.controller('a'), frames.edge_controller('b')], observed_at=frames.iso(2.5))),
    'snapshot frame as itself': ('SnapshotFrame', frames.snapshot_frame()),
    'delta frame, no group': ('StreamFrame', frames.delta_frame()),
    'delta frame, one group': ('StreamFrame', frames.delta_frame(
        'cell-1', {'motion': frames.motion_group(as_of=frames.iso(1))},
        observed_at=frames.iso(1))),
    'delta frame, identity null': ('StreamFrame', frames.delta_frame(
        'cell-1', {'identity': None})),
    'delta frame, new controller': ('StreamFrame', _full_delta()),
    'delta frame as itself': ('DeltaFrame', _full_delta()),
    'removed frame': ('StreamFrame', frames.removed_frame('cell-1', observed_at=frames.iso(3))),
    'removed frame as itself': ('RemovedFrame', frames.removed_frame()),
    'health': ('HealthAnswer', frames.health()),
    'health, recording': ('HealthAnswer', frames.health(recording=True, uptime_s=12)),
    'health, replay': ('HealthAnswer', frames.health(mode='replay')),
    'health, replay paused': ('HealthAnswer', frames.health(
        mode='replay', replay=frames.replay_health(state='paused', position_ms=1500,
                                                   speed=2.0, loop=True, loops=3,
                                                   time_mode='recorded'))),
    'replay health': ('ReplayHealth', frames.replay_health()),
    'controller list': ('ControllerList', frames.controller_list()),
    'controller list, empty': ('ControllerList', frames.controller_list([])),
    'controller list, edge': ('ControllerList', frames.controller_list(
        [frames.controller(), frames.edge_controller()], observed_at=frames.iso(60))),
    'motion watch': ('MotionWatch', frames.motion_watch_answer()),
    'motion watch, shared lease': ('MotionWatch', frames.motion_watch_answer(
        lease_ms=15000, target_hz=2.0, lease_id=None)),
    'io watch': ('IoWatch', frames.io_watch_answer()),
    'io watch, accepted and rejected': ('IoWatch', frames.io_watch_answer(
        ['do_gripper'], [('do_none', 'not in the signal table')], lease_id='abc_DEF-1')),
    'io watch, shared lease': ('IoWatch', frames.io_watch_answer(['do_a'], lease_id=None)),
}
VALID.update({
    f'error body {code}': ('ErrorBody', frames.error_answer(code)[2])
    for code in frames.ERROR_CODES
})

BROKEN = {
    'a joint in radians': lambda c: c['motion']['units'][0]['joints'][0].update(unit='rad'),
    'a joint without a unit key': lambda c: c['motion']['units'][0]['joints'][0].pop('unit'),
    'a missing group': lambda c: c.pop('events'),
    'an unknown key': lambda c: c.update(extra=1),
    'a number as text': lambda c: c['motion']['units'][0]['joints'][0].update(value='1.0'),
    'a time that is not ISO 8601': lambda c: c['motion'].update(asOf='yesterday'),
    'a time without Z': lambda c: c['link'].update(since='2026-01-01T00:00:00.000'),
    'a quaternion of three': lambda c: c['motion']['units'][0].update(
        pose=frames.tcp_pose(q=(1.0, 0.0, 0.0))),
    'a link quality that does not exist': lambda c: c['link'].update(quality='fine'),
}


@pytest.mark.parametrize('label', sorted(VALID))
def test_builder_output_is_valid(label):
    definition, data = VALID[label]
    assert frames.schema_errors(definition, data, SCHEMA) == []


@pytest.mark.parametrize('definition', [
    'ControllerSnapshot', 'StreamFrame', 'HealthAnswer', 'ControllerList', 'MotionWatch',
    'IoWatch', 'ErrorBody'])
def test_every_definition_the_bridge_reads_is_covered(definition):
    assert any(covered == definition for covered, _ in VALID.values())


@pytest.mark.parametrize('label', sorted(BROKEN))
def test_the_check_is_not_vacuous(label):
    snapshot = frames.controller()
    BROKEN[label](snapshot)
    assert frames.schema_errors('ControllerSnapshot', snapshot, SCHEMA) != []
    frame = frames.snapshot_frame([snapshot])
    assert frames.schema_errors('StreamFrame', frame, SCHEMA) != []


def test_schema_version_is_the_vendored_one():
    assert frames.SCHEMA_VERSION == SCHEMA['x-schema-version']
    assert frames.snapshot_frame()['schemaVersion'] == SCHEMA['x-schema-version']
    assert SCHEMA['$id'] == 'urn:abb-rws-daemon:schema:v0'
    assert set(frames.GROUPS) == set(SCHEMA['definitions']['GroupName']['enum'])
    assert frames.load_schema() == SCHEMA and frames.load_schema() is not SCHEMA


def test_schema_errors_refuses_an_unknown_definition():
    with pytest.raises(KeyError):
        frames.schema_errors('NoSuchDefinition', {}, SCHEMA)
    assert frames.schema_errors('ControllerList', frames.controller_list()) == []


def test_error_answers_carry_status_headers_and_body():
    statuses = {code: frames.error_answer(code)[0] for code in frames.ERROR_CODES}
    assert statuses == {
        'TOKEN_MISSING': 401, 'TOKEN_INVALID': 401, 'NO_CONTROLLER': 404,
        'IDENTITY_MISMATCH': 409, 'HOST_NOT_ALLOWED': 421, 'LEASE_LIMIT': 429}
    for code in frames.ERROR_CODES:
        status, headers, body = frames.error_answer(code, controller_id='cell-7')
        assert body['code'] == code and isinstance(body['error'], str) and body['error']
        assert headers['Content-Type'].startswith('application/json')
        assert ('Retry-After' in headers) == (code == 'LEASE_LIMIT')
    assert 'cell-7' in frames.error_answer('NO_CONTROLLER', controller_id='cell-7')[2]['error']
    assert frames.error_answer('LEASE_LIMIT', retry_after_s=3)[1]['Retry-After'] == '3'
    with pytest.raises(KeyError):
        frames.error_answer('NOT_A_CODE')


def test_io_signal_table_has_the_shape_of_the_route():
    assert frames.io_signal_table() == {
        'signals': [{'name': 'do_gripper', 'type': 'DO', 'value': '0'}]}
    table = frames.io_signal_table([frames.signal('di_a', 'DI', '1'), ('ao_b', 'AO', None)])
    assert table == {'signals': [{'name': 'di_a', 'type': 'DI', 'value': '1'},
                                 {'name': 'ao_b', 'type': 'AO', 'value': None}]}
    assert frames.io_signal_table([]) == {'signals': []}


def test_edge_controller_holds_every_edge_case():
    edge = frames.EDGE_CONTROLLER
    assert edge['identity'] is None
    units = {unit['name']: unit for unit in edge['motion']['units']}
    joints = {entry['name']: entry for entry in units['ROB_1']['joints']}
    assert joints['E1']['unit'] == 'mm' and joints['E1']['value'] is not None
    assert joints['J4']['unit'] is None and joints['J4']['value'] is not None
    assert joints['J3']['value'] is None and joints['J3']['unit'] == 'deg'
    assert joints['J1']['unit'] == 'deg' and joints['J1']['value'] is not None
    assert units['ROB_1']['error'] is None
    with_error = [unit for unit in edge['motion']['units'] if unit['error'] is not None]
    assert len(with_error) == 1 and with_error[0]['joints'][0]['value'] is not None
    types = [entry['type'] for entry in edge['io']['signals']]
    assert sorted(kind for kind in types if kind) == sorted(frames.SIGNAL_TYPES)
    assert types.count(None) == 1
    assert edge['motion']['watching'] is True and edge['io']['watching'] is True
    assert edge['link']['connected'] is True


def test_builders_return_new_objects():
    first = frames.controller()
    first['motion']['units'][0]['joints'][0]['value'] = 99.0
    first['identity']['options'].append('changed')
    assert frames.controller() == frames.controller() != first
    again = frames.edge_controller()
    assert again == frames.EDGE_CONTROLLER and again is not frames.EDGE_CONTROLLER
    again['io']['signals'].clear()
    assert frames.EDGE_CONTROLLER['io']['signals']
    assert frames.edge_controller('other')['id'] == 'other'
    snapshot = frames.controller()
    groups = frames.groups_of(snapshot)
    assert tuple(groups) == frames.GROUPS and groups['motion'] == snapshot['motion']
    assert groups['motion'] is not snapshot['motion']
    assert copy.deepcopy(snapshot) == snapshot


def test_default_controller_and_group_defaults():
    snapshot = frames.controller()
    assert snapshot['id'] == snapshot['name'] == 'cell-1'
    joints = snapshot['motion']['units'][0]['joints']
    assert [entry['name'] for entry in joints] == ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']
    assert all(entry['unit'] == 'deg' for entry in joints)
    assert snapshot['io'] == {'watching': False, 'signals': [], 'asOf': None}
    assert frames.motion_group(watching=False) == {
        'watching': False, 'units': [],
        'rate': {'targetHz': None, 'achievedHz': None, 'via': None}, 'asOf': None}
    assert frames.events_group([frames.event(3), frames.event(5)])['lastSeq'] == 5
    assert frames.health()['replay'] is None and frames.health(mode='replay')['replay']
    assert 'leaseId' not in frames.motion_watch_answer(lease_id=None)
    assert frames.motion_watch_answer()['leaseId'] == frames.LEASE_ID


def test_iso_counts_from_t0_in_milliseconds():
    assert frames.iso() == frames.T0 == '2026-01-01T00:00:00.000Z'
    assert frames.iso(1.5) == '2026-01-01T00:00:01.500Z'
    assert frames.iso(0.25) == '2026-01-01T00:00:00.250Z'
    assert frames.iso(3600) == '2026-01-01T01:00:00.000Z'
    assert frames.iso(-1) == '2025-12-31T23:59:59.000Z'

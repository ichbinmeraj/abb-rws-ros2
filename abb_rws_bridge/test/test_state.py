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

"""The fleet state: stream frames write it, controller lists can only confirm it."""

import copy
import dataclasses
import json
import random

from abb_rws_bridge import state
import frames
import pytest

ALL = frozenset(state.GROUPS)
# The groups with a stamp (asOf) of their own: link has none, and freshness has one per
# group it rates.
STAMPED = ALL - {'link', 'freshness'}
CLOCK = {'offsetMs': 0, 'measuredAt': frames.T0, 'problem': None}

# One change of a value per group: where it is, and the new value.
VALUE_CHANGES = {
    'identity': (('identity', 'systemName'), 'Other_System'),
    'link': (('link', 'quality'), 'polling'),
    'panel': (('panel', 'opMode'), 'MANR'),
    'rapid': (('rapid', 'execState'), 'running'),
    'motion': (('motion', 'units', 0, 'joints', 5, 'value'), 5.0),
    'io': (('io', 'signals', 0, 'value'), '1'),
    'events': (('events', 'lastSeq'), 99),
    'access': (('access', 'locks', 0, 'held'), True),
    'capabilities': (('capabilities', 'tested'), False),
    'freshness': (('freshness', 'level'), 'degraded'),
}


def _controller(controller_id='cell-1', seconds=0.0, *, j6=0.0, achieved_hz=7.9):
    """Return a controller whose every asOf is seconds after T0; all else stays the same."""
    stamp = frames.iso(seconds)
    joints = [frames.joint(f'J{axis}') for axis in range(1, 6)] + [frames.joint('J6', j6)]
    return frames.controller(
        controller_id,
        identity=frames.identity_group(as_of=stamp),
        panel=frames.panel_group(as_of=stamp),
        rapid=frames.rapid_group(as_of=stamp),
        motion=frames.motion_group([frames.motion_unit(joints=joints)], as_of=stamp,
                                   achieved_hz=achieved_hz),
        io=frames.io_group([frames.signal(as_of=stamp)], as_of=stamp),
        events=frames.events_group([frames.event()], as_of=stamp, clock=dict(CLOCK)),
        access=frames.access_group(
            [frames.access_lock('edit', as_of=stamp), frames.access_lock('motion', as_of=stamp)],
            as_of=stamp),
        capabilities=frames.capabilities_group(as_of=stamp),
        freshness=frames.freshness_group(as_of=stamp))


def _with_change(controller, group):
    """Change one value in a group of the controller, and return the controller."""
    path, value = VALUE_CHANGES[group]
    target = controller
    for key in path[:-1]:
        target = target[key]
    assert target[path[-1]] != value
    target[path[-1]] = value
    return controller


def _fleet(*controllers, observed_at=frames.T0, wall_ns=1000, mono=1.0):
    fleet = state.FleetState(frames.SCHEMA_VERSION)
    applied = fleet.apply(
        frames.snapshot_frame(list(controllers) or [_controller()], observed_at=observed_at),
        wall_ns, mono)
    assert applied.kind == 'snapshot'
    assert applied.problem is None
    return fleet


def _values(controller):
    """Return everything of a controller that a confirm may not change."""
    values = {name: state.comparable(name, controller[name]) for name in state.GROUPS}
    values['id'] = controller['id']
    values['name'] = controller['name']
    return values


def _joint(controller, index=5):
    return controller['motion']['units'][0]['joints'][index]['value']


# For the long random run: one value per group that a number can make new every time.
FRESH_VALUES = {
    'identity': (('identity', 'systemName'), 'System_{}'.format),
    'link': (('link', 'qualityReason'), 'reason {}'.format),
    'panel': (('panel', 'opMode'), 'MODE_{}'.format),
    'rapid': (('rapid', 'cycle'), 'cycle_{}'.format),
    'motion': (('motion', 'units', 0, 'joints', 5, 'value'), lambda number: number + 0.125),
    'io': (('io', 'signals', 0, 'value'), str),
    'events': (('events', 'lastSeq'), lambda number: 1000 + number),
    'access': (('access', 'problem'), 'problem {}'.format),
    'capabilities': (('capabilities', 'robotWare'), '7.{}.0'.format),
    'freshness': (('freshness', 'groups', 0, 'reason'), 'reason {}'.format),
}


def _with_fresh_value(controller, group, number):
    path, make = FRESH_VALUES[group]
    target = controller
    for key in path[:-1]:
        target = target[key]
    assert target[path[-1]] != make(number)
    target[path[-1]] = make(number)
    return controller


def _restamped(value, stamp):
    """Return a deep copy in which every asOf that is set is the stamp."""
    if isinstance(value, dict):
        return {key: stamp if key == 'asOf' and item is not None else _restamped(item, stamp)
                for key, item in value.items()}
    if isinstance(value, list):
        return [_restamped(item, stamp) for item in value]
    return value


def test_the_groups_are_those_of_the_schema():
    assert state.GROUPS == frames.GROUPS
    assert list(state.GROUPS) == frames.load_schema()['definitions']['GroupName']['enum']


def test_the_synthetic_controller_of_these_tests_fits_the_schema():
    assert frames.schema_errors('ControllerSnapshot', _controller(seconds=1.5, j6=5.0)) == []
    assert _values(_controller(seconds=0.0)) == _values(_controller(seconds=9.0))


def test_a_new_state_is_empty():
    fleet = state.FleetState('0.12')
    assert fleet.ids() == ()
    assert fleet.get('cell-1') is None
    assert fleet.seen('cell-1') is None
    assert fleet.incomplete() == ()
    assert fleet.schema_version is None
    assert fleet.unknown_frames == 0
    assert fleet.messages_schema == '0.12'


@pytest.mark.parametrize('version', ['', '12', 'v0.12', '0.12.1', '0.x', ' 0.12', None, 0.12])
def test_the_state_needs_the_schema_version_of_the_messages(version):
    with pytest.raises(ValueError):
        state.FleetState(version)


def test_seen_and_applied_cannot_be_changed():
    seen = state.Seen(frames.T0, 1, 1.0, None, None, None)
    applied = state.Applied('ignored')
    with pytest.raises(dataclasses.FrozenInstanceError):
        seen.mono = 2.0
    with pytest.raises(dataclasses.FrozenInstanceError):
        applied.kind = 'delta'
    assert (applied.changed, applied.removed, applied.confirmed, applied.motion_moved,
            applied.problem) == ({}, (), (), (), None)


def test_a_snapshot_fills_the_fleet():
    first, second = _controller('cell-2', 0.5), _controller('cell-1', 0.25)
    fleet = state.FleetState(frames.SCHEMA_VERSION)
    applied = fleet.apply(frames.snapshot_frame([first, second], observed_at=frames.iso(1)),
                          1000, 1.0)
    assert applied == state.Applied(
        'snapshot', changed={'cell-1': ALL, 'cell-2': ALL}, motion_moved=('cell-2', 'cell-1'))
    assert fleet.ids() == ('cell-1', 'cell-2')
    assert fleet.get('cell-1') == second
    assert fleet.get('cell-2') == first
    assert fleet.get('cell-3') is None
    assert fleet.schema_version == frames.SCHEMA_VERSION
    assert fleet.seen('cell-1') == state.Seen(
        frames.iso(1), 1000, 1.0, frames.iso(0.25), 1000, 1.0)
    assert fleet.seen('cell-2') == state.Seen(
        frames.iso(1), 1000, 1.0, frames.iso(0.5), 1000, 1.0)
    assert fleet.seen('cell-3') is None


def test_a_snapshot_replaces_the_whole_fleet():
    fleet = _fleet(_controller('a'), _controller('b'))
    applied = fleet.apply(
        frames.snapshot_frame([_controller('b', 2.0), _controller('c', 2.0)],
                              observed_at=frames.iso(2)), 2000, 2.0)
    assert applied.kind == 'snapshot'
    assert applied.removed == ('a',)
    assert applied.changed == {'b': ALL, 'c': ALL}
    assert applied.problem is None
    assert fleet.ids() == ('b', 'c')
    assert fleet.get('a') is None
    assert fleet.seen('a') is None
    assert fleet.get('b') == _controller('b', 2.0)


def test_an_empty_snapshot_removes_everything():
    fleet = _fleet(_controller('a'), _controller('b'))
    applied = fleet.apply(frames.snapshot_frame([], observed_at=frames.iso(1)), 2000, 2.0)
    assert applied == state.Applied('snapshot', removed=('a', 'b'))
    assert fleet.ids() == ()


def test_the_edge_controller_is_applied_whole():
    fleet = _fleet(frames.edge_controller('edge-1'))
    assert fleet.ids() == ('edge-1',)
    assert fleet.get('edge-1') == frames.EDGE_CONTROLLER
    assert fleet.get('edge-1')['identity'] is None
    assert fleet.seen('edge-1').motion_as_of == frames.iso(0.5)


def test_a_delta_replaces_the_groups_it_names_whole_and_the_name():
    fleet = _fleet()
    before = fleet.get('cell-1')
    panel = frames.panel_group(speed_ratio=50.0, as_of=frames.iso(3))
    applied = fleet.apply(
        frames.delta_frame('cell-1', {'panel': panel}, name='Renamed', observed_at=frames.iso(3)),
        3000, 3.0)
    assert applied == state.Applied('delta', changed={'cell-1': frozenset({'panel'})})
    after = fleet.get('cell-1')
    assert after['panel'] is panel
    assert after['name'] == 'Renamed'
    assert after['id'] == 'cell-1'
    for name in state.GROUPS:
        if name != 'panel':
            assert after[name] is before[name]
    # The frame moved the controller's time, not the time of its motion.
    assert fleet.seen('cell-1') == state.Seen(frames.iso(3), 3000, 3.0, frames.T0, 1000, 1.0)


def test_a_group_is_replaced_never_edited_in_place():
    fleet = _fleet()
    held_controller = fleet.get('cell-1')
    held_motion = held_controller['motion']
    as_it_was = copy.deepcopy(held_controller)
    moved = _controller(seconds=1.0, j6=5.0)
    fleet.apply(frames.delta_frame('cell-1', {'motion': moved['motion']},
                                   observed_at=frames.iso(1)), 2000, 2.0)
    fleet.confirm(frames.controller_list([_controller(seconds=2.0, j6=5.0)],
                                         observed_at=frames.iso(2)), 3000, 3.0)
    assert held_controller == as_it_was
    assert held_controller['motion'] is held_motion
    assert _joint(held_controller) == 0.0
    assert fleet.get('cell-1') is not held_controller
    assert _joint(fleet.get('cell-1')) == 5.0
    assert fleet.get('cell-1')['motion']['asOf'] == frames.iso(2)


def test_a_motion_delta_moves_the_motion_stamp_only_forward():
    fleet = _fleet()

    def motion_delta(as_of_s, observed_s):
        group = frames.motion_group(as_of=frames.iso(as_of_s))
        return frames.delta_frame('cell-1', {'motion': group}, observed_at=frames.iso(observed_s))

    applied = fleet.apply(motion_delta(0.25, 0.3), 2000, 2.0)
    assert applied.motion_moved == ('cell-1',)
    assert fleet.seen('cell-1') == state.Seen(
        frames.iso(0.3), 2000, 2.0, frames.iso(0.25), 2000, 2.0)

    # The same stamp again: news for the controller, not for its motion.
    applied = fleet.apply(motion_delta(0.25, 0.6), 3000, 3.0)
    assert applied.motion_moved == ()
    assert applied.changed == {'cell-1': frozenset({'motion'})}
    assert fleet.seen('cell-1') == state.Seen(
        frames.iso(0.6), 3000, 3.0, frames.iso(0.25), 2000, 2.0)

    # An older stamp: kept as the daemon gave it, and it did not move forward.
    applied = fleet.apply(motion_delta(0.125, 0.9), 4000, 4.0)
    assert applied.motion_moved == ()
    assert fleet.seen('cell-1') == state.Seen(
        frames.iso(0.9), 4000, 4.0, frames.iso(0.125), 2000, 2.0)

    applied = fleet.apply(motion_delta(0.5, 1.2), 5000, 5.0)
    assert applied.motion_moved == ('cell-1',)
    assert fleet.seen('cell-1').motion_wall_ns == 5000
    assert fleet.seen('cell-1').motion_mono == 5.0


def test_motion_that_was_never_read_has_no_motion_time():
    idle = frames.controller('cell-1', motion=frames.motion_group(watching=False))
    fleet = state.FleetState(frames.SCHEMA_VERSION)
    applied = fleet.apply(frames.snapshot_frame([idle]), 1000, 1.0)
    assert applied.motion_moved == ()
    assert fleet.seen('cell-1') == state.Seen(frames.T0, 1000, 1.0, None, None, None)

    watched = frames.motion_group(as_of=frames.iso(4))
    applied = fleet.apply(
        frames.delta_frame('cell-1', {'motion': watched}, observed_at=frames.iso(4)), 5000, 5.0)
    assert applied.motion_moved == ('cell-1',)
    assert fleet.seen('cell-1') == state.Seen(frames.iso(4), 5000, 5.0, frames.iso(4), 5000, 5.0)


def test_a_delta_for_an_unknown_controller_with_every_group_adds_it():
    fleet = _fleet()
    new = _controller('cell-9', 2.0)
    applied = fleet.apply(
        frames.delta_frame('cell-9', frames.groups_of(new), name='Nine',
                           observed_at=frames.iso(2)), 2000, 2.0)
    assert applied == state.Applied('delta', changed={'cell-9': ALL}, motion_moved=('cell-9',))
    assert fleet.ids() == ('cell-1', 'cell-9')
    expected = dict(new, name='Nine')
    assert fleet.get('cell-9') == expected
    assert list(fleet.get('cell-9'))[:2] == ['id', 'name']
    assert fleet.seen('cell-9') == state.Seen(frames.iso(2), 2000, 2.0, frames.iso(2), 2000, 2.0)
    assert fleet.incomplete() == ()


@pytest.mark.parametrize('missing', state.GROUPS)
def test_a_delta_for_an_unknown_controller_without_all_groups_is_held_back(missing):
    fleet = _fleet()
    new = _controller('cell-9', 2.0)
    groups = frames.groups_of(new)
    late = groups.pop(missing)
    applied = fleet.apply(frames.delta_frame('cell-9', groups, observed_at=frames.iso(2)),
                          2000, 2.0)
    assert applied.kind == 'delta'
    assert applied.changed == {}
    assert applied.motion_moved == ()
    assert 'cell-9' in applied.problem
    assert 'held back' in applied.problem
    assert missing in applied.problem
    assert fleet.ids() == ('cell-1',)
    assert fleet.get('cell-9') is None
    assert fleet.seen('cell-9') is None
    assert fleet.incomplete() == ('cell-9',)

    # A list of controllers confirms nothing about a controller that is held back.
    confirmed = fleet.confirm(frames.controller_list([new], observed_at=frames.iso(2.5)),
                              2500, 2.5)
    assert confirmed.confirmed == ()
    assert fleet.get('cell-9') is None

    # The group that was missing arrives: the controller is whole and appears.
    applied = fleet.apply(
        frames.delta_frame('cell-9', {missing: late}, observed_at=frames.iso(3)), 3000, 3.0)
    assert applied.problem is None
    assert applied.changed == {'cell-9': ALL}
    assert fleet.ids() == ('cell-1', 'cell-9')
    assert fleet.get('cell-9') == new
    assert fleet.incomplete() == ()
    assert fleet.seen('cell-9').observed_at == frames.iso(3)


def test_a_removed_frame_drops_the_controller():
    fleet = _fleet(_controller('a'), _controller('b'))
    applied = fleet.apply(frames.removed_frame('a', observed_at=frames.iso(1)), 2000, 2.0)
    assert applied == state.Applied('removed', removed=('a',))
    assert fleet.ids() == ('b',)
    assert fleet.get('a') is None
    assert fleet.seen('a') is None

    # Removing what is not there harms nothing.
    applied = fleet.apply(frames.removed_frame('a'), 3000, 3.0)
    assert applied == state.Applied('removed')
    assert fleet.ids() == ('b',)


def test_a_removed_frame_also_drops_a_controller_that_is_held_back():
    fleet = _fleet()
    fleet.apply(frames.delta_frame('cell-9', {'panel': frames.panel_group()}), 2000, 2.0)
    assert fleet.incomplete() == ('cell-9',)
    applied = fleet.apply(frames.removed_frame('cell-9'), 3000, 3.0)
    assert applied == state.Applied('removed')
    assert fleet.incomplete() == ()


def test_a_removed_controller_that_comes_back_starts_afresh():
    fleet = _fleet(_controller('a', 5.0))
    fleet.apply(frames.removed_frame('a'), 2000, 2.0)
    # Back with an older motion stamp than it had: it still counts as moved, from nothing.
    back = _controller('a', 1.0)
    applied = fleet.apply(frames.delta_frame('a', frames.groups_of(back),
                                             observed_at=frames.iso(6)), 3000, 3.0)
    assert applied.motion_moved == ('a',)
    assert fleet.seen('a') == state.Seen(frames.iso(6), 3000, 3.0, frames.iso(1), 3000, 3.0)


@pytest.mark.parametrize('frame', [
    {'type': 'heartbeat', 'observedAt': frames.T0},
    {'type': 'snapshot2', 'observedAt': frames.T0, 'controllers': []},
    {'type': 'Delta', 'observedAt': frames.T0, 'id': 'cell-1', 'groups': {}},
    {'type': None},
    {'type': 7},
    {},
])
def test_an_unknown_frame_type_is_ignored_and_counted(frame):
    fleet = _fleet()
    before = copy.deepcopy(fleet.get('cell-1'))
    seen = fleet.seen('cell-1')
    applied = fleet.apply(frame, 9000, 9.0)
    assert applied == state.Applied('ignored')
    assert fleet.unknown_frames == 1
    fleet.apply(frame, 9500, 9.5)
    assert fleet.unknown_frames == 2
    assert fleet.ids() == ('cell-1',)
    assert fleet.get('cell-1') == before
    assert fleet.seen('cell-1') == seen
    assert fleet.schema_version == frames.SCHEMA_VERSION


def test_an_unknown_frame_type_is_counted_even_before_a_snapshot():
    fleet = state.FleetState(frames.SCHEMA_VERSION)
    assert fleet.apply({'type': 'heartbeat'}, 1, 1.0) == state.Applied('ignored')
    assert fleet.unknown_frames == 1


def test_an_unknown_group_is_kept_and_harms_nothing():
    fleet = _fleet()
    before = fleet.get('cell-1')
    panel = frames.panel_group(speed_ratio=25.0)
    applied = fleet.apply(
        frames.delta_frame('cell-1', {'diag': {'load': 3}, 'panel': panel},
                           observed_at=frames.iso(1)), 2000, 2.0)
    assert applied.kind == 'delta'
    assert applied.problem is None
    assert applied.changed == {'cell-1': frozenset({'diag', 'panel'})}
    after = fleet.get('cell-1')
    assert after['diag'] == {'load': 3}
    assert after['panel'] is panel
    for name in state.GROUPS:
        if name != 'panel':
            assert after[name] is before[name]

    # A delta that names only a group this version does not know still touches the controller.
    applied = fleet.apply(
        frames.delta_frame('cell-1', {'diag': [1, 2]}, observed_at=frames.iso(2)), 3000, 3.0)
    assert applied == state.Applied('delta', changed={'cell-1': frozenset({'diag'})})
    assert fleet.get('cell-1')['diag'] == [1, 2]
    assert fleet.seen('cell-1').observed_at == frames.iso(2)

    # An answer that says something else about it changes nothing and still confirms the rest.
    answer = _controller(seconds=5.0)
    answer['diag'] = 'something else'
    confirmed = fleet.confirm(frames.controller_list([answer], observed_at=frames.iso(5)),
                              6000, 6.0)
    assert confirmed.confirmed == ()
    assert 'panel' not in confirmed.changed['cell-1']
    assert fleet.get('cell-1')['diag'] == [1, 2]
    assert fleet.get('cell-1')['panel'] is panel


def test_an_unknown_key_of_a_snapshot_controller_is_kept():
    newer = _controller()
    newer['diag'] = {'load': 3}
    newer['label'] = 'x'
    fleet = _fleet(newer)
    assert fleet.get('cell-1')['diag'] == {'load': 3}
    assert fleet.get('cell-1')['label'] == 'x'
    # The groups of an answer are confirmed whatever else the controller holds.
    answer = _controller(seconds=1.0)
    applied = fleet.confirm(frames.controller_list([answer], observed_at=frames.iso(1)), 2000, 2.0)
    assert applied.confirmed == ('cell-1',)
    assert fleet.get('cell-1')['diag'] == {'load': 3}


def test_a_delta_cannot_replace_the_id_or_smuggle_a_name():
    fleet = _fleet()
    frame = frames.delta_frame('cell-1', {'id': 'other', 'name': 'smuggled',
                                          'panel': frames.panel_group(speed_ratio=10.0)},
                               name='Proper', observed_at=frames.iso(1))
    applied = fleet.apply(frame, 2000, 2.0)
    assert applied.changed == {'cell-1': frozenset({'panel'})}
    assert fleet.ids() == ('cell-1',)
    assert fleet.get('cell-1')['id'] == 'cell-1'
    assert fleet.get('cell-1')['name'] == 'Proper'


@pytest.mark.parametrize('version', ['0.12', '0.13', '0.99', '0.120'])
def test_a_snapshot_of_the_same_or_a_higher_minor_is_accepted(version):
    fleet = state.FleetState('0.12')
    applied = fleet.apply(frames.snapshot_frame(schema_version=version), 1000, 1.0)
    assert applied.problem is None
    assert applied.changed == {'cell-1': ALL}
    assert fleet.ids() == ('cell-1',)
    assert fleet.schema_version == version


@pytest.mark.parametrize('version', ['0.11', '0.2', '0.0', '1.0', '1.12', '2.99', '00.11'])
def test_a_snapshot_of_a_lower_minor_or_another_major_is_refused(version):
    fleet = state.FleetState('0.12')
    applied = fleet.apply(frames.snapshot_frame(schema_version=version), 1000, 1.0)
    assert applied.kind == 'snapshot'
    assert applied.changed == {}
    assert version in applied.problem
    assert '0.12' in applied.problem
    assert fleet.ids() == ()
    assert fleet.get('cell-1') is None
    # The version the daemon named stays readable, for the status that reports it.
    assert fleet.schema_version == version


@pytest.mark.parametrize('version', ['', 'abc', '0', '0.12.1', 'v0.12', '0.12 ', None, 12, 0.12])
def test_a_snapshot_without_a_readable_schema_version_is_refused(version):
    fleet = state.FleetState('0.12')
    frame = frames.snapshot_frame(schema_version=version)
    applied = fleet.apply(frame, 1000, 1.0)
    assert applied.kind == 'snapshot'
    assert applied.problem is not None
    assert fleet.ids() == ()
    assert fleet.schema_version is None


def test_the_minor_is_compared_as_a_number_against_the_messages():
    fleet = state.FleetState('0.9')
    assert fleet.apply(frames.snapshot_frame(schema_version='0.10'), 1, 1.0).problem is None
    assert fleet.apply(frames.snapshot_frame(schema_version='0.8'), 2, 2.0).problem is not None
    other_major = state.FleetState('1.3')
    assert other_major.apply(frames.snapshot_frame(schema_version='1.3'), 1, 1.0).problem is None
    assert other_major.apply(frames.snapshot_frame(schema_version='0.12'), 2, 2.0).problem


def test_a_refused_snapshot_empties_the_fleet_and_nothing_is_taken_until_a_good_one():
    fleet = _fleet(_controller('a'), _controller('b'))
    applied = fleet.apply(frames.snapshot_frame([_controller('a')], schema_version='0.11'),
                          2000, 2.0)
    assert applied.removed == ('a', 'b')
    assert applied.problem is not None
    assert fleet.ids() == ()

    whole = frames.delta_frame('c', frames.groups_of(_controller('c')), observed_at=frames.iso(3))
    applied = fleet.apply(whole, 3000, 3.0)
    assert applied.kind == 'ignored'
    assert 'snapshot' in applied.problem
    assert fleet.ids() == ()
    assert fleet.incomplete() == ()

    applied = fleet.apply(frames.removed_frame('a'), 3500, 3.5)
    assert applied.kind == 'ignored'
    assert applied.problem is not None

    applied = fleet.confirm(frames.controller_list([_controller('a', 4.0)],
                                                   observed_at=frames.iso(4)), 4000, 4.0)
    assert applied == state.Applied('confirm')
    assert fleet.ids() == ()

    applied = fleet.apply(frames.snapshot_frame([_controller('a', 5.0)],
                                                observed_at=frames.iso(5)), 5000, 5.0)
    assert applied.problem is None
    assert fleet.ids() == ('a',)
    assert fleet.apply(whole, 6000, 6.0).changed == {'c': ALL}


def test_the_state_says_whether_it_holds_an_accepted_snapshot():
    fleet = state.FleetState(frames.SCHEMA_VERSION)
    assert fleet.has_snapshot is False
    fleet.apply({'type': 'heartbeat'}, 500, 0.5)
    fleet.apply(frames.delta_frame('a', frames.groups_of(_controller('a'))), 600, 0.6)
    assert fleet.has_snapshot is False
    fleet.apply(frames.snapshot_frame([]), 1000, 1.0)
    assert fleet.has_snapshot is True
    fleet.apply(frames.snapshot_frame(schema_version='0.11'), 2000, 2.0)
    assert fleet.has_snapshot is False
    fleet.apply(frames.snapshot_frame(), 3000, 3.0)
    assert fleet.has_snapshot is True
    fleet.confirm(frames.controller_list(), 3500, 3.5)
    fleet.apply(frames.removed_frame('cell-1'), 3600, 3.6)
    assert fleet.has_snapshot is True
    fleet.clear()
    assert fleet.has_snapshot is False
    with pytest.raises(AttributeError):
        fleet.has_snapshot = True


def test_frames_before_the_first_snapshot_are_refused():
    fleet = state.FleetState(frames.SCHEMA_VERSION)
    whole = frames.delta_frame('c', frames.groups_of(_controller('c')))
    for frame in (whole, frames.removed_frame('c')):
        applied = fleet.apply(frame, 1000, 1.0)
        assert applied.kind == 'ignored'
        assert 'snapshot' in applied.problem
    assert fleet.ids() == ()
    assert fleet.incomplete() == ()
    assert fleet.unknown_frames == 0


def test_clear_forgets_the_fleet_and_waits_for_a_snapshot():
    fleet = _fleet(_controller('a'), _controller('b'))
    fleet.apply(frames.delta_frame('c', {'panel': frames.panel_group()}), 1500, 1.5)
    fleet.apply({'type': 'heartbeat'}, 1600, 1.6)
    assert fleet.incomplete() == ('c',)
    fleet.clear()
    assert fleet.ids() == ()
    assert fleet.get('a') is None
    assert fleet.seen('a') is None
    assert fleet.incomplete() == ()
    assert fleet.schema_version is None
    assert fleet.unknown_frames == 1
    whole = frames.delta_frame('a', frames.groups_of(_controller('a')))
    assert fleet.apply(whole, 2000, 2.0).kind == 'ignored'
    assert fleet.confirm(frames.controller_list([_controller('a')]), 2500, 2.5) == (
        state.Applied('confirm'))
    assert fleet.ids() == ()
    # After a reconnect the motion stamp counts from nothing again.
    applied = fleet.apply(frames.snapshot_frame([_controller('a')]), 3000, 3.0)
    assert applied.motion_moved == ('a',)
    assert fleet.seen('a') == state.Seen(frames.T0, 3000, 3.0, frames.T0, 3000, 3.0)


@pytest.mark.parametrize('frame', [None, [], 'snapshot', 5, [frames.snapshot_frame()]])
def test_a_frame_that_is_not_an_object_is_refused(frame):
    fleet = _fleet()
    applied = fleet.apply(frame, 2000, 2.0)
    assert applied.kind == 'ignored'
    assert applied.problem is not None
    assert fleet.ids() == ('cell-1',)
    assert fleet.unknown_frames == 0


@pytest.mark.parametrize('change', [
    {'controllers': None},
    {'controllers': {'cell-1': {}}},
    {'observedAt': None},
    {'observedAt': 'yesterday'},
    {'observedAt': 1767225600},
])
def test_a_snapshot_that_is_not_shaped_as_one_is_refused(change):
    fleet = _fleet(_controller('a'))
    frame = dict(frames.snapshot_frame([_controller('b')]), **change)
    applied = fleet.apply(frame, 2000, 2.0)
    assert applied.kind == 'snapshot'
    assert applied.problem is not None
    assert applied.removed == ('a',)
    assert fleet.ids() == ()
    # Nothing is taken from a stream whose snapshot could not be read.
    whole = frames.delta_frame('c', frames.groups_of(_controller('c')))
    assert fleet.apply(whole, 3000, 3.0).kind == 'ignored'


def test_snapshot_entries_without_an_id_are_left_out_and_reported():
    nameless = _controller('x')
    del nameless['id']
    numbered = dict(_controller('y'), id=7)
    frame = frames.snapshot_frame([_controller('a'), nameless, numbered, 'text', None])
    fleet = state.FleetState(frames.SCHEMA_VERSION)
    applied = fleet.apply(frame, 1000, 1.0)
    assert fleet.ids() == ('a',)
    assert applied.changed == {'a': ALL}
    assert '4 controller(s) without an id' in applied.problem


def test_a_controller_listed_twice_in_a_snapshot_is_taken_once():
    first, second = _controller('a', 1.0), _controller('a', 2.0, j6=5.0)
    fleet = state.FleetState(frames.SCHEMA_VERSION)
    applied = fleet.apply(frames.snapshot_frame([first, _controller('b'), second]), 1000, 1.0)
    assert fleet.ids() == ('a', 'b')
    assert fleet.get('a') == first
    assert _joint(fleet.get('a')) == 0.0
    assert applied.changed == {'a': ALL, 'b': ALL}
    assert applied.motion_moved == ('a', 'b')
    assert 'a is listed more than once' in applied.problem


@pytest.mark.parametrize('group', state.GROUPS)
def test_a_snapshot_controller_without_a_group_is_held_back(group):
    broken = _controller('b')
    del broken[group]
    fleet = _fleet(_controller('a'), _controller('b'))
    applied = fleet.apply(frames.snapshot_frame([_controller('a'), broken]), 2000, 2.0)
    assert fleet.ids() == ('a',)
    assert applied.changed == {'a': ALL}
    assert applied.removed == ('b',)
    assert fleet.incomplete() == ('b',)
    assert fleet.get('b') is None
    assert 'b is held back' in applied.problem
    assert group in applied.problem
    # The stream completes it later.
    late = frames.delta_frame('b', {group: _controller('b')[group]}, observed_at=frames.iso(1))
    assert fleet.apply(late, 3000, 3.0).changed == {'b': ALL}
    assert fleet.get('b') == _controller('b')


@pytest.mark.parametrize('group, value', [
    ('link', None), ('panel', None), ('rapid', []), ('motion', 'none'), ('io', 5),
    ('events', None), ('identity', 'unknown'), ('identity', []),
])
def test_a_group_that_is_not_an_object_is_never_stored(group, value):
    broken = dict(_controller('b'), **{group: value})
    fleet = state.FleetState(frames.SCHEMA_VERSION)
    applied = fleet.apply(frames.snapshot_frame([_controller('a'), broken]), 1000, 1.0)
    assert fleet.ids() == ('a',)
    assert fleet.incomplete() == ('b',)
    assert f'{group} is not an object' in applied.problem

    # In a delta it spoils the whole frame: the good group beside it is not taken either.
    before = fleet.get('a')
    good = 'rapid' if group != 'rapid' else 'panel'
    applied = fleet.apply(
        frames.delta_frame('a', {good: _controller('a', 1.0)[good], group: value},
                           observed_at=frames.iso(1)), 2000, 2.0)
    assert applied.kind == 'ignored'
    assert f'{group} is not an object' in applied.problem
    assert applied.changed == {}
    assert fleet.get('a') is before
    assert fleet.get('a')[good]['asOf'] == frames.T0
    assert fleet.seen('a').observed_at == frames.T0


def test_a_null_identity_is_a_group_like_any_other():
    fleet = _fleet()
    applied = fleet.apply(frames.delta_frame('cell-1', {'identity': None},
                                             observed_at=frames.iso(1)), 2000, 2.0)
    assert applied == state.Applied('delta', changed={'cell-1': frozenset({'identity'})})
    assert fleet.get('cell-1')['identity'] is None
    # The answer knows the identity again: that is a value, so it is left for the stream.
    applied = fleet.confirm(frames.controller_list([_controller(seconds=2.0)],
                                                   observed_at=frames.iso(2)), 3000, 3.0)
    assert applied.confirmed == ()
    assert fleet.get('cell-1')['identity'] is None
    assert fleet.seen('cell-1').observed_at == frames.iso(1)


@pytest.mark.parametrize('change', [
    {'id': None},
    {'id': ''},
    {'id': 5},
    {'groups': None},
    {'groups': [['panel', {}]]},
    {'observedAt': None},
    {'observedAt': 'soon'},
])
def test_a_delta_that_is_not_shaped_as_one_is_refused(change):
    fleet = _fleet()
    before = fleet.get('cell-1')
    seen = fleet.seen('cell-1')
    frame = dict(frames.delta_frame('cell-1', {'panel': frames.panel_group(speed_ratio=1.0)},
                                    observed_at=frames.iso(1)), **change)
    applied = fleet.apply(frame, 2000, 2.0)
    assert applied.kind == 'ignored'
    assert applied.problem is not None
    assert applied.changed == {}
    assert fleet.get('cell-1') is before
    assert fleet.seen('cell-1') == seen
    assert fleet.incomplete() == ()


@pytest.mark.parametrize('controller_id', [None, '', 5, ['a']])
def test_a_removed_frame_without_an_id_is_refused(controller_id):
    fleet = _fleet()
    applied = fleet.apply(dict(frames.removed_frame(), id=controller_id), 2000, 2.0)
    assert applied.kind == 'ignored'
    assert applied.problem is not None
    assert fleet.ids() == ('cell-1',)


def test_a_problem_shows_an_id_on_one_line_and_not_at_full_length():
    fleet = _fleet()
    strange = 'bad\nid\x00' + 'x' * 500
    applied = fleet.apply(frames.delta_frame(strange, {'panel': frames.panel_group()}), 2000, 2.0)
    assert 'held back' in applied.problem
    assert '\n' not in applied.problem
    assert '\x00' not in applied.problem
    assert len(applied.problem) < 250
    assert fleet.incomplete() == (strange,)


def test_a_confirm_moves_the_stamps_of_a_robot_that_stands_still():
    fleet = _fleet()
    before = fleet.get('cell-1')
    answer = _controller(seconds=1.0, achieved_hz=7.6)
    applied = fleet.confirm(frames.controller_list([answer], observed_at=frames.iso(1.1)),
                            2000, 2.0)
    # The link group carries no stamp: it was confirmed as it is, nothing in it moved.
    assert applied == state.Applied(
        'confirm', changed={'cell-1': ALL - {'link'}}, confirmed=('cell-1',),
        motion_moved=('cell-1',))
    after = fleet.get('cell-1')
    assert after == answer
    assert _values(after) == _values(before)
    for name in ALL - {'link'}:
        assert after[name] is answer[name]
    for name in STAMPED:
        assert after[name]['asOf'] == frames.iso(1.0)
    assert after['link'] is before['link']
    assert after['io']['signals'][0]['asOf'] == frames.iso(1.0)
    assert after['motion']['rate']['achievedHz'] == 7.6
    assert fleet.seen('cell-1') == state.Seen(
        frames.iso(1.1), 2000, 2.0, frames.iso(1.0), 2000, 2.0)


def test_confirm_after_confirm_keeps_a_still_robot_fresh_until_the_daemon_stops_reading():
    fleet = _fleet()
    for step in range(1, 6):
        seconds = step * 0.5
        applied = fleet.confirm(
            frames.controller_list([_controller(seconds=seconds)],
                                   observed_at=frames.iso(seconds + 0.01)),
            1000 + step, 1.0 + seconds)
        assert applied.confirmed == ('cell-1',)
        assert applied.motion_moved == ('cell-1',)
        seen = fleet.seen('cell-1')
        assert seen.observed_at == frames.iso(seconds + 0.01)
        assert seen.motion_as_of == frames.iso(seconds)
        assert seen.motion_mono == 1.0 + seconds
        assert _joint(fleet.get('cell-1')) == 0.0

    # The daemon still answers, but motion.asOf no longer advances: confirmed, yet not moved.
    for step in range(1, 4):
        applied = fleet.confirm(
            frames.controller_list([_controller(seconds=2.5)], observed_at=frames.iso(3 + step)),
            2000 + step, 10.0 + step)
        assert applied.confirmed == ('cell-1',)
        assert applied.motion_moved == ()
        assert applied.changed == {}
        seen = fleet.seen('cell-1')
        assert seen.observed_at == frames.iso(3 + step)
        assert (seen.wall_ns, seen.mono) == (2000 + step, 10.0 + step)
        assert (seen.motion_as_of, seen.motion_wall_ns, seen.motion_mono) == (
            frames.iso(2.5), 1005, 3.5)


@pytest.mark.parametrize('group', state.GROUPS)
def test_a_confirm_never_changes_a_value(group):
    fleet = _fleet()
    before = fleet.get('cell-1')
    seen = fleet.seen('cell-1')
    answer = _with_change(_controller(seconds=1.0), group)
    answer['name'] = 'Another name'
    applied = fleet.confirm(frames.controller_list([answer], observed_at=frames.iso(1.1)),
                            2000, 2.0)
    after = fleet.get('cell-1')
    assert applied.kind == 'confirm'
    assert applied.problem is None
    assert _values(after) == _values(before)
    assert after['name'] == 'cell-1'
    # The group that differs is the stored one, stamp and all: it is left for the stream.
    assert after[group] is before[group]
    assert group not in applied.changed.get('cell-1', frozenset())
    assert applied.confirmed == ()
    assert fleet.seen('cell-1').observed_at == seen.observed_at
    assert (fleet.seen('cell-1').wall_ns, fleet.seen('cell-1').mono) == (1000, 1.0)
    # The groups that do agree took their new stamps.
    for other in ALL - {group, 'link'}:
        assert other in applied.changed['cell-1']
    for other in STAMPED - {group}:
        assert after[other]['asOf'] == frames.iso(1.0)
    assert (applied.motion_moved == ()) == (group == 'motion')


def test_a_moved_joint_is_left_for_the_stream():
    fleet = _fleet()
    answer = _controller(seconds=1.0, j6=5.0)
    applied = fleet.confirm(frames.controller_list([answer], observed_at=frames.iso(1.1)),
                            2000, 2.0)
    assert applied.motion_moved == ()
    assert applied.confirmed == ()
    assert _joint(fleet.get('cell-1')) == 0.0
    assert fleet.get('cell-1')['motion']['asOf'] == frames.T0
    assert fleet.seen('cell-1') == state.Seen(frames.T0, 1000, 1.0, frames.T0, 1000, 1.0)

    # The stream delivers the move; the next answer then confirms the new value.
    fleet.apply(frames.delta_frame('cell-1', {'motion': answer['motion']},
                                   observed_at=frames.iso(1.2)), 3000, 3.0)
    assert _joint(fleet.get('cell-1')) == 5.0
    applied = fleet.confirm(
        frames.controller_list([_controller(seconds=2.0, j6=5.0)], observed_at=frames.iso(2.1)),
        4000, 4.0)
    assert applied.confirmed == ('cell-1',)
    assert applied.motion_moved == ('cell-1',)
    assert _joint(fleet.get('cell-1')) == 5.0
    assert fleet.seen('cell-1') == state.Seen(
        frames.iso(2.1), 4000, 4.0, frames.iso(2.0), 4000, 4.0)


def test_an_older_answer_is_ignored():
    fleet = _fleet(observed_at=frames.iso(10))
    before = fleet.get('cell-1')
    seen = fleet.seen('cell-1')
    older = frames.controller_list([_controller(seconds=9.0)], observed_at=frames.iso(9.999))
    applied = fleet.confirm(older, 2000, 2.0)
    assert applied == state.Applied('confirm')
    assert fleet.get('cell-1') is before
    assert fleet.seen('cell-1') == seen

    # Not older: the same instant is taken.
    same = frames.controller_list([_controller(seconds=9.5)], observed_at=frames.iso(10))
    applied = fleet.confirm(same, 3000, 3.0)
    assert applied.confirmed == ('cell-1',)
    assert fleet.get('cell-1')['motion']['asOf'] == frames.iso(9.5)


def test_an_answer_is_older_or_not_per_controller():
    fleet = _fleet(_controller('a'), _controller('b'))
    fleet.apply(frames.delta_frame('a', {'panel': frames.panel_group(as_of=frames.iso(5))},
                                   observed_at=frames.iso(5)), 2000, 2.0)
    before = fleet.get('a')
    answer = frames.controller_list([_controller('a', 3.0), _controller('b', 3.0)],
                                    observed_at=frames.iso(3))
    applied = fleet.confirm(answer, 3000, 3.0)
    assert applied.confirmed == ('b',)
    assert applied.motion_moved == ('b',)
    assert set(applied.changed) == {'b'}
    assert fleet.get('a') is before
    assert fleet.seen('a').observed_at == frames.iso(5)
    assert fleet.seen('b').observed_at == frames.iso(3)


def test_an_older_answer_after_a_confirm_is_ignored_too():
    fleet = _fleet()
    fleet.confirm(frames.controller_list([_controller(seconds=5.0)], observed_at=frames.iso(5)),
                  2000, 2.0)
    applied = fleet.confirm(
        frames.controller_list([_controller(seconds=4.0)], observed_at=frames.iso(4)), 3000, 3.0)
    assert applied == state.Applied('confirm')
    assert fleet.get('cell-1')['motion']['asOf'] == frames.iso(5.0)
    assert fleet.seen('cell-1').observed_at == frames.iso(5)


def test_the_times_of_an_answer_are_compared_as_times_not_as_text():
    fleet = _fleet(observed_at='2026-01-01T00:00:10.5Z')
    newer = frames.controller_list([_controller(seconds=10.0)],
                                   observed_at='2026-01-01T00:00:10.500001Z')
    assert fleet.confirm(newer, 2000, 2.0).confirmed == ('cell-1',)
    # And the other way round: later as text, earlier as a time.
    fleet = _fleet(observed_at='2026-01-01T00:00:10.25Z')
    older = frames.controller_list([_controller(seconds=10.0)],
                                   observed_at='2026-01-01T00:00:10.2Z')
    assert '2026-01-01T00:00:10.2Z' > '2026-01-01T00:00:10.25Z'
    assert fleet.confirm(older, 2000, 2.0) == state.Applied('confirm')


def test_the_motion_rate_is_ignored_in_the_comparison():
    fleet = _fleet()
    answer = _controller(seconds=1.0, achieved_hz=6.5)
    answer['motion']['rate']['targetHz'] = 4.0
    applied = fleet.confirm(frames.controller_list([answer], observed_at=frames.iso(1.1)),
                            2000, 2.0)
    assert applied.confirmed == ('cell-1',)
    assert applied.motion_moved == ('cell-1',)
    assert fleet.get('cell-1')['motion']['rate'] == {
        'targetHz': 4.0, 'achievedHz': 6.5, 'via': 'poll'}


def test_a_rate_that_wobbles_alone_is_taken_without_moving_the_motion_stamp():
    fleet = _fleet()
    answer = _controller(seconds=0.0, achieved_hz=7.1)
    applied = fleet.confirm(frames.controller_list([answer], observed_at=frames.iso(1)),
                            2000, 2.0)
    assert applied == state.Applied(
        'confirm', changed={'cell-1': frozenset({'motion'})}, confirmed=('cell-1',))
    assert fleet.get('cell-1')['motion']['rate']['achievedHz'] == 7.1
    assert fleet.seen('cell-1') == state.Seen(frames.iso(1), 2000, 2.0, frames.T0, 1000, 1.0)


def test_an_answer_equal_to_the_state_confirms_it_without_replacing_anything():
    fleet = _fleet()
    before = fleet.get('cell-1')
    applied = fleet.confirm(frames.controller_list([_controller()], observed_at=frames.iso(2)),
                            2000, 2.0)
    assert applied == state.Applied('confirm', confirmed=('cell-1',))
    assert fleet.get('cell-1') is before
    assert fleet.seen('cell-1') == state.Seen(frames.iso(2), 2000, 2.0, frames.T0, 1000, 1.0)


@pytest.mark.parametrize('missing', state.GROUPS)
def test_a_controller_is_confirmed_only_when_every_group_was_taken(missing):
    fleet = _fleet()
    whole = _controller(seconds=1.0)
    partial = dict(whole)
    del partial[missing]
    applied = fleet.confirm(frames.controller_list([partial], observed_at=frames.iso(1.1)),
                            2000, 2.0)
    assert applied.confirmed == ()
    assert applied.changed == {'cell-1': ALL - {missing, 'link'}}
    assert (applied.motion_moved == ('cell-1',)) == (missing != 'motion')
    seen = fleet.seen('cell-1')
    assert (seen.observed_at, seen.wall_ns, seen.mono) == (frames.T0, 1000, 1.0)

    # All groups but one, taken again, still confirm nothing; the whole controller does.
    applied = fleet.confirm(frames.controller_list([partial], observed_at=frames.iso(1.2)),
                            2500, 2.5)
    assert applied.confirmed == ()
    applied = fleet.confirm(frames.controller_list([whole], observed_at=frames.iso(1.3)),
                            3000, 3.0)
    assert applied.confirmed == ('cell-1',)
    seen = fleet.seen('cell-1')
    assert (seen.observed_at, seen.wall_ns, seen.mono) == (frames.iso(1.3), 3000, 3.0)


def test_seen_follows_a_confirm_that_takes_motion_but_not_the_whole_controller():
    fleet = _fleet()
    answer = _with_change(_controller(seconds=1.0), 'panel')
    applied = fleet.confirm(frames.controller_list([answer], observed_at=frames.iso(1.1)),
                            2000, 2.0)
    assert applied.confirmed == ()
    assert applied.motion_moved == ('cell-1',)
    # The motion times are the answer's arrival; the controller's own stay the snapshot's.
    assert fleet.seen('cell-1') == state.Seen(frames.T0, 1000, 1.0, frames.iso(1.0), 2000, 2.0)


def test_a_confirm_adds_no_controller_and_removes_none():
    fleet = _fleet(_controller('a'), _controller('b'))
    answer = frames.controller_list([_controller('a', 1.0), _controller('zz', 1.0)],
                                    observed_at=frames.iso(1))
    applied = fleet.confirm(answer, 2000, 2.0)
    assert applied.confirmed == ('a',)
    assert applied.removed == ()
    assert set(applied.changed) == {'a'}
    assert fleet.ids() == ('a', 'b')
    assert fleet.get('zz') is None
    assert fleet.seen('b').observed_at == frames.T0
    assert fleet.incomplete() == ()


def test_a_controller_listed_twice_in_an_answer_is_confirmed_once():
    fleet = _fleet()
    first = _controller(seconds=1.0)
    second = _with_change(_controller(seconds=2.0), 'panel')
    applied = fleet.confirm(
        frames.controller_list([first, second], observed_at=frames.iso(2)), 2000, 2.0)
    assert applied.confirmed == ('cell-1',)
    assert applied.motion_moved == ('cell-1',)
    assert fleet.get('cell-1') == first
    assert fleet.seen('cell-1') == state.Seen(frames.iso(2), 2000, 2.0, frames.iso(1.0), 2000, 2.0)


def test_a_confirm_on_an_empty_state_does_nothing():
    fleet = state.FleetState(frames.SCHEMA_VERSION)
    applied = fleet.confirm(frames.controller_list([_controller()]), 1000, 1.0)
    assert applied == state.Applied('confirm')
    assert fleet.ids() == ()
    assert fleet.schema_version is None


@pytest.mark.parametrize('answer', [
    None,
    [],
    'controllers',
    {},
    {'schemaVersion': '0.12', 'controllers': []},
    {'schemaVersion': '0.12', 'observedAt': 'now', 'controllers': []},
    {'schemaVersion': '0.12', 'observedAt': None, 'controllers': []},
    {'schemaVersion': '0.12', 'observedAt': frames.iso(5)},
    {'schemaVersion': '0.12', 'observedAt': frames.iso(5), 'controllers': None},
    {'schemaVersion': '0.12', 'observedAt': frames.iso(5), 'controllers': {'cell-1': {}}},
    {'error': 'no', 'code': 'TOKEN_INVALID'},
])
def test_an_answer_that_is_not_a_controller_list_is_refused(answer):
    fleet = _fleet()
    before = fleet.get('cell-1')
    seen = fleet.seen('cell-1')
    applied = fleet.confirm(answer, 2000, 2.0)
    assert applied.kind == 'ignored'
    assert applied.problem is not None
    assert applied.confirmed == ()
    assert fleet.get('cell-1') is before
    assert fleet.seen('cell-1') == seen


@pytest.mark.parametrize('entry', [None, 'cell-1', 5, [], {}, {'id': None}, {'id': 5},
                                   {'id': 'cell-1'}, {'id': 'cell-1', 'motion': None},
                                   {'id': 'cell-1', 'motion': 'x', 'panel': []}])
def test_an_entry_that_is_no_controller_confirms_nothing(entry):
    fleet = _fleet()
    before = fleet.get('cell-1')
    seen = fleet.seen('cell-1')
    applied = fleet.confirm(
        {'schemaVersion': frames.SCHEMA_VERSION, 'observedAt': frames.iso(5),
         'controllers': [entry]}, 2000, 2.0)
    assert applied == state.Applied('confirm')
    assert fleet.get('cell-1') is before
    assert fleet.seen('cell-1') == seen


def test_an_answer_of_another_schema_version_than_the_stream_is_refused():
    fleet = _fleet()
    answer = frames.controller_list([_controller(seconds=1.0)], observed_at=frames.iso(1),
                                    schema_version=frames.NEWER_SCHEMA_VERSION)
    applied = fleet.confirm(answer, 2000, 2.0)
    assert applied.kind == 'ignored'
    assert 'schema' in applied.problem
    assert fleet.get('cell-1')['motion']['asOf'] == frames.T0
    assert fleet.seen('cell-1').observed_at == frames.T0


def test_a_delta_after_a_confirm_is_compared_afresh():
    fleet = _fleet()
    fleet.confirm(frames.controller_list([_controller(seconds=1.0)], observed_at=frames.iso(1)),
                  2000, 2.0)
    moved = _controller(seconds=2.0, j6=5.0)
    fleet.apply(frames.delta_frame('cell-1', {'motion': moved['motion']},
                                   observed_at=frames.iso(2)), 3000, 3.0)
    # An answer that still holds the old joint value cannot bring it back.
    stale = frames.controller_list([_controller(seconds=3.0, j6=0.0)], observed_at=frames.iso(3))
    applied = fleet.confirm(stale, 4000, 4.0)
    assert applied.confirmed == ()
    assert applied.motion_moved == ()
    assert _joint(fleet.get('cell-1')) == 5.0
    assert fleet.get('cell-1')['motion']['asOf'] == frames.iso(2.0)
    # One that agrees with the new value confirms it.
    fresh = frames.controller_list([_controller(seconds=3.5, j6=5.0)], observed_at=frames.iso(4))
    applied = fleet.confirm(fresh, 5000, 5.0)
    assert applied.confirmed == ('cell-1',)
    assert applied.motion_moved == ('cell-1',)
    assert _joint(fleet.get('cell-1')) == 5.0


def test_the_edge_controller_is_confirmed_like_any_other():
    fleet = _fleet(frames.edge_controller('edge-1'))
    answer = frames.edge_controller('edge-1')
    answer['motion']['asOf'] = frames.iso(3)
    answer['io']['signals'][0]['asOf'] = frames.iso(3)
    applied = fleet.confirm(frames.controller_list([answer], observed_at=frames.iso(3)),
                            2000, 2.0)
    assert applied == state.Applied(
        'confirm', changed={'edge-1': frozenset({'motion', 'io'})}, confirmed=('edge-1',),
        motion_moved=('edge-1',))
    stored = fleet.get('edge-1')
    assert stored == answer
    joints = stored['motion']['units'][0]['joints']
    # The joint without a unit is still the daemon's number, in no unit at all.
    assert (joints[3]['name'], joints[3]['value'], joints[3]['unit']) == ('J4', 5.0, None)


@pytest.mark.parametrize('seed', range(6))
def test_whatever_the_order_of_frames_and_answers_the_values_are_the_streams(seed):
    rng = random.Random(seed)
    every_id = ('a', 'b', 'c')
    # What a consumer that only applies the stream would hold, and when each was last touched.
    reference = {controller_id: _controller(controller_id) for controller_id in every_id}
    last_ms = dict.fromkeys(every_id, 0)
    fleet = _fleet(*copy.deepcopy(list(reference.values())))
    now_ms = 0
    confirmations = 0
    for number in range(1, 401):
        now_ms += rng.choice([40, 125, 250, 1000])
        stamp = frames.iso(now_ms / 1000)
        arrival = (number, float(number))
        action = rng.choice(['delta', 'delta', 'confirm', 'confirm', 'confirm', 'remove', 'add',
                             'unknown'])
        if action == 'delta':
            controller_id = rng.choice(sorted(reference))
            group = rng.choice(state.GROUPS)
            changed = _restamped(
                _with_fresh_value(copy.deepcopy(reference[controller_id]), group, number), stamp)
            applied = fleet.apply(
                frames.delta_frame(controller_id, {group: changed[group]}, observed_at=stamp),
                *arrival)
            assert applied.changed == {controller_id: frozenset({group})}
            reference[controller_id][group] = copy.deepcopy(changed[group])
            last_ms[controller_id] = now_ms
        elif action == 'remove' and len(reference) > 1:
            controller_id = rng.choice(sorted(reference))
            del reference[controller_id]
            applied = fleet.apply(frames.removed_frame(controller_id, observed_at=stamp), *arrival)
            assert applied.removed == (controller_id,)
        elif action == 'add' and len(reference) < len(every_id):
            controller_id = rng.choice(sorted(set(every_id) - set(reference)))
            reference[controller_id] = _controller(controller_id, now_ms / 1000)
            applied = fleet.apply(
                frames.delta_frame(controller_id, frames.groups_of(reference[controller_id]),
                                   observed_at=stamp), *arrival)
            assert applied.changed == {controller_id: ALL}
            last_ms[controller_id] = now_ms
        elif action == 'unknown':
            assert fleet.apply({'type': 'tick', 'observedAt': stamp}, *arrival).kind == 'ignored'
        elif action == 'confirm':
            # Some answers are late: built by the daemon three seconds before they arrive.
            answer_ms = now_ms - rng.choice([0, 0, 0, 3000])
            answer_stamp = frames.iso(answer_ms / 1000)
            entries = [_controller('stranger', now_ms / 1000)]
            expected = set()
            for controller_id in sorted(reference):
                if rng.random() < 0.15:
                    continue
                entry = _restamped(reference[controller_id], answer_stamp)
                entry['motion']['rate']['achievedHz'] = rng.choice([7.5, 7.9, 8.0])
                if rng.random() < 0.3:
                    _with_fresh_value(entry, rng.choice(state.GROUPS), number)
                elif answer_ms >= last_ms[controller_id]:
                    expected.add(controller_id)
                entries.append(entry)
            applied = fleet.confirm(
                frames.controller_list(entries, observed_at=answer_stamp), *arrival)
            assert set(applied.confirmed) == expected
            assert applied.removed == ()
            for controller_id in expected:
                last_ms[controller_id] = answer_ms
            confirmations += len(expected)
        assert fleet.ids() == tuple(sorted(reference))
        for controller_id, controller in reference.items():
            assert _values(fleet.get(controller_id)) == _values(controller)
            assert fleet.seen(controller_id).observed_at == (
                frames.iso(last_ms[controller_id] / 1000))
    assert confirmations > 20
    assert fleet.unknown_frames > 0


def test_comparable_leaves_out_every_as_of():
    group = frames.io_group([frames.signal('a', as_of=frames.iso(1)),
                             frames.signal('b', as_of=frames.iso(2))], as_of=frames.iso(3))
    text = state.comparable('io', group)
    assert isinstance(text, str)
    assert 'asOf' not in text
    assert json.loads(text) == {
        'watching': True,
        'signals': [
            {'name': 'a', 'type': 'DO', 'value': '0', 'via': 'subscription', 'error': None},
            {'name': 'b', 'type': 'DO', 'value': '0', 'via': 'subscription', 'error': None},
        ],
    }
    later = frames.io_group([frames.signal('a', as_of=frames.iso(7)),
                             frames.signal('b', as_of=None)], as_of=None)
    assert state.comparable('io', later) == text
    changed = frames.io_group([frames.signal('a', value='1', as_of=frames.iso(1)),
                               frames.signal('b', as_of=frames.iso(2))], as_of=frames.iso(3))
    assert state.comparable('io', changed) != text


def test_comparable_keeps_the_times_that_are_values():
    one = frames.events_group([frames.event(received_at=frames.iso(1))])
    other = frames.events_group([frames.event(received_at=frames.iso(2))])
    assert state.comparable('events', one) != state.comparable('events', other)
    assert state.comparable('link', frames.link_group(since=frames.iso(1))) != (
        state.comparable('link', frames.link_group(since=frames.iso(2))))
    clock = dict(CLOCK, measuredAt=frames.iso(9))
    assert state.comparable('events', frames.events_group(clock=clock)) != (
        state.comparable('events', frames.events_group(clock=dict(CLOCK))))


def test_comparable_leaves_out_the_rate_of_motion_and_of_nothing_else():
    slow = frames.motion_group(achieved_hz=3.0, target_hz=4.0, as_of=frames.iso(1))
    fast = frames.motion_group(achieved_hz=7.9, target_hz=8.0, as_of=frames.iso(2))
    assert state.comparable('motion', slow) == state.comparable('motion', fast)
    assert 'rate' not in json.loads(state.comparable('motion', slow))
    assert state.comparable('panel', slow) != state.comparable('panel', fast)
    assert 'rate' in json.loads(state.comparable('panel', slow))
    # Only the rate object of the group itself: a key of that name further down is a value.
    assert state.comparable('motion', {'units': [{'rate': 1}]}) != (
        state.comparable('motion', {'units': [{'rate': 2}]}))
    watched = frames.motion_group()
    unwatched = dict(frames.motion_group(), watching=False)
    assert state.comparable('motion', watched) != state.comparable('motion', unwatched)


def test_comparable_does_not_depend_on_key_order_and_changes_nothing():
    group = frames.panel_group(as_of=frames.iso(1))
    as_it_was = copy.deepcopy(group)
    backwards = dict(reversed(list(group.items())))
    assert list(backwards) != list(group)
    assert state.comparable('panel', backwards) == state.comparable('panel', group)
    assert group == as_it_was
    motion = frames.motion_group()
    as_it_was = copy.deepcopy(motion)
    state.comparable('motion', motion)
    assert motion == as_it_was


def test_comparable_of_a_null_identity():
    assert state.comparable('identity', None) == 'null'
    assert state.comparable('identity', frames.identity_group()) != 'null'


@pytest.mark.parametrize('group', state.GROUPS)
def test_comparable_sees_a_changed_value_in_every_group(group):
    plain = _controller(seconds=1.0)
    changed = _with_change(_controller(seconds=1.0), group)
    assert state.comparable(group, plain[group]) != state.comparable(group, changed[group])
    for other in ALL - {group}:
        assert state.comparable(other, plain[other]) == state.comparable(other, changed[other])


# ----- a joint stamp that nothing came before ------------------------------------------


def _limited(limit_s=2.0):
    """Return a state that takes a first joint stamp only when it is fresh."""
    return state.FleetState(frames.SCHEMA_VERSION, first_sight_max_age_s=limit_s)


@pytest.mark.parametrize('limit', [0, -1.0, float('inf'), float('nan'), '2', True])
def test_the_first_sight_limit_is_none_or_a_number_above_zero(limit):
    with pytest.raises(ValueError):
        state.FleetState(frames.SCHEMA_VERSION, first_sight_max_age_s=limit)


def test_the_first_sight_limit_is_kept():
    assert state.FleetState('0.12').first_sight_max_age_s is None
    assert _limited(1.5).first_sight_max_age_s == 1.5


def test_an_old_joint_stamp_in_the_first_snapshot_did_not_move_forward():
    # The daemon read the joints 60 s before it sent the snapshot: nothing says they
    # were read since, so they are not confirmed, whenever the snapshot arrives here.
    fleet = _limited(2.0)
    applied = fleet.apply(
        frames.snapshot_frame([_controller(seconds=0.0)], observed_at=frames.iso(60)),
        1000, 1.0)
    assert applied.problem is None
    assert applied.changed == {'cell-1': ALL}
    assert applied.motion_moved == ()
    assert fleet.seen('cell-1') == state.Seen(frames.iso(60), 1000, 1.0, frames.T0, None, None)

    # The same stamp again, in a frame and in an answer: still nothing moved.
    again = _controller(seconds=0.0)
    applied = fleet.apply(frames.delta_frame('cell-1', {'motion': again['motion']},
                                             observed_at=frames.iso(60.2)), 2000, 2.0)
    assert applied.motion_moved == ()
    applied = fleet.confirm(frames.controller_list([again], observed_at=frames.iso(61)),
                            3000, 3.0)
    assert applied.confirmed == ('cell-1',)
    assert applied.motion_moved == ()
    assert fleet.seen('cell-1').motion_mono is None

    # The next sample is later than the old stamp: that one counts, with its arrival.
    applied = fleet.confirm(
        frames.controller_list([_controller(seconds=61.4)], observed_at=frames.iso(61.5)),
        4000, 4.0)
    assert applied.motion_moved == ('cell-1',)
    assert fleet.seen('cell-1') == state.Seen(
        frames.iso(61.5), 4000, 4.0, frames.iso(61.4), 4000, 4.0)


@pytest.mark.parametrize('lag_s, fresh', [
    (0.0, True), (0.125, True), (1.999, True), (2.0, False), (2.5, False), (3600.0, False),
    # A stamp after its carrier is no fresher: the clock of the daemon stepped.
    (-0.5, True), (-2.0, False), (-3600.0, False),
])
def test_a_first_joint_stamp_counts_only_within_the_limit_of_its_carrier(lag_s, fresh):
    fleet = _limited(2.0)
    applied = fleet.apply(
        frames.snapshot_frame([_controller(seconds=100.0 - lag_s)],
                              observed_at=frames.iso(100)), 1000, 1.0)
    assert (applied.motion_moved == ('cell-1',)) == fresh
    seen = fleet.seen('cell-1')
    assert seen.motion_as_of == frames.iso(100.0 - lag_s)
    assert (seen.motion_wall_ns, seen.motion_mono) == ((1000, 1.0) if fresh else (None, None))


def test_a_controller_a_delta_adds_is_a_first_sight_too():
    fleet = _limited(2.0)
    fleet.apply(frames.snapshot_frame([_controller('a', 9.9)], observed_at=frames.iso(10)),
                1000, 1.0)
    old = _controller('b', 1.0)
    applied = fleet.apply(frames.delta_frame('b', frames.groups_of(old),
                                             observed_at=frames.iso(11)), 2000, 2.0)
    assert applied.changed == {'b': ALL}
    assert applied.motion_moved == ()
    assert fleet.seen('b') == state.Seen(frames.iso(11), 2000, 2.0, frames.iso(1.0), None, None)
    fresh = _controller('c', 11.9)
    applied = fleet.apply(frames.delta_frame('c', frames.groups_of(fresh),
                                             observed_at=frames.iso(12)), 3000, 3.0)
    assert applied.motion_moved == ('c',)
    assert fleet.seen('c').motion_mono == 3.0


def test_joints_read_for_the_first_time_count_only_when_the_stamp_is_fresh():
    idle = frames.controller('cell-1', motion=frames.motion_group(watching=False))
    for as_of_s, fresh in ((49.9, True), (10.0, False)):
        fleet = _limited(2.0)
        fleet.apply(frames.snapshot_frame([idle], observed_at=frames.iso(40)), 1000, 1.0)
        assert fleet.seen('cell-1').motion_as_of is None
        watched = frames.motion_group(as_of=frames.iso(as_of_s))
        applied = fleet.apply(frames.delta_frame('cell-1', {'motion': watched},
                                                 observed_at=frames.iso(50)), 2000, 2.0)
        assert (applied.motion_moved == ('cell-1',)) == fresh
        assert fleet.seen('cell-1').motion_mono == (2.0 if fresh else None)


def test_a_snapshot_on_a_connection_that_has_seen_the_controller_is_no_first_sight():
    # The stamp before it is known: later is later, whatever the age.
    fleet = _limited(2.0)
    fleet.apply(frames.snapshot_frame([_controller(seconds=9.9)], observed_at=frames.iso(10)),
                1000, 1.0)
    applied = fleet.apply(
        frames.snapshot_frame([_controller(seconds=12.0)], observed_at=frames.iso(60)),
        2000, 2.0)
    assert applied.motion_moved == ('cell-1',)
    assert fleet.seen('cell-1').motion_mono == 2.0
    # After clear() (the stream was lost) the controller is seen for the first time again.
    fleet.clear()
    applied = fleet.apply(
        frames.snapshot_frame([_controller(seconds=12.0)], observed_at=frames.iso(90)),
        3000, 3.0)
    assert applied.motion_moved == ()
    assert fleet.seen('cell-1').motion_mono is None


def test_without_a_limit_every_first_joint_stamp_counts():
    fleet = state.FleetState(frames.SCHEMA_VERSION)
    applied = fleet.apply(
        frames.snapshot_frame([_controller(seconds=0.0)], observed_at=frames.iso(3600)),
        1000, 1.0)
    assert applied.motion_moved == ('cell-1',)


# ----- what the stream owes ------------------------------------------------------------


def test_a_new_state_owes_nothing():
    fleet = _fleet()
    assert fleet.behind() == ()
    assert fleet.behind_since() is None


@pytest.mark.parametrize('group', state.GROUPS)
def test_a_list_that_shows_another_value_opens_a_debt_of_the_stream(group):
    fleet = _fleet()
    answer = _with_change(_controller(seconds=1.0), group)
    fleet.confirm(frames.controller_list([answer], observed_at=frames.iso(1.1)), 2000, 2.0)
    assert fleet.behind() == (('cell-1', group),)
    assert fleet.behind_since() == 2.0
    # It stays open, from the first answer that showed it, for as long as no frame comes.
    fleet.confirm(frames.controller_list([answer], observed_at=frames.iso(1.6)), 3000, 2.5)
    fleet.confirm(frames.controller_list([answer], observed_at=frames.iso(2.1)), 4000, 3.0)
    assert fleet.behind() == (('cell-1', group),)
    assert fleet.behind_since() == 2.0
    # No value was changed by any of it.
    assert _values(fleet.get('cell-1')) == _values(_controller())


def test_the_frame_that_delivers_the_value_settles_the_debt():
    fleet = _fleet()
    moved = _controller(seconds=1.0, j6=5.0)
    fleet.confirm(frames.controller_list([moved], observed_at=frames.iso(1.1)), 2000, 2.0)
    assert fleet.behind() == (('cell-1', 'motion'),)
    fleet.apply(frames.delta_frame('cell-1', {'motion': moved['motion']},
                                   observed_at=frames.iso(1.2)), 3000, 2.1)
    assert fleet.behind() == ()
    assert fleet.behind_since() is None


def test_any_frame_for_the_controller_settles_its_debt_and_no_other():
    fleet = _fleet(_controller('a'), _controller('b'))
    answer = [_with_change(_controller('a', 1.0), 'io'), _with_change(_controller('b', 1.0), 'io')]
    fleet.confirm(frames.controller_list(answer, observed_at=frames.iso(1.1)), 2000, 2.0)
    assert fleet.behind() == (('a', 'io'), ('b', 'io'))
    # The stream is alive for "a": whatever it sent, it is not stalled for it.
    fleet.apply(frames.delta_frame('a', {'panel': frames.panel_group(as_of=frames.iso(1.2))},
                                   observed_at=frames.iso(1.2)), 3000, 2.2)
    assert fleet.behind() == (('b', 'io'),)
    assert fleet.behind_since() == 2.0


def test_an_answer_that_agrees_again_settles_the_debt():
    fleet = _fleet()
    fleet.confirm(frames.controller_list([_with_change(_controller(seconds=1.0), 'panel')],
                                         observed_at=frames.iso(1.1)), 2000, 2.0)
    assert fleet.behind_since() == 2.0
    # The value went back before a frame was due: the list shows nothing else any more.
    applied = fleet.confirm(
        frames.controller_list([_controller(seconds=2.0)], observed_at=frames.iso(2.1)),
        3000, 3.0)
    assert applied.confirmed == ('cell-1',)
    assert fleet.behind() == ()


def test_an_answer_inside_the_window_of_the_daemon_opens_no_debt():
    # The list is ahead of the last frame by less than the window in which the daemon
    # sends at most one frame: the frame may simply be on its way.
    fleet = _fleet(observed_at=frames.iso(10))
    changed = _with_change(_controller(seconds=10.1), 'io')
    for lead_s in (0.0, 0.1, state.COALESCE_WINDOW_S):
        fleet.confirm(frames.controller_list([changed], observed_at=frames.iso(10 + lead_s)),
                      2000, 2.0)
        assert fleet.behind() == (), lead_s
    fleet.confirm(frames.controller_list([changed], observed_at=frames.iso(10.251)), 3000, 3.0)
    assert fleet.behind() == (('cell-1', 'io'),)
    assert fleet.behind_since() == 3.0


def test_the_window_is_measured_from_the_last_frame_or_the_last_full_confirm():
    fleet = _fleet()
    fleet.confirm(frames.controller_list([_controller(seconds=5.0)], observed_at=frames.iso(5)),
                  2000, 2.0)
    changed = _with_change(_controller(seconds=5.1), 'panel')
    fleet.confirm(frames.controller_list([changed], observed_at=frames.iso(5.2)), 3000, 2.2)
    assert fleet.behind() == ()
    fleet.confirm(frames.controller_list([changed], observed_at=frames.iso(5.7)), 4000, 2.7)
    assert fleet.behind_since() == 2.7


def test_an_older_answer_opens_no_debt():
    fleet = _fleet(observed_at=frames.iso(10))
    fleet.confirm(frames.controller_list([_with_change(_controller(seconds=9.0), 'panel')],
                                         observed_at=frames.iso(9.0)), 2000, 2.0)
    assert fleet.behind() == ()


def test_stamps_and_the_measured_rate_are_nothing_the_stream_owes():
    fleet = _fleet()
    answer = _controller(seconds=30.0, achieved_hz=2.5)
    applied = fleet.confirm(frames.controller_list([answer], observed_at=frames.iso(30.1)),
                            2000, 2.0)
    assert applied.confirmed == ('cell-1',)
    assert fleet.behind() == ()


def test_the_reconnect_bookkeeping_of_a_link_is_nothing_the_stream_owes():
    # The daemon counts its attempts to reach a controller that is down, and books the
    # next one, without a frame. The list then differs from the state for as long as
    # the controller stays down, and the stream is not stalled for that.
    down = frames.link_group(connected=False, quality='disconnected',
                             quality_reason='connection refused', subscriptions=False,
                             reconnect={'failures': 1, 'nextAttemptAt': frames.iso(5)})
    fleet = _fleet(frames.controller('cell-1', link=down))
    before = fleet.get('cell-1')
    for step in range(2, 6):
        retried = dict(down, reconnect={'failures': step, 'nextAttemptAt': frames.iso(5 * step)})
        answer = frames.controller('cell-1', link=retried)
        applied = fleet.confirm(
            frames.controller_list([answer], observed_at=frames.iso(step)), 1000 * step,
            1.0 * step)
        # Not taken (a confirm never changes a value), so not confirmed; and not owed.
        assert applied.confirmed == ()
        assert fleet.get('cell-1')['link'] is before['link']
        assert fleet.behind() == ()
    cleared = frames.controller('cell-1', link=dict(down, reconnect=None))
    fleet.confirm(frames.controller_list([cleared], observed_at=frames.iso(9)), 9000, 9.0)
    assert fleet.behind() == ()
    # Anything else in the link group is owed like every other value.
    up = frames.controller('cell-1', link=frames.link_group())
    fleet.confirm(frames.controller_list([up], observed_at=frames.iso(10)), 10000, 10.0)
    assert fleet.behind() == (('cell-1', 'link'),)


def test_owed_is_comparable_without_the_reconnect_object_of_a_link():
    link = frames.link_group(reconnect={'failures': 2, 'nextAttemptAt': frames.iso(5)})
    assert state.owed('link', link) == state.owed('link', frames.link_group())
    assert state.comparable('link', link) != state.comparable('link', frames.link_group())
    assert 'reconnect' not in json.loads(state.owed('link', link))
    assert state.owed('link', dict(link, quality='stale')) != state.owed('link', link)
    # Only the link group has such an object; a key of that name elsewhere is a value.
    assert state.owed('panel', {'reconnect': 1}) != state.owed('panel', {'reconnect': 2})
    for name in ALL - {'link'}:
        group = _controller()[name]
        assert state.owed(name, group) == state.comparable(name, group)


def test_a_controller_the_list_no_longer_holds_is_owed_its_removal():
    fleet = _fleet(_controller('a'), _controller('b'))
    only_a = frames.controller_list([_controller('a', 1.0)], observed_at=frames.iso(1.0))
    fleet.confirm(only_a, 2000, 2.0)
    assert fleet.behind() == (('b', state.NOT_LISTED),)
    assert fleet.behind_since() == 2.0
    assert fleet.get('b') is not None
    fleet.apply(frames.removed_frame('b', observed_at=frames.iso(1.1)), 3000, 2.1)
    assert fleet.behind() == ()


def test_a_controller_missing_from_a_list_inside_the_window_is_not_owed_yet():
    fleet = _fleet(_controller('a'), _controller('b'), observed_at=frames.iso(10))
    fleet.confirm(frames.controller_list([_controller('a', 10.1)], observed_at=frames.iso(10.2)),
                  2000, 2.0)
    assert fleet.behind() == ()


def test_a_controller_the_state_does_not_hold_is_no_debt():
    fleet = _fleet()
    answer = [_controller(seconds=1.0), _controller('cell-2', 1.0)]
    fleet.confirm(frames.controller_list(answer, observed_at=frames.iso(1.1)), 2000, 2.0)
    assert fleet.behind() == ()


def test_a_group_the_answer_lacks_or_misshapes_is_no_debt():
    fleet = _fleet()
    answer = _controller(seconds=1.0)
    del answer['io']
    answer['panel'] = 'not a group'
    applied = fleet.confirm(frames.controller_list([answer], observed_at=frames.iso(1.1)),
                            2000, 2.0)
    assert applied.confirmed == ()
    assert fleet.behind() == ()


def test_the_debt_names_what_the_last_answer_showed_and_keeps_its_first_time():
    fleet = _fleet()
    answer = _with_change(_controller(seconds=1.0), 'io')
    fleet.confirm(frames.controller_list([answer], observed_at=frames.iso(1.1)), 2000, 2.0)
    answer = _with_change(_with_change(_controller(seconds=2.0), 'io'), 'panel')
    fleet.confirm(frames.controller_list([answer], observed_at=frames.iso(2.1)), 3000, 3.0)
    assert fleet.behind() == (('cell-1', 'panel, io'),)
    assert fleet.behind_since() == 2.0


def test_a_snapshot_and_clear_forget_every_debt():
    for forget in ('snapshot', 'clear'):
        fleet = _fleet()
        fleet.confirm(frames.controller_list([_with_change(_controller(seconds=1.0), 'io')],
                                             observed_at=frames.iso(1.1)), 2000, 2.0)
        assert fleet.behind_since() == 2.0
        if forget == 'clear':
            fleet.clear()
        else:
            fleet.apply(frames.snapshot_frame([_controller(seconds=2.0)],
                                              observed_at=frames.iso(2.0)), 3000, 3.0)
        assert fleet.behind() == ()
        assert fleet.behind_since() is None


def test_a_healthy_stream_under_a_moving_robot_never_owes_for_long():
    # The robot moves every 0.4 s and every move is a frame; a list arrives 0.3 s after
    # each frame, newer than it by more than the window and showing the next position:
    # a debt opens every time, and the frame that follows settles it 0.1 s later.
    fleet = _fleet()
    opened = 0
    longest = 0.0
    for step in range(1, 41):
        at = step * 0.4
        moved = _controller(seconds=at, j6=float(step))
        fleet.apply(frames.delta_frame('cell-1', {'motion': moved['motion']},
                                       observed_at=frames.iso(at)), int(at * 1000), at)
        assert fleet.behind() == ()
        ahead = _controller(seconds=at + 0.3, j6=step + 0.5)
        fleet.confirm(frames.controller_list([ahead], observed_at=frames.iso(at + 0.3)),
                      int(at * 1000) + 300, at + 0.3)
        since = fleet.behind_since()
        if since is not None:
            opened += 1
            longest = max(longest, (at + 0.4) - since)
    assert opened == 40
    assert longest <= 0.1 + 1e-9

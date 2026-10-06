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
import math

from abb_rws_bridge import joint_state
from abb_rws_bridge import names
from abb_rws_bridge.joint_state import Skipped
import frames
import pytest

DEFAULT = names.JointNamer('{unit}_{joint}', {})


def build(motion, namer=DEFAULT, controller_id='cell-1', token='cell_1'):
    return joint_state.build(controller_id, token, motion, namer)


def test_six_joints_in_degrees_become_radians():
    degrees = [10.0, -20.5, 30.25, 0.0, 45.0, 90.0]
    unit = frames.motion_unit('ROB_1', [
        frames.joint(f'J{axis}', value) for axis, value in enumerate(degrees, start=1)])
    data = build(frames.motion_group([unit], as_of=frames.iso(1.5)))
    assert data.stamp == frames.iso(1.5)
    assert data.names == tuple(f'ROB_1_J{axis}' for axis in range(1, 7))
    assert len(data.positions) == 6
    for position, value in zip(data.positions, degrees):
        assert abs(position - value * math.pi / 180) <= 1e-12
    assert data.skipped == ()
    assert data.problem is None


def test_the_default_motion_group():
    data = build(frames.controller()['motion'])
    assert data.stamp == frames.T0
    assert data.names == ('ROB_1_J1', 'ROB_1_J2', 'ROB_1_J3', 'ROB_1_J4', 'ROB_1_J5',
                          'ROB_1_J6')
    assert data.positions == (0.0,) * 6
    assert all(isinstance(position, float) for position in data.positions)
    assert data.problem is None


def test_no_as_of_gives_none():
    assert build(frames.motion_group(watching=False)) is None
    assert build(frames.motion_group(as_of=None)) is None
    assert build(None) is None
    assert build('motion') is None


def test_values_that_are_no_longer_watched_are_still_built():
    # Whether they are current is decided elsewhere; here asOf alone decides.
    data = build(frames.motion_group([frames.motion_unit()], watching=False, as_of=frames.T0))
    assert data is not None
    assert len(data.names) == 6
    assert data.problem is None


def test_the_edge_controller():
    edge = frames.edge_controller()
    before = copy.deepcopy(edge)
    data = build(edge['motion'], controller_id='edge-1', token='edge_1')
    assert edge == before
    assert data.stamp == frames.iso(0.5)
    assert data.names == ('ROB_1_J1', 'ROB_1_J2', 'ROB_1_J5', 'ROB_1_J6', 'ROB_1_E1')
    expected = (10.0 * math.pi / 180, -20.5 * math.pi / 180, 45.0 * math.pi / 180,
                90.0 * math.pi / 180, 1.25)
    assert len(data.positions) == len(expected)
    for position, wanted in zip(data.positions, expected):
        assert abs(position - wanted) <= 1e-12
    assert data.skipped == (
        Skipped('ROB_1', 'J3', 'no-value'),
        Skipped('ROB_1', 'J4', 'unit-unknown'),
        Skipped('STN_1', 'J1', 'unit-error'),
    )
    assert data.problem is None


def test_a_null_unit_joint_is_never_in_the_output_whatever_its_value():
    for value in (0.0, 5.0, -3.14, 90.0, 1250.0, None):
        unit = frames.motion_unit('ROB_1', [
            frames.joint('J1', 1.0), frames.joint('J2', value, None, None, None)])
        data = build(frames.motion_group([unit]))
        assert data.names == ('ROB_1_J1',)
        assert data.positions == (math.radians(1.0),)
        assert data.skipped == (Skipped('ROB_1', 'J2', 'unit-unknown'),)
        assert data.problem is None


@pytest.mark.parametrize('unit_text', ['rad', 'm', 'DEG', 'degrees', '', 'cm', 5, True])
def test_an_unsupported_unit_is_left_out(unit_text):
    unit = frames.motion_unit('ROB_1', [
        frames.joint('J1', 1.0), frames.joint('J2', 2.0, unit_text)])
    data = build(frames.motion_group([unit]))
    assert data.names == ('ROB_1_J1',)
    assert data.skipped == (Skipped('ROB_1', 'J2', 'unit-unsupported'),)


@pytest.mark.parametrize('value', [None, float('nan'), float('inf'), float('-inf'), '5.0',
                                   True, 10 ** 400])
@pytest.mark.parametrize('unit_text', ['deg', 'mm'])
def test_a_missing_or_not_finite_value_is_left_out(value, unit_text):
    unit = frames.motion_unit('ROB_1', [
        frames.joint('J1', value, unit_text), frames.joint('J2', 2.0, unit_text)])
    data = build(frames.motion_group([unit]))
    assert data.names == ('ROB_1_J2',)
    assert data.skipped == (Skipped('ROB_1', 'J1', 'no-value'),)
    assert data.problem is None


def test_millimetres_become_metres():
    unit = frames.motion_unit('TRACK', [frames.joint('E1', 1250.0, 'mm', 0.0, 5000.0),
                                        frames.joint('E2', -0.5, 'mm')], kind='external')
    data = build(frames.motion_group([unit]))
    assert data.names == ('TRACK_E1', 'TRACK_E2')
    assert data.positions == (1.25, -0.0005)


def test_a_unit_with_an_error_is_left_out_whole():
    broken = frames.motion_unit(
        'STN_1', [frames.joint('J1', 30.0), frames.joint('J2', None),
                  frames.joint('J3', 1.0, None)],
        error=frames.group_error('CONTROLLER_ERROR', 'the unit could not be read'))
    data = build(frames.motion_group([frames.motion_unit('ROB_1', [frames.joint('J1', 90.0)]),
                                      broken]))
    assert data.names == ('ROB_1_J1',)
    assert data.skipped == (
        Skipped('STN_1', 'J1', 'unit-error'),
        Skipped('STN_1', 'J2', 'unit-error'),
        Skipped('STN_1', 'J3', 'unit-error'),
    )
    assert data.problem is None


def test_units_and_joints_keep_the_order_of_the_daemon():
    motion = frames.motion_group([
        frames.motion_unit('ROB_2', [frames.joint('J2', 2.0), frames.joint('J1', 1.0)]),
        frames.motion_unit('ROB_1', [frames.joint('J6', 6.0), frames.joint('E1', 7.0, 'mm')]),
    ])
    data = build(motion)
    assert data.names == ('ROB_2_J2', 'ROB_2_J1', 'ROB_1_J6', 'ROB_1_E1')
    assert data.positions == (math.radians(2.0), math.radians(1.0), math.radians(6.0), 0.007)


def test_no_joint_left_publishes_nothing():
    unit = frames.motion_unit('ROB_1', [frames.joint('J1', None),
                                        frames.joint('J2', 1.0, None)])
    data = build(frames.motion_group([unit]))
    assert data.names == ()
    assert data.positions == ()
    assert data.problem == joint_state.NO_JOINT
    assert data.skipped == (Skipped('ROB_1', 'J1', 'no-value'),
                            Skipped('ROB_1', 'J2', 'unit-unknown'))

    empty = build(frames.motion_group([], as_of=frames.T0))
    assert empty.names == ()
    assert empty.skipped == ()
    assert empty.problem == joint_state.NO_JOINT

    no_joints = build(frames.motion_group([frames.motion_unit('ROB_1', [])]))
    assert no_joints.problem == joint_state.NO_JOINT


def test_a_duplicate_name_is_a_problem_that_names_the_joints():
    unit = frames.motion_unit('ROB_1', [frames.joint('J1', 1.0), frames.joint('J2', 2.0),
                                        frames.joint('E1', 3.0, 'mm')])
    data = build(frames.motion_group([unit]), names.JointNamer('joint_{axis}', {}))
    assert data.problem is not None
    assert data.problem != joint_state.NO_JOINT
    assert '"joint_1"' in data.problem
    assert 'ROB_1/J1' in data.problem
    assert 'ROB_1/E1' in data.problem
    assert 'ROB_1/J2' not in data.problem
    # Nothing is renamed: the names are what the format gave.
    assert data.names == ('joint_1', 'joint_2', 'joint_1')


def test_the_same_joint_name_on_two_units_is_a_duplicate_with_a_format_without_unit():
    motion = frames.motion_group([frames.motion_unit('ROB_1', [frames.joint('J1', 1.0)]),
                                  frames.motion_unit('ROB_2', [frames.joint('J1', 2.0)])])
    assert build(motion).problem is None
    data = build(motion, names.JointNamer('{joint}', {}))
    assert 'ROB_1/J1' in data.problem and 'ROB_2/J1' in data.problem


def test_a_duplicate_with_a_joint_that_is_left_out_is_no_duplicate():
    unit = frames.motion_unit('ROB_1', [frames.joint('J1', 1.0),
                                        frames.joint('E1', 3.0, None)])
    data = build(frames.motion_group([unit]), names.JointNamer('joint_{axis}', {}))
    assert data.problem is None
    assert data.names == ('joint_1',)


def test_an_override_can_cause_and_cure_a_duplicate():
    unit = frames.motion_unit('ROB_1', [frames.joint('J1', 1.0), frames.joint('E1', 3.0, 'mm')])
    motion = frames.motion_group([unit])
    cured = names.JointNamer('joint_{axis}', {('cell-1', 'ROB_1', 'E1'): 'track'})
    assert build(motion, cured).names == ('joint_1', 'track')
    assert build(motion, cured).problem is None
    caused = names.JointNamer('{unit}_{joint}', {('cell-1', 'ROB_1', 'E1'): 'ROB_1_J1'})
    assert '"ROB_1_J1"' in build(motion, caused).problem
    # The override belongs to cell-1 only.
    assert build(motion, caused, controller_id='cell-2', token='cell_2').problem is None


def test_the_controller_token_reaches_the_namer():
    namer = names.JointNamer('{controller}_{unit}_{joint}', {})
    motion = frames.motion_group([frames.motion_unit('ROB_1', [frames.joint('J1', 1.0)])])
    assert build(motion, namer, token='cell_1').names == ('cell_1_ROB_1_J1',)
    assert build(motion, namer, controller_id='x', token='other').names == ('other_ROB_1_J1',)


def test_a_namer_that_cannot_name_a_joint_is_a_problem():
    unit = frames.motion_unit('ROB_1', [frames.joint('J1', 1.0), frames.joint('JX', 2.0)])
    data = build(frames.motion_group([unit]), names.JointNamer('joint_{axis}', {}))
    assert data.problem is not None
    assert data.problem != joint_state.NO_JOINT
    assert '{axis}' in data.problem

    class EmptyNamer:

        def name(self, controller_id, controller_token, unit, joint):
            return ''

    data = build(frames.motion_group([unit]), EmptyNamer())
    assert 'empty name' in data.problem
    assert data.problem != joint_state.NO_JOINT


def test_velocity_and_effort_are_not_part_of_the_data():
    fields = {field.name for field in dataclasses.fields(joint_state.JointStateData)}
    assert fields == {'stamp', 'names', 'positions', 'skipped', 'problem'}
    assert {field.name for field in dataclasses.fields(Skipped)} == {'unit', 'joint', 'reason'}


def test_the_data_cannot_be_changed():
    data = build(frames.controller()['motion'])
    with pytest.raises(dataclasses.FrozenInstanceError):
        data.problem = 'x'
    assert isinstance(data.names, tuple)
    assert isinstance(data.positions, tuple)
    assert isinstance(data.skipped, tuple)


def test_the_motion_group_is_not_changed():
    motion = frames.edge_controller()['motion']
    before = copy.deepcopy(motion)
    build(motion)
    assert motion == before


def test_shapes_that_are_not_a_motion_group_do_not_raise():
    assert build({}) is None
    assert build({'asOf': frames.T0}).problem == joint_state.NO_JOINT
    assert build({'asOf': frames.T0, 'units': None}).problem == joint_state.NO_JOINT
    odd_units = build({'asOf': frames.T0, 'units': [None, 5, 'x', {}]})
    assert odd_units.problem == joint_state.NO_JOINT
    nameless = build({'asOf': frames.T0, 'units': [{'name': 'ROB_1', 'joints': [None, {}]}]})
    assert nameless.problem is not None
    assert nameless.problem != joint_state.NO_JOINT
    data = build({'asOf': 12345, 'units': []})
    assert data.problem is not None and data.names == ()
    assert data.problem != joint_state.NO_JOINT


def test_the_reasons_are_the_documented_ones():
    assert joint_state.UNIT_UNKNOWN == 'unit-unknown'
    assert joint_state.UNIT_UNSUPPORTED == 'unit-unsupported'
    assert joint_state.NO_VALUE == 'no-value'
    assert joint_state.UNIT_ERROR == 'unit-error'

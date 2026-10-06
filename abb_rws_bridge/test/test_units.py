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

import ast
import math
from pathlib import Path
import sys

from abb_rws_bridge import units
import frames
import pytest

DEGREES = (0.0, 1.0, -1.0, 10.0, -20.5, 45.0, 90.0, 180.0, -180.0, 359.999, 400.0, -400.0,
           1e-9, 123456.789)
# The modules that hold names and values: no ROS, no socket, nothing but each other.
PURE_MODULES = ('units', 'names', 'joint_state', 'io_values', 'health', 'parameters')


def _imports(path):
    for node in ast.walk(ast.parse(path.read_text(encoding='utf-8'))):
        if isinstance(node, ast.Import):
            yield from (alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            assert node.level == 0, f'{path.name}: relative import'
            if node.module == 'abb_rws_bridge':
                yield from (f'abb_rws_bridge.{alias.name}' for alias in node.names)
            else:
                yield node.module


@pytest.mark.parametrize('module', PURE_MODULES)
def test_the_module_imports_only_the_standard_library_and_its_own_kind(module):
    path = Path(units.__file__).resolve().parent / f'{module}.py'
    imported = sorted(set(_imports(path)))
    for name in imported:
        parts = name.split('.')
        if parts[0] == 'abb_rws_bridge':
            assert len(parts) > 1 and parts[1] in PURE_MODULES, f'{module} imports {name}'
        else:
            assert parts[0] in sys.stdlib_module_names, f'{module} imports {name}'
    assert 'rclpy' not in imported


@pytest.mark.parametrize('deg', DEGREES)
def test_degrees_become_radians(deg):
    assert abs(units.deg_to_rad(deg) - deg * math.pi / 180) <= 1e-12
    assert abs(units.to_si(deg, 'deg') - deg * math.pi / 180) <= 1e-12


def test_known_angles():
    assert units.deg_to_rad(180.0) == pytest.approx(math.pi, abs=1e-15)
    assert units.deg_to_rad(90.0) == pytest.approx(math.pi / 2, abs=1e-15)
    assert units.deg_to_rad(0.0) == 0.0


@pytest.mark.parametrize('mm, metres', [
    (0.0, 0.0), (1.0, 0.001), (1250.0, 1.25), (-12.25, -0.01225), (5000.0, 5.0),
])
def test_millimetres_become_metres(mm, metres):
    assert units.mm_to_m(mm) == pytest.approx(metres, abs=1e-15)
    assert units.to_si(mm, 'mm') == pytest.approx(metres, abs=1e-15)


def test_a_whole_number_is_a_number():
    assert units.to_si(180, 'deg') == pytest.approx(math.pi, abs=1e-15)
    assert units.to_si(1000, 'mm') == 1.0
    assert isinstance(units.to_si(180, 'deg'), float)


@pytest.mark.parametrize('unit', [
    None, '', 'rad', 'm', 'DEG', 'Deg', 'MM', 'deg ', ' mm', 'degrees', 'cm', 'unknown',
    0, 1, True, False, ('deg',), ['mm'], b'deg',
])
def test_a_null_or_unknown_unit_is_never_converted(unit):
    for value in (0.0, 5.0, -90.0, 1250.0):
        assert units.to_si(value, unit) is None


@pytest.mark.parametrize('value', [
    None, float('nan'), float('inf'), float('-inf'), '5.0', '5', '', True, False, [5.0],
    (5.0,), {'value': 5.0}, 10 ** 400, -(10 ** 400), 5 + 0j,
])
def test_a_value_that_is_not_a_finite_number_gives_none(value):
    assert units.to_si(value, 'deg') is None
    assert units.to_si(value, 'mm') is None
    assert units.to_si(value, None) is None


def test_the_null_unit_joint_of_the_edge_controller_stays_as_it_is():
    robot = frames.EDGE_CONTROLLER['motion']['units'][0]
    by_name = {joint['name']: joint for joint in robot['joints']}
    before = dict(by_name['J4'])
    assert before['unit'] is None and before['value'] == 5.0
    assert units.to_si(by_name['J4']['value'], by_name['J4']['unit']) is None
    assert by_name['J4'] == before
    assert units.to_si(by_name['J3']['value'], by_name['J3']['unit']) is None
    assert units.to_si(by_name['E1']['value'], by_name['E1']['unit']) == 1.25
    assert units.to_si(by_name['J6']['value'], by_name['J6']['unit']) == pytest.approx(
        math.pi / 2, abs=1e-15)


def test_every_joint_of_the_edge_controller_converts_or_gives_none():
    for unit in frames.EDGE_CONTROLLER['motion']['units']:
        for joint in unit['joints']:
            converted = units.to_si(joint['value'], joint['unit'])
            if joint['unit'] == 'deg' and joint['value'] is not None:
                assert abs(converted - joint['value'] * math.pi / 180) <= 1e-12
            elif joint['unit'] == 'mm' and joint['value'] is not None:
                assert converted == joint['value'] / 1000.0
            else:
                assert converted is None

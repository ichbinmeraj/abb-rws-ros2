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

import re

from abb_rws_bridge import names
import frames
import pytest

TOKEN = re.compile(r'[A-Za-z_][A-Za-z0-9_]*')


@pytest.mark.parametrize('text, token', [
    ('cell-1', 'cell_1'),
    ('a--b', 'a_b'),
    ('a__b', 'a_b'),
    ('a-_-b', 'a_b'),
    ('x-', 'x'),
    ('x_', 'x'),
    ('x---', 'x'),
    ('1cell', '_1cell'),
    ('9', '_9'),
    ('1-', '_1'),
    ('-x', '_x'),
    ('_x', '_x'),
    ('__x', '_x'),
    ('-1', '_1'),
    ('ROB_1', 'ROB_1'),
    ('Cell', 'Cell'),
    ('a b', 'a_b'),
    ('a.b', 'a_b'),
    ('a/b', 'a_b'),
    ('do_gripper', 'do_gripper'),
    ('caf\u00e9', 'caf'),
    ('\u00fcnit', '_nit'),
])
def test_ros_token(text, token):
    assert names.ros_token(text) == token


@pytest.mark.parametrize('text', ['', '-', '--', '_', '___', ' ', '/', '-_-', '\u00e9\u00e9'])
def test_ros_token_with_nothing_left_is_an_error(text):
    with pytest.raises(ValueError):
        names.ros_token(text)


@pytest.mark.parametrize('text', [None, 1, b'cell', ['cell']])
def test_ros_token_needs_text(text):
    with pytest.raises(ValueError):
        names.ros_token(text)


@pytest.mark.parametrize('text', [
    'cell-1', 'a--b', 'x-', '1cell', '-x', 'a b c', 'Local/DRV_1/go_program', '0', 'a__',
    '__a__b__', 'x.y.z', '9-9',
])
def test_a_token_is_always_a_legal_topic_name_token(text):
    token = names.ros_token(text)
    assert TOKEN.fullmatch(token)
    assert '__' not in token
    assert not token.endswith('_')
    assert names.ros_token(token) == token


def test_reserved_tokens():
    assert names.REPLAY_TOKEN == 'replay'
    assert names.RESERVED_TOKENS == frozenset(
        {'joint_states', 'controller_snapshot', 'io', 'diagnostics', 'replay'})
    assert isinstance(names.RESERVED_TOKENS, frozenset)


def test_controller_tokens_plain():
    tokens, problems = names.controller_tokens(['cell-2', 'cell-1', 'irb1600'])
    assert tokens == {'cell-1': 'cell_1', 'cell-2': 'cell_2', 'irb1600': 'irb1600'}
    assert problems == {}


def test_the_primary_controller_has_the_empty_token():
    tokens, problems = names.controller_tokens(['cell-1', 'cell-2'], primary='cell-1')
    assert tokens == {'cell-1': '', 'cell-2': 'cell_2'}
    assert problems == {}


def test_a_primary_that_is_not_served_changes_nothing():
    tokens, problems = names.controller_tokens(['cell-1'], primary='cell-9')
    assert tokens == {'cell-1': 'cell_1'}
    assert problems == {}


def test_no_controllers():
    assert names.controller_tokens([]) == ({}, {})
    assert names.controller_tokens([], primary='cell-1') == ({}, {})


def test_a_token_collision_refuses_the_later_id():
    tokens, problems = names.controller_tokens(['a_b', 'a-b', 'a--b'])
    # Sorted: 'a--b', 'a-b', 'a_b'. The first keeps the token, the others get none.
    assert tokens == {'a--b': 'a_b'}
    assert set(problems) == {'a-b', 'a_b'}
    for problem in problems.values():
        assert '"a_b"' in problem
        assert 'controller "a--b"' in problem


def test_a_collision_does_not_depend_on_the_order_given():
    first = names.controller_tokens(['cell-1', 'cell_1', 'cell.1'])
    second = names.controller_tokens(['cell.1', 'cell_1', 'cell-1'])
    third = names.controller_tokens(iter(['cell_1', 'cell-1', 'cell.1', 'cell-1']))
    assert first == second == third
    tokens, problems = first
    assert tokens == {'cell-1': 'cell_1'}
    assert set(problems) == {'cell.1', 'cell_1'}


def test_no_suffix_is_invented():
    tokens, _ = names.controller_tokens(['a-b', 'a_b', 'a.b'])
    assert sorted(tokens.values()) == ['a_b']


@pytest.mark.parametrize('reserved', sorted(names.RESERVED_TOKENS))
def test_a_reserved_token_is_refused(reserved):
    tokens, problems = names.controller_tokens([reserved, 'cell-1'])
    assert tokens == {'cell-1': 'cell_1'}
    assert set(problems) == {reserved}
    assert f'"{reserved}"' in problems[reserved]


def test_an_id_that_becomes_a_reserved_token_is_refused():
    tokens, problems = names.controller_tokens(['joint-states', 'io-', 'replay_'])
    assert tokens == {}
    assert set(problems) == {'joint-states', 'io-', 'replay_'}


def test_the_primary_takes_no_token_from_another_controller():
    tokens, problems = names.controller_tokens(['cell-1', 'cell_1'], primary='cell-1')
    assert tokens == {'cell-1': '', 'cell_1': 'cell_1'}
    assert problems == {}


def test_a_reserved_id_may_be_the_primary():
    # The primary's topics carry no token of its own, so its id cannot shadow a name.
    tokens, problems = names.controller_tokens(['replay', 'io'], primary='replay')
    assert tokens == {'replay': ''}
    assert set(problems) == {'io'}


def test_an_id_with_nothing_left_has_a_problem():
    tokens, problems = names.controller_tokens(['---', 'cell-1', ''])
    assert tokens == {'cell-1': 'cell_1'}
    assert set(problems) == {'---', ''}


def test_ids_that_are_not_text_are_left_out():
    tokens, problems = names.controller_tokens(['cell-1', None, 7])
    assert tokens == {'cell-1': 'cell_1'}
    assert problems == {}


@pytest.mark.parametrize('signal, token', [
    ('do_gripper', 'do_gripper'),
    ('Local/DRV_1/go_program', 'Local_DRV_1_go_program'),
    ('network/device/signal', 'network_device_signal'),
    ('a//b', 'a_b'),
    ('1st', '_1st'),
    ('diValve-2', 'diValve_2'),
])
def test_io_topic_token(signal, token):
    assert names.io_topic_token(signal) == token


@pytest.mark.parametrize('signal', ['', '/', '//', '-', None, 5])
def test_io_topic_token_with_nothing_left_is_an_error(signal):
    with pytest.raises(ValueError):
        names.io_topic_token(signal)


def test_every_signal_name_of_the_edge_controller_has_a_token():
    tokens = [names.io_topic_token(signal['name'])
              for signal in frames.EDGE_CONTROLLER['io']['signals']]
    assert len(set(tokens)) == len(tokens)
    assert 'Local_DRV_1_go_program' in tokens
    assert all(TOKEN.fullmatch(token) for token in tokens)


def test_parse_io_signals():
    signals, problems = names.parse_io_signals([
        'cell-1:do_gripper', 'cell-2:di_ready', 'cell-1:Local/DRV_1/go_program',
        ' cell-1 : ai_pressure ',
    ])
    assert problems == []
    assert signals == {
        'cell-1': ('do_gripper', 'Local/DRV_1/go_program', 'ai_pressure'),
        'cell-2': ('di_ready',),
    }
    assert all(isinstance(value, tuple) for value in signals.values())


@pytest.mark.parametrize('entries', [None, [], ()])
def test_parse_io_signals_empty(entries):
    assert names.parse_io_signals(entries) == ({}, [])


@pytest.mark.parametrize('entry', [
    '', 'do_gripper', 'cell-1:', ':do_gripper', ':', 'cell-1 do_gripper', ' : ',
])
def test_an_io_entry_of_another_form_is_a_problem(entry):
    signals, problems = names.parse_io_signals(['cell-1:di_ready', entry])
    assert signals == {'cell-1': ('di_ready',)}
    assert len(problems) == 1
    assert '<controller id>:<signal>' in problems[0]
    assert f'"{entry}"' in problems[0]


@pytest.mark.parametrize('entry', [
    'cell/1:do_x', 'a b:do_x', '../admin:do_x', '-cell:do_x', 'cell=1:do_x', 'c\u00e9ll:do_x',
])
def test_an_io_entry_with_an_impossible_controller_id_is_a_problem(entry):
    signals, problems = names.parse_io_signals([entry])
    assert signals == {}
    assert len(problems) == 1
    assert 'cannot be a controller id' in problems[0]


def test_an_io_signal_with_nothing_for_a_topic_name_is_a_problem():
    signals, problems = names.parse_io_signals(['cell-1:---', 'cell-1:/'])
    assert signals == {}
    assert len(problems) == 2


def test_an_io_entry_listed_twice_is_a_problem():
    signals, problems = names.parse_io_signals(
        ['cell-1:do_gripper', 'cell-2:do_gripper', 'cell-1:do_gripper'])
    assert signals == {'cell-1': ('do_gripper',), 'cell-2': ('do_gripper',)}
    assert len(problems) == 1
    assert 'listed twice' in problems[0]


def test_an_io_topic_collision_refuses_the_later_signal():
    signals, problems = names.parse_io_signals([
        'cell-1:Local/DRV_1/do_x', 'cell-1:Local_DRV_1_do_x', 'cell-2:Local_DRV_1_do_x',
        'cell-1:do-y', 'cell-1:do_y',
    ])
    assert signals == {
        'cell-1': ('Local/DRV_1/do_x', 'do-y'),
        'cell-2': ('Local_DRV_1_do_x',),
    }
    assert len(problems) == 2
    assert '"io/Local_DRV_1_do_x"' in problems[0]
    assert '"Local/DRV_1/do_x"' in problems[0]
    assert '"cell-1:Local_DRV_1_do_x"' in problems[0]
    assert '"io/do_y"' in problems[1]


def test_io_entries_that_are_not_a_list_of_text():
    assert names.parse_io_signals('cell-1:do_x') == ({}, [
        'a list of entries is needed, not the single text "cell-1:do_x"'])
    signals, problems = names.parse_io_signals(['cell-1:do_x', 5, None])
    assert signals == {'cell-1': ('do_x',)}
    assert len(problems) == 2


def test_parse_joint_overrides():
    overrides, problems = names.parse_joint_overrides([
        'cell-1/ROB_1/J1=joint_1', 'cell-1/ROB_1/J2 = joint_2', ' cell-2/STN_1/E1=table ',
        'cell-1/ROB_1/J3=a=b',
    ])
    assert problems == []
    assert overrides == {
        ('cell-1', 'ROB_1', 'J1'): 'joint_1',
        ('cell-1', 'ROB_1', 'J2'): 'joint_2',
        ('cell-2', 'STN_1', 'E1'): 'table',
        ('cell-1', 'ROB_1', 'J3'): 'a=b',
    }


@pytest.mark.parametrize('entries', [None, [], ()])
def test_parse_joint_overrides_empty(entries):
    assert names.parse_joint_overrides(entries) == ({}, [])


@pytest.mark.parametrize('entry', [
    '', 'joint_1', 'cell-1/ROB_1/J1', 'cell-1/ROB_1/J1=', '=joint_1', 'ROB_1/J1=joint_1',
    'cell-1/ROB_1/J1/x=joint_1', 'cell-1//J1=joint_1', '/ROB_1/J1=joint_1',
    'cell-1/ROB_1/=joint_1', 'cell-1/ROB_1/J1= ', '//=x',
])
def test_an_override_of_another_form_is_a_problem(entry):
    overrides, problems = names.parse_joint_overrides(['cell-1/ROB_1/J6=wrist', entry])
    assert overrides == {('cell-1', 'ROB_1', 'J6'): 'wrist'}
    assert len(problems) == 1
    assert '<controller id>/<unit>/<joint>=<name>' in problems[0]


def test_an_override_with_an_impossible_controller_id_is_a_problem():
    overrides, problems = names.parse_joint_overrides(['a b/ROB_1/J1=joint_1'])
    assert overrides == {}
    assert len(problems) == 1
    assert 'cannot be a controller id' in problems[0]


def test_a_joint_named_twice_keeps_its_first_name():
    overrides, problems = names.parse_joint_overrides(
        ['cell-1/ROB_1/J1=joint_1', 'cell-1/ROB_1/J1=other', 'cell-1/ROB_1/J1=joint_1'])
    assert overrides == {('cell-1', 'ROB_1', 'J1'): 'joint_1'}
    assert len(problems) == 2
    assert 'already named "joint_1"' in problems[0]


def test_override_entries_that_are_not_a_list_of_text():
    overrides, problems = names.parse_joint_overrides('cell-1/ROB_1/J1=joint_1')
    assert overrides == {}
    assert len(problems) == 1
    overrides, problems = names.parse_joint_overrides([7])
    assert overrides == {}
    assert len(problems) == 1


@pytest.mark.parametrize('text, accepted', [
    ('cell-1', True), ('a', True), ('0', True), ('Cell_1.b', True), ('a' * 128, True),
    ('', False), ('a' * 129, False), ('-a', False), ('.a', False), ('_a', False),
    ('a b', False), ('a/b', False), ('a:b', False), ('a=b', False), ('a%2f', False),
    ('a?', False), ('a#', False), ('a\n', False), ('caf\u00e9', False), (None, False),
    (5, False),
])
def test_is_controller_id(text, accepted):
    assert names.is_controller_id(text) is accepted


def test_the_default_joint_names():
    namer = names.JointNamer('{unit}_{joint}', {})
    assert namer.name('cell-1', 'cell_1', 'ROB_1', 'J1') == 'ROB_1_J1'
    assert namer.name('cell-1', '', 'ROB_1', 'E1') == 'ROB_1_E1'
    assert names.JointNamer('{unit}_{joint}').name('c', 'c', 'STN_1', 'J1') == 'STN_1_J1'


def test_every_placeholder():
    namer = names.JointNamer('{controller}/{unit}/{joint}/{axis}', None)
    assert namer.name('cell-1', 'cell_1', 'ROB_1', 'J6') == 'cell_1/ROB_1/J6/6'
    assert namer.name('cell-1', '', 'ROB_1', 'E12') == '/ROB_1/E12/12'
    assert names.JointNamer('joint_{axis}', {}).name('c', 'c', 'ROB_1', 'J3') == 'joint_3'
    assert names.JointNamer('{joint}{joint}', {}).name('c', 'c', 'ROB_1', 'J3') == 'J3J3'
    assert names.JointNamer('{{{joint}}}', {}).name('c', 'c', 'ROB_1', 'J3') == '{J3}'


def test_an_override_wins_over_the_format():
    overrides, _ = names.parse_joint_overrides(
        ['cell-1/ROB_1/J1=joint_1', 'cell-1/ROB_1/E1=track'])
    namer = names.JointNamer('{unit}_{joint}', overrides)
    assert namer.name('cell-1', 'cell_1', 'ROB_1', 'J1') == 'joint_1'
    assert namer.name('cell-1', '', 'ROB_1', 'E1') == 'track'
    assert namer.name('cell-1', 'cell_1', 'ROB_1', 'J2') == 'ROB_1_J2'
    assert namer.name('cell-2', 'cell_2', 'ROB_1', 'J1') == 'ROB_1_J1'
    assert namer.name('cell-1', 'cell_1', 'ROB_2', 'J1') == 'ROB_2_J1'


def test_the_namer_keeps_its_own_copy_of_the_overrides():
    overrides = {('cell-1', 'ROB_1', 'J1'): 'joint_1'}
    namer = names.JointNamer('{unit}_{joint}', overrides)
    overrides.clear()
    assert namer.name('cell-1', 'cell_1', 'ROB_1', 'J1') == 'joint_1'


@pytest.mark.parametrize('fmt', [
    '{robot}_{joint}', '{}', '{0}', '{unit.name}', '{unit[0]}', '{Unit}', '{ joint }',
    '{unit}_{joint}_{id}',
])
def test_an_unknown_placeholder_is_an_error(fmt):
    with pytest.raises(ValueError) as caught:
        names.JointNamer(fmt, {})
    assert 'placeholder' in str(caught.value)


@pytest.mark.parametrize('fmt', ['{axis:02d}', '{joint!r}', '{unit:>8}', '{unit:{joint}}'])
def test_a_placeholder_with_a_conversion_or_a_specification_is_an_error(fmt):
    with pytest.raises(ValueError):
        names.JointNamer(fmt, {})


@pytest.mark.parametrize('fmt', ['{unit', 'unit}', '{unit}_{joint', '{', '}'])
def test_unbalanced_braces_are_an_error(fmt):
    with pytest.raises(ValueError) as caught:
        names.JointNamer(fmt, {})
    assert 'cannot be read' in str(caught.value)


@pytest.mark.parametrize('fmt', ['', '   ', None, 5])
def test_an_empty_format_is_an_error(fmt):
    with pytest.raises(ValueError):
        names.JointNamer(fmt, {})


def test_axis_needs_a_number_in_the_joint_name():
    namer = names.JointNamer('joint_{axis}', {('cell-1', 'ROB_1', 'JX'): 'spare'})
    with pytest.raises(ValueError) as caught:
        namer.name('cell-1', 'cell_1', 'ROB_1', 'J')
    assert '{axis}' in str(caught.value)
    assert namer.name('cell-1', 'cell_1', 'ROB_1', 'JX') == 'spare'
    assert names.JointNamer('{unit}_{joint}', {}).name('c', 'c', 'ROB_1', 'J') == 'ROB_1_J'

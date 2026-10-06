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

"""The mapping rules of the generator, one small synthetic schema per rule."""

import copy
import importlib.util
import json
from pathlib import Path
import sys

import pytest

PACKAGE_DIR = Path(__file__).resolve().parent.parent
GENERATOR = PACKAGE_DIR / 'scripts' / 'generate_msgs.py'
GENERATED_LINE = (
    '# Generated from the abb-rws-daemon JSON Schema by generate_msgs.py. Do not edit.'
)

EXPECTED_MESSAGES = {
    'ControllerAnswer', 'ControllerEvent', 'ControllerSnapshot', 'EventArg', 'EventClock',
    'EventTime', 'EventsGroup', 'GroupError', 'IdentityCheck', 'IdentityGroup', 'IoGroup',
    'IoWatchedSignal', 'LinkGroup', 'LinkSubscriptions', 'MotionGroup', 'MotionJoint',
    'MotionPose', 'MotionRate', 'MotionUnitInfo', 'PanelGroup', 'RapidGroup', 'RapidPointer',
    'RapidTaskInfo', 'ReconnectInfo', 'RobotWareVersion',
}


def _load_generator():
    """Import the generator from its file without leaving bytecode beside it."""
    before = sys.dont_write_bytecode
    sys.dont_write_bytecode = True
    try:
        spec = importlib.util.spec_from_file_location('generate_msgs', GENERATOR)
        module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = module
        spec.loader.exec_module(module)
    finally:
        sys.dont_write_bytecode = before
    return module


gen = _load_generator()

STRING = {'type': 'string'}
CHILD = {'type': 'object', 'additionalProperties': False, 'required': ['n'],
         'properties': {'n': {'type': 'integer', 'x-ros-name': 'n'}}}


def ref(name):
    return {'$ref': f'#/definitions/{name}'}


def nullable_ref(name):
    return {'anyOf': [ref(name), {'type': 'null'}]}


def obj(properties, required=None, **extra):
    """Return an object definition; a property without x-ros-name gets its own name."""
    named = {}
    for name, node in properties.items():
        node = dict(node)
        node.setdefault('x-ros-name', name)
        named[name] = node
    return {'type': 'object', 'additionalProperties': False, 'properties': named,
            'required': list(properties) if required is None else list(required), **extra}


def schema_of(definitions):
    return {'$id': 'urn:test:schema:v0', 'x-schema-version': '0.1', 'definitions': definitions}


def run(properties, required=None, definitions=None, renames=None):
    """Generate from one root named Root; return (generated, Root.msg lines, Root fields)."""
    all_definitions = {'Root': obj(properties, required, **{'x-ros': 'msg'})}
    all_definitions.update(definitions or {})
    generated = gen.generate(schema_of(all_definitions), renames or {})
    fields = {field['ros']: field for field in generated.mapping['messages']['Root']['fields']}
    return generated, generated.messages['Root'].splitlines(), fields


def fails(properties, match, required=None, definitions=None, renames=None):
    with pytest.raises(gen.GeneratorError, match=match):
        run(properties, required, definitions, renames)


def test_plain_types():
    _, lines, fields = run({
        'text': STRING,
        'ratio': {'type': 'number'},
        'count': {'type': 'integer'},
        'flag': {'type': 'boolean'},
        'seenAt': {'type': 'string', 'format': 'date-time', 'x-ros-name': 'seen_at'},
    })
    for line in ('string text', 'float64 ratio', 'int64 count', 'bool flag',
                 'builtin_interfaces/Time seen_at'):
        assert line in lines
    assert [fields[name]['kind'] for name in ('text', 'ratio', 'count', 'flag', 'seen_at')] == [
        'string', 'float', 'int', 'bool', 'time']
    assert fields['seen_at'] == {
        'json': 'seenAt', 'ros': 'seen_at', 'kind': 'time', 'required': True, 'nullable': False}
    assert lines[0] == GENERATED_LINE
    assert not any('_known' in line for line in lines)


def test_enum_const_and_named_enum_become_string_constants():
    generated, lines, fields = run(
        {
            'state': {'type': 'string', 'enum': ['on', 'size-limit']},
            'kind': {'type': 'string', 'const': 'snapshot'},
            'quality': {'allOf': [ref('Quality')]},
            'generation': nullable_ref('Generation'),
            'via': {'type': ['string', 'null'], 'enum': ['poll', None]},
        },
        definitions={
            'Quality': {'type': 'string', 'enum': ['live', 'stale']},
            'Generation': {'type': 'string', 'enum': ['rws1', 'rws2']},
        })
    for line in ('string state', 'string STATE_ON="on"', 'string STATE_SIZE_LIMIT="size-limit"',
                 'string kind', 'string KIND_SNAPSHOT="snapshot"',
                 'string quality', 'string QUALITY_LIVE="live"', 'string QUALITY_STALE="stale"',
                 'string generation', 'string GENERATION_RWS1="rws1"',
                 'string via', 'string VIA_POLL="poll"'):
        assert line in lines
    assert fields['state'] == {'json': 'state', 'ros': 'state', 'kind': 'enum', 'required': True,
                               'nullable': False, 'values': ['on', 'size-limit']}
    assert fields['generation']['nullable'] is True
    assert fields['generation']['values'] == ['rws1', 'rws2']
    assert fields['via']['nullable'] is True and fields['via']['values'] == ['poll']
    # Empty when null: no companion flag for an enum.
    assert not any('_known' in line for line in lines)
    assert lines[lines.index('string via') - 1] == '# Empty when not known.'
    # A named enum is never a message of its own.
    assert set(generated.messages) == {'Root'}


def test_nullable_string_integer_and_time_get_a_known_flag():
    _, lines, fields = run({
        'text': {'type': ['string', 'null']},
        'count': {'type': ['integer', 'null']},
        'seenAt': {'type': ['string', 'null'], 'format': 'date-time', 'x-ros-name': 'seen_at'},
    })
    for name, ros_type in (('text', 'string'), ('count', 'int64'),
                           ('seen_at', 'builtin_interfaces/Time')):
        at = lines.index(f'{ros_type} {name}')
        assert lines[at - 1] == f'# Valid only when {name}_known is true.'
        assert lines[at + 1] == f'bool {name}_known'
        assert fields[name]['known'] == f'{name}_known'
        assert fields[name]['nullable'] is True


def test_nullable_number_is_nan_without_a_flag():
    _, lines, fields = run({'value': {'type': ['number', 'null']}})
    at = lines.index('float64 value')
    assert lines[at - 1] == '# NaN when not known.'
    assert 'known' not in fields['value']
    assert not any('_known' in line for line in lines)


def test_nullable_boolean_is_a_tristate_int8():
    _, lines, fields = run({'flag': {'type': ['boolean', 'null']}})
    at = lines.index('int8 flag')
    assert lines[at + 1:at + 4] == [
        'int8 FLAG_UNKNOWN=-1', 'int8 FLAG_FALSE=0', 'int8 FLAG_TRUE=1']
    assert fields['flag']['tristate'] is True
    assert 'known' not in fields['flag']


def test_object_reference_is_a_nested_message_and_optional_when_nullable():
    generated, lines, fields = run(
        {'always': {'allOf': [ref('Child')]}, 'direct': ref('Child'),
         'maybe': nullable_ref('Child')},
        definitions={'Child': CHILD})
    assert 'Child always' in lines and 'Child direct' in lines
    at = lines.index('Child[<=1] maybe')
    assert lines[at - 1] == '# Empty or one element.'
    assert fields['always'] == {'json': 'always', 'ros': 'always', 'kind': 'msg',
                                'required': True, 'nullable': False, 'msg': 'Child'}
    assert fields['maybe']['optional'] is True and fields['maybe']['nullable'] is True
    assert 'optional' not in fields['always']
    assert list(generated.messages) == ['Root', 'Child']
    assert 'int64 n' in generated.messages['Child'].splitlines()


def test_property_missing_from_required_is_nullable_for_its_kind():
    _, lines, fields = run(
        {'child': {'allOf': [ref('Child')]},
         'details': {'allOf': [ref('Details')], 'x-ros': 'json'},
         'text': STRING, 'value': {'type': 'number'}, 'flag': {'type': 'boolean'}},
        required=[],
        definitions={'Child': CHILD, 'Details': {'type': 'object', 'x-ros': 'json'}})
    assert 'Child[<=1] child' in lines
    assert fields['child'] == {'json': 'child', 'ros': 'child', 'kind': 'msg', 'required': False,
                               'nullable': False, 'msg': 'Child', 'optional': True}
    assert 'string details' in lines and fields['details']['kind'] == 'json'
    assert 'bool text_known' in lines and fields['text']['known'] == 'text_known'
    assert lines[lines.index('float64 value') - 1] == '# NaN when not known.'
    assert 'int8 flag' in lines and fields['flag']['tristate'] is True


def test_arrays_bounded_only_with_max_items_and_its_constant():
    _, lines, fields = run(
        {'names': {'type': 'array', 'items': STRING},
         'children': {'type': 'array', 'items': ref('Child')},
         'recent': {'type': 'array', 'items': ref('Child'), 'maxItems': 5,
                    'x-max-items-from': 'RECENT_MAX'},
         'loose': {'type': 'array', 'items': {'type': 'integer'}, 'maxItems': 5},
         'modes': {'type': 'array', 'items': ref('Mode')}},
        definitions={'Child': CHILD, 'Mode': {'type': 'string', 'enum': ['a', 'b']}})
    for line in ('string[] names', 'Child[] children', 'Child[<=5] recent', 'int64[] loose',
                 'string[] modes', 'string MODES_A="a"', 'string MODES_B="b"'):
        assert line in lines
    assert fields['names']['items'] == {'kind': 'string'}
    assert fields['recent'] == {
        'json': 'recent', 'ros': 'recent', 'kind': 'array', 'required': True, 'nullable': False,
        'items': {'kind': 'msg', 'msg': 'Child'}, 'max': 5}
    assert 'max' not in fields['loose'] and 'max' not in fields['children']
    assert fields['modes']['items'] == {'kind': 'enum', 'values': ['a', 'b']}


def test_tuple_of_numbers_is_a_fixed_array():
    number = {'type': 'number'}
    _, lines, fields = run({'q': {
        'type': 'array', 'items': [number, number, number, number], 'additionalItems': False,
        'minItems': 4, 'maxItems': 4, 'x-unit': 'quaternion-wxyz'}})
    at = lines.index('float64[4] q')
    assert lines[at - 1] == '# Unit: quaternion-wxyz.'
    assert fields['q'] == {'json': 'q', 'ros': 'q', 'kind': 'tuple', 'required': True,
                           'nullable': False, 'size': 4}


def test_json_is_a_string_and_is_never_followed():
    details = {'type': 'object', 'additionalProperties': True, 'x-ros': 'json',
               'properties': {'broken': {'oneOf': [STRING, {'type': 'integer'}]}}}
    generated, lines, fields = run(
        {'byDefinition': {'allOf': [ref('Details')], 'x-ros-name': 'by_definition'},
         'byProperty': {'allOf': [ref('Child')], 'x-ros': 'json', 'x-ros-name': 'by_property'},
         'maybe': nullable_ref('Details')},
        definitions={'Details': details, 'Child': CHILD})
    assert 'string by_definition' in lines and 'string by_property' in lines
    at = lines.index('string maybe')
    assert lines[at - 2:at] == ['# JSON text.', '# Empty when not known.']
    assert fields['by_definition'] == {'json': 'byDefinition', 'ros': 'by_definition',
                                       'kind': 'json', 'required': True, 'nullable': False}
    assert fields['maybe']['nullable'] is True
    # Neither the json definition nor the object behind a json property is a message.
    assert set(generated.messages) == {'Root'}


def test_alias_is_not_a_message_and_resolves_to_its_target():
    generated, lines, fields = run(
        {'old': {'allOf': [ref('OldChild')]}},
        definitions={'Child': CHILD, 'OldChild': {**ref('Child'), 'x-ros': 'alias'},
                     'Unused': {**ref('Child'), 'x-ros': 'alias'}})
    assert 'Child old' in lines
    assert fields['old']['msg'] == 'Child'
    assert set(generated.messages) == {'Root', 'Child'}


def test_header_on_a_root_and_only_on_a_root():
    definitions = {
        'First': obj({'child': ref('Child')}, **{'x-ros': 'msg'}),
        'Second': obj({'text': STRING}, **{'x-ros': 'msg'}),
        'Child': CHILD,
        'NotReached': obj({'text': STRING}),
    }
    generated = gen.generate(schema_of(definitions), {})
    assert list(generated.messages) == ['First', 'Child', 'Second']
    for name in ('First', 'Second'):
        lines = [line for line in generated.messages[name].splitlines()
                 if line and not line.startswith('#')]
        assert lines[0] == 'std_msgs/Header header'
        assert generated.mapping['messages'][name]['header'] is True
        assert all(field['ros'] != 'header'
                   for field in generated.mapping['messages'][name]['fields'])
    assert 'Header' not in generated.messages['Child']
    assert 'header' not in generated.mapping['messages']['Child']


def test_a_root_property_named_header_collides_with_the_header():
    fails({'header': STRING}, r"Root\.header: the field name 'header' is used twice")


def test_keyword_field_name_fails_unless_the_rename_table_names_it():
    for name in ('virtual', 'class', 'lambda', 'restrict'):
        fails({name: STRING}, rf'Root\.{name}: the field name .* keyword')
    generated, lines, fields = run({'virtual': {'type': ['boolean', 'null']}},
                                   renames={'Root.virtual': 'is_virtual'})
    assert 'int8 is_virtual' in lines and 'int8 IS_VIRTUAL_UNKNOWN=-1' in lines
    assert fields['is_virtual']['json'] == 'virtual'
    assert generated.mapping['renames'] == {'Root.virtual': 'is_virtual'}


def test_rename_that_names_no_property_fails():
    fails({'text': STRING}, 'rename table names no mapped property: Gone.field',
          renames={'Gone.field': 'x'})


def test_idl_keyword_and_min_max_are_warnings_not_renames():
    generated, lines, _ = run({'module': STRING, 'port': {'type': 'integer'},
                               'min': {'type': 'number'}, 'max': {'type': 'number'},
                               'plain': STRING})
    for line in ('string module', 'int64 port', 'float64 min', 'float64 max'):
        assert line in lines
    assert generated.mapping['warnings'] == [
        {'message': 'Root', 'field': 'module', 'why': 'IDL keyword'},
        {'message': 'Root', 'field': 'port', 'why': 'IDL keyword'},
        {'message': 'Root', 'field': 'min', 'why': 'macro name in some C++ headers'},
        {'message': 'Root', 'field': 'max', 'why': 'macro name in some C++ headers'},
    ]


@pytest.mark.parametrize('node, match', [
    ({'type': ['array', 'null'], 'items': STRING}, 'nullable array'),
    ({'oneOf': [ref('Child'), ref('Other')]}, 'oneOf'),
    ({'anyOf': [ref('Child'), ref('Other')]}, 'anyOf'),
    ({'allOf': [ref('Child'), ref('Other')]}, 'allOf'),
    ({'type': 'object', 'properties': {}}, 'inline object'),
    ({'type': 'array', 'items': {'type': 'array', 'items': STRING}}, 'array of arrays'),
    ({'type': 'array', 'items': {'type': ['string', 'null']}}, 'nullable items'),
    ({'type': 'array', 'items': [STRING, STRING], 'minItems': 2, 'maxItems': 2}, 'fixed tuple'),
    ({'type': 'array', 'items': [{'type': 'number'}], 'minItems': 0, 'maxItems': 1},
     'fixed tuple'),
    ({'type': 'array'}, 'without an item schema'),
    ({'type': ['string', 'integer']}, 'type'),
    ({'type': 'integer', 'enum': [1, 2]}, 'enum of integer'),
    ({'type': 'string', 'enum': ['say "x"']}, 'cannot be a string constant'),
    ({'type': 'string', 'enum': ['a#b']}, 'cannot be a string constant'),
    ({}, 'without a type'),
    ({'$ref': '#/elsewhere/Child'}, 'reference'),
    ({'$ref': '#/definitions/Missing'}, 'unknown definition Missing'),
])
def test_unsupported_shape_stops_with_the_property_name(node, match):
    definitions = {'Child': CHILD, 'Other': CHILD}
    with pytest.raises(gen.GeneratorError, match=match) as raised:
        run({'bad': node}, definitions=definitions)
    assert str(raised.value).startswith('Root.bad')


def test_optional_array_and_recursion_fail():
    fails({'items': {'type': 'array', 'items': STRING}}, r'Root\.items: an optional array',
          required=[])
    fails({'next': nullable_ref('Root')}, 'refers to itself')


def test_names_are_checked():
    fails({'text': {'type': 'string', 'x-ros-name': 'Text'}}, 'not a legal field name')
    fails({'text': {'type': 'string', 'x-ros-name': 'a__b'}}, 'not a legal field name')
    fails({'text': {'type': 'string', 'x-ros-name': 'a_'}}, 'not a legal field name')
    fails({'state': {'type': 'string', 'enum': ['-x']}}, 'not a legal constant name')
    fails({'a': {'type': ['string', 'null']}, 'a_known': {'type': 'boolean'}},
          "field name 'a_known' is used twice")
    fails({'a': {'type': 'string', 'enum': ['b_c']}, 'a_b': {'type': 'string', 'enum': ['c']}},
          'constant A_B_C is used twice')
    fails({'child': ref('bad_name')}, 'not a legal message name',
          definitions={'bad_name': CHILD})
    with pytest.raises(gen.GeneratorError, match='has no x-ros-name'):
        gen.generate(schema_of({'Root': {
            'type': 'object', 'x-ros': 'msg', 'properties': {'text': STRING}}}), {})
    with pytest.raises(gen.GeneratorError, match='no definition is marked'):
        gen.generate(schema_of({'Root': obj({'text': STRING})}), {})
    with pytest.raises(gen.GeneratorError, match='x-schema-version'):
        gen.generate({'$id': 'urn:x', 'definitions': {}}, {})


def test_comments_carry_description_unit_and_stay_ascii():
    long_text = ' '.join(['word'] * 60)
    generated, lines, _ = run(
        {'speed': {'type': 'number', 'x-unit': 'percent', 'description': 'First line.\nSecond.'},
         'value': {'type': ['number', 'null'], 'x-unit-from': 'theUnit'},
         'theUnit': {'type': 'string', 'x-ros-name': 'the_unit'},
         'count': {'type': 'integer', 'x-unit': 'count', 'description': long_text},
         'plain': {'type': 'integer', 'x-unit': 'none'},
         'odd': {'type': 'string',
                 'description': 'a\\b caf' + chr(233) + ' [x]\tend\n\nNew paragraph.'}})
    at = lines.index('float64 speed')
    assert lines[at - 2:at] == ['# First line. Second.', '# Unit: percent.']
    at = lines.index('float64 value')
    assert lines[at - 2:at] == ['# In the unit named by the_unit.', '# NaN when not known.']
    assert lines[lines.index('int64 plain') - 1] == ''
    at = lines.index('string odd')
    assert lines[at - 3:at] == ['# aU+005Cb cafU+00E9 [x] end', '#', '# New paragraph.']
    text = generated.messages['Root']
    assert text.isascii() and '\\' not in text and '\r' not in text and text.endswith('\n')
    assert max(len(line) for line in lines) <= 99
    wrapped = [line for line in lines if line.startswith('# word')]
    assert len(wrapped) > 1 and ' '.join(line[2:] for line in wrapped) == long_text


def test_comments_name_fields_not_json_properties():
    child = obj({'asOf': {'type': 'string', 'format': 'date-time', 'x-ros-name': 'as_of',
                          'description': 'Same meaning as `other.asOf`.'},
                 'kind': {'type': 'string', 'x-ros-name': 'child_kind'}},
                description='Carries `asOf`; see `Root.titleFromCode` and `kind`.')
    generated, lines, _ = run(
        {'titleFromCode': {'type': 'boolean', 'x-ros-name': 'title_from_code',
                           'description': 'True when `title` was made from `code`.'},
         'title': {'type': 'string',
                   'description': 'With `titleFromCode` true: "Event <code>". `GET '
                                  '/v0/x?titleFromCode=1` and `leaseId` stay as they are.'},
         'code': {'type': 'integer'},
         'kind': STRING,
         'child': ref('Child')},
        definitions={'Child': child})
    at = lines.index('string title')
    assert lines[at - 2:at] == [
        '# With `title_from_code` true: "Event <code>". `GET /v0/x?titleFromCode=1` and `leaseId` '
        'stay as',
        '# they are.']
    assert lines[lines.index('bool title_from_code') - 1] == (
        '# True when `title` was made from `code`.')
    child_lines = generated.messages['Child'].splitlines()
    # A dotted path is rewritten part by part; a name that is no property stays. `kind`
    # has two field names (kind, child_kind), so it names nothing a comment could follow.
    assert '# Carries `as_of`; see `Root.title_from_code` and `kind`.' in child_lines
    assert '# Same meaning as `other.as_of`.' in child_lines
    assert gen.member_names({}) == {}
    assert gen.in_field_names('`a.bC` and bC and `x y`', {'bC': 'b_c'}) == (
        '`a.b_c` and bC and `x y`')


def test_a_time_field_is_not_called_iso_8601():
    _, lines, _ = run(
        {'receivedAt': {'type': 'string', 'format': 'date-time', 'x-ros-name': 'received_at',
                        'description': 'When the daemon received it (UTC, ISO 8601).'},
         'utc': {'type': ['string', 'null'], 'format': 'date-time',
                 'description': '`raw` in UTC (ISO 8601). Shifted when corrected.'},
         'raw': {'type': 'string',
                 'description': 'As reported - not ISO 8601 on any generation (ISO 8601).'}})
    assert lines[lines.index('builtin_interfaces/Time received_at') - 1] == (
        '# When the daemon received it (UTC).')
    at = lines.index('builtin_interfaces/Time utc')
    assert lines[at - 2:at] == ['# `raw` in UTC. Shifted when corrected.',
                                '# Valid only when utc_known is true.']
    # Text stays text: what a string field says about its own format is left alone.
    assert lines[lines.index('string raw') - 1] == (
        '# As reported - not ISO 8601 on any generation (ISO 8601).')


def test_the_vendored_comments_name_no_camel_case_member_and_no_iso_time():
    schema = json.loads((PACKAGE_DIR / 'schema' / 'v0.schema.json').read_bytes())
    generated = gen.generate(schema)
    members = gen.member_names({
        name: [type('Field', (), {'entry': field})() for field in entry['fields']]
        for name, entry in generated.mapping['messages'].items()})
    assert {'asOf', 'titleFromCode', 'offsetMs', 'receivedAt', 'targetHz'} <= set(members)
    for name, text in generated.messages.items():
        lines = text.splitlines()
        for match in gen.MEMBER_PATH.finditer(text):
            assert not set(match.group(1).split('.')) & set(members), (name, match.group(0))
        for index, line in enumerate(lines):
            if line.startswith('builtin_interfaces/Time '):
                above = index
                while above and lines[above - 1].startswith('#'):
                    above -= 1
                    assert 'ISO 8601' not in lines[above], (name, lines[above])


def test_cmake_list_is_sorted_and_names_every_message_and_nothing_else():
    definitions = {
        'Zeta': obj({'beta': ref('Beta'), 'alpha': ref('Alpha')}, **{'x-ros': 'msg'}),
        'Beta': CHILD, 'Alpha': CHILD, 'Unreached': CHILD,
        'Mode': {'type': 'string', 'enum': ['a']},
    }
    generated = gen.generate(schema_of(definitions), {})
    listed = [line.strip() for line in generated.cmake.splitlines()
              if line.startswith('  "')]
    assert listed == ['"msg/Alpha.msg"', '"msg/Beta.msg"', '"msg/Zeta.msg"']
    assert generated.cmake.startswith('# Copyright 2026 Meraj Safari\n')
    assert GENERATED_LINE in generated.cmake.splitlines()
    assert 'set(MSG_FILES\n' in generated.cmake and generated.cmake.endswith(')\n')
    files = generated.files()
    assert sorted(files) == ['msg/Alpha.msg', 'msg/Beta.msg', 'msg/Zeta.msg',
                             'msg/interfaces.cmake', 'schema/v0.mapping.json']


def test_two_runs_give_the_same_bytes_and_fields_keep_the_property_order():
    definitions = {
        'Root': obj({'zulu': STRING, 'alpha': ref('Child'), 'mike': {'type': ['integer', 'null']}},
                    **{'x-ros': 'msg'}),
        'Child': CHILD,
    }
    first = gen.generate(schema_of(copy.deepcopy(definitions)), {})
    second = gen.generate(schema_of(copy.deepcopy(definitions)), {})
    assert first.files() == second.files()
    assert first.definitions_sha256 == second.definitions_sha256
    names = [field['ros'] for field in first.mapping['messages']['Root']['fields']]
    assert names == ['zulu', 'alpha', 'mike']


def test_definitions_hash_follows_the_mapped_definitions_only():
    def digest(change=None, version='0.1', renames=None):
        definitions = {
            'Root': obj({'child': ref('Child'), 'mode': ref('Mode')}, **{'x-ros': 'msg'}),
            'Child': copy.deepcopy(CHILD),
            'Mode': {'type': 'string', 'enum': ['a']},
            'Unmapped': obj({'text': STRING}),
        }
        if change:
            change(definitions)
        schema = schema_of(definitions)
        schema['x-schema-version'] = version
        return gen.generate(schema, renames or {}).definitions_sha256

    base = digest()
    assert len(base) == 64 and base == digest()
    assert base == digest(version='0.2')
    assert base == digest(lambda d: d['Unmapped']['properties'].update(more=STRING))
    assert base != digest(lambda d: d['Child'].update(description='Now with a comment.'))
    assert base != digest(lambda d: d['Mode']['enum'].append('b'))
    renamed = digest(renames={'Root.child': 'kid'})
    assert renamed != base and renamed != digest(renames={'Root.child': 'offspring'})


def test_vendored_schema_gives_the_25_messages():
    schema_bytes = (PACKAGE_DIR / 'schema' / 'v0.schema.json').read_bytes()
    generated = gen.generate(json.loads(schema_bytes))
    assert set(generated.messages) == EXPECTED_MESSAGES and len(generated.messages) == 25
    mapping = generated.mapping
    assert [name for name, entry in mapping['messages'].items() if entry.get('header')] == [
        'ControllerSnapshot']
    root_lines = [line for line in generated.messages['ControllerSnapshot'].splitlines()
                  if line and not line.startswith('#')]
    assert root_lines[0] == 'std_msgs/Header header'
    assert mapping['renames'] == {'IdentityGroup.virtual': 'is_virtual'}
    assert 'int8 is_virtual' in generated.messages['IdentityGroup'].splitlines()
    assert {(entry['message'], entry['field']) for entry in mapping['warnings']} == {
        ('LinkGroup', 'port'), ('RapidPointer', 'module'),
        ('MotionJoint', 'min'), ('MotionJoint', 'max')}
    joint = {field['ros']: field for field in mapping['messages']['MotionJoint']['fields']}
    assert joint['unit'] == {'json': 'unit', 'ros': 'unit', 'kind': 'enum', 'required': True,
                             'nullable': True, 'values': ['deg', 'mm']}
    assert joint['value']['kind'] == 'float' and joint['value']['nullable'] is True
    for text in generated.messages.values():
        assert text.startswith(GENERATED_LINE + '\n') and text.isascii()
    record = gen.source_record(schema_bytes, generated)
    assert list(record) == ['source', 'schemaId', 'schemaVersion', 'schemaSha256',
                            'definitionsSha256', 'messages', 'renames']
    assert record['source'] == 'abb-rws-daemon, schema/v0.schema.json'
    assert record['messages'] == 25
    assert record['schemaVersion'] == json.loads(schema_bytes)['x-schema-version']


def _tree(folder):
    return {path.relative_to(folder).as_posix(): path.read_bytes()
            for path in sorted(folder.rglob('*')) if path.is_file()}


def test_command_line_writes_the_tree_elsewhere_and_removes_a_stale_message(tmp_path, capsys):
    stale = tmp_path / 'msg' / 'Gone.msg'
    stale.parent.mkdir()
    stale.write_text('string gone\n')
    (tmp_path / 'msg' / 'notes.txt').write_text('kept\n')
    assert gen.main(['--out-dir', str(tmp_path)]) == 0
    tree = _tree(tmp_path)
    messages = [name for name in tree if name.endswith('.msg')]
    assert len(messages) == 25 and 'msg/Gone.msg' not in tree and 'msg/notes.txt' in tree
    assert {'msg/interfaces.cmake', 'schema/v0.mapping.json', 'schema/SOURCE.json'} <= set(tree)
    assert 'schema/v0.schema.json' not in tree
    assert all(b'\r' not in data for data in tree.values())
    for name, data in tree.items():
        if name != 'msg/notes.txt':
            assert data == (PACKAGE_DIR / name).read_bytes(), name
    assert '1 removed' in capsys.readouterr().out
    # A second run changes nothing.
    assert gen.main(['--out-dir', str(tmp_path)]) == 0
    assert '0 written, 0 removed' in capsys.readouterr().out
    assert _tree(tmp_path) == tree


def test_vendor_copies_the_schema_with_lf_and_records_it(tmp_path):
    schema_bytes = (PACKAGE_DIR / 'schema' / 'v0.schema.json').read_bytes()
    incoming = tmp_path / 'incoming.json'
    incoming.write_bytes(schema_bytes.replace(b'\n', b'\r\n'))
    out = tmp_path / 'out'
    assert gen.main(['--vendor', str(incoming), '--out-dir', str(out)]) == 0
    assert (out / 'schema' / 'v0.schema.json').read_bytes() == schema_bytes
    assert gen.check_tree(out) == []
    broken = tmp_path / 'broken.json'
    broken.write_text('{"definitions": ')
    assert gen.main(['--vendor', str(broken), '--out-dir', str(tmp_path / 'never')]) == 2
    assert not (tmp_path / 'never').exists()


def test_check_lists_missing_stale_and_extra_files_and_a_changed_schema(tmp_path, capsys):
    schema_path = PACKAGE_DIR / 'schema' / 'v0.schema.json'
    assert gen.main(['--vendor', str(schema_path), '--out-dir', str(tmp_path)]) == 0
    assert gen.check_tree(tmp_path) == []
    (tmp_path / 'msg' / 'MotionJoint.msg').unlink()
    with open(tmp_path / 'msg' / 'MotionPose.msg', 'a', newline='\n') as handle:
        handle.write('string added\n')
    (tmp_path / 'msg' / 'Extra.msg').write_text('string extra\n')
    cmake = tmp_path / 'msg' / 'interfaces.cmake'
    cmake.write_bytes(cmake.read_bytes().replace(b'\n', b'\r\n'))
    assert gen.check_tree(tmp_path) == [
        'missing: msg/MotionJoint.msg', 'stale: msg/MotionPose.msg',
        'stale: msg/interfaces.cmake', 'extra: msg/Extra.msg']
    # Another schema file than the one SOURCE.json names: its hash and version are reported.
    schema = json.loads(schema_path.read_bytes())
    schema['x-schema-version'] = '0.999'
    (tmp_path / 'schema' / 'v0.schema.json').write_text(
        json.dumps(schema, indent=2) + '\n', newline='\n')
    problems = gen.check_tree(tmp_path)
    assert any(line.startswith('schema/SOURCE.json: schemaSha256 is ') for line in problems)
    assert any(line.startswith("schema/SOURCE.json: schemaVersion is '0.12'")
               for line in problems)
    assert 'stale: schema/SOURCE.json' in problems and 'stale: schema/v0.mapping.json' in problems
    (tmp_path / 'schema' / 'v0.schema.json').unlink()
    assert gen.check_tree(tmp_path) == ['missing: schema/v0.schema.json']
    capsys.readouterr()
    with pytest.raises(SystemExit) as stopped:
        gen.main(['--check', '--out-dir', str(tmp_path)])
    assert stopped.value.code == 2

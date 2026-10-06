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

"""The installed message package holds the 25 messages its mapping file names."""

import hashlib
import importlib
import json
from pathlib import Path
import re

from ament_index_python.packages import get_package_share_directory
import pytest

PACKAGE = 'abb_rws_ros2_msgs'
SCHEMA_DIR = Path(get_package_share_directory(PACKAGE)) / 'schema'

PLAIN_TYPES = {
    'string': 'string',
    'enum': 'string',
    'json': 'string',
    'time': 'builtin_interfaces/Time',
    'float': 'double',
    'int': 'int64',
    'bool': 'boolean',
}


def _read(name):
    with open(SCHEMA_DIR / name, encoding='utf-8') as handle:
        return json.load(handle)


MAPPING = _read('v0.mapping.json')
MESSAGE_NAMES = sorted(MAPPING['messages'])


def _message(name):
    return getattr(importlib.import_module(f'{PACKAGE}.msg'), name)


def _type_text(entry):
    """Return the type rclpy reports for a mapping entry (a field or the items of an array)."""
    kind = entry['kind']
    if kind == 'msg':
        nested = f'{PACKAGE}/{entry["msg"]}'
        return f'sequence<{nested}, 1>' if entry.get('optional') else nested
    if kind == 'bool' and entry.get('tristate'):
        return 'int8'
    if kind == 'tuple':
        return f'double[{entry["size"]}]'
    if kind == 'array':
        bound = f', {entry["max"]}' if 'max' in entry else ''
        return f'sequence<{_type_text(entry["items"])}{bound}>'
    return PLAIN_TYPES[kind]


def _constant(field, value):
    return f'{field["ros"].upper()}_' + re.sub(r'[^A-Z0-9]+', '_', value.upper())


def test_there_are_25_messages_and_the_package_is_the_mapped_one():
    assert MAPPING['package'] == PACKAGE
    assert len(MESSAGE_NAMES) == 25
    assert 'ControllerSnapshot' in MESSAGE_NAMES


@pytest.mark.parametrize('name', MESSAGE_NAMES)
def test_message_imports_with_the_fields_the_mapping_names(name):
    entry = MAPPING['messages'][name]
    message_class = _message(name)
    expected = {'header': 'std_msgs/Header'} if entry.get('header') else {}
    for field in entry['fields']:
        expected[field['ros']] = _type_text(field)
        if 'known' in field:
            expected[field['known']] = 'boolean'
    actual = message_class.get_fields_and_field_types()
    assert actual == expected
    assert list(actual) == list(expected), 'the field order is the property order'
    # A default-constructed message can be made and holds every field.
    message = message_class()
    for ros_name in expected:
        assert hasattr(message, ros_name)


@pytest.mark.parametrize('name', MESSAGE_NAMES)
def test_message_has_the_constants_of_its_enums_and_tristates(name):
    message_class = _message(name)
    for field in MAPPING['messages'][name]['fields']:
        values = field.get('values', field.get('items', {}).get('values', []))
        for value in values:
            assert getattr(message_class, _constant(field, value)) == value
        if field.get('tristate'):
            prefix = field['ros'].upper()
            assert getattr(message_class, f'{prefix}_UNKNOWN') == -1
            assert getattr(message_class, f'{prefix}_FALSE') == 0
            assert getattr(message_class, f'{prefix}_TRUE') == 1


def test_controller_snapshot_has_a_std_msgs_header_and_no_other_message_has():
    with_header = [name for name in MESSAGE_NAMES
                   if 'header' in _message(name).get_fields_and_field_types()]
    assert with_header == ['ControllerSnapshot']
    fields = _message('ControllerSnapshot').get_fields_and_field_types()
    assert next(iter(fields)) == 'header'
    assert fields['header'] == 'std_msgs/Header'
    from std_msgs.msg import Header
    assert isinstance(_message('ControllerSnapshot')().header, Header)
    assert MAPPING['messages']['ControllerSnapshot']['header'] is True


def test_installed_schema_files_belong_together():
    source = _read('SOURCE.json')
    schema_bytes = (SCHEMA_DIR / 'v0.schema.json').read_bytes()
    schema = json.loads(schema_bytes)
    assert hashlib.sha256(schema_bytes).hexdigest() == source['schemaSha256']
    assert schema['x-schema-version'] == source['schemaVersion'] == MAPPING['schemaVersion']
    assert schema['$id'] == source['schemaId'] == MAPPING['schemaId']
    assert source['definitionsSha256'] == MAPPING['definitionsSha256']
    assert source['messages'] == len(MESSAGE_NAMES)
    assert source['renames'] == MAPPING['renames']

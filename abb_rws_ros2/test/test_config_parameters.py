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
The shipped parameter file against the parameter table of the bridge node.

``config/bridge.yaml`` must list every parameter of ``abb_rws_bridge.parameters.SPECS``
with its default, and nothing else. The file is read from the source folder, as it is
now, not from an installed copy.
"""

import json
from pathlib import Path
import re

from abb_rws_bridge import parameters
import pytest
import rclpy
from rclpy.context import Context
import yaml

CONFIG = Path(__file__).resolve().parent.parent / 'config' / 'bridge.yaml'

SPECS = {spec.name: spec for spec in parameters.SPECS}
BOUNDED = sorted(
    name for name, spec in SPECS.items()
    if spec.minimum is not None or spec.maximum is not None)

_KEY = re.compile(r' {4}([A-Za-z0-9_]+):(?: (.*))?')
_COMMENT = re.compile(r' {4}#(?: (.*))?')
_QOS_START = '    # qos_overrides:'
_QOS_MORE = '    #   '
_QOS_POLICIES = {'depth', 'history', 'reliability'}


def _text():
    return CONFIG.read_bytes().decode('ascii')


def _content_lines(text):
    """Return the lines that are neither blank nor a comment."""
    return [
        line for line in text.split('\n')
        if line.strip() and not line.lstrip().startswith('#')
    ]


def _written(text):
    """
    Return [(name, value text, comment)] for the keys under ros__parameters.

    Taken from the text itself, so a key that is written twice shows twice. The
    comment of a key is the block of comment lines directly above it, as one line.
    """
    lines = text.split('\n')
    start = lines.index('  ros__parameters:') + 1
    found = []
    comment = []
    for line in lines[start:]:
        key = _KEY.fullmatch(line)
        note = _COMMENT.fullmatch(line)
        if key:
            found.append((key.group(1), key.group(2) or '', ' '.join(comment)))
            comment = []
        elif note:
            comment.append((note.group(1) or '').strip())
        else:
            comment = []
    return found


def _values(text):
    """Return the parameters of the file as PyYAML reads them."""
    return yaml.safe_load(text)['/**']['ros__parameters']


def _with_qos_example(text):
    """Return the text with the commented-out qos_overrides example switched on."""
    lines = text.split('\n')
    start = lines.index(_QOS_START)
    end = start + 1
    while end < len(lines) and lines[end].startswith(_QOS_MORE):
        end += 1
    for index in range(start, end):
        lines[index] = lines[index].replace('# ', '', 1)
    return '\n'.join(lines), end - start


def _rcl_reads(path):
    """Let rcl itself parse a parameter file; no node is made, so no ROS graph."""
    context = Context()
    rclpy.init(args=['--ros-args', '--params-file', str(path)], context=context)
    try:
        return context.ok()
    finally:
        rclpy.shutdown(context=context)


def test_file_is_plain_ascii_with_lf_line_endings():
    data = CONFIG.read_bytes()
    assert data, 'bridge.yaml is empty'
    assert b'\r' not in data, 'bridge.yaml has a carriage return; it must use LF'
    assert b'\t' not in data, 'bridge.yaml has a tab'
    assert max(data) < 127, 'bridge.yaml has a character that is not ASCII'
    assert data.endswith(b'\n') and not data.endswith(b'\n\n')
    trailing = [line for line in data.decode('ascii').split('\n') if line != line.rstrip()]
    assert not trailing, f'lines end with spaces: {trailing}'


def test_file_opens_with_the_licence_header():
    lines = _text().split('\n')
    assert lines[0] == '# Copyright 2026 Meraj Safari'
    assert '# Licensed under the Apache License, Version 2.0 (the "License");' in lines[:5]


def test_file_starts_with_the_wildcard_and_ros_parameters():
    content = _content_lines(_text())
    assert content[0] == '/**:'
    assert content[1] == '  ros__parameters:'


def test_document_is_one_wildcard_entry_with_ros_parameters_only():
    document = yaml.safe_load(_text())
    assert isinstance(document, dict)
    assert list(document) == ['/**']
    assert isinstance(document['/**'], dict)
    assert list(document['/**']) == ['ros__parameters']
    assert isinstance(document['/**']['ros__parameters'], dict)


def test_every_spec_is_in_the_file_and_every_key_is_a_spec():
    names = [name for name, _, _ in _written(_text())]
    missing = sorted(set(SPECS) - set(names))
    unknown = sorted(set(names) - set(SPECS))
    assert not missing, f'parameters of SPECS that bridge.yaml does not list: {missing}'
    assert not unknown, f'keys of bridge.yaml that are not in SPECS: {unknown}'


def test_no_key_is_written_twice_and_the_text_matches_the_document():
    text = _text()
    names = [name for name, _, _ in _written(text)]
    twice = sorted({name for name in names if names.count(name) > 1})
    assert not twice, f'written twice: {twice}'
    assert names == list(_values(text))


def test_there_are_as_many_keys_as_specs():
    assert len(_values(_text())) == len(parameters.SPECS) == len(SPECS)


@pytest.mark.parametrize('name', sorted(SPECS))
def test_default_equals_the_spec(name):
    spec = SPECS[name]
    value = _values(_text())[name]
    if spec.kind == 'string':
        assert type(value) is str
        assert value == spec.default
    elif spec.kind == 'bool':
        assert type(value) is bool
        assert value is spec.default
    elif spec.kind == 'double':
        assert type(value) is float, f'{name} must be written with a decimal point'
        assert value == spec.default
    elif spec.kind == 'string_array':
        assert spec.default == ()
        assert type(value) is list
        assert value == []
    else:
        pytest.fail(f'{name} has a kind this test does not know: {spec.kind}')


@pytest.mark.parametrize('name', sorted(SPECS))
def test_value_is_written_the_way_rcl_reads_its_type(name):
    """Text in double quotes, a number with a decimal point, true or false, []."""
    spec = SPECS[name]
    written = {key: value for key, value, _ in _written(_text())}[name]
    if spec.kind == 'string':
        expected = json.dumps(spec.default)
    elif spec.kind == 'bool':
        expected = 'true' if spec.default else 'false'
    elif spec.kind == 'double':
        expected = repr(float(spec.default))
        assert re.fullmatch(r'[0-9]+\.[0-9]+', expected)
    else:
        expected = '[]'
    assert written == expected


@pytest.mark.parametrize('name', sorted(SPECS))
def test_every_parameter_has_a_comment_above_it(name):
    comments = {key: comment for key, _, comment in _written(_text())}
    assert len(comments[name]) >= 20, f'{name} has no comment directly above it'


@pytest.mark.parametrize('name', BOUNDED)
def test_comment_of_a_bounded_number_names_its_range(name):
    spec = SPECS[name]
    comments = {key: comment for key, _, comment in _written(_text())}
    assert f'{spec.minimum:g} to {spec.maximum:g}' in comments[name]


def test_bounded_numbers_are_the_doubles():
    assert BOUNDED, 'SPECS has no bounded number; the range test would check nothing'
    assert all(SPECS[name].kind == 'double' for name in BOUNDED)


def test_defaults_pass_the_rules_of_the_node():
    settings, problems = parameters.settings_from(_values(_text()))
    assert problems == []
    expected, _ = parameters.settings_from({})
    assert settings is not None
    assert settings == expected


def test_rcl_reads_the_file():
    assert _rcl_reads(CONFIG)


def test_qos_overrides_example_is_commented_out():
    text = _text()
    assert _QOS_START in text.split('\n')
    assert not any(
        name.startswith('qos_overrides') for name, _, _ in _written(text))
    assert 'qos_overrides' not in _values(text)


def test_qos_overrides_example_is_valid_once_switched_on(tmp_path):
    text, switched = _with_qos_example(_text())
    assert switched >= 4
    values = _values(text)
    assert set(values) == set(SPECS) | {'qos_overrides'}

    topics = values['qos_overrides']
    assert isinstance(topics, dict) and topics
    for topic, entry in topics.items():
        assert topic.startswith('/'), 'the key is the full name of the topic'
        assert list(entry) == ['publisher']
        policies = entry['publisher']
        assert policies and set(policies) <= _QOS_POLICIES
        for policy in policies:
            flat = f'qos_overrides.{topic}.publisher.{policy}'
            assert flat.startswith(parameters.QOS_OVERRIDE_PREFIX)
        if 'depth' in policies:
            assert type(policies['depth']) is int and policies['depth'] > 0
        if 'history' in policies:
            assert policies['history'] in ('keep_last', 'keep_all')
        if 'reliability' in policies:
            assert policies['reliability'] in ('reliable', 'best_effort')

    switched_on = tmp_path / 'bridge.yaml'
    switched_on.write_bytes(text.encode('ascii'))
    assert _rcl_reads(switched_on)


def test_rcl_refuses_a_broken_file(tmp_path):
    """The check above means something: rcl does refuse a file it cannot read."""
    broken = tmp_path / 'broken.yaml'
    broken.write_bytes(b'/**:\n  ros__parameters:\n    refresh_hz: [1.0, "a"]\n')
    with pytest.raises(Exception, match='parse params file'):
        _rcl_reads(broken)

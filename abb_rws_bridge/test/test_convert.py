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

"""Daemon data becomes messages by the mapping table: every kind, the header, the way back."""

import copy
import datetime
import importlib
import json
import math
from pathlib import Path
import pickle
import subprocess
import sys
import types

from abb_rws_bridge import convert
from ament_index_python.packages import get_package_share_directory
import frames
import pytest

PACKAGE = 'abb_rws_ros2_msgs'
PACKAGE_ROOT = Path(__file__).resolve().parent.parent
T0_SEC = 1767225600
STAMP = (T0_SEC + 3, 250000000)
SENTINEL = 'SENTINEL-value'


def _read_mapping():
    path = Path(get_package_share_directory(PACKAGE)) / 'schema' / 'v0.mapping.json'
    with open(path, encoding='utf-8') as handle:
        return json.load(handle)


MAPPING = _read_mapping()
MESSAGES = MAPPING['messages']
FIELDS = [(name, entry['json']) for name, message in MESSAGES.items()
          for entry in message['fields']]
TYPED_FIELDS = [(name, key) for name, key in FIELDS
                if next(entry for entry in MESSAGES[name]['fields']
                        if entry['json'] == key)['kind'] != 'json']

# A value no field of that kind takes.
WRONG = {
    'string': 5, 'enum': 5, 'time': 5, 'float': SENTINEL, 'int': SENTINEL, 'bool': SENTINEL,
    'msg': SENTINEL, 'array': SENTINEL, 'tuple': SENTINEL,
}


@pytest.fixture(scope='module')
def converter():
    return convert.Converter.from_installed()


def _entry(type_name, key):
    return next(entry for entry in MESSAGES[type_name]['fields'] if entry['json'] == key)


def _may_miss(entry):
    return entry['nullable'] or not entry['required']


def _sample_value(entry):
    kind = entry['kind']
    if kind == 'msg':
        return _sample(entry['msg'])
    if kind == 'array':
        return [_sample_value(entry['items'])]
    if kind == 'tuple':
        return [0.5] * entry['size']
    if kind == 'enum':
        return entry['values'][0]
    return {'string': 'text', 'time': frames.iso(1.5), 'float': 1.5, 'int': 3, 'bool': True,
            'json': {'key': [1, 'two']}}[kind]


def _sample(type_name):
    """Return data for a message with a value of the right kind in every field."""
    return {entry['json']: _sample_value(entry) for entry in MESSAGES[type_name]['fields']}


def _convert(converter, type_name, data):
    stamp = STAMP if MESSAGES[type_name].get('header') else None
    return converter.to_msg(type_name, data, stamp)


def _iso(stamp):
    moment = datetime.datetime.fromtimestamp(stamp.sec, tz=datetime.timezone.utc)
    return moment.strftime('%Y-%m-%dT%H:%M:%S.') + f'{stamp.nanosec // 1000000:03d}Z'


def _value_back(entry, value):
    kind = entry['kind']
    if kind == 'time':
        return _iso(value)
    if kind == 'msg':
        return _back(entry['msg'], value)
    if kind == 'json':
        return json.loads(value)
    if kind == 'tuple':
        return [float(item) for item in value]
    if kind == 'array':
        return [_value_back(entry['items'], item) for item in value]
    return value


def _field_back(entry, message):
    """Return what the daemon said for one field of a message; None stands for null."""
    value = getattr(message, entry['ros'])
    kind = entry['kind']
    if 'known' in entry and not getattr(message, entry['known']):
        return None
    if kind == 'msg' and entry.get('optional'):
        return _back(entry['msg'], value[0]) if value else None
    if kind == 'bool' and entry.get('tristate'):
        return {-1: None, 0: False, 1: True}[value]
    if kind in ('enum', 'json') and _may_miss(entry) and value == '':
        return None
    if kind == 'float' and _may_miss(entry) and math.isnan(value):
        return None
    return _value_back(entry, value)


def _back(type_name, message):
    """Return the daemon data a message was built from: the mapping read the other way."""
    data = {}
    for entry in MESSAGES[type_name]['fields']:
        value = _field_back(entry, message)
        if value is None and not entry['required']:
            continue
        data[entry['json']] = value
    return data


def _full_controller():
    """Return a controller in which everything that may be null has a value."""
    error = frames.group_error(
        controller=(404, -1073445879, 'synthetic controller message'),
        refusal={'b': [1, {'z': None, 'a': True}], 'a': 'x'})
    return frames.controller(
        'full-1',
        name='Everything set',
        identity=frames.identity_group(virtual=False,
                                       version=frames.robot_ware('6.16.03', 6, 16, 'rws1')),
        link=frames.link_group(connected=False, quality='reconnecting',
                               quality_reason='no answer', identity_status='mismatch',
                               actual='00000000-0000-4000-8000-000000000002',
                               reconnect={'failures': 2, 'nextAttemptAt': frames.iso(5)}),
        panel=frames.panel_group(collision_detection='on', speed_ratio=50.0),
        rapid=frames.rapid_group([frames.rapid_task(
            pp=frames.rapid_pointer(), mp=frames.rapid_pointer('Other', 'work', 3))]),
        motion=frames.motion_group([frames.motion_unit(
            pose=frames.tcp_pose(tool='tool0', wobj='wobj0', coord='Base'), error=error)]),
        io=frames.io_group([frames.signal(error=error)]),
        events=frames.events_group(
            [frames.event(1, seqnum=10, code=10010, severity='error', description='d',
                          causes='c', consequences='q', actions='a',
                          args=[('string', 'x')], source='src')],
            clock={'offsetMs': -1250, 'measuredAt': frames.iso(1), 'problem': None},
            error=error))


def test_the_installed_converter_names_its_versions(converter):
    assert converter.schema_version == MAPPING['schemaVersion'] == frames.SCHEMA_VERSION
    assert converter.definitions_sha256 == MAPPING['definitionsSha256']
    assert len(converter.definitions_sha256) == 64
    assert int(converter.definitions_sha256, 16) >= 0


def test_the_installed_mapping_is_the_one_of_the_source_tree():
    source = PACKAGE_ROOT.parent / PACKAGE / 'schema' / 'v0.mapping.json'
    if not source.is_file():
        pytest.skip('the message package is not beside this package')
    with open(source, encoding='utf-8') as handle:
        assert json.load(handle) == MAPPING


def test_a_converter_from_the_mapping_alone_imports_the_message_module():
    by_name = convert.Converter(MAPPING)
    message = by_name.to_msg('MotionJoint', frames.joint('J2', 12.5))
    assert type(message) is importlib.import_module(f'{PACKAGE}.msg').MotionJoint
    assert (message.name, message.value, message.unit) == ('J2', 12.5, 'deg')


def test_the_edge_controller_survives_the_round_trip(converter):
    message = converter.to_msg('ControllerSnapshot', frames.EDGE_CONTROLLER, STAMP)
    assert _back('ControllerSnapshot', message) == frames.EDGE_CONTROLLER
    assert message.identity == []
    assert (message.header.stamp.sec, message.header.stamp.nanosec) == STAMP


def test_the_default_controller_survives_the_round_trip(converter):
    data = frames.controller()
    message = converter.to_msg('ControllerSnapshot', data, STAMP)
    assert _back('ControllerSnapshot', message) == data
    assert len(message.identity) == 1


def test_a_controller_with_everything_set_survives_the_round_trip(converter):
    data = _full_controller()
    assert frames.schema_errors('ControllerSnapshot', data) == []
    message = converter.to_msg('ControllerSnapshot', data, STAMP)
    assert _back('ControllerSnapshot', message) == data
    assert message.link.reconnect[0].failures == 2
    assert message.motion.units[0].pose[0].tool == 'tool0'
    assert message.io.signals[0].error[0].controller[0].code == -1073445879


def test_the_edge_controller_passes_through_the_wire_format(converter):
    from rclpy.serialization import deserialize_message, serialize_message

    message = converter.to_msg('ControllerSnapshot', frames.EDGE_CONTROLLER, STAMP)
    received = deserialize_message(serialize_message(message), type(message))
    assert _back('ControllerSnapshot', received) == frames.EDGE_CONTROLLER
    assert (received.header.stamp.sec, received.header.stamp.nanosec) == STAMP
    assert received.header.frame_id == ''


def test_the_header_carries_the_stamp_and_no_frame(converter):
    message = converter.to_msg('ControllerSnapshot', frames.controller(), (T0_SEC, 1))
    assert message.header.stamp.sec == T0_SEC
    assert message.header.stamp.nanosec == 1
    assert message.header.frame_id == ''
    last = converter.to_msg('ControllerSnapshot', frames.controller(), (2**31 - 1, 999999999))
    assert (last.header.stamp.sec, last.header.stamp.nanosec) == (2**31 - 1, 999999999)
    from_a_list = converter.to_msg('ControllerSnapshot', frames.controller(), [5, 6])
    assert (from_a_list.header.stamp.sec, from_a_list.header.stamp.nanosec) == (5, 6)


def test_only_the_root_message_is_marked_with_a_header():
    assert [name for name, message in MESSAGES.items() if message.get('header')] == [
        'ControllerSnapshot']


def test_a_message_with_a_header_needs_a_stamp(converter):
    with pytest.raises(ValueError) as caught:
        converter.to_msg('ControllerSnapshot', frames.controller())
    assert not isinstance(caught.value, convert.ConversionError)
    assert 'stamp' in str(caught.value)


@pytest.mark.parametrize('stamp', [
    5, 5.0, '5', (5,), (5, 6, 7), (5.0, 6), (5, 6.0), ('5', '6'), (True, 0), (-1, 0), (5, -1),
    (5, 1000000000), (2**31, 0), {'sec': 5, 'nanosec': 6},
])
def test_a_stamp_that_is_no_ros_time_is_a_mistake_of_the_caller(converter, stamp):
    with pytest.raises(ValueError) as caught:
        converter.to_msg('ControllerSnapshot', frames.controller(), stamp)
    assert not isinstance(caught.value, convert.ConversionError)


def test_a_message_without_a_header_takes_no_stamp(converter):
    assert converter.to_msg('MotionJoint', frames.joint()).name == 'J1'
    with pytest.raises(ValueError) as caught:
        converter.to_msg('MotionJoint', frames.joint(), STAMP)
    assert not isinstance(caught.value, convert.ConversionError)


@pytest.mark.parametrize('type_name', ['', 'Nothing', 'controllersnapshot', 'Header', 'Time',
                                       'Generation', 'RefusalDetails', None])
def test_an_unknown_message_name_is_a_mistake_of_the_caller(converter, type_name):
    with pytest.raises(ValueError) as caught:
        converter.to_msg(type_name, frames.controller(), STAMP)
    assert not isinstance(caught.value, convert.ConversionError)


def test_a_conversion_error_is_a_value_error_that_names_the_place(converter):
    data = frames.controller()
    data['motion']['units'][0]['joints'][2]['value'] = SENTINEL
    with pytest.raises(ValueError) as caught:
        converter.to_msg('ControllerSnapshot', data, STAMP)
    error = caught.value
    assert isinstance(error, convert.ConversionError)
    assert error.path == 'ControllerSnapshot.motion.units[0].joints[2].value'
    assert str(error) == f'{error.path}: {error.reason}'
    assert 'number' in error.reason
    assert SENTINEL not in str(error)


def test_a_conversion_error_can_be_copied_and_pickled():
    error = convert.ConversionError('Thing.n', 'a number is required, found text')
    assert error.args == ('Thing.n', 'a number is required, found text')
    for other in (copy.copy(error), pickle.loads(pickle.dumps(error))):
        assert type(other) is convert.ConversionError
        assert (other.path, other.reason) == (error.path, error.reason)
        assert str(other) == 'Thing.n: a number is required, found text'


@pytest.mark.parametrize('type_name, key', FIELDS, ids=str)
def test_every_mapped_field_takes_a_value_of_its_kind(converter, type_name, key):
    entry = _entry(type_name, key)
    data = _sample(type_name)
    message = _convert(converter, type_name, data)
    assert _field_back(entry, message) == data[key]
    if 'known' in entry:
        assert getattr(message, entry['known']) is True
    assert _back(type_name, message) == data


@pytest.mark.parametrize('type_name, key', FIELDS, ids=str)
@pytest.mark.parametrize('absent', [False, True], ids=['null', 'absent'])
def test_every_mapped_field_treats_null_and_absence_by_its_rule(converter, type_name, key,
                                                                absent):
    entry = _entry(type_name, key)
    data = _sample(type_name)
    if absent:
        del data[key]
    else:
        data[key] = None
    refused = entry['required'] if absent else not _may_miss(entry)
    if refused:
        with pytest.raises(convert.ConversionError) as caught:
            _convert(converter, type_name, data)
        assert caught.value.path == f'{type_name}.{key}'
        assert ('missing' in caught.value.reason) == absent
        return
    message = _convert(converter, type_name, data)
    value = getattr(message, entry['ros'])
    kind = entry['kind']
    if 'known' in entry:
        assert getattr(message, entry['known']) is False
    if kind in ('string', 'enum', 'json'):
        assert value == ''
    elif kind == 'time':
        assert (value.sec, value.nanosec) == (0, 0)
    elif kind == 'float':
        assert math.isnan(value)
    elif kind == 'int':
        assert value == 0
    elif kind == 'bool':
        assert value == -1 == getattr(type(message), f'{entry["ros"].upper()}_UNKNOWN')
    else:
        assert kind == 'msg'
        assert value == []
    assert _field_back(entry, message) is None


@pytest.mark.parametrize('type_name, key', TYPED_FIELDS, ids=str)
def test_every_mapped_field_refuses_a_value_of_another_type(converter, type_name, key):
    entry = _entry(type_name, key)
    data = _sample(type_name)
    data[key] = WRONG[entry['kind']]
    with pytest.raises(convert.ConversionError) as caught:
        _convert(converter, type_name, data)
    assert caught.value.path == f'{type_name}.{key}'
    assert SENTINEL not in str(caught.value)


def test_the_mapping_uses_every_kind_the_converter_knows():
    kinds = set()

    def collect(entry):
        kinds.add(entry['kind'])
        if entry['kind'] == 'array':
            collect(entry['items'])

    for message in MESSAGES.values():
        for entry in message['fields']:
            collect(entry)
    assert kinds == {'string', 'enum', 'time', 'float', 'int', 'bool', 'msg', 'array', 'tuple',
                     'json'}


def test_a_joint_stays_in_the_unit_of_the_daemon(converter):
    message = converter.to_msg('ControllerSnapshot', frames.EDGE_CONTROLLER, STAMP)
    joint_class = type(message.motion.units[0].joints[0])
    joints = {joint.name: joint for joint in message.motion.units[0].joints}
    # Degrees stay degrees and millimetres stay millimetres: nothing here is SI.
    assert (joints['J1'].value, joints['J1'].unit) == (10.0, joint_class.UNIT_DEG)
    assert joints['J1'].value != pytest.approx(math.radians(10.0))
    assert (joints['J2'].value, joints['J2'].min, joints['J2'].max) == (-20.5, -180.0, 180.0)
    assert (joints['E1'].value, joints['E1'].unit) == (1250.0, joint_class.UNIT_MM)
    assert joints['E1'].max == 5000.0
    # A joint without a value: NaN, never zero.
    assert math.isnan(joints['J3'].value)
    assert joints['J3'].unit == 'deg'
    # A joint without a unit: the daemon's number, untouched, and no unit named.
    assert joints['J4'].value == 5.0
    assert joints['J4'].unit == ''
    assert math.isnan(joints['J4'].min)
    assert math.isnan(joints['J4'].max)
    # A unit with an error keeps its last values and says so.
    station = message.motion.units[1]
    assert station.joints[0].value == 30.0
    assert station.error[0].code == 'CONTROLLER_ERROR'
    assert (station.error[0].controller[0].http_status, station.error[0].controller[0].code) == (
        503, -1)
    assert station.kind == type(station).KIND_POSITIONER


def test_a_pose_stays_as_the_daemon_gives_it(converter):
    pose = converter.to_msg('ControllerSnapshot', frames.EDGE_CONTROLLER,
                            STAMP).motion.units[0].pose[0]
    assert (pose.x, pose.y, pose.z) == (815.5, -12.25, 962.0)
    assert [float(part) for part in pose.q] == [0.5, 0.0, 0.866025, 0.0]
    assert (pose.tool, pose.tool_known, pose.wobj_known, pose.coord_known) == (
        '', False, False, False)


def test_a_whole_number_where_a_float_is_mapped_becomes_a_float(converter):
    message = converter.to_msg('MotionPose', frames.tcp_pose(500, 0, -600, (1, 0, 0, 0)))
    assert (message.x, message.y, message.z) == (500.0, 0.0, -600.0)
    assert type(message.x) is float
    assert [float(part) for part in message.q] == [1.0, 0.0, 0.0, 0.0]
    joint = converter.to_msg('MotionJoint', frames.joint('J1', 10, 'deg', -180, 180))
    assert (joint.value, joint.min, joint.max) == (10.0, -180.0, 180.0)


@pytest.mark.parametrize('value', [True, False, '1.5', [1.5], {'value': 1.5}])
def test_a_float_field_takes_numbers_only(converter, value):
    with pytest.raises(convert.ConversionError) as caught:
        converter.to_msg('MotionJoint', frames.joint('J1', value))
    assert caught.value.path == 'MotionJoint.value'


def test_a_number_too_large_for_a_float_is_refused(converter):
    with pytest.raises(convert.ConversionError) as caught:
        converter.to_msg('MotionJoint', frames.joint('J1', 10**400))
    assert caught.value.path == 'MotionJoint.value'


def test_a_whole_float_where_an_integer_is_mapped_is_taken(converter):
    message = converter.to_msg('LinkGroup', frames.link_group(port=5466.0))
    assert message.port == 5466
    assert type(message.port) is int


@pytest.mark.parametrize('value', [5466.5, True, '5466', [5466], float('nan'), float('inf'),
                                   2**63, -2**63 - 1])
def test_an_integer_field_takes_whole_numbers_of_64_bits_only(converter, value):
    with pytest.raises(convert.ConversionError) as caught:
        converter.to_msg('LinkGroup', frames.link_group(port=value))
    assert caught.value.path == 'LinkGroup.port'


def test_the_ends_of_the_64_bit_range_fit(converter):
    assert converter.to_msg('LinkGroup', frames.link_group(port=2**63 - 1)).port == 2**63 - 1
    assert converter.to_msg('LinkGroup', frames.link_group(port=-2**63)).port == -2**63


@pytest.mark.parametrize('value', [1, 0, 'true', 'false', [], 1.0])
def test_a_bool_field_takes_true_and_false_only(converter, value):
    with pytest.raises(convert.ConversionError) as caught:
        converter.to_msg('LinkGroup', frames.link_group(connected=value))
    assert caught.value.path == 'LinkGroup.connected'
    with pytest.raises(convert.ConversionError) as caught:
        converter.to_msg('IdentityGroup', frames.identity_group(virtual=value))
    assert caught.value.path == 'IdentityGroup.virtual'


@pytest.mark.parametrize('virtual, expected', [(None, 'IS_VIRTUAL_UNKNOWN'),
                                               (False, 'IS_VIRTUAL_FALSE'),
                                               (True, 'IS_VIRTUAL_TRUE')])
def test_a_bool_that_may_be_null_has_three_states(converter, virtual, expected):
    message = converter.to_msg('IdentityGroup', frames.identity_group(virtual=virtual))
    assert message.is_virtual == getattr(type(message), expected)
    assert type(message.is_virtual) is int


def test_text_with_a_known_flag(converter):
    known = converter.to_msg('PanelGroup', frames.panel_group(ctrl_state='motoron'))
    assert (known.ctrl_state, known.ctrl_state_known) == ('motoron', True)
    unknown = converter.to_msg('PanelGroup', frames.panel_group(ctrl_state=None))
    assert (unknown.ctrl_state, unknown.ctrl_state_known) == ('', False)
    # An empty text the daemon did report is known.
    empty = converter.to_msg('PanelGroup', frames.panel_group(ctrl_state=''))
    assert (empty.ctrl_state, empty.ctrl_state_known) == ('', True)


def test_an_enum_value_the_mapping_does_not_list_is_passed_on(converter):
    message = converter.to_msg('LinkGroup', frames.link_group(quality='degraded'))
    assert message.quality == 'degraded'
    assert 'degraded' not in _entry('LinkGroup', 'quality')['values']
    listed = converter.to_msg('LinkGroup', frames.link_group(quality='polling'))
    assert listed.quality == type(listed).QUALITY_POLLING


def test_a_replayed_link_says_so_and_when_it_was_recorded(converter):
    live = converter.to_msg('LinkGroup', frames.link_group())
    assert (live.mode, live.recorded_at_known) == (type(live).MODE_LIVE, False)
    data = frames.link_group(mode='replay', recorded_at=frames.iso(-3600))
    assert frames.schema_errors('LinkGroup', data) == []
    replayed = converter.to_msg('LinkGroup', data)
    assert replayed.mode == type(replayed).MODE_REPLAY
    assert replayed.quality == type(replayed).QUALITY_LIVE
    assert (replayed.recorded_at.sec, replayed.recorded_at_known) == (T0_SEC - 3600, True)
    assert _back('LinkGroup', replayed) == data


@pytest.mark.parametrize('holder, reported, constant', [
    (frames.access_holder(), 'local', 'FLEX_PENDANT_TRUE'),
    (frames.access_holder('station', name='FlexPendant', application=None), None,
     'FLEX_PENDANT_TRUE'),
    (frames.access_holder('station', name='TPU', application=None, flex_pendant=False), None,
     'FLEX_PENDANT_FALSE'),
    (frames.access_holder('remote', application='RobotStudio', flex_pendant=False), 'remote',
     'FLEX_PENDANT_FALSE'),
    (frames.access_holder(application=None, flex_pendant=None), 'local',
     'FLEX_PENDANT_UNKNOWN'),
])
def test_a_lock_holder_says_whether_it_is_the_flex_pendant(converter, holder, reported,
                                                           constant):
    data = frames.access_lock(held=True, holder=holder, reported=reported)
    assert frames.schema_errors('AccessLockInfo', data) == []
    message = converter.to_msg('AccessLockInfo', data)
    held_by = message.holder[0]
    assert held_by.flex_pendant == getattr(type(held_by), constant)
    assert _back('AccessLockInfo', message) == data


def test_a_time_with_a_known_flag(converter):
    message = converter.to_msg('PanelGroup', frames.panel_group(as_of=frames.iso(2.125)))
    assert (message.as_of.sec, message.as_of.nanosec, message.as_of_known) == (
        T0_SEC + 2, 125000000, True)
    unread = converter.to_msg('PanelGroup', frames.panel_group(as_of=None))
    assert (unread.as_of.sec, unread.as_of.nanosec, unread.as_of_known) == (0, 0, False)


@pytest.mark.parametrize('text', ['', 'yesterday', '2026-01-01 T 00:00:00', '2026-01-01',
                                  '2026-01-01T00:00:00+01:00', '1969-12-31T23:59:59Z',
                                  '2040-01-01T00:00:00Z'])
def test_a_time_that_cannot_be_read_sets_its_flag_false(converter, text):
    message = converter.to_msg('PanelGroup', frames.panel_group(as_of=text))
    assert message.as_of_known is False
    assert (message.as_of.sec, message.as_of.nanosec) == (0, 0)
    # The rest of the message is there.
    assert (message.op_mode, message.speed_ratio) == ('AUTO', 100.0)


@pytest.mark.parametrize('text', ['', 'yesterday', '2026-01-01', '1969-12-31T23:59:59Z',
                                  '2040-01-01T00:00:00Z'])
def test_a_time_without_a_flag_must_be_readable(converter, text):
    with pytest.raises(convert.ConversionError) as caught:
        converter.to_msg('LinkGroup', frames.link_group(since=text))
    assert caught.value.path == 'LinkGroup.since'
    assert 'time' in caught.value.reason


@pytest.mark.parametrize('value', [1767225600, 1767225600.5, True, ['2026-01-01T00:00:00Z'],
                                   {'sec': 1}])
def test_a_time_must_be_text(converter, value):
    with pytest.raises(convert.ConversionError) as caught:
        converter.to_msg('PanelGroup', frames.panel_group(as_of=value))
    assert caught.value.path == 'PanelGroup.asOf'
    with pytest.raises(convert.ConversionError) as caught:
        converter.to_msg('LinkGroup', frames.link_group(since=value))
    assert caught.value.path == 'LinkGroup.since'


def test_the_times_inside_a_message_are_the_daemon_times_to_the_nanosecond(converter):
    message = converter.to_msg(
        'LinkGroup', frames.link_group(since='2026-01-01T00:00:01.123456789Z'))
    assert (message.since.sec, message.since.nanosec) == (T0_SEC + 1, 123456789)


def test_an_object_that_may_be_null_is_a_list_of_none_or_one(converter):
    without = converter.to_msg('ControllerSnapshot', frames.controller(identity=None), STAMP)
    assert without.identity == []
    with_one = converter.to_msg('ControllerSnapshot', frames.controller(), STAMP)
    assert len(with_one.identity) == 1
    assert with_one.identity[0].system_name == 'Test_System'
    assert with_one.identity[0].robot_ware.generation == 'rws2'


def test_an_object_the_schema_does_not_require_may_be_absent(converter):
    bare = converter.to_msg('GroupError', frames.group_error())
    assert (bare.code, bare.controller, bare.refusal) == ('RESOURCE_NOT_FOUND', [], '')
    answered = converter.to_msg(
        'GroupError', frames.group_error(controller=(None, None, None)))
    assert len(answered.controller) == 1
    assert (answered.controller[0].http_status_known, answered.controller[0].code_known,
            answered.controller[0].message_known) == (False, False, False)


def test_free_form_details_become_json_text_with_sorted_keys(converter):
    refusal = {'b': [1, {'z': None, 'a': True}], 'a': 'x', 'c': 1.5}
    message = converter.to_msg('GroupError', frames.group_error(refusal=refusal))
    assert message.refusal == '{"a": "x", "b": [1, {"a": true, "z": null}], "c": 1.5}'
    assert json.loads(message.refusal) == refusal
    same = converter.to_msg('GroupError', frames.group_error(
        refusal={'c': 1.5, 'a': 'x', 'b': [1, {'a': True, 'z': None}]}))
    assert same.refusal == message.refusal
    empty = converter.to_msg('GroupError', frames.group_error(refusal={}))
    assert empty.refusal == '{}'
    for absent in (frames.group_error(), dict(frames.group_error(), refusal=None)):
        assert converter.to_msg('GroupError', absent).refusal == ''


@pytest.mark.parametrize('value', [float('nan'), float('inf'), {'a': float('-inf')},
                                   [1, {'b': float('nan')}]])
def test_details_that_are_not_json_are_refused(converter, value):
    with pytest.raises(convert.ConversionError) as caught:
        converter.to_msg('GroupError', dict(frames.group_error(), refusal=value))
    assert caught.value.path == 'GroupError.refusal'
    assert 'JSON' in caught.value.reason


def test_json_text_is_ascii_whatever_the_daemon_sent(converter):
    message = converter.to_msg('GroupError', frames.group_error(refusal={'fix': 'gr\u00fcn'}))
    assert message.refusal.isascii()
    assert json.loads(message.refusal) == {'fix': 'gr\u00fcn'}


def test_a_list_keeps_its_order_and_its_items(converter):
    options = ['RobotWare Base', 'English', '', 'English']
    message = converter.to_msg('IdentityGroup', frames.identity_group(options=options))
    assert list(message.options) == options
    assert list(converter.to_msg('IdentityGroup',
                                 frames.identity_group(options=())).options) == []


@pytest.mark.parametrize('options', ['English', {'0': 'English'}, 5, True])
def test_a_list_field_takes_lists_only(converter, options):
    data = dict(frames.identity_group(), options=options)
    with pytest.raises(convert.ConversionError) as caught:
        converter.to_msg('IdentityGroup', data)
    assert caught.value.path == 'IdentityGroup.options'


def test_a_wrong_item_is_named_by_its_place_in_the_list(converter):
    data = dict(frames.identity_group(), options=['a', 'b', 5, 'd'])
    with pytest.raises(convert.ConversionError) as caught:
        converter.to_msg('IdentityGroup', data)
    assert caught.value.path == 'IdentityGroup.options[2]'
    data = dict(frames.identity_group(), options=['a', None])
    with pytest.raises(convert.ConversionError) as caught:
        converter.to_msg('IdentityGroup', data)
    assert caught.value.path == 'IdentityGroup.options[1]'
    group = frames.io_group([frames.signal('a'), 'not a signal'])
    with pytest.raises(convert.ConversionError) as caught:
        converter.to_msg('IoGroup', group)
    assert caught.value.path == 'IoGroup.signals[1]'


def test_a_list_up_to_its_bound_fits_and_one_more_is_refused_not_cut(converter):
    bound = _entry('IoGroup', 'signals')['max']
    assert bound == 24
    signals = [frames.signal(f'do_{index}') for index in range(bound + 1)]
    full = converter.to_msg('IoGroup', frames.io_group(signals[:bound]))
    assert [entry.name for entry in full.signals] == [f'do_{index}' for index in range(bound)]
    with pytest.raises(convert.ConversionError) as caught:
        converter.to_msg('IoGroup', frames.io_group(signals))
    assert caught.value.path == 'IoGroup.signals'
    assert '25' in caught.value.reason
    assert '24' in caught.value.reason

    bound = _entry('EventsGroup', 'recent')['max']
    assert bound == 20
    events = [frames.event(seq) for seq in range(1, bound + 2)]
    assert len(converter.to_msg('EventsGroup', frames.events_group(events[:bound])).recent) == 20
    with pytest.raises(convert.ConversionError) as caught:
        converter.to_msg('ControllerSnapshot',
                         frames.controller(events=frames.events_group(events)), STAMP)
    assert caught.value.path == 'ControllerSnapshot.events.recent'


@pytest.mark.parametrize('q', [(), (1.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0, 0.0)])
def test_a_tuple_has_exactly_its_size(converter, q):
    with pytest.raises(convert.ConversionError) as caught:
        converter.to_msg('MotionPose', frames.tcp_pose(q=q))
    assert caught.value.path == 'MotionPose.q'
    assert '4' in caught.value.reason


@pytest.mark.parametrize('bad', [None, 'x', True, [0.0]])
def test_a_tuple_holds_numbers_only(converter, bad):
    with pytest.raises(convert.ConversionError) as caught:
        converter.to_msg('MotionPose', frames.tcp_pose(q=(1.0, 0.0, bad, 0.0)))
    assert caught.value.path == 'MotionPose.q[2]'


@pytest.mark.parametrize('data', [None, [], 'MotionJoint', 5, [frames.joint()]])
def test_a_message_is_built_from_an_object_only(converter, data):
    with pytest.raises(convert.ConversionError) as caught:
        converter.to_msg('MotionJoint', data)
    assert caught.value.path == 'MotionJoint'
    broken = frames.controller()
    broken['motion']['units'][0]['joints'][4] = data
    with pytest.raises(convert.ConversionError) as caught:
        converter.to_msg('ControllerSnapshot', broken, STAMP)
    assert caught.value.path == 'ControllerSnapshot.motion.units[0].joints[4]'


@pytest.mark.parametrize('key', ['id', 'name', 'identity', 'link', 'panel', 'rapid', 'motion',
                                 'io', 'events'])
def test_a_controller_without_one_of_its_keys_is_not_converted(converter, key):
    data = frames.controller()
    del data[key]
    with pytest.raises(convert.ConversionError) as caught:
        converter.to_msg('ControllerSnapshot', data, STAMP)
    assert caught.value.path == f'ControllerSnapshot.{key}'
    assert 'missing' in caught.value.reason


def test_a_missing_key_deep_inside_is_named_by_its_path(converter):
    data = frames.edge_controller()
    del data['motion']['units'][0]['joints'][6]['unit']
    with pytest.raises(convert.ConversionError) as caught:
        converter.to_msg('ControllerSnapshot', data, STAMP)
    assert caught.value.path == 'ControllerSnapshot.motion.units[0].joints[6].unit'
    data = frames.edge_controller()
    del data['io']['signals'][6]['error']['message']
    with pytest.raises(convert.ConversionError) as caught:
        converter.to_msg('ControllerSnapshot', data, STAMP)
    assert caught.value.path == 'ControllerSnapshot.io.signals[6].error.message'


def test_unknown_keys_are_ignored_at_every_depth(converter):
    plain = frames.controller()
    newer = frames.controller()
    newer['diag'] = {'load': 3}
    newer['label'] = 'x'
    newer['link']['latencyMs'] = 4
    newer['motion']['units'][0]['payload'] = {'kg': 1.5}
    newer['motion']['units'][0]['joints'][0]['torque'] = 0.25
    newer['identity']['robotWare']['patch'] = 3
    assert converter.to_msg('ControllerSnapshot', newer, STAMP) == (
        converter.to_msg('ControllerSnapshot', plain, STAMP))
    # A key that only looks like a mapped one is unknown too.
    newer['motion']['as_of'] = 'not the key the daemon uses'
    newer['Name'] = 'not the name'
    assert converter.to_msg('ControllerSnapshot', newer, STAMP) == (
        converter.to_msg('ControllerSnapshot', plain, STAMP))


def test_the_data_is_left_as_it_was_and_two_messages_share_nothing(converter):
    data = frames.edge_controller()
    as_it_was = copy.deepcopy(data)
    first = converter.to_msg('ControllerSnapshot', data, STAMP)
    second = converter.to_msg('ControllerSnapshot', data, STAMP)
    assert data == as_it_was
    assert first.motion is not second.motion
    assert first.motion.units[0].joints[0] is not second.motion.units[0].joints[0]
    assert first.header.stamp is not second.header.stamp
    first.motion.units[0].joints[0].value = 99.0
    first.io.signals.clear()
    assert second.motion.units[0].joints[0].value == 10.0
    assert len(second.io.signals) == 7
    assert data == as_it_was


def test_an_error_never_repeats_a_value(converter):
    for key, value in [('name', 12345678), ('link', SENTINEL), ('motion', [SENTINEL])]:
        data = dict(frames.controller(), **{key: value})
        with pytest.raises(convert.ConversionError) as caught:
            converter.to_msg('ControllerSnapshot', data, STAMP)
        assert caught.value.path == f'ControllerSnapshot.{key}'
        assert SENTINEL not in str(caught.value)
        assert '12345678' not in str(caught.value)
    data = frames.controller()
    data['panel']['asOf'] = {SENTINEL: SENTINEL}
    with pytest.raises(convert.ConversionError) as caught:
        converter.to_msg('ControllerSnapshot', data, STAMP)
    assert SENTINEL not in str(caught.value)


def test_the_pure_modules_load_nothing_from_ros_and_no_other_bridge_module():
    code = '\n'.join([
        'import sys',
        'from abb_rws_bridge import convert, stamps, state, timeconv',
        'mine = {"abb_rws_bridge." + name for name in ("convert", "stamps", "state", "timeconv")}',
        'mine.add("abb_rws_bridge")',
        'watched = ("abb_rws_bridge", "abb_rws_ros2_msgs", "ament_index_python",',
        '           "builtin_interfaces", "rclpy", "std_msgs", "websocket")',
        'loaded = {name for name in sys.modules if name.split(".")[0] in watched}',
        'print(sorted(loaded - mine))',
        'converter = convert.Converter.from_installed()',
        'joint = {"name": "J1", "value": 1.0, "unit": "deg", "min": None, "max": None}',
        'converter.to_msg("MotionJoint", joint)',
        'converter.to_msg("ReconnectInfo",',
        '                 {"failures": 1, "nextAttemptAt": "2026-01-01T00:00:00.000Z"})',
        'fleet = state.FleetState(converter.schema_version)',
        'print(sorted(name for name in sys.modules if name.split(".")[0] == "rclpy"))',
        'print(sorted(set(name for name in sys.modules',
        '                 if name.startswith("abb_rws_bridge")) - mine))',
    ])
    result = subprocess.run([sys.executable, '-c', code], cwd=PACKAGE_ROOT,
                            capture_output=True, text=True, timeout=60, check=False)
    assert result.returncode == 0, result.stderr
    assert result.stdout.split('\n')[:3] == ['[]', '[]', '[]']


# What follows uses a hand-made mapping and stand-in classes: the shapes the generator can
# write that the daemon's schema does not use today, and what a mapping must not hold.

class _Bag:
    """A stand-in for a message class: it takes any field."""

    def __init__(self):
        self.header = types.SimpleNamespace(stamp=None, frame_id='')


class _Strict:
    """A stand-in for a message class that checks what it is given, as the real ones do."""

    __slots__ = ('_count',)

    def __init__(self):
        self._count = 0

    @property
    def count(self):
        return self._count

    @count.setter
    def count(self, value):
        if value > 10:
            raise AssertionError('count too large')
        self._count = value


def _field(json_name, kind, *, required=True, nullable=False, **more):
    return dict(json=json_name, ros=json_name.lower(), kind=kind, required=required,
                nullable=nullable, **more)


def _mapping(fields, *, header=False, more=None):
    messages = {'Thing': {'fields': fields}, 'Part': {'fields': [_field('n', 'int')]}}
    if header:
        messages['Thing']['header'] = True
    messages.update(more or {})
    return {'schemaVersion': '0.1', 'definitionsSha256': 'abc123', 'package': 'nowhere',
            'messages': messages}


def _made(fields, **more):
    module = types.SimpleNamespace(Thing=_Bag, Part=_Bag)
    return convert.Converter(_mapping(fields, **more), module)


LISTS = [
    _field('texts', 'array', items={'kind': 'string'}),
    _field('modes', 'array', items={'kind': 'enum', 'values': ['a', 'b']}),
    _field('times', 'array', items={'kind': 'time'}),
    _field('numbers', 'array', items={'kind': 'float'}),
    _field('counts', 'array', items={'kind': 'int'}, max=3),
    _field('flags', 'array', items={'kind': 'bool'}),
    _field('blobs', 'array', items={'kind': 'json'}),
    _field('parts', 'array', items={'kind': 'msg', 'msg': 'Part'}),
]
LIST_DATA = {
    'texts': ['a', ''], 'modes': ['b', 'a'], 'times': [frames.T0, frames.iso(1.5)],
    'numbers': [1, 2.5], 'counts': [1, 2, 3], 'flags': [True, False],
    'blobs': [{'b': 1, 'a': 2}, [1, 2], 'x'], 'parts': [{'n': 1}, {'n': 2, 'extra': 0}],
}


def test_a_hand_made_converter_names_its_versions():
    made = _made([_field('n', 'int')])
    assert (made.schema_version, made.definitions_sha256) == ('0.1', 'abc123')
    assert made.to_msg('Thing', {'n': 4}).n == 4


def test_lists_of_every_item_kind():
    message = _made(LISTS).to_msg('Thing', LIST_DATA)
    assert message.texts == ['a', '']
    assert message.modes == ['b', 'a']
    assert [(stamp.sec, stamp.nanosec) for stamp in message.times] == [
        (T0_SEC, 0), (T0_SEC + 1, 500000000)]
    assert message.numbers == [1.0, 2.5]
    assert all(type(number) is float for number in message.numbers)
    assert message.counts == [1, 2, 3]
    assert message.flags == [True, False]
    assert message.blobs == ['{"a": 2, "b": 1}', '[1, 2]', '"x"']
    assert [part.n for part in message.parts] == [1, 2]
    assert all(isinstance(part, _Bag) for part in message.parts)
    empty = _made(LISTS).to_msg('Thing', {key: [] for key in LIST_DATA})
    assert all(getattr(empty, key) == [] for key in LIST_DATA)


@pytest.mark.parametrize('key, bad, place', [
    ('texts', ['a', 5], 'texts[1]'),
    ('modes', [None], 'modes[0]'),
    ('times', [frames.T0, 'soon'], 'times[1]'),
    ('times', [5], 'times[0]'),
    ('numbers', [1.0, 2.0, '3'], 'numbers[2]'),
    ('numbers', [None], 'numbers[0]'),
    ('counts', [1, 2.5], 'counts[1]'),
    ('counts', [1, 2, 3, 4], 'counts'),
    ('flags', [True, 1], 'flags[1]'),
    ('parts', [{'n': 1}, {}], 'parts[1].n'),
    ('parts', [{'n': 1}, None], 'parts[1]'),
    ('parts', {'n': 1}, 'parts'),
])
def test_a_list_item_that_does_not_fit_is_refused(key, bad, place):
    with pytest.raises(convert.ConversionError) as caught:
        _made(LISTS).to_msg('Thing', dict(LIST_DATA, **{key: bad}))
    assert caught.value.path == f'Thing.{place}'


def test_a_hand_made_root_gets_its_stamp():
    made = _made([_field('n', 'int')], header=True)
    message = made.to_msg('Thing', {'n': 1}, (7, 8))
    assert (message.header.stamp.sec, message.header.stamp.nanosec) == (7, 8)
    assert message.header.frame_id == ''
    with pytest.raises(ValueError):
        made.to_msg('Thing', {'n': 1})
    with pytest.raises(ValueError):
        made.to_msg('Part', {'n': 1}, (7, 8))


def test_a_field_that_may_be_absent_or_null_in_a_hand_made_mapping():
    made = _made([
        _field('a', 'int', required=False, known='a_known'),
        _field('b', 'float', required=False),
        _field('c', 'bool', nullable=True, tristate=True),
        _field('d', 'msg', nullable=True, msg='Part', optional=True),
        _field('e', 'time', required=False, nullable=True, known='e_known'),
        _field('f', 'enum', nullable=True, values=['x']),
    ])
    message = made.to_msg('Thing', {'c': None, 'd': None, 'f': None})
    assert (message.a, message.a_known) == (0, False)
    assert math.isnan(message.b)
    assert message.c == -1
    assert message.d == []
    assert (message.e.sec, message.e.nanosec, message.e_known) == (0, 0, False)
    assert message.f == ''
    message = made.to_msg('Thing', {'a': 5, 'b': 2, 'c': False, 'd': {'n': 9},
                                    'e': frames.iso(2), 'f': 'x'})
    assert (message.a, message.a_known, message.b, message.c) == (5, True, 2.0, 0)
    assert [part.n for part in message.d] == [9]
    assert (message.e.sec, message.e_known, message.f) == (T0_SEC + 2, True, 'x')


def test_a_message_class_that_refuses_a_value_is_a_conversion_error():
    module = types.SimpleNamespace(Thing=_Strict, Part=_Bag)
    made = convert.Converter(_mapping([_field('count', 'int')]), module)
    assert made.to_msg('Thing', {'count': 10}).count == 10
    with pytest.raises(convert.ConversionError) as caught:
        made.to_msg('Thing', {'count': 11})
    assert caught.value.path == 'Thing.count'
    # A field the class does not have: the mapping and the messages do not belong together.
    made = convert.Converter(_mapping([_field('other', 'int')]), module)
    with pytest.raises(convert.ConversionError) as caught:
        made.to_msg('Thing', {'other': 1})
    assert caught.value.path == 'Thing.other'


@pytest.mark.parametrize('fields', [
    [_field('a', 'number')],
    [_field('a', None)],
    [dict(_field('a', 'int'), kind='object')],
    [_field('a', 'msg', msg='Nowhere')],
    [_field('a', 'msg')],
    [_field('a', 'array')],
    [_field('a', 'array', items='string')],
    [_field('a', 'array', items={'kind': 'array', 'items': {'kind': 'int'}})],
    [_field('a', 'array', items={'kind': 'tuple', 'size': 2})],
    [_field('a', 'array', items={'kind': 'msg', 'msg': 'Nowhere'})],
    [_field('a', 'array', items={'kind': 'bits'})],
    [_field('a', 'array', items={'kind': 'int'}, max=0)],
    [_field('a', 'array', items={'kind': 'int'}, max='3')],
    [_field('a', 'array', items={'kind': 'int'}, max=True)],
    [_field('a', 'tuple')],
    [_field('a', 'tuple', size=0)],
    [_field('a', 'tuple', size='4')],
    [_field('a', 'string', nullable=True)],
    [_field('a', 'int', required=False)],
    [_field('a', 'time', nullable=True)],
    [_field('a', 'int', nullable=True, known='')],
    [_field('a', 'int', known=5)],
    [_field('a', 'bool', nullable=True)],
    [_field('a', 'bool', required=False, tristate='yes')],
    [_field('a', 'msg', msg='Part', nullable=True)],
    [_field('a', 'msg', msg='Part', required=False)],
    [_field('a', 'array', items={'kind': 'int'}, nullable=True)],
    [_field('a', 'tuple', size=2, required=False)],
    [{'ros': 'a', 'kind': 'int', 'required': True, 'nullable': False}],
    [{'json': 'a', 'kind': 'int', 'required': True, 'nullable': False}],
    [{'json': 'a', 'ros': '', 'kind': 'int', 'required': True, 'nullable': False}],
    [{'json': 'a', 'ros': 'a', 'kind': 'int', 'nullable': False}],
    [{'json': 'a', 'ros': 'a', 'kind': 'int', 'required': True}],
    [{'json': 'a', 'ros': 'a', 'kind': 'int', 'required': 'yes', 'nullable': False}],
    ['a'],
    [None],
])
def test_a_mapping_the_converter_has_no_rule_for_is_refused_at_once(fields):
    with pytest.raises(ValueError) as caught:
        _made(fields)
    assert not isinstance(caught.value, convert.ConversionError)


@pytest.mark.parametrize('change', [
    {'messages': None},
    {'messages': {}},
    {'messages': ['Thing']},
    {'messages': {'Thing': None}},
    {'messages': {'Thing': {}}},
    {'messages': {'Thing': {'fields': 'none'}}},
    {'messages': {'Thing': {'fields': []}, 'Missing': {'fields': []}}},
    {'schemaVersion': None},
    {'schemaVersion': 0.12},
    {'definitionsSha256': None},
])
def test_a_mapping_that_is_not_shaped_as_the_generator_writes_it_is_refused(change):
    mapping = dict(_mapping([_field('n', 'int')]), **change)
    module = types.SimpleNamespace(Thing=_Bag, Part=_Bag)
    with pytest.raises(ValueError):
        convert.Converter(mapping, module)


@pytest.mark.parametrize('mapping', [None, [], 'mapping', 5])
def test_a_mapping_must_be_an_object(mapping):
    with pytest.raises(ValueError):
        convert.Converter(mapping, types.SimpleNamespace())


@pytest.mark.parametrize('package', [None, '', 5])
def test_without_a_module_the_mapping_must_name_its_package(package):
    mapping = dict(_mapping([_field('n', 'int')]), package=package)
    with pytest.raises(ValueError):
        convert.Converter(mapping)


def test_a_package_that_is_not_installed_cannot_be_imported():
    with pytest.raises(ImportError):
        convert.Converter(_mapping([_field('n', 'int')]))

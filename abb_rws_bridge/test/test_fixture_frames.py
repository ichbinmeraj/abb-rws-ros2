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
The recorded fixtures against the schema, the state, the messages and the joint rules.

A fixture is a sanitized, frames-only recording of the abb-rws-daemon stream (see
``fixtures/README.md``). The fixture tests read every ``*.jsonl`` in the fixtures folder:
the folder named by ABB_RWS_TEST_FIXTURES when that is set, otherwise ``fixtures`` beside
this file. With no recording there they are skipped and say so.

Every check is a function that takes one recording. The second half of this file runs
the same checks, and the reader itself, on recordings written by hand from the ``frames``
builders: what must pass does pass and what a check is for is caught, also while the
fixtures folder is empty.
"""

import collections
import json
import math

from abb_rws_bridge import convert
from abb_rws_bridge import joint_state
from abb_rws_bridge import names
from abb_rws_bridge import parameters
from abb_rws_bridge import state
from abb_rws_bridge import timeconv
import frames
import pytest
import recording

# The joint_name_format a node has when nobody sets one: ROB_1 and J1 give ROB_1_J1.
DEFAULT_JOINT_NAMES = next(
    spec.default for spec in parameters.SPECS if spec.name == 'joint_name_format')
# How far a published position may lie from value * pi / 180 (or value / 1000).
TOLERANCE = 1e-12
WALL_NS = 1_700_000_000_000_000_000
MARKER = 'MARKER-in-the-line'

_FRAME_DEFINITIONS = {
    'snapshot': 'SnapshotFrame', 'delta': 'DeltaFrame', 'removed': 'RemovedFrame'}
# How much of one schema error is shown, and how many problems of one recording.
_ERROR_LENGTH = 160
_PROBLEMS_SHOWN = 8


def refuse(path, text):
    """Fail a check: name the recording and say what is wrong with it."""
    raise AssertionError(f'{path.name}: {text}')


def applied(path, messages_schema):
    """Yield (frame number, frame, what applying it did, the state after it)."""
    fleet = state.FleetState(messages_schema)
    for number, (seconds, frame) in enumerate(recording.read_frames(path), start=1):
        done = fleet.apply(frame, WALL_NS + round(seconds * 1e9), seconds)
        yield number, frame, done, fleet


# ----- the checks: each takes one recording ---------------------------------------------

def check_header(path, schema):
    """Refuse a recording that is not a sanitized, frames-only one of the vendored schema."""
    vendored = schema['x-schema-version']
    header = recording.read_header(path)
    if header['schemaVersion'] != vendored:
        refuse(path, f'was made with schema {header["schemaVersion"]}; the vendored schema '
                     f'is {vendored}: record it again')
    if header.get('sanitized') is not True:
        refuse(path, 'its header does not say that it was sanitized')
    if header.get('content') != ['frames']:
        refuse(path, 'its header names other content than frames')
    kinds = collections.Counter(record['kind'] for _, record in recording.read_records(path))
    others = sorted(kind[:24] for kind in kinds if kind not in ('frame', 'end'))
    if others:
        refuse(path, 'holds records that are neither a frame nor the end record: '
                     + ', '.join(others))
    if not kinds['frame']:
        refuse(path, 'holds no frame')
    if kinds['end'] > 1:
        refuse(path, 'holds more than one end record')
    if b'\r' in path.read_bytes():
        refuse(path, 'has line endings other than LF')


def check_frames(path, schema):
    """Refuse a recording in which a frame does not validate against the vendored schema."""
    vendored = schema['x-schema-version']
    problems = []
    for number, (_, frame) in enumerate(recording.read_frames(path), start=1):
        kind = frame['type']
        if number == 1 and kind != 'snapshot':
            problems.append('frame 1 is not a snapshot')
        if kind == 'snapshot' and frame.get('schemaVersion') != vendored:
            problems.append(f'frame {number} is a snapshot of another schema version')
        errors = frames.schema_errors('StreamFrame', frame, schema)
        if errors and kind in _FRAME_DEFINITIONS:
            # The definition of the frame's own type says where it does not fit.
            errors = frames.schema_errors(_FRAME_DEFINITIONS[kind], frame, schema) or errors
        elif errors:
            errors = ['a frame type the schema does not have']
        problems.extend(f'frame {number}: {error[:_ERROR_LENGTH]}' for error in errors)
        if len(problems) >= _PROBLEMS_SHOWN:
            break
    if problems:
        refuse(path, 'does not fit the vendored schema: ' + '; '.join(problems[:_PROBLEMS_SHOWN]))


def check_application(path, schema):
    """Refuse a recording whose frames the state does not take without a problem."""
    fleet = None
    for number, frame, done, fleet in applied(path, schema['x-schema-version']):
        if done.problem is not None:
            refuse(path, f'frame {number}: {done.problem}')
        if done.kind != frame['type']:
            refuse(path, f'frame {number} was not applied ({done.kind})')
        if fleet.incomplete():
            refuse(path, f'frame {number} leaves a controller held back')
    if fleet is None or not fleet.ids():
        refuse(path, 'holds no controller at its end')
    if fleet.unknown_frames:
        refuse(path, f'holds {fleet.unknown_frames} frame(s) of a type the state does not know')


def _message_problem(message, data, stamp):
    """Return what a ControllerSnapshot message lost of its data; None when nothing."""
    if (message.id, message.name) != (data['id'], data['name']):
        return 'the message has another id or name'
    if (message.header.stamp.sec, message.header.stamp.nanosec) != stamp:
        return 'the message has another header stamp'
    units = data['motion']['units']
    if len(message.motion.units) != len(units):
        return 'the message has another number of mechanical units'
    for unit_message, unit in zip(message.motion.units, units):
        if len(unit_message.joints) != len(unit['joints']):
            return f'unit {unit["name"]} has another number of joints in the message'
        for joint_message, joint in zip(unit_message.joints, unit['joints']):
            where = f'joint {unit["name"]}/{joint["name"]}'
            if (joint_message.name, joint_message.unit) != (joint['name'], joint['unit'] or ''):
                return f'{where} has another name or unit in the message'
            value = joint['value']
            if value is None:
                kept = math.isnan(joint_message.value)
            else:
                kept = joint_message.value == value
            if not kept:
                return f'{where} does not hold the value the daemon reported'
    return None


def check_messages(path, converter):
    """
    Refuse a recording with a controller that does not become a ControllerSnapshot.

    Every controller is converted after every frame that changed it. The message keeps
    the daemon's own units: a joint holds the number the daemon reported, in degrees or
    millimetres, and names that unit.
    """
    converted = 0
    for number, frame, done, fleet in applied(path, converter.schema_version):
        for controller_id in sorted(done.changed):
            data = fleet.get(controller_id)
            if data is None:
                continue
            where = f'frame {number}: controller {controller_id}'
            try:
                stamp = timeconv.iso_to_sec_nanosec(frame.get('observedAt'))
                message = converter.to_msg('ControllerSnapshot', data, stamp)
            except ValueError as error:
                refuse(path, f'{where} does not convert: {error}')
            problem = _message_problem(message, data, stamp)
            if problem is not None:
                refuse(path, f'{where}: {problem}')
            converted += 1
    if not converted:
        refuse(path, 'holds no controller to convert')


def expected_joints(motion):
    """
    Return (name, position, unit) of every joint a JointState must hold, by the rule.

    Degrees become radians and millimetres metres. A joint without a value, a joint
    whose unit is unknown and every joint of a unit that reports an error are left out.
    The name is the default one: unit and joint, joined by an underscore.
    """
    expected = []
    for unit in motion['units']:
        if unit['error'] is not None:
            continue
        for joint in unit['joints']:
            value, kind = joint['value'], joint['unit']
            if value is None or kind not in ('deg', 'mm'):
                continue
            position = value * math.pi / 180 if kind == 'deg' else value / 1000.0
            expected.append((f'{unit["name"]}_{joint["name"]}', position, kind))
    return expected


def check_joints(path, schema):
    """
    Refuse a recording whose joints would not be published by the rule.

    After every frame that carries a motion group: the JointState content holds exactly
    the joints of ``expected_joints``, in the daemon's order, under their default names,
    each position within 1e-12 of ``value * pi / 180`` (or ``value / 1000``). A recording
    without one joint in degrees checks nothing and is refused too.
    """
    namer = names.JointNamer(DEFAULT_JOINT_NAMES)
    degrees = 0
    for number, _, done, fleet in applied(path, schema['x-schema-version']):
        tokens, _ = names.controller_tokens(fleet.ids())
        for controller_id in sorted(done.changed):
            data = fleet.get(controller_id)
            if data is None or 'motion' not in done.changed[controller_id]:
                continue
            motion = data['motion']
            where = f'frame {number}: controller {controller_id}'
            built = joint_state.build(
                controller_id, tokens.get(controller_id, ''), motion, namer)
            if motion['asOf'] is None:
                if built is not None:
                    refuse(path, f'{where}: joints although the motion group has no time')
                continue
            expected = expected_joints(motion)
            if built is None or built.stamp != motion['asOf']:
                refuse(path, f'{where}: the joints do not carry the time of the motion group')
            if list(built.names) != [name for name, _, _ in expected]:
                refuse(path, f'{where}: the joint names are not the default ones')
            for position, (name, wanted, _) in zip(built.positions, expected, strict=True):
                if not abs(position - wanted) <= TOLERANCE:
                    refuse(path, f'{where}: {name} is not the reported value in radians '
                                 '(or metres)')
            if built.problem != (None if expected else joint_state.NO_JOINT):
                refuse(path, f'{where}: {built.problem}')
            degrees += sum(kind == 'deg' for _, _, kind in expected)
    if not degrees:
        refuse(path, 'holds no joint in degrees: record it while a motion watch is held')


# ----- the fixture tests: every recording in the fixtures folder ------------------------

@pytest.fixture(scope='module')
def schema():
    return frames.load_schema()


@pytest.fixture(scope='module')
def recordings(fixtures_dir):
    found = recording.find(fixtures_dir)
    if not found:
        pytest.skip(recording.skip_reason(fixtures_dir))
    return found


@pytest.fixture(scope='module')
def converter():
    return convert.Converter.from_installed()


def test_every_fixture_is_a_sanitized_recording_of_the_vendored_schema(recordings, schema):
    for path in recordings:
        check_header(path, schema)


def test_every_frame_of_a_fixture_validates_against_the_vendored_schema(recordings, schema):
    for path in recordings:
        check_frames(path, schema)


def test_applying_the_frames_of_a_fixture_never_reports_a_problem(recordings, schema):
    for path in recordings:
        check_application(path, schema)


def test_every_controller_of_a_fixture_converts_to_a_controller_snapshot(recordings, converter):
    for path in recordings:
        check_messages(path, converter)


def test_every_joint_of_a_fixture_is_in_radians_under_its_default_name(recordings, schema):
    for path in recordings:
        check_joints(path, schema)


# ----- recordings written by hand: the reader -------------------------------------------

def moving(controller_id, seconds):
    """Return a motion delta: the six robot joints at angles that depend on the time."""
    joints = [frames.joint(f'J{axis}', 10.0 * axis + seconds) for axis in range(1, 7)]
    motion = frames.motion_group([frames.motion_unit(joints=joints)], as_of=frames.iso(seconds))
    return frames.delta_frame(controller_id, {'motion': motion}, observed_at=frames.iso(seconds))


def timed_frames():
    """
    Return a short recording as (seconds, frame) pairs: synthetic, from the builders.

    A snapshot of a plain controller and of the edge-case one (a joint in millimetres,
    one without a unit, one without a value, a unit with an error), four motion deltas,
    a controller added by a delta and one removed.
    """
    timed = [(0.0, frames.snapshot_frame(
        [frames.controller('cell-1'), frames.edge_controller('edge-1')]))]
    timed.extend((step * 0.25, moving('cell-1', step * 0.25)) for step in range(1, 5))
    added = frames.controller('cell-2', motion=frames.motion_group(watching=False))
    timed.append((1.25, frames.delta_frame('cell-2', frames.groups_of(added),
                                           observed_at=frames.iso(1.25))))
    timed.append((1.5, frames.removed_frame('edge-1', observed_at=frames.iso(1.5))))
    return timed


def write_lines(path, *records, ending=b'\n'):
    """Write records (dicts, or text for a line that is not JSON) as the lines of a file."""
    lines = [record if isinstance(record, str) else json.dumps(record) for record in records]
    path.write_bytes(b''.join(line.encode('utf-8') + ending for line in lines))
    return path


def header(**changes):
    made = {'kind': 'header', 'format': recording.FORMAT, 'formatVersion': 1,
            'schemaVersion': frames.SCHEMA_VERSION}
    made.update(changes)
    return made


def frame_record(moment, frame=None, **more):
    record = {'kind': 'frame', 't': moment,
              'frame': frames.removed_frame() if frame is None else frame}
    record.update(more)
    return record


def test_a_recording_is_read_back_as_it_was_written(tmp_path):
    timed = timed_frames()
    path = recording.write(tmp_path / 'made.jsonl', timed, frames.SCHEMA_VERSION)
    head = recording.read_header(path)
    assert (head['format'], head['formatVersion']) == ('abb-rws-recording', 1)
    assert (head['schemaVersion'], head['content'], head['sanitized']) == (
        frames.SCHEMA_VERSION, ['frames'], True)
    read = list(recording.read_frames(path))
    assert [seconds for seconds, _ in read] == [seconds for seconds, _ in timed]
    assert [frame for _, frame in read] == [frame for _, frame in timed]
    raw = path.read_bytes()
    assert raw.endswith(b'\n') and b'\r' not in raw
    lines = raw.split(b'\n')[:-1]
    assert all(line.startswith(b'{"kind":"') for line in lines)
    records = list(recording.read_records(path))
    assert [number for number, _ in records] == list(range(2, len(lines) + 1))
    assert [record['kind'] for _, record in records] == ['frame'] * len(timed) + ['end']
    end = records[-1][1]
    assert (end['records'], end['bytes']) == (len(lines), len(raw) - len(lines[-1]) - 1)
    assert end['t'] == 1500


def test_records_that_are_no_frames_are_passed_over(tmp_path):
    first, second = frames.snapshot_frame(), frames.removed_frame()
    path = write_lines(
        tmp_path / 'mixed.jsonl',
        header(sanitized=False, somethingNew=1),
        {'kind': 'controller', 't': 0, 'id': 'cell-1'},
        frame_record(250, first, resync=True, somethingNew=[1]),
        '',
        {'kind': 'note', 't': 300, 'text': 'synthetic'},
        {'kind': 'a-later-addition', 'whatever': True},
        frame_record(1500.5, second, masked=2),
        {'kind': 'end', 't': 1600},
        ending=b'\r\n')
    assert list(recording.read_frames(path)) == [(0.25, first), (1.5005, second)]
    records = list(recording.read_records(path))
    assert [(number, record['kind']) for number, record in records] == [
        (2, 'controller'), (3, 'frame'), (5, 'note'), (6, 'a-later-addition'), (7, 'frame'),
        (8, 'end')]
    assert recording.read_header(path)['somethingNew'] == 1


def test_a_time_that_goes_back_is_held_at_the_one_before(tmp_path):
    path = write_lines(tmp_path / 'back.jsonl', header(), frame_record(0), frame_record(900),
                       frame_record(400), frame_record(900.5), frame_record(2000))
    assert [seconds for seconds, _ in recording.read_frames(path)] == [
        0.0, 0.9, 0.9, 0.9005, 2.0]


NOT_A_RECORDING = 'is not a recording: its first line is not a recording header'
REFUSED = {
    'an empty file': ([], NOT_A_RECORDING),
    'a first line that is not JSON': ([MARKER, frame_record(0)], NOT_A_RECORDING),
    'a first line that is a frame': ([frame_record(0, note=MARKER)], NOT_A_RECORDING),
    'a header of another format': ([header(format=MARKER)], NOT_A_RECORDING),
    'a header that is a list': (['[1, 2]'], NOT_A_RECORDING),
    'format version 2': ([header(formatVersion=2)],
                         'is recording format 2; this reader reads format 1'),
    'a format version that is text': ([header(formatVersion=MARKER)],
                                      'is recording format ?; this reader reads format 1'),
    'a format version that is true': ([header(formatVersion=True)],
                                      'is recording format ?; this reader reads format 1'),
    'no schema version': ([header(schemaVersion=None)],
                          'has a recording header without a valid schemaVersion'),
    'a schema version that is no version': ([header(schemaVersion=MARKER)],
                                            'has a recording header without a valid '
                                            'schemaVersion'),
    'a line that is not JSON': ([header(), frame_record(0), '{"kind": "frame", ' + MARKER],
                                'line 3 of refused.jsonl is not JSON'),
    'a line with a number that is not JSON': ([header(), '{"kind": "frame", "t": NaN}'],
                                              'line 2 of refused.jsonl is not JSON'),
    'a line that is a list': ([header(), f'["{MARKER}"]'],
                              'line 2 of refused.jsonl is not a record'),
    'a record without a kind': ([header(), frame_record(0), {'t': 1, 'text': MARKER}],
                                'line 3 of refused.jsonl is not a record'),
    'a frame record without a time': ([header(), {'kind': 'frame', 'frame': {'type': MARKER}}],
                                      'line 2 of refused.jsonl is a frame record without a '
                                      'valid time'),
    'a frame record with a time before the start': (
        [header(), frame_record(-1)],
        'line 2 of refused.jsonl is a frame record without a valid time'),
    'a frame record with a time that is true': (
        [header(), frame_record(True)],
        'line 2 of refused.jsonl is a frame record without a valid time'),
    'a frame record without a frame': ([header(), {'kind': 'frame', 't': 0, 'frame': MARKER}],
                                       'line 2 of refused.jsonl is a frame record without a '
                                       'frame'),
    'a frame without a type': ([header(), frame_record(0, {'observedAt': MARKER})],
                               'line 2 of refused.jsonl is a frame record without a frame'),
}


@pytest.mark.parametrize('label', sorted(REFUSED))
def test_what_is_not_a_recording_is_refused(tmp_path, label):
    lines, reason = REFUSED[label]
    path = write_lines(tmp_path / 'refused.jsonl', *lines)
    with pytest.raises(recording.RecordingError) as refused:
        recording.read_header(path)
        list(recording.read_frames(path))
    # The error names the file and the line and never repeats what a line holds.
    text = str(refused.value)
    assert text == (reason if reason.startswith('line ') else f'refused.jsonl {reason}')
    assert MARKER not in text


def test_the_last_line_is_read_as_strictly_as_any_other(tmp_path):
    path = write_lines(tmp_path / 'cut.jsonl', header(), frame_record(0))
    path.write_bytes(path.read_bytes() + b'{"kind": "frame", "t": 40, "fra')
    with pytest.raises(recording.RecordingError, match='line 3 of cut.jsonl is not JSON'):
        list(recording.read_frames(path))


def test_the_recordings_of_a_folder_are_found_in_name_order(tmp_path, monkeypatch):
    for name in ('b.jsonl', 'a.jsonl', 'README.md', 'c.jsonl.txt'):
        (tmp_path / name).write_text('', encoding='ascii')
    (tmp_path / 'folder.jsonl').mkdir()
    assert recording.find(tmp_path) == [tmp_path / 'a.jsonl', tmp_path / 'b.jsonl']
    assert recording.find(str(tmp_path)) == recording.find(tmp_path)
    nowhere = tmp_path / 'nowhere'
    assert recording.find(nowhere) == [] and recording.find(tmp_path / 'README.md') == []
    monkeypatch.delenv(recording.FIXTURES_VARIABLE, raising=False)
    assert recording.skip_reason(nowhere).startswith(
        f'no recording (*.jsonl) in {nowhere} (ABB_RWS_TEST_FIXTURES is not set)')
    monkeypatch.setenv(recording.FIXTURES_VARIABLE, str(nowhere))
    assert recording.skip_reason(nowhere).startswith(
        f'no recording (*.jsonl) in {nowhere} (the folder ABB_RWS_TEST_FIXTURES names)')


# ----- recordings written by hand: the checks -------------------------------------------

def test_a_recording_written_by_hand_passes_every_check(tmp_path, schema, converter):
    path = recording.write(tmp_path / 'made.jsonl', timed_frames(), frames.SCHEMA_VERSION)
    check_header(path, schema)
    check_frames(path, schema)
    check_application(path, schema)
    check_messages(path, converter)
    check_joints(path, schema)


def test_the_rule_names_the_joints_of_the_edge_cases_and_their_positions():
    expected = expected_joints(frames.EDGE_CONTROLLER['motion'])
    assert [(name, kind) for name, _, kind in expected] == [
        ('ROB_1_J1', 'deg'), ('ROB_1_J2', 'deg'), ('ROB_1_J5', 'deg'), ('ROB_1_J6', 'deg'),
        ('ROB_1_E1', 'mm')]
    assert [position for _, position, _ in expected] == pytest.approx(
        [0.17453292519943295, -0.35779249665883756, 0.7853981633974483, 1.5707963267948966,
         1.25], abs=TOLERANCE)
    assert DEFAULT_JOINT_NAMES == '{unit}_{joint}'


def _with_a_call_record(path):
    raw = path.read_bytes()
    call = json.dumps({'kind': 'call', 't': 5, 'target': 'manager', 'id': 'cell-1',
                       'method': 'synthetic', 'args': [], 'ok': True})
    path.write_bytes(raw + call.encode('ascii') + b'\n')


def _with_two_end_records(path):
    raw = path.read_bytes()
    path.write_bytes(raw + raw.splitlines(keepends=True)[-1])


def _with_other_line_endings(path):
    path.write_bytes(path.read_bytes().replace(b'\n', b'\r\n'))


def _unit_in_radians():
    timed = timed_frames()
    timed[2][1]['groups']['motion']['units'][0]['joints'][0]['unit'] = 'rad'
    return timed


def _unknown_key():
    timed = timed_frames()
    timed[0][1]['controllers'][0]['panel']['somethingNew'] = 1
    return timed


def _unreadable_time():
    timed = timed_frames()
    timed[3][1]['observedAt'] = '2026-01-01 00:00:00'
    return timed


def _unknown_frame_type():
    timed = timed_frames()
    timed.append((2.0, {'type': 'something-new', 'observedAt': frames.iso(2)}))
    return timed


def _newer_snapshot():
    timed = timed_frames()
    timed.append((2.0, frames.snapshot_frame(observed_at=frames.iso(2),
                                             schema_version=frames.NEWER_SCHEMA_VERSION)))
    return timed


def _part_of_a_controller():
    timed = timed_frames()
    timed.append((2.0, frames.delta_frame('cell-9', {'motion': frames.motion_group()},
                                          observed_at=frames.iso(2))))
    return timed


def _nobody_left():
    timed = timed_frames()
    timed.extend((2.0, frames.removed_frame(controller_id, observed_at=frames.iso(2)))
                 for controller_id in ('cell-1', 'cell-2'))
    return timed


def _speed_as_text():
    timed = timed_frames()
    timed[0][1]['controllers'][0]['panel']['speedRatio'] = 'fast'
    return timed


def _nothing_watched():
    idle = frames.controller('cell-1', motion=frames.motion_group(watching=False))
    return [(0.0, frames.snapshot_frame([idle]))]


def _millimetres_only():
    rail = frames.motion_unit('TRACK_1', [frames.joint('E1', 1250.0, 'mm', 0.0, 5000.0)],
                              kind='external')
    return [(0.0, frames.snapshot_frame(
        [frames.controller('cell-1', motion=frames.motion_group([rail]))]))]


# label: (the check, what it must say, the frames, changes of the header, a change of the file)
CAUGHT = {
    'a recording of an older schema': (
        check_header, 'was made with schema 0.11; the vendored schema is ',
        timed_frames, {'schemaVersion': '0.11'}, None),
    'a recording that was not sanitized': (
        check_header, 'does not say that it was sanitized', timed_frames,
        {'sanitized': False}, None),
    'a header without the sanitized mark': (
        check_header, 'does not say that it was sanitized', timed_frames,
        {'sanitized': 'yes'}, None),
    'a recording with calls in its content': (
        check_header, 'names other content than frames', timed_frames,
        {'content': ['frames', 'calls']}, None),
    'a call record': (
        check_header, 'neither a frame nor the end record: call', timed_frames, {},
        _with_a_call_record),
    'two end records': (
        check_header, 'more than one end record', timed_frames, {}, _with_two_end_records),
    'no frame': (check_header, 'holds no frame', list, {}, None),
    'line endings other than LF': (
        check_header, 'line endings other than LF', timed_frames, {}, _with_other_line_endings),
    'a first frame that is a delta': (
        check_frames, 'frame 1 is not a snapshot', lambda: timed_frames()[1:], {}, None),
    'a unit the schema does not have': (
        check_frames, 'frame 3: groups/motion/units/0/joints/0/unit: ', _unit_in_radians, {},
        None),
    'a key the schema does not have': (
        check_frames, 'frame 1: controllers/0/panel: ', _unknown_key, {}, None),
    'a time the daemon does not write': (
        check_frames, 'frame 4: observedAt: ', _unreadable_time, {}, None),
    'a frame type the schema does not have': (
        check_frames, 'frame 8: a frame type the schema does not have', _unknown_frame_type,
        {}, None),
    'a snapshot of a newer schema': (
        check_frames, 'frame 8 is a snapshot of another schema version', _newer_snapshot, {},
        None),
    'a delta before the snapshot': (
        check_application, 'frame 1: a delta before an accepted snapshot',
        lambda: timed_frames()[1:], {}, None),
    'a time the state cannot read': (
        check_application, 'frame 4: a delta for cell-1 without a readable observedAt',
        _unreadable_time, {}, None),
    'a delta with a part of a new controller': (
        check_application, 'frame 8: controller cell-9 is held back', _part_of_a_controller,
        {}, None),
    'a frame of an unknown type': (
        check_application, 'frame 8 was not applied (ignored)', _unknown_frame_type, {}, None),
    'nobody left at the end': (
        check_application, 'holds no controller at its end', _nobody_left, {}, None),
    'a value that fits no message': (
        check_messages, 'frame 1: controller cell-1 does not convert: '
                        'ControllerSnapshot.panel.speedRatio: ', _speed_as_text, {}, None),
    'no motion watched': (
        check_joints, 'holds no joint in degrees', _nothing_watched, {}, None),
    'joints in millimetres only': (
        check_joints, 'holds no joint in degrees', _millimetres_only, {}, None),
}


@pytest.mark.parametrize('label', sorted(CAUGHT))
def test_a_check_catches_what_it_is_for(tmp_path, schema, converter, label):
    check, said, make, header_changes, change = CAUGHT[label]
    path = recording.write(tmp_path / 'made.jsonl', make(), frames.SCHEMA_VERSION,
                           **header_changes)
    if change is not None:
        change(path)
    with pytest.raises(AssertionError) as caught:
        check(path, converter if check is check_messages else schema)
    text = str(caught.value)
    assert text.startswith('made.jsonl: ') and said in text

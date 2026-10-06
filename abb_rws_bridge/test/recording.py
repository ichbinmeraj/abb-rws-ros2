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
Recordings of the abb-rws-daemon stream, read for the tests.

A recording is a JSON Lines file (UTF-8, one JSON object per line): a header first,
then records. Every record has a ``kind`` and ``t``, the milliseconds since the recording
began. A ``frame`` record holds one frame of the daemon's stream as a consumer received
it. A fixture holds frame records and a closing ``end`` record and nothing else (see
``fixtures/README.md``).

Import it as a top-level module (``import recording``): this folder has no ``__init__.py``
and pytest puts it on the import path. It lives in the test folder and is never
installed.

    header = recording.read_header(path)
    fake.play(recording.read_frames(path), speed=50.0)

Like the daemon's own reader this one passes over record kinds and keys it does not know
and refuses a format version it does not know. It is stricter in one point: a line that
cannot be read is refused wherever it stands, the last one included, because a fixture is
a closed file. An error names the file and the line and never repeats what a line holds.
"""

from collections.abc import Iterable, Iterator
import json
import math
import os
from pathlib import Path
import re

FORMAT = 'abb-rws-recording'
FORMAT_VERSION = 1
SUFFIX = '.jsonl'
FIXTURES_VARIABLE = 'ABB_RWS_TEST_FIXTURES'

# The start of every recording written by hand: the T0 of the ``frames`` builders.
SYNTHETIC_START = '2026-01-01T00:00:00.000Z'

_SCHEMA_VERSION = re.compile(r'[0-9]+\.[0-9]+')


class RecordingError(ValueError):
    """A file that is not a recording this reader can read; the text says why."""


def find(folder) -> list[Path]:
    """Return the recordings (``*.jsonl``) of a folder in name order; none without it."""
    folder = Path(folder)
    if not folder.is_dir():
        return []
    return sorted(path for path in folder.glob('*' + SUFFIX) if path.is_file())


def skip_reason(folder) -> str:
    """Return what a fixture test says when it is skipped: the folder has no recording."""
    if os.environ.get(FIXTURES_VARIABLE):
        named = f'the folder {FIXTURES_VARIABLE} names'
    else:
        named = f'{FIXTURES_VARIABLE} is not set'
    return (f'no recording (*{SUFFIX}) in {folder} ({named}); a recording is added only '
            'after its approval, see test/fixtures/README.md')


def read_header(path) -> dict:
    """
    Return the header of a recording: its first line.

    Raises RecordingError when the first line is not the header of a recording, names a
    format version other than 1 or has no readable ``schemaVersion``.
    """
    path = Path(path)
    with open(path, 'rb') as handle:
        return _header(handle.readline(), path.name)


def read_records(path) -> Iterator[tuple[int, dict]]:
    """
    Yield ``(line number, record)`` for every record after the header, in file order.

    Records of every kind are given, also kinds this reader does not know. Empty lines
    are passed over. Raises RecordingError for a header ``read_header`` refuses, for a
    line that is not JSON and for one that is not a record (an object with a ``kind``).
    """
    path = Path(path)
    name = path.name
    with open(path, 'rb') as handle:
        _header(handle.readline(), name)
        for number, raw in enumerate(handle, start=2):
            if not raw.strip():
                continue
            record = _parsed(raw, number, name)
            if not isinstance(record, dict) or not isinstance(record.get('kind'), str):
                raise RecordingError(f'line {number} of {name} is not a record')
            yield number, record


def read_frames(path) -> Iterator[tuple[float, dict]]:
    """
    Yield ``(seconds from the start of the recording, frame)`` for every frame record.

    This is the shape ``fake_daemon.FakeDaemon.play`` takes. Frames come in file order
    and times never go back: a record with a lower ``t`` than the one before is given
    that one's time, as the daemon's own replay does. Records of other kinds are passed
    over. Raises RecordingError for what ``read_records`` refuses and for a frame record
    without a usable ``t`` or without a frame that names its ``type``.
    """
    name = Path(path).name
    last_ms = 0.0
    for number, record in read_records(path):
        if record['kind'] != 'frame':
            continue
        moment = record.get('t')
        if (isinstance(moment, bool) or not isinstance(moment, (int, float))
                or not math.isfinite(moment) or moment < 0):
            raise RecordingError(f'line {number} of {name} is a frame record without a '
                                 'valid time')
        frame = record.get('frame')
        if not isinstance(frame, dict) or not isinstance(frame.get('type'), str):
            raise RecordingError(f'line {number} of {name} is a frame record without a frame')
        last_ms = max(last_ms, float(moment))
        yield last_ms / 1000.0, frame


def write(path, timed_frames: Iterable[tuple[float, dict]], schema_version: str,
          **header_changes) -> Path:
    """
    Write a frames-only recording made by hand, shaped as a sanitized one.

    For tests that need a recording and must not depend on a recorded one: the frames
    come from the ``frames`` builders, so nothing in the file was read from a
    controller. timed_frames is an iterable of ``(seconds from the start, frame)``, what
    ``read_frames`` gives back. header_changes replace keys of the header. Lines end in
    LF on every platform. Returns the path.
    """
    path = Path(path)
    header = {
        'kind': 'header', 'format': FORMAT, 'formatVersion': FORMAT_VERSION,
        'schemaVersion': schema_version, 'source': 'daemon', 'session': 'rec-synthetic',
        'part': 1, 'previous': None, 'startedAt': SYNTHETIC_START,
        'sessionStartedAt': SYNTHETIC_START, 'label': 'synthetic', 'content': ['frames'],
        'daemon': {'version': 'synthetic'}, 'client': {'version': 'synthetic'},
        'controllers': [], 'limits': None,
        'masking': {'rules': [], 'values': 0, 'derived': 0, 'weakValues': 0},
        'sanitized': True,
    }
    header.update(header_changes)
    lines = [_line(header)]
    last_ms = 0
    for seconds, frame in timed_frames:
        last_ms = round(seconds * 1000)
        lines.append(_line({'kind': 'frame', 't': last_ms, 'frame': frame}))
    lines.append(_line({
        'kind': 'end', 't': last_ms, 'reason': 'stopped', 'records': len(lines) + 1,
        'bytes': sum(len(line) for line in lines),
        'dropped': {'call': 0, 'state': 0, 'sub': 0},
        'masking': {'values': 0, 'derived': 0, 'weakValues': 0, 'maskedRecords': 0,
                    'droppedKeys': 0, 'refusedLines': 0},
        'stats': {'stateDiffMsMax': 0}, 'next': None,
    }))
    path.write_bytes(b''.join(lines))
    return path


def _line(record) -> bytes:
    text = json.dumps(record, ensure_ascii=False, allow_nan=False, separators=(',', ':'))
    return text.encode('utf-8') + b'\n'


def _not_json(constant):
    raise ValueError(f'{constant} is not JSON')


def _parsed(raw, number, name):
    """Return one line parsed as strict JSON (UTF-8; NaN and Infinity are not JSON)."""
    try:
        return json.loads(raw.decode('utf-8'), parse_constant=_not_json)
    except (ValueError, RecursionError):
        raise RecordingError(f'line {number} of {name} is not JSON') from None


def _header(raw, name):
    """Return the first line of a file as a recording header, or raise RecordingError."""
    try:
        header = _parsed(raw, 1, name)
    except RecordingError:
        header = None
    if (not isinstance(header, dict) or header.get('kind') != 'header'
            or header.get('format') != FORMAT):
        raise RecordingError(f'{name} is not a recording: its first line is not a '
                             'recording header')
    version = header.get('formatVersion')
    if isinstance(version, bool) or version != FORMAT_VERSION:
        shown = version if isinstance(version, int) and not isinstance(version, bool) else '?'
        raise RecordingError(f'{name} is recording format {shown}; this reader reads format '
                             f'{FORMAT_VERSION}')
    schema_version = header.get('schemaVersion')
    if not isinstance(schema_version, str) or not _SCHEMA_VERSION.fullmatch(schema_version):
        raise RecordingError(f'{name} has a recording header without a valid schemaVersion')
    return header

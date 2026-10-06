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
Daemon data to ROS messages, driven by the mapping file of the message package.

``abb_rws_ros2_msgs`` installs ``schema/v0.mapping.json`` beside its messages: for each
message, how every JSON property of the daemon's schema maps to a field. ``Converter``
builds a message from that table and from nothing else, so the conversion cannot drift
from the ``.msg`` files. No value is converted to another unit here: a message carries
the daemon's own units and times.

How a mapped property becomes a field (``kind`` is the word the mapping uses):

- ``string``: the text; with a ``known`` flag the flag is false and the text empty for
  null;
- ``enum``: the text, empty for null; a value the mapping does not list is passed on as
  it is (a newer daemon may know more values);
- ``time``: ``builtin_interfaces/Time``; with a ``known`` flag the flag is false for
  null and for a text that is not a readable time;
- ``float``: the number as a float, NaN for null;
- ``int``: the whole number; with a ``known`` flag the flag is false for null;
- ``bool``: the value; marked ``tristate``: -1 for null, 0 for false, 1 for true;
- ``msg``: the nested message; marked ``optional``: a list with no element for null and
  one element otherwise;
- ``array``: a list of its items, each converted by the items' kind;
- ``tuple``: a list of floats of the fixed size;
- ``json``: JSON text with sorted keys, empty for null.

A property the schema does not require may also be absent; it is then treated as null.
Keys the mapping does not name are ignored.

Pure: nothing from ROS is imported when this module is. The message classes come from
the module the caller hands over or, by name, from the mapped package.
"""

from collections.abc import Mapping
import importlib
import json
import os

from abb_rws_bridge import timeconv

MESSAGE_PACKAGE = 'abb_rws_ros2_msgs'
MAPPING_FILE = ('schema', 'v0.mapping.json')

_KINDS = frozenset({
    'string', 'enum', 'time', 'float', 'int', 'bool', 'msg', 'array', 'tuple', 'json'})
_WITH_KNOWN = frozenset({'string', 'time', 'int'})
_INT64_MIN = -2**63
_INT64_MAX = 2**63 - 1
_TRISTATE_UNKNOWN = -1


class ConversionError(ValueError):
    """
    Daemon data that does not fit the message it should become.

    ``path`` names the place, in the daemon's own property names
    (``ControllerSnapshot.motion.units[0].joints[2].value``); ``reason`` says what did
    not fit. Neither repeats a value.
    """

    def __init__(self, path: str, reason: str) -> None:
        super().__init__(path, reason)
        self.path = path
        self.reason = reason

    def __str__(self) -> str:
        return f'{self.path}: {self.reason}'


class Converter:
    """
    The builder of messages from daemon data, by the mapping table.

    ``schema_version`` is the daemon schema version the messages were generated from,
    ``definitions_sha256`` the version of the messages themselves.
    """

    schema_version: str
    definitions_sha256: str

    def __init__(self, mapping: Mapping, msg_module=None) -> None:
        """
        Take a parsed mapping file and the module that holds the message classes.

        Without msg_module the module ``<package>.msg`` of the mapping is imported.
        Raises ValueError when the mapping is not shaped as the generator writes it or
        names a message the module does not have.
        """
        if not isinstance(mapping, Mapping):
            raise ValueError('the mapping must be an object')
        messages = mapping.get('messages')
        version = mapping.get('schemaVersion')
        digest = mapping.get('definitionsSha256')
        if not isinstance(messages, Mapping) or not messages:
            raise ValueError('the mapping names no messages')
        if not isinstance(version, str) or not isinstance(digest, str):
            raise ValueError('the mapping lacks schemaVersion or definitionsSha256')
        if msg_module is None:
            package = mapping.get('package')
            if not isinstance(package, str) or not package:
                raise ValueError('the mapping names no package')
            msg_module = importlib.import_module(f'{package}.msg')
        self.schema_version = version
        self.definitions_sha256 = digest
        self._headers = {}
        self._fields = {}
        self._classes = {}
        self._time_class = None
        for name, entry in messages.items():
            fields = entry.get('fields') if isinstance(entry, Mapping) else None
            if not isinstance(fields, (list, tuple)):
                raise ValueError(f'the mapping of {name} has no field list')
            message_class = getattr(msg_module, name, None)
            if message_class is None:
                raise ValueError(f'the message module has no {name}')
            self._headers[name] = entry.get('header') is True
            self._fields[name] = tuple(fields)
            self._classes[name] = message_class
        for name, fields in self._fields.items():
            for entry in fields:
                self._check(name, entry, item=False)

    @classmethod
    def from_installed(cls) -> 'Converter':
        """
        Return a converter for the installed message package.

        Reads ``schema/v0.mapping.json`` from the share folder of ``abb_rws_ros2_msgs``
        (found through the ament index) and imports its message module.
        """
        from ament_index_python.packages import get_package_share_directory

        share = get_package_share_directory(MESSAGE_PACKAGE)
        with open(os.path.join(share, *MAPPING_FILE), encoding='utf-8') as handle:
            return cls(json.load(handle))

    def to_msg(self, type_name: str, data: Mapping,
               stamp: tuple[int, int] | None = None) -> object:
        """
        Return the message type_name built from data, a mapping in the daemon's shape.

        For a message the mapping marks with a header (``ControllerSnapshot``), stamp is
        required: (seconds, nanoseconds) for ``header.stamp``. ``header.frame_id`` stays
        empty. Any other message takes no stamp.

        Raises ConversionError when the data does not fit: a required key is missing,
        a value has the wrong type (null where the schema allows none included), or a
        list holds more items than its bound. Nothing is cut to make it fit. Raises a
        plain ValueError for a mistake of the caller: an unknown type_name, a missing
        or unusable stamp, a stamp for a message without a header.
        """
        if type_name not in self._fields:
            raise ValueError(f'the mapping has no message {type_name}')
        if self._headers[type_name]:
            if stamp is None:
                raise ValueError(f'{type_name} has a header: a stamp is required')
            sec, nanosec = _stamp(stamp)
        elif stamp is not None:
            raise ValueError(f'{type_name} has no header: it takes no stamp')
        message = self._build(type_name, data, type_name)
        if self._headers[type_name]:
            _assign(message.header, 'stamp', self._time(sec, nanosec), type_name)
        return message

    def _build(self, type_name, data, path):
        if not isinstance(data, Mapping):
            raise ConversionError(path, f'an object is required, found {_found(data)}')
        message = self._classes[type_name]()
        for entry in self._fields[type_name]:
            key = entry['json']
            where = f'{path}.{key}'
            if key not in data and entry['required']:
                raise ConversionError(where, 'the key is missing')
            value, known = self._field(entry, data.get(key), where)
            _assign(message, entry['ros'], value, where)
            if 'known' in entry:
                _assign(message, entry['known'], known, where)
        return message

    def _field(self, entry, raw, where):
        """Return the value of one field and what its known flag would be."""
        kind = entry['kind']
        if raw is None:
            if not (entry['nullable'] or not entry['required']):
                raise ConversionError(where, 'null where a value is required')
            return self._unknown(entry), False
        if kind == 'time' and 'known' in entry and isinstance(raw, str):
            try:
                return self._time(*timeconv.iso_to_sec_nanosec(raw)), True
            except ValueError:
                return self._time(0, 0), False
        value = self._value(entry, raw, where)
        if kind == 'msg' and entry.get('optional'):
            return [value], True
        if kind == 'bool' and entry.get('tristate'):
            return int(value), True
        return value, True

    def _unknown(self, entry):
        """Return what a field holds when the daemon reports null, or nothing."""
        kind = entry['kind']
        if kind == 'float':
            return float('nan')
        if kind == 'int':
            return 0
        if kind == 'time':
            return self._time(0, 0)
        if kind == 'bool':
            return _TRISTATE_UNKNOWN
        if kind == 'msg':
            return []
        return ''

    def _value(self, entry, raw, where):
        """Return one value that is not null, converted by the kind of its entry."""
        kind = entry['kind']
        if kind in ('string', 'enum'):
            if not isinstance(raw, str):
                raise ConversionError(where, f'text is required, found {_found(raw)}')
            return raw
        if kind == 'time':
            if not isinstance(raw, str):
                raise ConversionError(where, f'a time is required, found {_found(raw)}')
            try:
                return self._time(*timeconv.iso_to_sec_nanosec(raw))
            except ValueError:
                raise ConversionError(where, 'a time that cannot be read') from None
        if kind == 'float':
            return _number(raw, where)
        if kind == 'int':
            return _whole_number(raw, where)
        if kind == 'bool':
            if not isinstance(raw, bool):
                raise ConversionError(where, f'true or false is required, found {_found(raw)}')
            return raw
        if kind == 'msg':
            return self._build(entry['msg'], raw, where)
        if kind == 'json':
            try:
                return json.dumps(raw, sort_keys=True, allow_nan=False)
            except (TypeError, ValueError):
                raise ConversionError(where, 'a value that is not JSON') from None
        if not isinstance(raw, (list, tuple)):
            raise ConversionError(where, f'a list is required, found {_found(raw)}')
        if kind == 'tuple':
            if len(raw) != entry['size']:
                raise ConversionError(
                    where, f'{len(raw)} numbers, exactly {entry["size"]} are required')
            return [_number(item, f'{where}[{index}]') for index, item in enumerate(raw)]
        if 'max' in entry and len(raw) > entry['max']:
            raise ConversionError(where, f'{len(raw)} items, at most {entry["max"]} fit')
        items = entry['items']
        return [self._value(items, item, f'{where}[{index}]')
                for index, item in enumerate(raw)]

    def _time(self, sec, nanosec):
        if self._time_class is None:
            self._time_class = importlib.import_module('builtin_interfaces.msg').Time
        return self._time_class(sec=sec, nanosec=nanosec)

    def _check(self, message, entry, *, item):
        """Refuse a mapping entry this converter has no rule for."""
        if not isinstance(entry, Mapping):
            raise ValueError(f'the mapping of {message} holds an entry that is not an object')
        kind = entry.get('kind')
        where = f'{message}.{entry.get("json")}' if not item else f'the items of {message}'
        if kind not in _KINDS:
            raise ValueError(f'{where}: the kind {kind!r} is not known')
        if kind == 'msg' and entry.get('msg') not in self._fields:
            raise ValueError(f'{where}: refers to a message the mapping does not have')
        if kind == 'tuple' and not _is_count(entry.get('size')):
            raise ValueError(f'{where}: a tuple needs a size')
        if kind == 'array':
            if 'max' in entry and not _is_count(entry['max']):
                raise ValueError(f'{where}: the bound of an array must be a count')
            items = entry.get('items')
            if not isinstance(items, Mapping) or items.get('kind') in ('array', 'tuple'):
                raise ValueError(f'{where}: an array needs items that are not lists')
            self._check(f'{where}[]', items, item=True)
        if item:
            return
        for key in ('json', 'ros'):
            if not isinstance(entry.get(key), str) or not entry[key]:
                raise ValueError(f'{where}: the entry lacks {key}')
        for key in ('required', 'nullable'):
            if not isinstance(entry.get(key), bool):
                raise ValueError(f'{where}: the entry lacks {key}')
        if 'known' in entry and not (isinstance(entry['known'], str) and entry['known']):
            raise ValueError(f'{where}: the name of the known flag must be text')
        if not (entry['nullable'] or not entry['required']):
            return
        if kind in _WITH_KNOWN and 'known' not in entry:
            raise ValueError(f'{where}: a {kind} that may be null needs a known flag')
        if kind == 'bool' and entry.get('tristate') is not True:
            raise ValueError(f'{where}: a bool that may be null must be marked tristate')
        if kind == 'msg' and entry.get('optional') is not True:
            raise ValueError(f'{where}: a message that may be null must be marked optional')
        if kind in ('array', 'tuple'):
            raise ValueError(f'{where}: a list that may be null is not supported')


def _assign(message, name, value, where):
    try:
        setattr(message, name, value)
    except (AssertionError, AttributeError, TypeError, ValueError):
        raise ConversionError(where, f'the message field {name} does not take the value') \
            from None


def _found(value):
    """Name what was found instead, without repeating it."""
    if value is None:
        return 'null'
    if isinstance(value, bool):
        return 'true or false'
    if isinstance(value, (int, float)):
        return 'a number'
    if isinstance(value, str):
        return 'text'
    if isinstance(value, Mapping):
        return 'an object'
    if isinstance(value, (list, tuple)):
        return 'a list'
    return type(value).__name__


def _is_count(value):
    return isinstance(value, int) and not isinstance(value, bool) and value >= 1


def _number(raw, where):
    if isinstance(raw, bool) or not isinstance(raw, (int, float)):
        raise ConversionError(where, f'a number is required, found {_found(raw)}')
    try:
        return float(raw)
    except OverflowError:
        raise ConversionError(where, 'a number too large for a float') from None


def _whole_number(raw, where):
    if isinstance(raw, float) and raw.is_integer():
        raw = int(raw)
    if isinstance(raw, bool) or not isinstance(raw, int):
        raise ConversionError(where, f'a whole number is required, found {_found(raw)}')
    if not _INT64_MIN <= raw <= _INT64_MAX:
        raise ConversionError(where, 'a whole number too large for 64 bits')
    return raw


def _stamp(stamp):
    """Return a caller's stamp as (seconds, nanoseconds), or raise ValueError."""
    if not isinstance(stamp, (tuple, list)) or len(stamp) != 2:
        raise ValueError('a stamp is (seconds, nanoseconds)')
    sec, nanosec = stamp
    for part in (sec, nanosec):
        if isinstance(part, bool) or not isinstance(part, int):
            raise ValueError('a stamp is (seconds, nanoseconds), both whole numbers')
    if not 0 <= sec <= timeconv.MAX_SEC or not 0 <= nanosec < timeconv.NANOSEC_PER_SEC:
        raise ValueError('a stamp must fit a ROS time')
    return sec, nanosec

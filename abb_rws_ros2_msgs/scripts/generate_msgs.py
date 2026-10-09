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
Generate the ROS 2 messages of this package from the abb-rws-daemon JSON Schema.

The only input is ``schema/v0.schema.json``, a byte-for-byte copy of the daemon's
``schema/v0.schema.json``. The generated files are checked in:

- ``msg/<Name>.msg``: one message per definition marked ``x-ros: "msg"`` (a root) and per
  object definition a root reaches through its properties;
- ``msg/interfaces.cmake``: the sorted list of those files, included by ``CMakeLists.txt``;
- ``schema/v0.mapping.json``: how each JSON property maps to a ROS field;
- ``schema/SOURCE.json``: which schema the files were generated from.

Command line (standard library only, the same bytes on every run):

- no option: write the generated files and delete a ``.msg`` that is no longer generated;
- ``--check``: write nothing; exit with status 1 and list every difference when a generated
  file is missing, stale or extra, or when ``SOURCE.json`` names another schema file;
- ``--vendor <file>``: take in a new copy of the daemon's schema (LF line ends), then generate;
- ``--out-dir <dir>``: write the tree under another folder (for a read-only source tree).

How a schema property becomes a ROS field (``kind`` is the word the mapping file uses):

- a string is ``string`` (kind ``string``); when it may be null or absent a second field
  ``bool <name>_known`` says whether it holds a value;
- an ``enum``, a ``const`` or a reference to a named string enum is ``string`` plus one
  constant ``<FIELD>_<VALUE>`` per value (kind ``enum``); empty when null or absent;
- ``format: "date-time"`` is ``builtin_interfaces/Time`` (kind ``time``), with a
  ``<name>_known`` field when it may be null or absent;
- a ``number`` is ``float64`` (kind ``float``); NaN when null or absent;
- an ``integer`` is ``int64`` (kind ``int``), with a ``<name>_known`` field when it may be
  null or absent;
- a ``boolean`` is ``bool`` (kind ``bool``); when it may be null or absent it is ``int8``
  with the constants ``<FIELD>_UNKNOWN`` (-1), ``<FIELD>_FALSE`` (0), ``<FIELD>_TRUE`` (1);
- a reference to an object definition is a nested message (kind ``msg``); when it may be
  null or absent it is ``<Message>[<=1]``, empty or one element;
- an array is ``T[]``, or ``T[<=N]`` when it has ``maxItems`` and ``x-max-items-from``
  (kind ``array``); a fixed tuple of numbers is ``float64[N]`` (kind ``tuple``);
- a property or a definition marked ``x-ros: "json"`` is a ``string`` holding JSON text
  (kind ``json``), empty when null or absent; its content is never followed;
- a definition marked ``x-ros: "alias"`` is never a message of its own;
- a root message gets ``std_msgs/Header header`` as its first field; the header is not a
  property of the schema, so the mapping file marks the message with ``"header": true``;
- any other shape stops the generator with an error that names the property.

A field name comes from ``x-ros-name``, or from the rename table of this file (empty: the
daemon gives a keyword another ``x-ros-name``). A name that is a C, C++ or Python keyword
stops the generator. A name that is an IDL keyword,
or ``min`` / ``max``, is kept and listed under ``warnings`` in the mapping file.

The comments are the schema's descriptions, with two changes so that they speak of the
message and not of the JSON. A property name in back quotes is written as the name of
its field (``titleFromCode`` becomes ``title_from_code``, a dotted path part by part)
when every message that has the property gives it that one field name. And the
description of a time field loses the words "ISO 8601": a ``builtin_interfaces/Time``
is not text.

The mapping file has ``schemaId``, ``schemaVersion``, ``definitionsSha256``, ``package``,
``renames``, ``warnings`` (``message``, ``field``, ``why``) and ``messages``: per message
its ``fields`` in order (and ``"header": true`` on a root). A field has ``json`` (the
property), ``ros`` (the field), ``kind``, ``required`` (the property is always present)
and ``nullable`` (the schema allows null), and with them: ``values`` for an enum, ``msg``
for a nested message, ``items`` (a kind with what belongs to it) and ``max`` for an
array, ``size`` for a tuple, ``known`` (the name of the companion flag), ``tristate`` for
an ``int8`` boolean and ``optional`` for a ``[<=1]`` message.

``definitionsSha256`` is the version of the messages: the sha256 of the canonical JSON
(sorted keys, no spaces) of the mapped definitions, the named definitions they refer to,
the rename table and the rule version of this file. A schema version that changes none
of them changes no message.
"""

import argparse
import hashlib
import json
from pathlib import Path
import re
import sys

PACKAGE = 'abb_rws_ros2_msgs'

# Raised whenever a rule of this file changes what is generated from the same schema.
# 2: comments name fields, not JSON properties, and call no time field ISO 8601.
RULE_VERSION = 2

# Field names that replace the schema's x-ros-name: '<Definition>.<property>' -> name.
# Empty: the schema names every field for ROS itself. A name that will not do (a C, C++ or
# Python keyword) is renamed in the daemon, with @rosName (IdentityGroup.virtual is
# is_virtual there since schema 0.13); this table is for a schema that has not caught up.
RENAMES = {}

SCHEMA_FILE = 'schema/v0.schema.json'
SOURCE_FILE = 'schema/SOURCE.json'
MAPPING_FILE = 'schema/v0.mapping.json'
CMAKE_FILE = 'msg/interfaces.cmake'
SOURCE_NAME = 'abb-rws-daemon, schema/v0.schema.json'

GENERATED_LINE = (
    '# Generated from the abb-rws-daemon JSON Schema by generate_msgs.py. Do not edit.'
)

LICENSE_HEADER = """\
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

HEADER_FIELD = 'header'
HEADER_COMMENT = (
    'header.stamp is when the daemon reported the state this message holds: the observedAt '
    'of the stream frame, or of the controller list, that last delivered or confirmed it. '
    'A publisher set to stamp with its own clock uses the time that frame or list arrived. '
    'header.frame_id is empty.'
)

MESSAGE_NAME = re.compile(r'^[A-Z][A-Za-z0-9]*$')
FIELD_NAME = re.compile(r'^[a-z]([a-z0-9_]?[a-z0-9]+)*$')
CONSTANT_NAME = re.compile(r'^[A-Z]([A-Z0-9_]?[A-Z0-9]+)*$')
CONSTANT_VALUE = re.compile(r'^[ !$-&(-\[\]-~]+$')
REFERENCE = re.compile(r'^#/definitions/([^/]+)$')

COMMENT_WIDTH = 97

# A name, or a dotted path of names, in back quotes: how a description names a member.
MEMBER_PATH = re.compile(r'`([A-Za-z_][A-Za-z0-9_]*(?:\.[A-Za-z_][A-Za-z0-9_]*)*)`')
# How a description says that a time is written as text.
ISO_IN_LIST = re.compile(r',\s*ISO 8601\)')
ISO_ALONE = re.compile(r'\s*\(ISO 8601\)')

# A field with one of these names does not compile in C or C++, or cannot be used in Python.
FORBIDDEN_NAMES = frozenset("""
    alignas alignof and and_eq asm auto bitand bitor bool break case catch char char8_t
    char16_t char32_t class compl concept const consteval constexpr constinit const_cast
    continue co_await co_return co_yield decltype default delete do double dynamic_cast else
    enum explicit export extern false float for friend goto if inline int long mutable
    namespace new noexcept not not_eq nullptr operator or or_eq private protected public
    register reinterpret_cast requires restrict return short signed sizeof static
    static_assert static_cast struct switch template this thread_local throw true try
    typedef typeid typename typeof union unsigned using virtual void volatile wchar_t while
    xor xor_eq
    as assert async await def del elif except finally from global import in is lambda
    nonlocal pass raise with yield
""".split())

# Kept, but listed in the mapping file: these built on the tested platform and may not on
# another one.
IDL_KEYWORDS = frozenset("""
    abstract any alias attribute bitfield bitmask bitset boolean component connector
    consumes context custom emits eventtype exception factory finder fixed getraises home
    inout interface local manages map mirrorport module multiple native object octet oneway
    out port porttype primarykey provides publishes raises readonly sequence setraises
    string supports truncatable typeprefix uses valuebase valuetype wstring int8 uint8
    int16 int32 int64 uint16 uint32 uint64
""".split())
MACRO_NAMES = frozenset(('min', 'max'))
WHY_IDL_KEYWORD = 'IDL keyword'
WHY_MACRO_NAME = 'macro name in some C++ headers'

# x-unit values that name no physical unit.
NO_UNIT = frozenset(('none', 'count'))

ROS_TYPES = {
    'string': 'string',
    'enum': 'string',
    'json': 'string',
    'time': 'builtin_interfaces/Time',
    'float': 'float64',
    'int': 'int64',
    'bool': 'bool',
}


class GeneratorError(Exception):
    """A shape or a name in the schema that the generator does not map."""


class Generated:
    """What one schema maps to, before anything is written."""

    def __init__(self, messages, mapping):
        self.messages = messages
        self.mapping = mapping
        self.definitions_sha256 = mapping['definitionsSha256']
        self.cmake = render_cmake(messages)

    def files(self):
        """Return every generated file but SOURCE.json: relative path -> text."""
        out = {f'msg/{name}.msg': text for name, text in self.messages.items()}
        out[CMAKE_FILE] = self.cmake
        out[MAPPING_FILE] = json_text(self.mapping)
        return out


class _Field:
    """One field of a message: its mapping entry and what its comment says."""

    def __init__(self, entry, description, notes):
        self.entry = entry
        self.description = description
        self.notes = notes


class _Builder:
    """Walks the definitions from the roots and collects messages, fields and warnings."""

    def __init__(self, schema, renames):
        definitions = schema.get('definitions')
        if not isinstance(definitions, dict):
            raise GeneratorError('the schema has no definitions')
        self.definitions = definitions
        self.renames = dict(renames)
        self.unused_renames = set(self.renames)
        self.order = []
        self.fields = {}
        self.roots = [name for name, definition in definitions.items()
                      if isinstance(definition, dict) and definition.get('x-ros') == 'msg']
        self.used = set()
        self.warnings = []
        self.building = []

    def message(self, name):
        """Map one object definition and every message it refers to."""
        if name in self.fields:
            return
        if name in self.building:
            raise GeneratorError(f'{name}: refers to itself; a message cannot hold itself')
        if not MESSAGE_NAME.match(name):
            raise GeneratorError(f'{name}: not a legal message name')
        definition = self.definitions[name]
        properties = definition.get('properties')
        if definition.get('type') != 'object' or not isinstance(properties, dict):
            raise GeneratorError(f'{name}: not an object definition with properties')
        self.building.append(name)
        self.order.append(name)
        required = definition.get('required', [])
        fields = []
        names = {HEADER_FIELD} if name in self.roots else set()
        for json_name, node in properties.items():
            field = self._field(name, json_name, node, json_name in required)
            for ros_name in (field.entry['ros'], field.entry.get('known')):
                if ros_name is None:
                    continue
                if ros_name in names:
                    raise GeneratorError(f'{name}.{json_name}: the field name {ros_name!r} '
                                         'is used twice in the message')
                names.add(ros_name)
            fields.append(field)
        constants = set()
        for field in fields:
            for constant, _ in constant_lines(field.entry):
                if constant in constants:
                    raise GeneratorError(f'{name}.{field.entry["json"]}: the constant '
                                         f'{constant} is used twice in the message')
                constants.add(constant)
        self.building.pop()
        self.fields[name] = fields

    def _field(self, message, json_name, node, required):
        where = f'{message}.{json_name}'
        if not isinstance(node, dict):
            raise GeneratorError(f'{where}: not a schema object')
        ros_name = self._ros_name(message, json_name, node)
        entry = {'json': json_name, 'ros': ros_name}
        kind, nullable = self._kind(where, node)
        entry['kind'] = kind.pop('kind')
        entry['required'] = required
        entry['nullable'] = nullable
        entry.update(kind)
        missing = nullable or not required
        notes = unit_notes(self, message, node, entry)
        if entry['kind'] == 'json':
            notes.append('JSON text.')
        if missing:
            if entry['kind'] in ('string', 'time', 'int'):
                entry['known'] = f'{ros_name}_known'
                notes.append(f'Valid only when {ros_name}_known is true.')
            elif entry['kind'] in ('enum', 'json'):
                notes.append('Empty when not known.')
            elif entry['kind'] == 'float':
                notes.append('NaN when not known.')
            elif entry['kind'] == 'bool':
                entry['tristate'] = True
            elif entry['kind'] == 'msg':
                entry['optional'] = True
                notes.append('Empty or one element.')
            else:
                what = 'a nullable' if nullable else 'an optional'
                raise GeneratorError(f'{where}: {what} {entry["kind"]} is not supported')
        for constant, _ in constant_lines(entry):
            if not CONSTANT_NAME.match(constant):
                raise GeneratorError(f'{where}: {constant!r} is not a legal constant name')
        return _Field(entry, node.get('description'), notes)

    def _ros_name(self, message, json_name, node):
        where = f'{message}.{json_name}'
        ros_name = node.get('x-ros-name')
        if where in self.renames:
            ros_name = self.renames[where]
            self.unused_renames.discard(where)
        if not isinstance(ros_name, str):
            raise GeneratorError(f'{where}: has no x-ros-name')
        if not FIELD_NAME.match(ros_name):
            raise GeneratorError(f'{where}: {ros_name!r} is not a legal field name')
        if ros_name in FORBIDDEN_NAMES:
            raise GeneratorError(f'{where}: the field name {ros_name!r} is a C, C++ or Python '
                                 'keyword; give it another x-ros-name in the daemon (@rosName)')
        if ros_name in IDL_KEYWORDS:
            self.warnings.append({'message': message, 'field': ros_name,
                                  'why': WHY_IDL_KEYWORD})
        elif ros_name in MACRO_NAMES:
            self.warnings.append({'message': message, 'field': ros_name,
                                  'why': WHY_MACRO_NAME})
        return ros_name

    def _resolve(self, where, node):
        """Follow references: return the schema found, its definition name, and nullable."""
        nullable = False
        name = None
        while True:
            reference = reference_of(where, node)
            if reference is None:
                return node, name, nullable
            name, also_null = reference
            nullable = nullable or also_null
            if name not in self.definitions:
                raise GeneratorError(f'{where}: refers to the unknown definition {name}')
            if node.get('x-ros') == 'json':
                return node, name, nullable
            node = self.definitions[name]
            if not isinstance(node, dict):
                raise GeneratorError(f'{where}: the definition {name} is not a schema object')
            if node.get('x-ros') == 'json':
                return node, name, nullable
            if node.get('x-ros') == 'alias' or node.get('type') != 'object':
                self.used.add(name)

    def _kind(self, where, node, item=False):
        """Return the kind (with what belongs to it) and whether the schema allows null."""
        marked_json = node.get('x-ros') == 'json'
        target, name, nullable = self._resolve(where, node)
        if marked_json or target.get('x-ros') == 'json':
            types = target.get('type')
            if isinstance(types, list) and 'null' in types:
                nullable = True
            return {'kind': 'json'}, nullable
        base, null_type = base_type(where, target)
        nullable = nullable or null_type
        if base == 'object':
            if name is None:
                raise GeneratorError(f'{where}: an inline object is not supported; '
                                     'refer to a definition')
            self.message(name)
            return {'kind': 'msg', 'msg': name}, nullable
        if base == 'array':
            if item:
                raise GeneratorError(f'{where}: an array of arrays is not supported')
            return self._array(where, target), nullable
        if base == 'string':
            if 'enum' in target or 'const' in target:
                values = target['enum'] if 'enum' in target else [target['const']]
                if None in values:
                    nullable = True
                values = [value for value in values if value is not None]
                for value in values:
                    if not isinstance(value, str) or not CONSTANT_VALUE.match(value):
                        raise GeneratorError(f'{where}: the enum value {value!r} cannot be '
                                             'a string constant')
                if not values:
                    raise GeneratorError(f'{where}: an enum without a value')
                return {'kind': 'enum', 'values': values}, nullable
            if target.get('format') == 'date-time':
                return {'kind': 'time'}, nullable
            return {'kind': 'string'}, nullable
        if 'enum' in target or ('const' in target and base != 'boolean'):
            raise GeneratorError(f'{where}: an enum of {base} values is not supported')
        if base == 'number':
            return {'kind': 'float'}, nullable
        if base == 'integer':
            return {'kind': 'int'}, nullable
        if base == 'boolean':
            return {'kind': 'bool'}, nullable
        raise GeneratorError(f'{where}: the type {base!r} is not supported')

    def _array(self, where, target):
        items = target.get('items')
        if isinstance(items, list):
            size = len(items)
            numbers = all(isinstance(entry, dict) and entry.get('type') == 'number'
                          for entry in items)
            fixed = target.get('minItems') == size and target.get('maxItems') == size
            if not size or not numbers or not fixed:
                raise GeneratorError(f'{where}: only a fixed tuple of numbers is supported')
            return {'kind': 'tuple', 'size': size}
        if not isinstance(items, dict):
            raise GeneratorError(f'{where}: an array without an item schema is not supported')
        kind, nullable = self._kind(f'{where}[]', items, item=True)
        if nullable:
            raise GeneratorError(f'{where}: an array of nullable items is not supported')
        out = {'kind': 'array', 'items': kind}
        if 'maxItems' in target and 'x-max-items-from' in target:
            bound = target['maxItems']
            if isinstance(bound, bool) or not isinstance(bound, int) or bound < 1:
                raise GeneratorError(f'{where}: maxItems must be a whole number above zero')
            out['max'] = bound
        return out


def reference_of(where, node):
    """Return (definition name, nullable) when the node is a reference, else None."""
    nullable = False
    if '$ref' in node:
        reference = node['$ref']
    elif 'allOf' in node:
        parts = node['allOf']
        if not isinstance(parts, list) or len(parts) != 1 or '$ref' not in parts[0]:
            raise GeneratorError(f'{where}: allOf of anything but one reference is not '
                                 'supported')
        reference = parts[0]['$ref']
    elif 'anyOf' in node:
        parts = node['anyOf']
        references = [part for part in parts if isinstance(part, dict) and '$ref' in part]
        nulls = [part for part in parts if part == {'type': 'null'}]
        if len(references) != 1 or len(references) + len(nulls) != len(parts):
            raise GeneratorError(f'{where}: anyOf of anything but one reference and null is '
                                 'not supported')
        reference = references[0]['$ref']
        nullable = bool(nulls)
    elif 'oneOf' in node:
        raise GeneratorError(f'{where}: oneOf is not supported')
    else:
        return None
    match = REFERENCE.match(reference) if isinstance(reference, str) else None
    if match is None:
        raise GeneratorError(f'{where}: the reference {reference!r} is not supported')
    return match.group(1), nullable


def base_type(where, node):
    """Return the one type of a schema that is not null, and whether null is allowed."""
    types = node.get('type')
    if isinstance(types, str):
        types = [types]
    if not isinstance(types, list):
        raise GeneratorError(f'{where}: a schema without a type is not supported')
    rest = [entry for entry in types if entry != 'null']
    if len(rest) != 1:
        raise GeneratorError(f'{where}: the type {types!r} is not supported')
    return rest[0], 'null' in types


def unit_notes(builder, message, node, entry):
    """Return the comment line that names the unit of a number, if it has one."""
    kind = entry['items']['kind'] if entry['kind'] == 'array' else entry['kind']
    if kind not in ('float', 'int', 'tuple'):
        return []
    sibling = node.get('x-unit-from')
    if isinstance(sibling, str):
        where = f'{message}.{sibling}'
        named = builder.renames.get(where)
        if named is None:
            named = builder.definitions[message]['properties'].get(sibling, {}).get(
                'x-ros-name', sibling)
        return [f'In the unit named by {named}.']
    unit = node.get('x-unit')
    if isinstance(unit, str) and unit not in NO_UNIT:
        return [f'Unit: {unit}.']
    return []


def ros_type(entry):
    """Return the ROS type of a mapping entry (a field or the items of an array)."""
    kind = entry['kind']
    if kind == 'msg':
        return f'{entry["msg"]}[<=1]' if entry.get('optional') else entry['msg']
    if kind == 'bool' and entry.get('tristate'):
        return 'int8'
    if kind == 'tuple':
        return f'float64[{entry["size"]}]'
    if kind == 'array':
        bound = f'<={entry["max"]}' if 'max' in entry else ''
        return f'{ros_type(entry["items"])}[{bound}]'
    return ROS_TYPES[kind]


def constant_lines(entry):
    """Return the constants of a field: (name, the line that declares it)."""
    prefix = entry['ros'].upper()
    values = entry.get('values')
    if entry['kind'] == 'array':
        values = entry['items'].get('values')
    out = []
    for value in values or []:
        name = f'{prefix}_{re.sub(r"[^A-Z0-9]+", "_", value.upper())}'
        out.append((name, f'string {name}="{value}"'))
    if entry.get('tristate'):
        for suffix, number in (('UNKNOWN', -1), ('FALSE', 0), ('TRUE', 1)):
            out.append((f'{prefix}_{suffix}', f'int8 {prefix}_{suffix}={number}'))
    return out


def comment_lines(text):
    """Return a text as comment lines: wrapped, and in printable ASCII without backslash."""
    lines = []
    for index, paragraph in enumerate(re.split(r'\n[ \t]*\n', text.strip())):
        if index:
            lines.append('#')
        line = ''
        for word in paragraph.split():
            word = ''.join(char if ' ' < char <= '~' and char != '\\' else f'U+{ord(char):04X}'
                           for char in word)
            if line and len(line) + 1 + len(word) > COMMENT_WIDTH:
                lines.append(f'# {line}')
                line = word
            else:
                line = f'{line} {word}' if line else word
        if line:
            lines.append(f'# {line}')
    return lines


def member_names(fields_by_message):
    """
    Return {property name: field name} for the properties whose field has another name.

    Only a property that has that one field name in every message that holds it is
    listed: a name that maps two ways names nothing a comment could follow.
    """
    found = {}
    for fields in fields_by_message.values():
        for field in fields:
            found.setdefault(field.entry['json'], set()).add(field.entry['ros'])
    return {json_name: min(ros_names) for json_name, ros_names in sorted(found.items())
            if len(ros_names) == 1 and json_name not in ros_names}


def in_field_names(text, members):
    """Return a description with the property names in back quotes as field names."""
    def renamed(match):
        parts = [members.get(part, part) for part in match.group(1).split('.')]
        return '`' + '.'.join(parts) + '`'

    return MEMBER_PATH.sub(renamed, text)


def without_iso_wording(text):
    """Return the description of a time field without the words that call it text."""
    return ISO_ALONE.sub('', ISO_IN_LIST.sub(')', text))


def is_time(entry):
    """Return whether a mapping entry is a time, or an array of times."""
    kind = entry['items']['kind'] if entry['kind'] == 'array' else entry['kind']
    return kind == 'time'


def render_message(definition, fields, header, members=None):
    """
    Return the text of one .msg file.

    members maps the property names a comment may name to their field names (see
    member_names); without it the descriptions are taken as they are.
    """
    members = members or {}
    lines = [GENERATED_LINE]
    description = definition.get('description')
    if isinstance(description, str) and description.strip():
        lines.append('#')
        lines.extend(comment_lines(in_field_names(description, members)))
    if header:
        lines.append('')
        lines.extend(comment_lines(HEADER_COMMENT))
        lines.append(f'std_msgs/Header {HEADER_FIELD}')
    for field in fields:
        entry = field.entry
        lines.append('')
        if isinstance(field.description, str) and field.description.strip():
            description = in_field_names(field.description, members)
            if is_time(entry):
                description = without_iso_wording(description)
            lines.extend(comment_lines(description))
        for note in field.notes:
            lines.append(f'# {note}')
        lines.append(f'{ros_type(entry)} {entry["ros"]}')
        lines.extend(line for _, line in constant_lines(entry))
        if 'known' in entry:
            lines.append(f'bool {entry["known"]}')
    return '\n'.join(lines) + '\n'


def render_cmake(names):
    """Return the text of msg/interfaces.cmake for these message names."""
    lines = [LICENSE_HEADER.rstrip('\n'), '', GENERATED_LINE, '', 'set(MSG_FILES']
    lines.extend(f'  "msg/{name}.msg"' for name in sorted(names))
    lines.append(')')
    return '\n'.join(lines) + '\n'


def json_text(data):
    """Return the text of a generated JSON file."""
    return json.dumps(data, indent=2, ensure_ascii=True) + '\n'


def generate(schema, renames=None):
    """
    Map a schema (the parsed JSON) to messages, the cmake list and the mapping.

    Without a rename table the one of this file applies. A schema that cannot be mapped
    raises GeneratorError.
    """
    if not isinstance(schema, dict):
        raise GeneratorError('the schema is not a JSON object')
    renames = dict(RENAMES if renames is None else renames)
    for key in ('$id', 'x-schema-version'):
        if not isinstance(schema.get(key), str):
            raise GeneratorError(f'the schema has no {key}')
    builder = _Builder(schema, renames)
    if not builder.roots:
        raise GeneratorError('no definition is marked x-ros: "msg"')
    for name in builder.roots:
        builder.message(name)
    if builder.unused_renames:
        raise GeneratorError('the rename table names no mapped property: '
                             + ', '.join(sorted(builder.unused_renames)))
    hashed = {
        'definitions': {name: builder.definitions[name]
                        for name in sorted(set(builder.order) | builder.used)},
        'renames': renames,
        'ruleVersion': RULE_VERSION,
    }
    canonical = json.dumps(hashed, sort_keys=True, separators=(',', ':'), ensure_ascii=True)
    messages = {}
    mapped = {}
    members = member_names(builder.fields)
    for name in builder.order:
        fields = builder.fields[name]
        header = name in builder.roots
        messages[name] = render_message(builder.definitions[name], fields, header, members)
        mapped[name] = {'header': True} if header else {}
        mapped[name]['fields'] = [field.entry for field in fields]
    mapping = {
        'schemaId': schema['$id'],
        'schemaVersion': schema['x-schema-version'],
        'definitionsSha256': hashlib.sha256(canonical.encode('ascii')).hexdigest(),
        'package': PACKAGE,
        'renames': dict(sorted(renames.items())),
        'warnings': builder.warnings,
        'messages': mapped,
    }
    return Generated(messages, mapping)


def source_record(schema_bytes, generated):
    """Return the content of SOURCE.json for a schema file and what it generated."""
    mapping = generated.mapping
    return {
        'source': SOURCE_NAME,
        'schemaId': mapping['schemaId'],
        'schemaVersion': mapping['schemaVersion'],
        'schemaSha256': hashlib.sha256(schema_bytes).hexdigest(),
        'definitionsSha256': mapping['definitionsSha256'],
        'messages': len(generated.messages),
        'renames': mapping['renames'],
    }


def expected_files(schema_bytes, renames=None):
    """Return every generated file for a schema file: relative path -> text."""
    try:
        schema = json.loads(schema_bytes.decode('utf-8'))
    except ValueError as error:
        raise GeneratorError(f'{SCHEMA_FILE}: not JSON ({error})') from error
    generated = generate(schema, renames)
    files = generated.files()
    files[SOURCE_FILE] = json_text(source_record(schema_bytes, generated))
    return files


def write_tree(out_dir, files):
    """Write the files under a folder and delete a .msg that is no longer generated."""
    written = 0
    for relative, text in sorted(files.items()):
        path = out_dir / relative
        data = text if isinstance(text, bytes) else text.encode('utf-8')
        if path.is_file() and path.read_bytes() == data:
            continue
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)
        written += 1
    removed = 0
    for path in sorted((out_dir / 'msg').glob('*.msg')):
        if f'msg/{path.name}' not in files:
            path.unlink()
            removed += 1
    return written, removed


def check_tree(package_dir):
    """Return every difference between the tree and what a run would write."""
    schema_path = package_dir / SCHEMA_FILE
    if not schema_path.is_file():
        return [f'missing: {SCHEMA_FILE}']
    schema_bytes = schema_path.read_bytes()
    problems = []
    if b'\r' in schema_bytes:
        problems.append(f'{SCHEMA_FILE}: has CR line ends; vendor it again')
    expected = expected_files(schema_bytes)
    try:
        recorded = json.loads((package_dir / SOURCE_FILE).read_text(encoding='utf-8'))
    except (OSError, ValueError):
        recorded = None
    if isinstance(recorded, dict):
        wanted = json.loads(expected[SOURCE_FILE])
        for key in ('schemaSha256', 'schemaVersion'):
            if recorded.get(key) != wanted[key]:
                problems.append(f'{SOURCE_FILE}: {key} is {recorded.get(key)!r}, the '
                                f'vendored schema file has {wanted[key]!r}')
    for relative in sorted(expected):
        path = package_dir / relative
        if not path.is_file():
            problems.append(f'missing: {relative}')
        elif path.read_bytes() != expected[relative].encode('utf-8'):
            problems.append(f'stale: {relative}')
    for path in sorted((package_dir / 'msg').glob('*.msg')):
        if f'msg/{path.name}' not in expected:
            problems.append(f'extra: msg/{path.name}')
    return problems


def main(argv=None):
    """Run the command line; return the exit status."""
    parser = argparse.ArgumentParser(
        description='Generate the messages of abb_rws_ros2_msgs from the vendored '
                    'abb-rws-daemon JSON Schema.')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--check', action='store_true',
                      help='write nothing; exit 1 and list the differences when the '
                           'generated files are not what a run would write')
    mode.add_argument('--vendor', metavar='FILE',
                      help='copy a daemon schema/v0.schema.json in (LF), then generate')
    parser.add_argument('--out-dir', metavar='DIR',
                        help='write the generated tree under DIR instead of this package')
    args = parser.parse_args(argv)
    if args.check and args.out_dir:
        parser.error('--check writes nothing and takes no --out-dir')
    package_dir = Path(__file__).resolve().parent.parent
    try:
        if args.check:
            problems = check_tree(package_dir)
            for problem in problems:
                print(problem)
            if problems:
                print(f'{len(problems)} difference(s); run generate_msgs.py to regenerate')
                return 1
            print('the generated files match the vendored schema')
            return 0
        out_dir = Path(args.out_dir) if args.out_dir else package_dir
        if args.vendor:
            schema_bytes = Path(args.vendor).read_bytes().replace(b'\r\n', b'\n')
        else:
            schema_bytes = (package_dir / SCHEMA_FILE).read_bytes()
        files = expected_files(schema_bytes)
        if args.vendor:
            files[SCHEMA_FILE] = schema_bytes
        written, removed = write_tree(out_dir, files)
    except (GeneratorError, OSError) as error:
        print(f'generate_msgs.py: {error}', file=sys.stderr)
        return 2
    print(f'{len(files)} generated files under {out_dir}: {written} written, {removed} removed')
    return 0


if __name__ == '__main__':
    sys.exit(main())

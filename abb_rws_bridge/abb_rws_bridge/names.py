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
Names the bridge derives: topic name tokens, I/O topic tokens and joint names.

A name is derived by one fixed rule and is never repaired afterwards. Two things that
would end up under one name are not renamed and get no suffix: the later one is refused
and the caller reports it.
"""

from collections.abc import Iterable, Mapping
import re
import string

REPLAY_TOKEN = 'replay'
RESERVED_TOKENS = frozenset({
    'joint_states', 'controller_snapshot', 'io', 'diagnostics', REPLAY_TOKEN,
})

_PLACEHOLDERS = ('controller', 'unit', 'joint', 'axis')
_NOT_A_TOKEN_CHARACTER = re.compile(r'[^A-Za-z0-9_]')
_UNDERSCORES = re.compile(r'_+')
_NUMBER = re.compile(r'[0-9]+')
# What an entry may name as a controller: one path segment without the separators the
# entries use. The daemon's own ids (lower-case letters, digits and hyphens) all fit.
_CONTROLLER_ID = re.compile(r'[A-Za-z0-9][A-Za-z0-9._-]{0,127}')

_IO_SIGNAL_ENTRY = '<controller id>:<signal>'
_JOINT_OVERRIDE_ENTRY = '<controller id>/<unit>/<joint>=<name>'


def is_controller_id(text: object) -> bool:
    """
    Return whether text can name a controller in a parameter.

    Accepted: 1 to 128 letters, digits, ``.``, ``_`` and ``-``, starting with a letter or
    a digit. That leaves out the separators of the entries (``:``, ``/``, ``=``) and
    everything that would change a request path.
    """
    return isinstance(text, str) and _CONTROLLER_ID.fullmatch(text) is not None


def ros_token(text: str) -> str:
    """
    Return text as one token of a ROS topic name.

    Every character outside ``[A-Za-z0-9_]`` becomes ``_``, runs of ``_`` collapse to
    one, a trailing ``_`` is dropped and a leading digit gets ``_`` in front. Raises
    ValueError when nothing is left.
    """
    if not isinstance(text, str):
        raise ValueError(f'{text!r} is not text')
    token = _UNDERSCORES.sub('_', _NOT_A_TOKEN_CHARACTER.sub('_', text)).rstrip('_')
    if not token:
        raise ValueError(f'nothing of {text!r} can be used in a topic name')
    if '0' <= token[0] <= '9':
        token = '_' + token
    return token


def controller_tokens(ids: Iterable[str],
                      primary: str = '') -> tuple[dict[str, str], dict[str, str]]:
    """
    Return the namespace token of every controller, and why some have none.

    The first dictionary maps a controller id to its token; the primary controller's is
    the empty string (its topics sit directly in the node's namespace). The second maps
    an id to its problem: such a controller gets no token and is not bridged. The ids
    are handled in sorted order, so the result does not depend on the order given. A
    token that is reserved, or equal to the token of an earlier id, is refused for the
    later id; no suffix is invented.
    """
    tokens: dict[str, str] = {}
    problems: dict[str, str] = {}
    owners: dict[str, str] = {}
    for controller_id in sorted({entry for entry in ids if isinstance(entry, str)}):
        if primary and controller_id == primary:
            tokens[controller_id] = ''
            continue
        try:
            token = ros_token(controller_id)
        except ValueError:
            problems[controller_id] = (
                f'the id "{controller_id}" leaves nothing to use in a topic name')
            continue
        if token in RESERVED_TOKENS:
            problems[controller_id] = (
                f'its topic name "{token}" is one the bridge uses itself')
        elif token in owners:
            problems[controller_id] = (
                f'its topic name "{token}" is already taken by controller "{owners[token]}"')
        else:
            owners[token] = controller_id
            tokens[controller_id] = token
    return tokens, problems


def io_topic_token(signal: str) -> str:
    """
    Return the topic name token of an I/O signal.

    ``/`` becomes ``_``, then the rule of ros_token: ``Local/DRV_1/do_x`` gives
    ``Local_DRV_1_do_x``. Raises ValueError when nothing is left.
    """
    if not isinstance(signal, str):
        raise ValueError(f'{signal!r} is not text')
    return ros_token(signal.replace('/', '_'))


def parse_io_signals(
        entries: Iterable[str] | None) -> tuple[dict[str, tuple[str, ...]], list[str]]:
    """
    Read the entries ``<controller id>:<signal>`` of the io_signals parameter.

    The first value maps a controller id to its signals, in the order given, each as
    the daemon names it. The second lists the problems, one sentence per bad entry. An
    entry with a problem is in no tuple: an entry that cannot be read, one that is
    listed twice, and one whose topic name is already taken by an earlier signal of the
    same controller (the later signal is the one refused).
    """
    signals: dict[str, list[str]] = {}
    taken: dict[str, dict[str, str]] = {}
    problems: list[str] = []
    for entry in _entries(entries, problems):
        controller_id, separator, signal = (part.strip() for part in entry.partition(':'))
        if not separator or not controller_id or not signal:
            problems.append(f'entry "{entry}" is not of the form {_IO_SIGNAL_ENTRY}')
            continue
        if not is_controller_id(controller_id):
            problems.append(f'entry "{entry}": "{controller_id}" cannot be a controller id')
            continue
        try:
            token = io_topic_token(signal)
        except ValueError:
            problems.append(
                f'entry "{entry}": the signal name leaves nothing to use in a topic name')
            continue
        owners = taken.setdefault(controller_id, {})
        if owners.get(token) == signal:
            problems.append(f'entry "{entry}" is listed twice')
        elif token in owners:
            problems.append(
                f'entry "{entry}": its topic name "io/{token}" is already taken by signal '
                f'"{owners[token]}" of the same controller')
        else:
            owners[token] = signal
            signals.setdefault(controller_id, []).append(signal)
    return {controller_id: tuple(names) for controller_id, names in signals.items()}, problems


def parse_joint_overrides(
        entries: Iterable[str] | None,
) -> tuple[dict[tuple[str, str, str], str], list[str]]:
    """
    Read the entries ``<controller id>/<unit>/<joint>=<name>`` of joint_name_overrides.

    The first value maps (controller id, unit, joint) to the joint's ROS name. The
    second lists the problems, one sentence per bad entry; an entry with a problem is
    not in the dictionary, and a joint that is named twice keeps its first name.
    """
    overrides: dict[tuple[str, str, str], str] = {}
    problems: list[str] = []
    for entry in _entries(entries, problems):
        target, separator, name = (part.strip() for part in entry.partition('='))
        parts = tuple(part.strip() for part in target.split('/'))
        if not separator or not name or len(parts) != 3 or not all(parts):
            problems.append(f'entry "{entry}" is not of the form {_JOINT_OVERRIDE_ENTRY}')
            continue
        if not is_controller_id(parts[0]):
            problems.append(f'entry "{entry}": "{parts[0]}" cannot be a controller id')
            continue
        if parts in overrides:
            problems.append(
                f'entry "{entry}": that joint is already named "{overrides[parts]}"')
            continue
        overrides[parts] = name
    return overrides, problems


class JointNamer:
    """
    Gives every joint its name in ``sensor_msgs/JointState``.

    The format may hold the placeholders ``{controller}`` (the token the caller hands
    in: the node gives the controller's id as a name token, ``cell_1`` for ``cell-1``,
    for the primary controller too), ``{unit}`` (the mechanical unit as reported,
    ``ROB_1``), ``{joint}`` (``J1`` to ``J6``, ``E1`` to ``E6``) and ``{axis}`` (the
    number in the joint name). An override names one joint of one controller and wins
    over the format.
    """

    def __init__(self, fmt: str,
                 overrides: Mapping[tuple[str, str, str], str] | None = None) -> None:
        """
        Check the format and keep the overrides.

        Raises ValueError on an unknown placeholder, on a placeholder with a conversion
        or a format specification, on unbalanced braces and on an empty format.
        """
        if not isinstance(fmt, str) or not fmt.strip():
            raise ValueError('the joint name format is empty')
        try:
            fields = [(field, spec, conversion)
                      for _, field, spec, conversion in string.Formatter().parse(fmt)
                      if field is not None]
        except ValueError as error:
            raise ValueError(f'the joint name format "{fmt}" cannot be read: {error}') from None
        accepted = ', '.join('{' + name + '}' for name in _PLACEHOLDERS)
        for field, spec, conversion in fields:
            if field not in _PLACEHOLDERS:
                raise ValueError(
                    f'the joint name format "{fmt}" holds the unknown placeholder '
                    f'{{{field}}}; accepted: {accepted}')
            if spec or conversion:
                raise ValueError(
                    f'the joint name format "{fmt}" may hold only plain placeholders '
                    f'({accepted}), without a conversion or a format specification')
        self._fmt = fmt
        self._uses_axis = any(field == 'axis' for field, _, _ in fields)
        self._overrides = dict(overrides or {})

    def name(self, controller_id: str, controller_token: str, unit: str, joint: str) -> str:
        """
        Return the ROS name of one joint.

        Raises ValueError when the format asks for ``{axis}`` and the joint name holds
        no number.
        """
        override = self._overrides.get((controller_id, unit, joint))
        if override is not None:
            return override
        axis = ''
        if self._uses_axis:
            number = _NUMBER.search(joint) if isinstance(joint, str) else None
            if number is None:
                raise ValueError(
                    f'the joint name "{joint}" of unit "{unit}" holds no number for {{axis}}')
            axis = number.group()
        return self._fmt.format(
            controller=controller_token, unit=unit, joint=joint, axis=axis)


def _entries(entries: Iterable[str] | None, problems: list[str]) -> list[str]:
    """Return the entries that are text; anything else becomes a problem."""
    if entries is None:
        return []
    if isinstance(entries, str):
        problems.append(f'a list of entries is needed, not the single text "{entries}"')
        return []
    texts = []
    for entry in entries:
        if isinstance(entry, str):
            texts.append(entry)
        else:
            problems.append(f'entry {entry!r} is not text')
    return texts

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
What is current, and the diagnostic statuses the node publishes.

Two kinds of functions, all pure. The first kind judges a controller by its own groups:
is the controller current, are its joints, is a signal. Whether the daemon itself still
answers is the node's knowledge, and it hands that in. The second kind builds the plain
status records (name, hardware id, level, message, values) that the node copies into
``diagnostic_msgs/DiagnosticStatus`` messages.

No status holds a token, a password or the content of a file: the views below have no
field for one.
"""

from collections.abc import Mapping
from dataclasses import dataclass, replace
import re

from abb_rws_bridge import joint_state
from abb_rws_bridge.joint_state import JointStateData

OK, WARN, ERROR, STALE = 0, 1, 2, 3

# An io_unpublishable reason that starts with this text is a signal that changed its
# type: an error, where every other reason is a warning.
TYPE_CHANGED = 'type changed'

_DAEMON_HARDWARE_ID = 'abb-rws-daemon'
_LOCAL_NAMES = ('127.0.0.1', 'localhost', '::1')
_LIVE, _POLLING = 'live', 'polling'
_MODE_LIVE, _MODE_REPLAY = 'live', 'replay'
_REPLAY_MARK = 'REPLAY: '
_NOT_WATCHED = 'joints not watched'
_UNKNOWN = 'unknown'
_NONE = 'none'
_DETAIL_LIMIT = 200
_VERSION = re.compile(r'([0-9]{1,6})\.([0-9]{1,6})')
# The last label of a host name that resolvers read as a number, so as an address.
_NUMBER_LABEL = re.compile(r'[0-9]+|0x[0-9a-f]*')

# Why the stream is not connected, and what to do: reason -> sentence.
_REASONS = {
    'unreachable':
        'no daemon answers at {host_port}; start abb-rws-daemon, or correct daemon_url',
    'host-refused':
        'the daemon does not accept the host name of daemon_url; start abb-rws-daemon '
        'with --allowed-hosts {host}, or use 127.0.0.1. A daemon that knows the option '
        'names the extra hosts in a warning when it starts; one that does not know it '
        'ignores it without a word and must be updated',
    'not-a-daemon':
        'what answers at {host_port} is not abb-rws-daemon; correct daemon_url',
    'token-file':
        'the token file cannot be used; start abb-rws-daemon first (it writes the file), '
        'or correct token_file',
    'token-refused':
        'the daemon refused the token; point token_file at the token file of this daemon. '
        'Right after a good health answer this can also mean that something else holds '
        'the port',
    'replay-refused':
        'the daemon replays a recording; set allow_replay to publish a recording under '
        'replay/',
    'refresh-failed':
        'the daemon stopped answering (two refreshes in a row failed); reconnecting',
    'mode-changed':
        'the daemon changed between live and replay; reconnecting',
    'stream-stalled':
        'the stream stopped delivering what the controller list of the daemon shows (the '
        'connection was open in name only); reconnecting',
    'origin-refused':
        'the daemon refused the origin of the request; this is a fault of the bridge, '
        'please report it',
    'handshake-failed': 'the daemon did not open the stream',
    'protocol':
        'the stream did not follow the protocol (no snapshot first, or a frame that '
        'cannot be read); reconnecting',
    'silent': 'nothing arrived on the stream for too long; reconnecting',
    'daemon-stopping': 'the daemon is shutting down; start it again',
    'replay-restart': 'the replay started over; reconnecting',
    'too-many-consumers':
        'the daemon has no free stream for the bridge; close another consumer of the daemon',
    'policy':
        'the daemon closed the stream because of what the bridge sent; this is a fault of '
        'the bridge, please report it',
    'closed': 'the stream was closed; reconnecting',
}


@dataclass(frozen=True)
class Status:
    """One diagnostic status; every value is text."""

    name: str
    hardware_id: str
    level: int
    message: str
    values: tuple[tuple[str, str], ...]


@dataclass(frozen=True)
class DaemonView:
    """
    What the node knows about its connection to the daemon.

    ``replay`` is the replay part of the daemon's health answer. ``contact_age_s`` is
    the time since the last frame or controller list arrived (None: none yet on this
    connection). ``stamp_source`` is the resolved one, ``daemon`` or ``bridge``.
    ``stream_behind_s`` is for how long a controller list has shown a value that no
    stream frame delivered (None: the stream owes nothing); at ``stale_after_s`` the
    stream counts as stalled.
    """

    node_name: str
    host_port: str
    connected: bool
    reason: str
    detail: str
    retry_in_s: float | None
    mode: str | None
    allow_replay: bool
    replay: Mapping | None
    daemon_version: str | None
    daemon_schema: str | None
    messages_schema: str
    definitions_sha256: str
    connected_for_s: float | None
    reconnects: int
    frames: int
    contact_age_s: float | None
    stale_after_s: float
    clock_offset_ms: float | None
    stamp_source: str
    stamps_held_back: int
    stamps_held_back_recently: bool
    stream_behind_s: float | None = None


@dataclass(frozen=True)
class ControllerView:
    """
    What the node knows about one selected controller.

    ``controller`` is its snapshot as the daemon gave it, None when the daemon does not
    serve it. ``motion_age_s`` is the time since ``motion.asOf`` last moved forward.
    ``leases`` maps ``motion`` and ``io`` to (granted, the daemon's code).
    ``io_rejected`` holds (name, the daemon's reason) and ``io_unpublishable`` (name,
    why), where a why that starts with TYPE_CHANGED is a signal that changed its type.
    ``watch_motion`` is the parameter of that name: with it false the bridge asks for
    no joints, so joints that nobody watches are what was asked for, not a fault.
    """

    node_name: str
    controller_id: str
    controller: Mapping | None
    daemon_live: bool
    name_problem: str | None
    conversion_problem: str | None
    joints: JointStateData | None
    motion_age_s: float | None
    stale_after_s: float
    leases: Mapping[str, tuple[bool, str]]
    io_accepted: tuple[str, ...]
    io_rejected: tuple[tuple[str, str], ...]
    io_unpublishable: tuple[tuple[str, str], ...]
    watch_motion: bool = True


def controller_current(controller: Mapping | None) -> tuple[bool, str]:
    """
    Return whether a controller is current by its link group, and why not.

    Current: ``link.connected`` is true, ``link.quality`` is ``live`` or ``polling``
    and the identity check is not ``mismatch``. The node adds that the daemon is live.
    """
    if not isinstance(controller, Mapping):
        return False, 'not served by the daemon'
    link = controller.get('link')
    if not isinstance(link, Mapping):
        return False, 'the daemon reports no link for it'
    quality = _text(link.get('quality'))
    reason = _text(link.get('qualityReason'))
    if link.get('connected') is not True:
        return False, f'link not connected ({quality}: {reason})'
    if quality not in (_LIVE, _POLLING):
        return False, f'link {quality}: {reason}'
    check = link.get('identityCheck')
    status = check.get('status') if isinstance(check, Mapping) else None
    if status == 'mismatch':
        return False, 'identity mismatch: the link answers as a different controller'
    if not isinstance(status, str):
        return False, 'the daemon reports no identity check for it'
    return True, ''


def joints_current(controller: Mapping | None, age_s: float | None,
                   stale_after_s: float) -> tuple[bool, str]:
    """
    Return whether a controller's joint positions are current, and why not.

    Current: the controller is, ``motion.watching`` is true, ``motion.asOf`` is set and
    age_s, the time since ``motion.asOf`` last moved forward, is below stale_after_s.
    """
    current, why = controller_current(controller)
    if not current:
        return False, why
    motion = controller.get('motion')
    if not isinstance(motion, Mapping):
        return False, 'the daemon reports no motion group for it'
    if motion.get('watching') is not True:
        return False, _NOT_WATCHED
    if motion.get('asOf') is None:
        return False, 'joints not read yet'
    if age_s is None:
        return False, 'joints not confirmed yet'
    if not age_s < stale_after_s:
        return False, f'joints not confirmed for {_seconds(age_s, 1)} s'
    return True, ''


def signal_current(controller: Mapping | None, signal: Mapping | str,
                   granted: bool) -> tuple[bool, str]:
    """
    Return whether a watched signal is current, and why not.

    Current: the controller is, ``io.watching`` is true, the signal was granted to the
    bridge's own lease, its ``via`` is set and its ``error`` is null. signal is the
    entry of ``io.signals``, or its name. There is no age limit on the signal's own
    ``asOf``: the daemon moves it only when the value changes or is polled again.
    """
    current, why = controller_current(controller)
    if not current:
        return False, why
    io = controller.get('io')
    if not isinstance(io, Mapping):
        return False, 'the daemon reports no I/O group for it'
    if io.get('watching') is not True:
        return False, 'I/O not watched'
    if granted is not True:
        return False, 'not granted to the lease of the bridge'
    if isinstance(signal, str):
        signal = _signal_named(io, signal)
    if not isinstance(signal, Mapping):
        return False, 'not in the watch list of the daemon'
    error = signal.get('error')
    if error is not None:
        code = error.get('code') if isinstance(error, Mapping) else None
        return False, f'the last read failed ({_text(code)})'
    if signal.get('via') is None:
        return False, 'not served by the watch yet'
    return True, ''


def schema_verdict(daemon_schema: str | None, messages_schema: str) -> tuple[int, str]:
    """
    Return the level and the sentence for the daemon's schema version.

    OK when it equals the version the messages were generated from, or is not known
    yet. WARN for a higher minor (unknown fields are ignored). ERROR for a lower minor
    or another major: such frames are refused and nothing is published.
    """
    ours = _version(messages_schema)
    if ours is None:
        return ERROR, f'the messages name their schema {_quoted(messages_schema)}, not a version'
    if daemon_schema is None:
        return OK, 'the daemon schema is not known yet'
    theirs = _version(daemon_schema)
    if theirs is None:
        return ERROR, (f'the daemon names its schema {_quoted(daemon_schema)}, not a version; '
                       'nothing is published')
    if theirs[0] != ours[0]:
        return ERROR, (f'the daemon schema {daemon_schema} has another major version than '
                       f'the messages ({messages_schema}); nothing is published')
    if theirs[1] < ours[1]:
        return ERROR, (f'schema too old: the daemon speaks {daemon_schema}, the messages '
                       f'need {messages_schema} or later; update abb-rws-daemon. Nothing is '
                       'published')
    if theirs[1] > ours[1]:
        return WARN, (f'the daemon schema {daemon_schema} is newer than the messages '
                      f'({messages_schema}); what the messages do not know is left out')
    return OK, f'the daemon schema {daemon_schema} is the one of the messages'


def clock_offset_limit_s(stale_after_s: float) -> float:
    """Return the clock offset above which daemon stamps get a warning, in seconds."""
    return min(1.0, stale_after_s / 4)


def stream_stalled(stream_behind_s: float | None, stale_after_s: float) -> bool:
    """
    Return whether the stream counts as stalled.

    stream_behind_s is for how long a controller list has shown a value that no stream
    frame delivered, None when the stream owes nothing. A healthy stream delivers
    within a quarter of a second; one that still owes after stale_after_s is open in
    name only, and what it last delivered is not current any more.
    """
    return stream_behind_s is not None and not stream_behind_s < stale_after_s


def replay_marked(status: Status, replay: Mapping | None) -> Status:
    """
    Return a status as it is shown while a recording is published.

    The message starts with ``REPLAY: recorded <time>`` and a level of OK becomes WARN.
    replay is the replay part of the daemon's health answer. A status that is marked
    already (the daemon status marks itself) keeps its message.
    """
    level = WARN if status.level == OK else status.level
    message = status.message
    if not message.startswith(_REPLAY_MARK):
        message = f'{_replay_text(replay)}; {message}'
    return replace(status, level=level, message=message)


def daemon_status(view: DaemonView) -> Status:
    """Return the status ``<node name>: daemon``."""
    level, message = _daemon_verdict(view)
    return Status(f'{view.node_name}: daemon', _DAEMON_HARDWARE_ID, level, message,
                  _daemon_values(view))


def controller_status(view: ControllerView) -> Status:
    """
    Return the status ``<node name>: controller <id>`` of one selected controller.

    The word ``controller`` keeps the name apart from the daemon status whatever the
    id is (a controller may be called ``daemon``).
    """
    controller = view.controller if isinstance(view.controller, Mapping) else None
    errors: list[str] = []
    stale: list[str] = []
    warnings: list[str] = []
    notes: list[str] = []
    joints_ok = False
    if view.name_problem:
        errors.append(f'not bridged: {view.name_problem}')
    elif not view.daemon_live:
        stale.append('no current data: the daemon is not live (see the daemon status)')
    elif controller is None:
        errors.append('not served by the daemon')
    else:
        current, why = controller_current(controller)
        if not current:
            errors.append(why)
        else:
            link = _group(controller, 'link')
            if link.get('quality') == _POLLING:
                reason = _text(link.get('qualityReason'))
                warnings.append(f'link polling: {reason}')
            joints_ok, why = joints_current(controller, view.motion_age_s, view.stale_after_s)
            if not joints_ok:
                if why == _NOT_WATCHED and view.watch_motion is False:
                    # Asked for: the bridge holds no motion watch and nobody else does.
                    notes.append(f'{_NOT_WATCHED} (watch_motion is false)')
                else:
                    stale.append(why)
    if not view.name_problem:
        _controller_notes(view, errors, warnings)

    if errors:
        level = ERROR
    elif stale:
        level = STALE
    elif warnings:
        level = WARN
    else:
        level = OK
    problems = errors + stale + warnings
    message = '; '.join(problems + notes if problems else ['current'] + notes)
    return Status(f'{view.node_name}: controller {view.controller_id}', view.controller_id,
                  level, message, _controller_values(view, controller, joints_ok))


def _controller_notes(view: ControllerView, errors: list[str],
                      warnings: list[str]) -> None:
    """Add what the joints, the signals and the leases of a controller give."""
    if view.conversion_problem:
        errors.append(f'the snapshot does not fit the messages: {view.conversion_problem}')
    joints = view.joints
    if joints is not None:
        if joints.problem is not None and joints.problem != joint_state.NO_JOINT:
            errors.append(f'joint_states stopped: {joints.problem}')
        if joints.skipped:
            count = _count(len(joints.skipped), 'joint')
            warnings.append(f'{count} left out of joint_states: {_skipped_text(joints)}')
    changed = [entry for entry in view.io_unpublishable if _type_changed(entry[1])]
    others = [entry for entry in view.io_unpublishable if not _type_changed(entry[1])]
    if changed:
        count = _count(len(changed), 'signal')
        errors.append(f'{count} changed type and stopped: {_pairs_text(changed)}')
    if view.io_rejected:
        count = _count(len(view.io_rejected), 'signal')
        warnings.append(f'{count} refused by the daemon: {_pairs_text(view.io_rejected)}')
    if others:
        count = _count(len(others), 'signal')
        warnings.append(f'{count} not publishable: {_pairs_text(others)}')
    for kind in ('motion', 'io'):
        lease = view.leases.get(kind)
        if lease is not None and lease[0] is not True:
            warnings.append(f'{kind} lease refused: {_text(lease[1]) or _UNKNOWN}')


def _controller_values(view: ControllerView, controller: Mapping | None,
                       joints_ok: bool) -> tuple[tuple[str, str], ...]:
    """Return the values of a controller status, every one as text."""
    link = _group(controller, 'link')
    check = _group(link, 'identityCheck')
    panel = _group(controller, 'panel')
    motion = _group(controller, 'motion')
    rate = _group(motion, 'rate')
    joints = view.joints
    published = 0
    if joints is not None and joints.problem is None and joints_ok:
        published = len(joints.names)
    return (
        ('link quality', _text(link.get('quality'))),
        ('link quality reason', _text(link.get('qualityReason'))),
        ('identity check', _text(check.get('status'))),
        ('controller state', _text(panel.get('ctrlState'))),
        ('operating mode', _text(panel.get('opMode'))),
        ('motion watching', _text(motion.get('watching'))),
        ('motion asOf', _text(motion.get('asOf'))),
        ('motion asOf age (s)', _seconds(view.motion_age_s)),
        ('target rate (Hz)', _text(rate.get('targetHz'))),
        ('achieved rate (Hz)', _text(rate.get('achievedHz'))),
        ('joints published', str(published)),
        ('joints left out', _skipped_text(joints) if joints is not None else _NONE),
        ('motion lease', _lease_text(view.leases.get('motion'))),
        ('io lease', _lease_text(view.leases.get('io'))),
        ('signals accepted', ', '.join(view.io_accepted) or _NONE),
        ('signals refused', _pairs_text(view.io_rejected)),
        ('signals not publishable', _pairs_text(view.io_unpublishable)),
    )


def _daemon_verdict(view: DaemonView) -> tuple[int, str]:
    """Return the level and the message of the daemon status."""
    if not view.connected:
        return ERROR, _not_connected(view)
    schema_level, schema_text = schema_verdict(view.daemon_schema, view.messages_schema)
    if schema_level == ERROR:
        return ERROR, schema_text
    if view.mode == _MODE_REPLAY and not view.allow_replay:
        return ERROR, 'replay-refused: ' + _REASONS['replay-refused']
    if view.mode not in (_MODE_LIVE, _MODE_REPLAY):
        return ERROR, (f'the daemon reports the mode {_quoted(view.mode)}, which this bridge '
                       'does not know; nothing is published')
    if view.contact_age_s is None:
        return STALE, 'connected, waiting for the first snapshot'
    if not view.contact_age_s < view.stale_after_s:
        return STALE, f'no answer from the daemon for {_seconds(view.contact_age_s, 1)} s'
    if stream_stalled(view.stream_behind_s, view.stale_after_s):
        return STALE, ('the stream has not delivered what the controller list shows for '
                       f'{_seconds(view.stream_behind_s, 1)} s; reconnecting')

    warnings = []
    if view.mode == _MODE_REPLAY:
        warnings.append(_replay_text(view.replay))
    if schema_level == WARN:
        warnings.append(schema_text)
    if view.stamps_held_back_recently:
        count = _count(view.stamps_held_back, 'stamp')
        warnings.append(f'{count} held back: a stamp behind the last published one is not '
                        'published')
    offset = view.clock_offset_ms
    if view.stamp_source == 'daemon' and offset is not None:
        limit_ms = clock_offset_limit_s(view.stale_after_s) * 1000.0
        if not abs(offset) <= limit_ms:
            warnings.append(f'the clock of the daemon and this clock differ by '
                            f'{_number(offset, 0)} ms (limit {_number(limit_ms, 0)} ms) and '
                            "stamps are the daemon's; set stamp_source to bridge, or "
                            'correct the clocks')
    if warnings:
        return WARN, '; '.join(warnings)
    return OK, 'live'


def _not_connected(view: DaemonView) -> str:
    """Return the message of a daemon that is not connected: the reason, what to do."""
    reason = _clean(view.reason)
    host = _host_of(view.host_port)
    if not reason:
        message = 'not connected: no attempt to reach the daemon has finished yet'
    elif reason == 'host-refused' and host in _LOCAL_NAMES:
        message = ('host-refused: the daemon does not accept the host name of daemon_url '
                   f'although it is a local one; check what listens at {view.host_port}')
    elif reason == 'host-refused' and reads_as_address(host):
        # The daemon takes no address as an extra host: naming the option would be a
        # hint that stops the daemon at launch.
        message = ('host-refused: the daemon does not accept the address in daemon_url as '
                   'a host, and no option of the daemon changes that: --allowed-hosts '
                   'takes DNS host names only. An address other than loopback means a '
                   'forwarder in front of the daemon; use 127.0.0.1 on the machine of the '
                   'daemon, or the host name a container has for that machine')
    elif reason in _REASONS:
        text = _REASONS[reason].format(host=host, host_port=view.host_port)
        message = f'{reason}: {text}'
    else:
        message = f'not connected: {reason}'
    detail = _clean(view.detail)
    if detail:
        message += f' ({detail})'
    if view.retry_in_s is not None:
        message += f'; next try in {_seconds(view.retry_in_s, 1)} s'
    return message


def _daemon_values(view: DaemonView) -> tuple[tuple[str, str], ...]:
    """Return the values of the daemon status, every one as text."""
    values = [
        ('daemon', _text(view.host_port)),
        ('connected', _text(bool(view.connected))),
        ('daemon version', _text(view.daemon_version)),
        ('mode', _text(view.mode)),
        ('daemon schema', _text(view.daemon_schema)),
        ('messages schema', _text(view.messages_schema)),
        ('definitions sha256', _text(view.definitions_sha256)[:12]),
        ('connected for (s)', _seconds(view.connected_for_s)),
        ('reconnects', _text(view.reconnects)),
        ('frames received', _text(view.frames)),
        ('last contact age (s)', _seconds(view.contact_age_s)),
        ('stream behind for (s)', _stream_behind_text(view.stream_behind_s)),
        ('clock offset (ms)', _number(view.clock_offset_ms, 0)),
        ('stamp source', _text(view.stamp_source)),
        ('stamps held back', _text(view.stamps_held_back)),
    ]
    if not view.connected:
        values.append(('reason', _clean(view.reason) or _NONE))
        values.append(('detail', _clean(view.detail) or _NONE))
        values.append(('next try in (s)', _seconds(view.retry_in_s)))
    if isinstance(view.replay, Mapping):
        replay = view.replay
        values.extend([
            ('replay state', _text(replay.get('state'))),
            ('replay position (ms)', _text(replay.get('positionMs'))),
            ('replay speed', _text(replay.get('speed'))),
            ('replay time mode', _text(replay.get('time'))),
            ('recorded at', _text(replay.get('recordedAt'))),
        ])
    return tuple(values)


def _stream_behind_text(stream_behind_s: float | None) -> str:
    """Return for how long the stream has owed a frame, or that it owes none."""
    if stream_behind_s is None:
        return 'not behind'
    return _seconds(stream_behind_s, 1)


def _replay_text(replay: Mapping | None) -> str:
    """Return the start of every message while a recording is published."""
    recorded = replay.get('recordedAt') if isinstance(replay, Mapping) else None
    if isinstance(recorded, str) and recorded:
        return f'{_REPLAY_MARK}recorded {_clean(recorded, 40)}'
    return f'{_REPLAY_MARK}recorded at a time that is not known'


def _type_changed(why: object) -> bool:
    """Return whether an io_unpublishable reason says the signal changed its type."""
    return isinstance(why, str) and why.lower().startswith(TYPE_CHANGED)


def _skipped_text(joints: JointStateData) -> str:
    """Return the joints that are left out, each with its reason."""
    return ', '.join(f'{skipped.unit}/{skipped.joint}: {skipped.reason}'
                     for skipped in joints.skipped) or _NONE


def _pairs_text(pairs: tuple[tuple[str, str], ...]) -> str:
    """Return (name, text) pairs as one line."""
    return '; '.join(f'{name}: {text}' for name, text in pairs) or _NONE


def _lease_text(lease: tuple[bool, str] | None) -> str:
    """Return one lease result as text."""
    if lease is None:
        return 'not asked'
    if lease[0] is True:
        return 'granted'
    return f'refused: {_text(lease[1]) or _UNKNOWN}'


def _signal_named(io: Mapping, name: str) -> Mapping | None:
    """Return the entry of io.signals with this name, or None."""
    signals = io.get('signals')
    if not isinstance(signals, (list, tuple)):
        return None
    for signal in signals:
        if isinstance(signal, Mapping) and signal.get('name') == name:
            return signal
    return None


def _group(parent: Mapping | None, key: str) -> Mapping:
    """Return a nested mapping, or an empty one when it is missing or null."""
    value = parent.get(key) if isinstance(parent, Mapping) else None
    return value if isinstance(value, Mapping) else {}


def _version(text: object) -> tuple[int, int] | None:
    """Return (major, minor) of a schema version such as 0.12, or None."""
    match = _VERSION.fullmatch(text) if isinstance(text, str) else None
    if match is None:
        return None
    return int(match.group(1)), int(match.group(2))


def reads_as_address(host: str) -> bool:
    """
    Return whether a host is an IP address, or a name that is read as one.

    The daemon's own rule for the names it accepts besides loopback: anything with a
    colon (IPv6), a dotted IPv4 address, and every name whose last label is a number
    (all digits, or ``0x`` followed by hexadecimal digits or by nothing), because
    resolvers read such a name as an IPv4 address (``2130706433``, ``0x7f.1``).
    """
    if not isinstance(host, str) or not host:
        return False
    if ':' in host:
        return True
    last = host.rstrip('.').rsplit('.', 1)[-1].lower()
    return _NUMBER_LABEL.fullmatch(last) is not None


def _host_of(host_port: object) -> str:
    """Return the host of ``host:port`` or ``[v6]:port``, without brackets."""
    text = host_port if isinstance(host_port, str) else ''
    if text.startswith('['):
        return text[1:].partition(']')[0]
    host, separator, port = text.rpartition(':')
    return host if separator and port.isdigit() else text


def _count(number: int, noun: str) -> str:
    """Return ``1 joint`` or ``3 joints``."""
    return f'{number} {noun}' if number == 1 else f'{number} {noun}s'


def _text(value: object) -> str:
    """Return a value of a status as text."""
    if value is None:
        return _UNKNOWN
    if value is True:
        return 'true'
    if value is False:
        return 'false'
    return str(value)


def _number(value: float | None, digits: int) -> str:
    """Return a number with a fixed count of decimals, or ``unknown``."""
    if value is None:
        return _UNKNOWN
    try:
        return f'{float(value):.{digits}f}'
    except (TypeError, ValueError, OverflowError):
        return _UNKNOWN


def _seconds(value: float | None, digits: int = 3) -> str:
    """Return a time span in seconds as text, or ``unknown``."""
    return _number(value, digits)


def _quoted(value: object) -> str:
    """Return a short printable form of a value, in quotes."""
    return f'"{_clean(_text(value), 40)}"'


def _clean(text: object, limit: int = _DETAIL_LIMIT) -> str:
    """Return text cut to the limit, with nothing but printable ASCII in it."""
    if not isinstance(text, str):
        return ''
    return ''.join(character if ' ' <= character <= '~' else '?'
                   for character in text[:limit]).strip()

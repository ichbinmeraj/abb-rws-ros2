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
The parameters of the bridge node: the table, and the rules that span parameters.

``SPECS`` is the one list of parameters: the node declares them from it, and the
parameter file and the documentation are checked against it. ``settings_from`` turns the
values the node read into typed settings, or into the list of everything that is wrong
with them. Parameters are read once at start and never change afterwards.
"""

from collections.abc import Mapping
from dataclasses import dataclass
import ipaddress
import math
import re
from types import MappingProxyType
from urllib.parse import urlsplit

from abb_rws_bridge import names

_STRING, _BOOL, _DOUBLE, _STRING_ARRAY = 'string', 'bool', 'double', 'string_array'


@dataclass(frozen=True)
class Spec:
    """
    One parameter of the node.

    ``kind`` is ``string``, ``bool``, ``double`` or ``string_array``. The default of a
    string array is the empty tuple. ``minimum`` and ``maximum`` are set for a bounded
    double and are both inclusive.
    """

    name: str
    kind: str
    default: object
    description: str
    minimum: float | None = None
    maximum: float | None = None


SPECS: tuple[Spec, ...] = (
    Spec('daemon_url', _STRING, 'http://127.0.0.1:7810',
         'Where abb-rws-daemon listens: http://<host>:<port> and nothing else (no https, '
         'no user info, no path, no query). The host must be 127.0.0.1, localhost, ::1, '
         'host.docker.internal or host.containers.internal unless allow_remote_daemon is '
         'true.'),
    Spec('allow_remote_daemon', _BOOL, False,
         'Accept any host in daemon_url. The daemon listens on 127.0.0.1 only, so another '
         'host is a forwarder in front of it, and the access token then travels '
         'unencrypted to that host.'),
    Spec('token_file', _STRING, '',
         'File that holds the access token of the daemon. Empty: the file named by '
         'ABB_RWS_DAEMON_TOKEN_FILE, else abb-rws-daemon/token under XDG_CONFIG_HOME, else '
         'under ~/.config. Only the path is a parameter, never the token.'),
    Spec('controllers', _STRING_ARRAY, (),
         'Daemon controller ids to bridge. Empty: every controller the daemon serves.'),
    Spec('primary_controller', _STRING, '',
         'The topics of this controller sit directly in the namespace of the node '
         '(joint_states, controller_snapshot, io/<signal>). Every other controller gets a '
         'namespace of its own, made from its id.'),
    Spec('joint_name_format', _STRING, '{unit}_{joint}',
         'Joint names in joint_states. Placeholders: {controller} (the id of the '
         'controller as it is written in topic names, cell_1 for cell-1), {unit} (the '
         'mechanical unit, ROB_1), {joint} (J1 to J6, E1 to E6) and {axis} (the number in '
         'the joint name).'),
    Spec('joint_name_overrides', _STRING_ARRAY, (),
         'Entries <controller id>/<unit>/<joint>=<name> that name single joints, for '
         'example cell-1/ROB_1/J1=joint_1. They win over joint_name_format.'),
    Spec('joint_states_frame_id', _STRING, '',
         'header.frame_id of joint_states.'),
    Spec('stamp_source', _STRING, 'auto',
         'Which clock stamps joint_states and controller_snapshot: daemon (the times the '
         'daemon reports), bridge (the clock of this machine when the value arrived) or '
         'auto (daemon when daemon_url names 127.0.0.1, localhost or ::1, otherwise '
         'bridge).'),
    Spec('watch_motion', _BOOL, True,
         'Hold a motion watch on every bridged controller, which makes the daemon sample '
         'its joints. False: joints appear only while another consumer of the daemon '
         'watches them.'),
    Spec('io_signals', _STRING_ARRAY, (),
         'Entries <controller id>:<signal> to watch and publish under io/, the signal as '
         'the daemon names it (do_gripper, or network/device/signal). The daemon accepts '
         'at most 16 for one controller.'),
    Spec('allow_replay', _BOOL, False,
         'A daemon in replay mode serves a recording, not a robot. False: nothing is '
         'published from it. True: it is published under replay/ names only, never on '
         'the live topic names.'),
    Spec('refresh_hz', _DOUBLE, 2.0,
         'How often the bridge asks the daemon for all controllers, in Hz. This confirms '
         'values that did not change and shows that the daemon still answers.',
         0.2, 10.0),
    Spec('republish_hz', _DOUBLE, 1.0,
         'Keep-alive rate of controller_snapshot and the I/O topics while their data is '
         'current, in Hz. 0: on change only.',
         0.0, 10.0),
    Spec('stale_after_s', _DOUBLE, 2.0,
         'A value not confirmed for this many seconds is stale, and a daemon that has not '
         'answered for as long is treated as gone. Must be at least 2 / refresh_hz.',
         0.5, 60.0),
    Spec('diagnostics_period_s', _DOUBLE, 1.0,
         'Period of /diagnostics in seconds.',
         0.2, 10.0),
    Spec('reconnect_min_s', _DOUBLE, 0.5,
         'Shortest wait before the bridge connects to the daemon again, in seconds. Must '
         'not exceed reconnect_max_s.',
         0.1, 60.0),
    Spec('reconnect_max_s', _DOUBLE, 15.0,
         'Longest wait before the bridge connects to the daemon again, in seconds.',
         1.0, 300.0),
    Spec('http_timeout_s', _DOUBLE, 5.0,
         'Timeout of every request to the daemon and of opening the stream, in seconds.',
         0.5, 30.0),
)

BUILTIN_PARAMETERS = frozenset({'use_sim_time', 'start_type_description_service'})
QOS_OVERRIDE_PREFIX = 'qos_overrides.'
STAMP_SOURCES = ('auto', 'daemon', 'bridge')
TRUSTED_DAEMON_HOSTS = frozenset({
    '127.0.0.1', 'localhost', '::1', 'host.docker.internal', 'host.containers.internal',
})

_URL_FORM = 'http://<host>:<port>'
_HOST_LABEL = r'[a-z0-9](?:[a-z0-9-]{0,61}[a-z0-9])?'
_HOST_NAME = re.compile(rf'{_HOST_LABEL}(?:\.{_HOST_LABEL})*')
_HOST_NAME_LIMIT = 253
_TOLERANCE = 1e-9


@dataclass(frozen=True)
class Settings:
    """
    The checked parameters of the node, typed.

    One field per parameter. String arrays are tuples. ``io_signals`` maps a controller
    id to its signals and ``joint_name_overrides`` maps (controller id, unit, joint) to
    a name; both are read-only. ``daemon_url`` is written as ``http://<host>:<port>``,
    with the host in lower case. ``token_file`` is the parameter as given (empty: the
    default path).

    Derived: ``daemon_host`` (an IPv6 address without brackets), ``daemon_port``,
    ``ws_url`` (the stream) and ``refresh_timeout_s`` (the smaller of
    ``http_timeout_s`` and ``stale_after_s``).
    """

    daemon_url: str
    allow_remote_daemon: bool
    token_file: str
    controllers: tuple[str, ...]
    primary_controller: str
    joint_name_format: str
    joint_name_overrides: Mapping[tuple[str, str, str], str]
    joint_states_frame_id: str
    stamp_source: str
    watch_motion: bool
    io_signals: Mapping[str, tuple[str, ...]]
    allow_replay: bool
    refresh_hz: float
    republish_hz: float
    stale_after_s: float
    diagnostics_period_s: float
    reconnect_min_s: float
    reconnect_max_s: float
    http_timeout_s: float
    daemon_host: str
    daemon_port: int
    ws_url: str
    refresh_timeout_s: float


def settings_from(values: Mapping[str, object]) -> tuple[Settings | None, list[str]]:
    """
    Return (settings, no problems) or (None, every problem found).

    values maps parameter names to plain values (str, bool, float, a list or tuple of
    str). A name that is missing takes its default; names that are not in SPECS are not
    looked at, except ``use_sim_time``: true is a problem, because every stamp is a
    wall-clock time. Each problem is one sentence that starts with the parameter's
    name.
    """
    problems: list[str] = []
    wrong: set[str] = set()
    read = {spec.name: _value(spec, values, problems, wrong) for spec in SPECS}

    address = None
    if 'daemon_url' not in wrong:
        address = _daemon_address(read['daemon_url'], problems)
    if address is not None and 'allow_remote_daemon' not in wrong:
        host = address[0]
        if host not in TRUSTED_DAEMON_HOSTS and not read['allow_remote_daemon']:
            accepted = ', '.join(sorted(TRUSTED_DAEMON_HOSTS))
            problems.append(
                f'daemon_url: the host "{host}" is not this machine (accepted: {accepted}). '
                'The access token would travel unencrypted to it; set allow_remote_daemon '
                'to true only if that is what you want')

    controllers = read['controllers']
    primary = read['primary_controller']
    _check_controllers(controllers, primary, problems, wrong)

    overrides, found = names.parse_joint_overrides(read['joint_name_overrides'])
    problems.extend(f'joint_name_overrides: {problem}' for problem in found)
    _check_bridged('joint_name_overrides', {key[0] for key in overrides}, controllers,
                   problems, wrong)
    if 'joint_name_format' not in wrong:
        try:
            names.JointNamer(read['joint_name_format'], overrides)
        except ValueError as error:
            problems.append(f'joint_name_format: {error}')

    stamp_source = read['stamp_source']
    if 'stamp_source' not in wrong and stamp_source not in STAMP_SOURCES:
        accepted = ', '.join(STAMP_SOURCES)
        problems.append(
            f'stamp_source: "{_shown(stamp_source)}" is not one of {accepted}')

    signals, found = names.parse_io_signals(read['io_signals'])
    problems.extend(f'io_signals: {problem}' for problem in found)
    _check_bridged('io_signals', set(signals), controllers, problems, wrong)

    stale_after_s, refresh_hz = read['stale_after_s'], read['refresh_hz']
    if not wrong & {'stale_after_s', 'refresh_hz'}:
        needed = 2.0 / refresh_hz
        if stale_after_s < needed - _TOLERANCE:
            problems.append(
                f'stale_after_s: {stale_after_s:g} is below 2 / refresh_hz ({needed:g} with '
                f'refresh_hz {refresh_hz:g}); a value needs two chances to be confirmed '
                'before it counts as stale')
    reconnect_min_s, reconnect_max_s = read['reconnect_min_s'], read['reconnect_max_s']
    if not wrong & {'reconnect_min_s', 'reconnect_max_s'}:
        if reconnect_min_s > reconnect_max_s:
            problems.append(
                f'reconnect_min_s: {reconnect_min_s:g} is above reconnect_max_s '
                f'({reconnect_max_s:g})')

    if values.get('use_sim_time') is True:
        problems.append(
            'use_sim_time: simulated time is not supported; every stamp is a wall-clock '
            'time, from the daemon or from the system clock')

    if problems or address is None:
        return None, problems

    host, port = address
    host_text = f'[{host}]' if ':' in host else host
    return Settings(
        daemon_url=f'http://{host_text}:{port}',
        allow_remote_daemon=read['allow_remote_daemon'],
        token_file=read['token_file'],
        controllers=controllers,
        primary_controller=primary,
        joint_name_format=read['joint_name_format'],
        joint_name_overrides=MappingProxyType(overrides),
        joint_states_frame_id=read['joint_states_frame_id'],
        stamp_source=read['stamp_source'],
        watch_motion=read['watch_motion'],
        io_signals=MappingProxyType(signals),
        allow_replay=read['allow_replay'],
        refresh_hz=read['refresh_hz'],
        republish_hz=read['republish_hz'],
        stale_after_s=read['stale_after_s'],
        diagnostics_period_s=read['diagnostics_period_s'],
        reconnect_min_s=read['reconnect_min_s'],
        reconnect_max_s=read['reconnect_max_s'],
        http_timeout_s=read['http_timeout_s'],
        daemon_host=host,
        daemon_port=port,
        ws_url=f'ws://{host_text}:{port}/v0/stream',
        refresh_timeout_s=min(read['http_timeout_s'], read['stale_after_s']),
    ), []


def _value(spec: Spec, values: Mapping[str, object], problems: list[str],
           wrong: set[str]) -> object:
    """Return one parameter with the type of its spec; its default when it is wrong."""
    if spec.name not in values:
        return spec.default
    value = values[spec.name]
    problem = None
    if spec.kind == _STRING:
        if not isinstance(value, str):
            problem = f'needs text, not {_kind_of(value)}'
    elif spec.kind == _BOOL:
        if not isinstance(value, bool):
            problem = f'needs true or false, not {_kind_of(value)}'
    elif spec.kind == _DOUBLE:
        problem = _double_problem(spec, value)
        if problem is None:
            value = float(value)
    else:
        if value is None:
            value = ()
        elif isinstance(value, (list, tuple)) and all(isinstance(e, str) for e in value):
            value = tuple(value)
        else:
            problem = f'needs a list of texts, not {_kind_of(value)}'
    if problem is None:
        return value
    problems.append(f'{spec.name}: {problem}')
    wrong.add(spec.name)
    return spec.default


def _double_problem(spec: Spec, value: object) -> str | None:
    """Return why a value is not a double inside the range of its spec, or None."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return f'needs a number, not {_kind_of(value)}'
    if isinstance(value, int):
        return f'needs a number with a decimal point (write {value}.0, not {value})'
    if not math.isfinite(value):
        return 'needs a finite number'
    low, high = spec.minimum, spec.maximum
    if (low is not None and value < low - _TOLERANCE) or (
            high is not None and value > high + _TOLERANCE):
        return f'{value:g} is outside the range {low:g} to {high:g}'
    return None


def _check_controllers(controllers: tuple[str, ...], primary: str, problems: list[str],
                       wrong: set[str]) -> None:
    """Add what is wrong with controllers and primary_controller."""
    seen: set[str] = set()
    for controller_id in controllers:
        if not names.is_controller_id(controller_id):
            problems.append(
                f'controllers: "{_shown(controller_id)}" cannot be a controller id')
        elif controller_id in seen:
            problems.append(f'controllers: "{controller_id}" is listed twice')
        seen.add(controller_id)
    if 'primary_controller' in wrong or not primary:
        return
    if not names.is_controller_id(primary):
        problems.append(
            f'primary_controller: "{_shown(primary)}" cannot be a controller id')
    elif 'controllers' not in wrong and controllers and primary not in controllers:
        listed = ', '.join(controllers)
        problems.append(f'primary_controller: "{primary}" is not in controllers ({listed})')


def _check_bridged(parameter: str, named: set[str], controllers: tuple[str, ...],
                   problems: list[str], wrong: set[str]) -> None:
    """
    Add a problem for every controller a parameter names that is not bridged.

    With a controllers list, an entry for a controller outside it would have no effect
    and nobody would be told: no topic, no status. So it stops the node at start, like a
    primary_controller the list does not hold. With an empty list every controller the
    daemon serves is bridged, and a name it does not serve gets a status of its own.
    """
    if not controllers or wrong & {'controllers', parameter}:
        return
    listed = ', '.join(controllers)
    for controller_id in sorted(named - set(controllers)):
        problems.append(f'{parameter}: an entry names the controller '
                        f'"{_shown(controller_id)}", which is not in controllers ({listed})')


def _daemon_address(url: str, problems: list[str]) -> tuple[str, int] | None:
    """
    Return (host, port) of daemon_url, or None after adding what is wrong with it.

    The text of the URL is never repeated in a problem: it may hold a password.
    """
    found = _url_problems(url)
    if isinstance(found, tuple):
        return found
    problems.extend(f'daemon_url: {problem}' for problem in found)
    return None


def _url_problems(url: str) -> tuple[str, int] | list[str]:
    """Return (host, port) of a daemon URL, or the list of what is wrong with it."""
    if not url:
        return [f'is empty; write {_URL_FORM}']
    if any(not '!' <= character <= '~' for character in url):
        return ['holds a space or a character that is not printable ASCII; '
                f'write {_URL_FORM}']
    lowered = url.lower()
    if lowered.startswith('https://'):
        return ['https is not possible, the daemon has no TLS; '
                f'write {_URL_FORM}']
    if not lowered.startswith('http://'):
        return [f'must be {_URL_FORM}']
    try:
        parts = urlsplit(url)
        host = parts.hostname
    except ValueError:
        return [f'cannot be read; write {_URL_FORM}']
    found = []
    if '@' in parts.netloc:
        found.append('must not hold user info (a name or a password)')
    if '\\' in url:
        found.append('must not hold a backslash')
    if parts.path not in ('', '/'):
        found.append('must not hold a path')
    if '?' in url:
        found.append('must not hold a query')
    if '#' in url:
        found.append('must not hold a fragment')
    try:
        port = parts.port
    except ValueError:
        port = 0
    if port is None:
        found.append(f'needs an explicit port; write {_URL_FORM}')
    elif not 1 <= port <= 65535:
        found.append('needs a port from 1 to 65535')
    bracketed = '[' in parts.netloc or ']' in parts.netloc
    if not host:
        found.append(f'names no host; write {_URL_FORM}')
    elif not _is_host(host) or bracketed != (':' in host):
        found.append('names a host that is neither a host name nor an IP address')
    if found:
        return found
    return host, port


def _is_host(host: str) -> bool:
    """Return whether text is a DNS host name, an IPv4 or an IPv6 address."""
    if ':' in host:
        if '%' in host:
            return False
        try:
            ipaddress.IPv6Address(host)
        except ValueError:
            return False
        return True
    return len(host) <= _HOST_NAME_LIMIT and _HOST_NAME.fullmatch(host) is not None


def _kind_of(value: object) -> str:
    """Return what a value of the wrong type is, in plain words."""
    if value is None:
        return 'nothing'
    if isinstance(value, bool):
        return 'true or false'
    if isinstance(value, (int, float)):
        return 'a number'
    if isinstance(value, str):
        return 'a single text'
    if isinstance(value, (list, tuple)):
        return 'a list'
    return type(value).__name__


def _shown(text: str) -> str:
    """Return a short, printable form of a text for a problem."""
    return ''.join(character if ' ' <= character <= '~' else '?' for character in text[:60])

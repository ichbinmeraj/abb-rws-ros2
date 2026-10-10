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
Builders of daemon data for the tests: synthetic, hand-written, valid against the schema.

Every builder returns new plain dicts and lists in the shapes of the abb-rws-daemon JSON
Schema that the message package vendors; ``test_frames_schema.py`` validates each of them
against it. Nothing here was read from a controller.

Import it as a top-level module (``import frames``): this folder has no ``__init__.py``
and pytest puts it on the import path.

Times are the daemon's: ISO 8601 in UTC with milliseconds (``T0``, ``iso(seconds)``).
Joint values are in the joint's own unit (``deg``, ``mm`` or None), as on the wire.
"""

import copy
import datetime
import json
from pathlib import Path
import re

SCHEMA_VERSION = '0.13'
# The minors either side of it, for a daemon newer or older than the messages.
NEWER_SCHEMA_VERSION = f'0.{int(SCHEMA_VERSION.split(".")[1]) + 1}'
OLDER_SCHEMA_VERSION = f'0.{int(SCHEMA_VERSION.split(".")[1]) - 1}'
GROUPS = ('identity', 'link', 'panel', 'rapid', 'motion', 'io', 'events', 'access',
          'capabilities', 'freshness')
CAPABILITY_IDS = ('status-push', 'events-push', 'event-times', 'io-push', 'io-table',
                  'joint-limits', 'mastership', 'write-access', 'rmmp')
FRESH_GROUPS = ('identity', 'panel', 'rapid', 'motion', 'io', 'events', 'access')
SIGNAL_TYPES = ('DI', 'DO', 'AI', 'AO', 'GI', 'GO')

T0 = '2026-01-01T00:00:00.000Z'
SYS_ID = '00000000-0000-4000-8000-000000000001'
LEASE_ID = 'lease-1'
LEASE_MS = 15000
MOTION_HZ = 8.0

_EPOCH = datetime.datetime(2026, 1, 1, tzinfo=datetime.timezone.utc)
_UNSET = object()
_DAEMON_TIME = re.compile(r'^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}(\.\d+)?Z$')
_SCHEMA_FILE = ('schema', 'v0.schema.json')
_SCHEMA_PACKAGE = 'abb_rws_ros2_msgs'

# How the daemon answers a refused request: code -> (HTTP status, text of the error).
_ERRORS = {
    'TOKEN_MISSING': (401, "this request needs the daemon's access token"),
    'TOKEN_INVALID': (401, "the access token is not this daemon's"),
    'NO_CONTROLLER': (404, 'no controller with id "{id}"'),
    'IDENTITY_MISMATCH': (409, 'the link of "{id}" answers as a different controller'),
    'HOST_NOT_ALLOWED': (421, 'host not allowed'),
    'LEASE_LIMIT': (429, '"{id}" already holds every watch lease it may hold'),
}
ERROR_CODES = tuple(_ERRORS)


def iso(offset_s=0.0):
    """Return the time offset_s seconds after T0, as the daemon writes times."""
    moment = _EPOCH + datetime.timedelta(milliseconds=round(offset_s * 1000))
    return moment.strftime('%Y-%m-%dT%H:%M:%S.') + f'{moment.microsecond // 1000:03d}Z'


def joint(name='J1', value=0.0, unit='deg', minimum=-180.0, maximum=180.0):
    """Return one MotionJoint; value, unit, minimum and maximum may each be None."""
    return {'name': name, 'value': value, 'unit': unit, 'min': minimum, 'max': maximum}


def tcp_pose(x=500.0, y=0.0, z=600.0, q=(1.0, 0.0, 0.0, 0.0), *,
             tool=None, wobj=None, coord=None):
    """Return one MotionPose: millimetres, quaternion as w, x, y, z."""
    return {'x': x, 'y': y, 'z': z, 'q': list(q), 'tool': tool, 'wobj': wobj, 'coord': coord}


def group_error(code='RESOURCE_NOT_FOUND', message='no such resource on the controller', *,
                controller=None, refusal=None):
    """
    Return one GroupError.

    controller is (HTTP status, code, message) as the controller answered, each of which
    may be None; refusal is a mapping. Both keys are left out when not given.
    """
    error = {'code': code, 'message': message}
    if controller is not None:
        http_status, controller_code, controller_message = controller
        error['controller'] = {'httpStatus': http_status, 'code': controller_code,
                               'message': controller_message}
    if refusal is not None:
        error['refusal'] = dict(refusal)
    return error


def motion_unit(name='ROB_1', joints=None, *, kind='robot', type_raw='TCPRobot',
                task='T_ROB1', pose=None, error=None):
    """Return one MotionUnitInfo; without joints: J1 to J6 at 0 degrees."""
    if joints is None:
        joints = [joint(f'J{axis}') for axis in range(1, 7)]
    return {'name': name, 'kind': kind, 'typeRaw': type_raw, 'task': task,
            'joints': list(joints), 'pose': pose, 'error': error}


def signal(name='do_gripper', signal_type='DO', value='0', *,
           via='subscription', as_of=T0, error=None):
    """Return one IoWatchedSignal; the value is text, as the controller reports it."""
    return {'name': name, 'type': signal_type, 'value': value, 'via': via, 'asOf': as_of,
            'error': error}


def robot_ware(raw='7.21.0', major=7, minor=21, generation='rws2'):
    """Return one RobotWareVersion."""
    return {'raw': raw, 'major': major, 'minor': minor, 'generation': generation}


def identity_group(*, controller_name='Test_Controller', controller_id='TEST-0001',
                   controller_type='Virtual Controller', virtual=True,
                   mac='02:00:00:00:00:01', system_name='Test_System', sys_id=SYS_ID,
                   start_time='2026-01-01 T 00:00:00', version=None,
                   options=('RobotWare Base', 'English'), as_of=T0):
    """Return one IdentityGroup; version is a RobotWareVersion (default: robot_ware())."""
    return {
        'controllerName': controller_name, 'controllerId': controller_id,
        'controllerType': controller_type, 'virtual': virtual, 'mac': mac,
        'systemName': system_name, 'sysId': sys_id, 'startTime': start_time,
        'robotWare': robot_ware() if version is None else version,
        'options': list(options), 'asOf': as_of,
    }


def link_group(*, connected=True, quality='live', quality_reason='events streaming',
               since=T0, host='127.0.0.1', port=5466, use_https=True, subscriptions=True,
               identity_status='confirmed', expected=SYS_ID, actual=SYS_ID, reconnect=None,
               down_since=None, mode='live', recorded_at=None):
    """
    Return one LinkGroup: by default a healthy link to the controller that was added.

    reconnect is None or a ReconnectInfo, {'failures': 2, 'nextAttemptAt': iso(5)}.
    down_since is None while the link is healthy, else when it last was.
    mode is 'live', or 'replay' with recorded_at (when the recorded session began).
    """
    return {
        'connected': connected, 'quality': quality, 'qualityReason': quality_reason,
        'since': since, 'host': host, 'port': port, 'useHttps': use_https,
        'subscriptions': {'active': subscriptions},
        'identityCheck': {'status': identity_status, 'expected': expected, 'actual': actual},
        'reconnect': reconnect, 'downSince': down_since,
        'mode': mode, 'recordedAt': recorded_at,
    }


def panel_group(*, ctrl_state='motoron', op_mode='AUTO', speed_ratio=100.0,
                collision_detection=None, as_of=T0):
    """Return one PanelGroup."""
    return {'ctrlState': ctrl_state, 'opMode': op_mode, 'speedRatio': speed_ratio,
            'collisionDetection': collision_detection, 'asOf': as_of}


def rapid_pointer(module='MainModule', routine='main', line=1):
    """Return one RapidPointer."""
    return {'module': module, 'routine': routine, 'line': line}


def rapid_task(name='T_ROB1', *, task_type='NORMAL', task_state='started',
               exec_state='stopped', active=True, motion=True, modules=('MainModule',),
               pp=None, mp=None):
    """Return one RapidTaskInfo; pp and mp are None or a RapidPointer."""
    return {'name': name, 'type': task_type, 'taskState': task_state, 'execState': exec_state,
            'active': active, 'motion': motion, 'modules': list(modules), 'pp': pp, 'mp': mp}


def rapid_group(tasks=None, *, exec_state='stopped', cycle='forever', as_of=T0):
    """Return one RapidGroup; without tasks: one motion task."""
    return {'execState': exec_state, 'cycle': cycle,
            'tasks': [rapid_task()] if tasks is None else list(tasks), 'asOf': as_of}


def motion_group(units=None, *, watching=True, as_of=T0, target_hz=MOTION_HZ,
                 achieved_hz=7.9):
    """
    Return one MotionGroup.

    Watched and without units: one robot, J1 to J6 at 0 degrees. Not watched and without
    units: the group as it is before anybody watches (no units, no rate, asOf null). Not
    watched with units: the last values stay, the rate is null.
    """
    if units is None and not watching:
        as_of = None
    if units is None:
        units = [motion_unit()] if watching else []
    rate = {'targetHz': target_hz if watching else None,
            'achievedHz': achieved_hz if watching else None,
            'via': 'poll' if watching else None}
    return {'watching': watching, 'units': list(units), 'rate': rate, 'asOf': as_of}


def io_group(signals=None, *, watching=None, as_of=T0):
    """
    Return one IoGroup.

    Without signals: nothing watched and asOf null. watching defaults to "there are
    signals"; pass False with signals for a lease that ran out.
    """
    signals = list(signals or [])
    if watching is None:
        watching = bool(signals)
    return {'watching': watching, 'signals': signals, 'asOf': as_of if signals else None}


def event(seq=1, *, seqnum=None, code=99999, severity='info', title='Synthetic test event',
          title_from_code=False, description=None, causes=None, consequences=None,
          actions=None, args=(), source=None, raw_time='2026-01-01 T 00:00:00', utc=T0,
          corrected=True, received_at=T0, backlog=False):
    """Return one ControllerEvent; args is a sequence of (type, value) pairs of text."""
    return {
        'seq': seq, 'seqnum': seqnum, 'code': code, 'severity': severity, 'title': title,
        'titleFromCode': title_from_code, 'description': description, 'causes': causes,
        'consequences': consequences, 'actions': actions,
        'args': [{'type': arg_type, 'value': value} for arg_type, value in args],
        'source': source, 'time': {'raw': raw_time, 'utc': utc, 'corrected': corrected},
        'receivedAt': received_at, 'backlog': backlog,
    }


def events_group(recent=None, *, feed='feed-1', last_seq=None, via='subscription',
                 clock=None, as_of=T0, error=None):
    """
    Return one EventsGroup.

    last_seq defaults to the highest seq of recent (0 without events); clock defaults to
    a measured offset of 0 ms.
    """
    recent = list(recent or [])
    if last_seq is None:
        last_seq = max((entry['seq'] for entry in recent), default=0)
    if clock is None:
        clock = {'offsetMs': 0, 'measuredAt': as_of, 'problem': None}
    return {'feed': feed, 'lastSeq': last_seq, 'recent': recent, 'via': via, 'clock': clock,
            'asOf': as_of, 'error': error}


def access_lock(lock='motion', *, held=False, holder=None, held_by_daemon=False,
                reported='nomaster', as_of=T0, error=None):
    """Return one AccessLockInfo: by default a RobotWare 7 lock nobody holds."""
    return {'lock': lock, 'held': held, 'holder': holder, 'heldByDaemon': held_by_daemon,
            'reported': reported, 'asOf': as_of, 'error': error}


def access_group(locks=None, *, generation='rw7', model='mastership',
                 remote_access_enabled=None, rmmp=None, via='poll', problem=None, as_of=T0):
    """
    Return one AccessGroup.

    By default what a RobotWare 7 controller reports when nobody holds anything: its two
    locks (edit, motion) free, RMMP offered and the daemon's session holding no privilege.
    """
    if locks is None:
        locks = [access_lock('edit'), access_lock('motion')]
    if rmmp is None:
        rmmp = {'supported': True, 'privilege': 'none', 'asOf': as_of, 'error': None}
    return {'generation': generation, 'model': model, 'locks': list(locks),
            'remoteAccessEnabled': remote_access_enabled, 'rmmp': rmmp, 'via': via,
            'problem': problem, 'asOf': as_of}


def capabilities_group(entries=None, *, protocol='rws2', generation='rw7',
                       robot_ware_raw='7.21.0', tested=True, as_of=T0):
    """
    Return one CapabilitiesGroup; entries maps a capability id to (state, reason).

    By default every capability of a RobotWare 7 controller is available, except write
    access, which only RobotWare 8 has.
    """
    states = {cap: ('available', None) for cap in CAPABILITY_IDS}
    states['write-access'] = ('unavailable', 'RobotWare 8 only')
    states.update(entries or {})
    return {
        'protocol': protocol, 'generation': generation, 'robotWare': robot_ware_raw,
        'tested': tested,
        'entries': [{'id': cap, 'state': states[cap][0], 'reason': states[cap][1]}
                    for cap in CAPABILITY_IDS],
        'asOf': as_of,
    }


def freshness_group(states=None, *, level='current', reasons=(), as_of=T0):
    """
    Return one FreshnessGroup; states maps a group name to its state.

    By default what goes with the default controller(): every group current, except
    I/O, which is idle (nothing watched).
    """
    by_group = {group: 'current' for group in FRESH_GROUPS}
    by_group['io'] = 'idle'
    by_group.update(states or {})
    return {
        'level': level, 'reasons': list(reasons),
        'groups': [{'group': group, 'state': by_group[group],
                    'asOf': None if by_group[group] in ('idle', 'unread') else as_of,
                    'reason': None}
                   for group in FRESH_GROUPS],
    }


def controller(controller_id='cell-1', *, name=None, identity=_UNSET, link=None, panel=None,
               rapid=None, motion=None, io=None, events=None, access=None, capabilities=None,
               freshness=None):
    """
    Return one ControllerSnapshot.

    By default: a connected virtual controller, its six robot joints watched, no I/O
    watched, no events, nobody holding any lock. Every group can be replaced;
    identity=None gives the null identity of a controller that has not been read yet.
    name defaults to the id.
    """
    return {
        'id': controller_id,
        'name': controller_id if name is None else name,
        'identity': identity_group() if identity is _UNSET else identity,
        'link': link_group() if link is None else link,
        'panel': panel_group() if panel is None else panel,
        'rapid': rapid_group() if rapid is None else rapid,
        'motion': motion_group() if motion is None else motion,
        'io': io_group() if io is None else io,
        'events': events_group() if events is None else events,
        'access': access_group() if access is None else access,
        'capabilities': capabilities_group() if capabilities is None else capabilities,
        'freshness': freshness_group() if freshness is None else freshness,
    }


def groups_of(snapshot):
    """Return the ten groups of a ControllerSnapshot: what a delta for a new id carries."""
    return {group: copy.deepcopy(snapshot[group]) for group in GROUPS}


def snapshot_frame(controllers=None, *, observed_at=T0, schema_version=SCHEMA_VERSION):
    """Return one SnapshotFrame; without controllers: one default controller()."""
    return {
        'type': 'snapshot', 'schemaVersion': schema_version, 'observedAt': observed_at,
        'controllers': [controller()] if controllers is None else list(controllers),
    }


def delta_frame(controller_id='cell-1', groups=None, *, name=None, observed_at=T0):
    """Return one DeltaFrame; groups maps group names to their new, whole values."""
    return {'type': 'delta', 'observedAt': observed_at, 'id': controller_id,
            'name': controller_id if name is None else name, 'groups': dict(groups or {})}


def removed_frame(controller_id='cell-1', *, observed_at=T0):
    """Return one RemovedFrame."""
    return {'type': 'removed', 'observedAt': observed_at, 'id': controller_id}


def replay_health(*, state='playing', position_ms=0, duration_ms=60000, speed=1.0,
                  loop=False, loops=0, recorded_at=T0, time_mode='shifted'):
    """Return one ReplayHealth, the replay part of a health answer."""
    return {'state': state, 'positionMs': position_ms, 'durationMs': duration_ms,
            'speed': speed, 'loop': loop, 'loops': loops, 'recordedAt': recorded_at,
            'time': time_mode}


def health(*, mode='live', version='0.1.0', started_at=T0, uptime_s=0, recording=False,
           replay=_UNSET):
    """
    Return one HealthAnswer.

    replay defaults to null in live mode and to replay_health() in replay mode.
    """
    if replay is _UNSET:
        replay = replay_health() if mode == 'replay' else None
    return {
        'status': 'ok', 'version': version, 'startedAt': started_at, 'uptimeS': uptime_s,
        'mode': mode,
        'recording': {'active': recording, 'since': started_at if recording else None},
        'replay': replay,
    }


def controller_list(controllers=None, *, observed_at=T0, schema_version=SCHEMA_VERSION):
    """Return one ControllerList; without controllers: one default controller()."""
    return {
        'schemaVersion': schema_version, 'observedAt': observed_at,
        'controllers': [controller()] if controllers is None else list(controllers),
    }


def motion_watch_answer(*, lease_ms=LEASE_MS, target_hz=MOTION_HZ, lease_id=LEASE_ID):
    """Return one MotionWatch; lease_id=None leaves the id out (the shared lease)."""
    answer = {'leaseMs': lease_ms, 'targetHz': target_hz}
    if lease_id is not None:
        answer['leaseId'] = lease_id
    return answer


def io_watch_answer(accepted=(), rejected=(), *, lease_ms=LEASE_MS, lease_id=LEASE_ID):
    """
    Return one IoWatch.

    accepted is a sequence of signal names, rejected a sequence of (name, reason) pairs;
    lease_id=None leaves the id out (the shared lease).
    """
    answer = {
        'leaseMs': lease_ms, 'accepted': list(accepted),
        'rejected': [{'name': name, 'reason': reason} for name, reason in rejected],
    }
    if lease_id is not None:
        answer['leaseId'] = lease_id
    return answer


def error_answer(code, *, controller_id='cell-1', retry_after_s=15):
    """
    Return (HTTP status, headers, ErrorBody) of a refused request.

    code is one of ERROR_CODES: TOKEN_MISSING and TOKEN_INVALID (401), NO_CONTROLLER
    (404), IDENTITY_MISMATCH (409), HOST_NOT_ALLOWED (421), LEASE_LIMIT (429, with a
    Retry-After header of retry_after_s seconds). Any other code is a KeyError.
    """
    status, text = _ERRORS[code]
    headers = {'Content-Type': 'application/json; charset=utf-8'}
    if code == 'LEASE_LIMIT':
        headers['Retry-After'] = str(retry_after_s)
    return status, headers, {'error': text.format(id=controller_id), 'code': code}


def io_signal_table(signals=None):
    """
    Return the answer of GET /v0/controllers/<id>/io/signals.

    Its shape is {"signals": [{"name", "type", "value"}]}; the schema has no definition
    for it. signals is a sequence of signal() dicts or (name, type, value) triples;
    without it: one default signal().
    """
    rows = []
    for entry in [signal()] if signals is None else signals:
        if isinstance(entry, dict):
            name, signal_type, value = entry['name'], entry['type'], entry['value']
        else:
            name, signal_type, value = entry
        rows.append({'name': name, 'type': signal_type, 'value': value})
    return {'signals': rows}


def edge_controller(controller_id='edge-1'):
    """
    Return a new copy of the edge-case controller (EDGE_CONTROLLER).

    identity is null. Unit ROB_1 has J1 and J2 in degrees, J3 with no value, J4 with no
    unit, J5 and J6 in degrees and the external axis E1 in millimetres. Unit STN_1 has
    an error, so its joint holds a last good value. The I/O group has one signal of
    each type (DI, DO, AI, AO, GI, GO) and one that does not exist on the controller.
    The link is polled, one task has a program pointer, and there is one event.
    """
    robot = motion_unit('ROB_1', [
        joint('J1', 10.0),
        joint('J2', -20.5),
        joint('J3', None),
        joint('J4', 5.0, None, None, None),
        joint('J5', 45.0),
        joint('J6', 90.0, 'deg', -400.0, 400.0),
        joint('E1', 1250.0, 'mm', 0.0, 5000.0),
    ], pose=tcp_pose(815.5, -12.25, 962.0, (0.5, 0.0, 0.866025, 0.0)))
    station = motion_unit(
        'STN_1', [joint('J1', 30.0)], kind='positioner', type_raw=None, task=None,
        error=group_error('CONTROLLER_ERROR', 'the unit could not be read',
                          controller=(503, -1, 'synthetic controller message')))
    signals = [
        signal('di_ready', 'DI', '1'),
        signal('do_gripper', 'DO', '0'),
        signal('ai_pressure', 'AI', '2.5'),
        signal('ao_speed', 'AO', '-0.75', via='poll'),
        signal('gi_code', 'GI', '42'),
        signal('Local/DRV_1/go_program', 'GO', '4294967295', via='poll'),
        signal('do_missing', None, None, via=None, as_of=None,
               error=group_error(refusal={'kind': 'synthetic', 'fix': 'none'})),
    ]
    return controller(
        controller_id,
        name='Edge cases',
        identity=None,
        link=link_group(quality='polling', quality_reason='polled', subscriptions=False,
                        use_https=False, identity_status='unverified', expected=None,
                        actual=None),
        panel=panel_group(ctrl_state=None, op_mode='MANR', speed_ratio=None),
        rapid=rapid_group([
            rapid_task(exec_state='running', pp=rapid_pointer('MainModule', 'main', 12),
                       mp=rapid_pointer('MainModule', None, None)),
            rapid_task('T_BACK', task_type='STATIC', task_state=None, exec_state=None,
                       active=False, motion=False, modules=()),
        ], exec_state='running', cycle=None),
        motion=motion_group([robot, station], as_of=iso(0.5), achieved_hz=None),
        io=io_group(signals),
        events=events_group(
            [event(7, seqnum=None, code=None, severity=None, title_from_code=True,
                   args=[('string', 'ROB_1'), ('long', '3')], raw_time=None, utc=None,
                   corrected=False, backlog=True)],
            via='poll', clock={'offsetMs': None, 'measuredAt': None, 'problem': 'not read'}),
    )


EDGE_CONTROLLER = edge_controller()


def _schema_path():
    here = Path(__file__).resolve()
    source = here.parent.parent.parent.joinpath(_SCHEMA_PACKAGE, *_SCHEMA_FILE)
    if source.is_file():
        return source
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory(_SCHEMA_PACKAGE)).joinpath(*_SCHEMA_FILE)


def load_schema():
    """
    Return the vendored abb-rws-daemon schema, parsed.

    It is read from the message package's source folder beside this package when that
    folder exists, otherwise from the installed share folder of abb_rws_ros2_msgs. This
    is the one way the tests find the schema.
    """
    with open(_schema_path(), encoding='utf-8') as handle:
        return json.load(handle)


def schema_errors(definition, data, schema=None):
    """
    Return why data does not fit a definition of the schema; an empty list when it fits.

    Validates as JSON Schema draft 7 against load_schema() (or the schema given), and a
    date-time must be written as the daemon writes it (UTC, ending in Z). Needs the
    jsonschema package, which is imported only here.
    """
    import jsonschema

    schema = load_schema() if schema is None else schema
    if definition not in schema['definitions']:
        raise KeyError(f'the schema has no definition {definition}')
    checker = jsonschema.FormatChecker(formats=())

    @checker.checks('date-time')
    def _is_daemon_time(value):
        return not isinstance(value, str) or _DAEMON_TIME.match(value) is not None

    validator = jsonschema.Draft7Validator({
        '$schema': 'http://json-schema.org/draft-07/schema#',
        '$ref': f'#/definitions/{definition}',
        'definitions': schema['definitions'],
    }, format_checker=checker)
    return sorted(
        '/'.join(str(part) for part in error.absolute_path) + ': ' + error.message
        for error in validator.iter_errors(data))

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
The bridge node: what abb-rws-daemon reports, republished as ROS 2 topics.

One process, one node, one connection to one daemon, any number of controllers. The
node reads and republishes; it is not a motion driver. It offers no service of its own
and subscribes to nothing, so nothing on the ROS graph can make it write to a daemon or
to a controller.

Topics, relative to the node's namespace (``<c>/`` is a controller's namespace token,
nothing for the primary controller):

- ``<c>/joint_states`` (``sensor_msgs/JointState``): radians and metres, while current;
- ``<c>/controller_snapshot`` (``abb_rws_ros2_msgs/ControllerSnapshot``): the daemon's
  own units and times, on every change and as a keep-alive, while the daemon is live;
- ``<c>/io/<signal>`` (``std_msgs/Bool``, ``Float64`` or ``UInt32``): on change and as a
  keep-alive, while current;
- ``/diagnostics`` (``diagnostic_msgs/DiagnosticArray``): always.

A silent data topic means there is no current data, and ``/diagnostics`` says why. Data
of a daemon that replays a recording is never published on these names: with
``allow_replay`` it goes out under ``replay/`` instead.

This is the only module of the package that imports ``rclpy``. The daemon client hands
its events over from its own threads; the node takes them from a queue on the executor
thread, so everything below runs on one thread and needs no lock.
"""

from collections import deque
from collections.abc import Mapping
import logging
import queue
import time

from abb_rws_bridge import health
from abb_rws_bridge import io_values
from abb_rws_bridge import joint_state
from abb_rws_bridge import names
from abb_rws_bridge import parameters
from abb_rws_bridge import stamps
from abb_rws_bridge import timeconv
from abb_rws_bridge.convert import ConversionError, Converter
from abb_rws_bridge.daemon_client import ClientSettings
from abb_rws_bridge.daemon_client import Connected
from abb_rws_bridge.daemon_client import DaemonClient
from abb_rws_bridge.daemon_client import Disconnected
from abb_rws_bridge.daemon_client import FrameReceived
from abb_rws_bridge.daemon_client import HealthRead
from abb_rws_bridge.daemon_client import LeaseResult
from abb_rws_bridge.daemon_client import Refreshed
from abb_rws_bridge.state import FleetState
from abb_rws_ros2_msgs.msg import ControllerSnapshot
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rcl_interfaces.msg import FloatingPointRange, ParameterDescriptor
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.exceptions import InvalidParameterTypeException
from rclpy.exceptions import InvalidParameterValueException
from rclpy.executors import ExternalShutdownException
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos_overriding_options import QoSOverridingOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, UInt32

NODE_NAME = 'abb_rws_bridge'

DIAGNOSTICS_TOPIC = '/diagnostics'
JOINT_STATES = 'joint_states'
CONTROLLER_SNAPSHOT = 'controller_snapshot'
IO = 'io'

EXIT_OK = 0
EXIT_START_PROBLEM = 2

# How often the node looks at the daemon contact and at the age of the joints.
WATCH_TICK_S = 0.25
# Every publisher: reliable, volatile, keep last 10.
QOS_DEPTH = 10

_MODE_LIVE = 'live'
_MODE_REPLAY = 'replay'
_LIVE_PREFIX = ''
_SNAPSHOT_MESSAGE = 'ControllerSnapshot'
# The one module besides this one that logs: from its own threads, through ``logging``.
_CLIENT_LOGGER = DaemonClient.__module__
# Events taken from the queue in one turn of the executor; the rest waits for the next.
_EVENTS_PER_TURN = 200
# A count of held-back stamps is a warning for this long after it last grew.
_HELD_BACK_RECENT_S = 5.0
# How many frames and controller lists the clock offset is judged from.
_CLOCK_SAMPLES = 8
_THROTTLE_S = 10.0
_SHOWN = 80
_NOT_KNOWN = object()

_LEVELS = {
    health.OK: DiagnosticStatus.OK,
    health.WARN: DiagnosticStatus.WARN,
    health.ERROR: DiagnosticStatus.ERROR,
    health.STALE: DiagnosticStatus.STALE,
}
_IO_MESSAGES = {
    io_values.BOOL: Bool,
    io_values.FLOAT64: Float64,
    io_values.UINT32: UInt32,
}
_ACCEPTED = {
    'string': 'text',
    'bool': 'true or false',
    'string_array': 'a list of texts',
}


class StartProblem(Exception):
    """
    The node cannot start with what it was given.

    ``problems`` holds one readable sentence per problem. ``main`` writes each as one
    line and ends the process with exit code 2. ``logger_name`` is the name of the
    logger of the node that could not start.
    """

    def __init__(self, problems, logger_name: str = NODE_NAME) -> None:
        self.problems = tuple(problems)
        self.logger_name = logger_name
        super().__init__('; '.join(self.problems))


class _Signal:
    """One configured I/O signal of a controller, as the node publishes it."""

    __slots__ = ('topic', 'publisher', 'last')

    def __init__(self, topic: str) -> None:
        self.topic = topic
        # Made when the type of the signal is first known.
        self.publisher = None
        # What was last published, as (kind, value); None after a silence.
        self.last = None


class _Outputs:
    """The publishers of one bridged controller."""

    __slots__ = ('token', 'prefix', 'snapshot_topic', 'joints_topic', 'snapshot', 'joints',
                 'signals')

    def __init__(self, token: str, prefix: str) -> None:
        self.token = token
        self.prefix = prefix
        self.snapshot_topic = _topic(prefix, token, CONTROLLER_SNAPSHOT)
        self.joints_topic = _topic(prefix, token, JOINT_STATES)
        self.snapshot = None
        self.joints = None
        self.signals = {}


class _LogForwarder(logging.Handler):
    """Hands what the daemon client logs on to the logger of the node."""

    def __init__(self, logger) -> None:
        super().__init__(logging.DEBUG)
        self._logger = logger

    def emit(self, record: logging.LogRecord) -> None:
        """
        Write one record: errors and warnings as they are, everything else at DEBUG.

        The node itself says, once, when the connection changes; the client's line for
        every single attempt is detail.
        """
        try:
            # One call per severity: a call site of an rclpy logger keeps its severity.
            if record.levelno >= logging.ERROR:
                self._logger.error(record.getMessage())
            elif record.levelno >= logging.WARNING:
                self._logger.warning(record.getMessage())
            elif self._logger.is_enabled_for(LoggingSeverity.DEBUG):
                self._logger.debug(record.getMessage())
        except Exception:  # a log line must never end a thread of the client
            pass


class BridgeNode(Node):
    """
    The supervisory bridge.

    Parameters are read once, at construction, and are read-only afterwards. A
    parameter that cannot be used raises StartProblem and leaves no node behind. The
    daemon client starts with the node; ``close()`` (or ``destroy_node()``) stops it.
    Keyword arguments other than node_name go to ``rclpy.node.Node``.
    """

    def __init__(self, *, node_name: str = NODE_NAME, **kwargs) -> None:
        super().__init__(node_name, **kwargs)
        self._closed = False
        self._daemon = None
        self._tick_timers = []
        self._forwarder = None
        self._forwarder_level = logging.NOTSET
        try:
            self._settings = self._read_parameters()
            self._converter = _installed_converter()
        except StartProblem as problem:
            problem.logger_name = self.get_logger().name
            super().destroy_node()
            raise
        settings = self._settings
        self._host_port = settings.daemon_url[len('http://'):]
        self._stamp_source = stamps.resolve_source(settings.stamp_source, settings.daemon_host)
        self._namer = names.JointNamer(settings.joint_name_format,
                                       settings.joint_name_overrides)
        # A joint stamp that was there when a controller was first seen counts only when
        # it was fresh then: an old sample is never published as a current one.
        self._fleet = FleetState(self._converter.schema_version,
                                 first_sight_max_age_s=settings.stale_after_s)
        self._mono = time.monotonic
        self.faults = 0

        # The connection, as the events of the client describe it.
        self._connected = False
        self._connects = 0
        self._connected_mono = None
        self._reason = ''
        self._detail = ''
        self._retry_mono = None
        self._daemon_health = None
        self._health_is_fresh = False
        self._mode = None
        self._session_mode = None
        # Where data may go: '' (the live names), 'replay', or None (nowhere).
        self._prefix = None
        self._contact_mono = None
        self._frames = 0
        # The daemon's clock against this one, as the last frames and lists showed it.
        self._clock_offsets = deque(maxlen=_CLOCK_SAMPLES)
        self._held_back_mono = None
        self._was_live = False
        # Whether this connection was already ended for a stream that stalled.
        self._stall_ended = False

        # The controllers.
        self._known = set()
        self._tokens = {}
        self._name_problems = {}
        self._outputs = {}
        self._gates = {}
        self._io_kinds = {}
        self._leases = {}
        self._io_accepted = {}
        self._io_rejected = {}
        self._conversion_problems = {}
        self._wanted = ()
        self._last_levels = None
        # The controllers that io_signals and joint_name_overrides name.
        self._named = frozenset(settings.io_signals) | frozenset(
            key[0] for key in settings.joint_name_overrides)
        if settings.controllers:
            self._tokens, self._name_problems = names.controller_tokens(
                settings.controllers, settings.primary_controller)
            for controller_id, problem in sorted(self._name_problems.items()):
                self.get_logger().error(
                    f'controller "{controller_id}" is not bridged: {problem}')

        self._diagnostics = self.create_publisher(DiagnosticArray, DIAGNOSTICS_TOPIC,
                                                  _qos_profile())
        self._events = queue.SimpleQueue()
        self._wakeup = self.create_guard_condition(self._on_events)
        # Wall timers: a step of the system clock must not stop or hurry them.
        self._steady = Clock(clock_type=ClockType.STEADY_TIME)
        self._add_timer(settings.diagnostics_period_s, self._on_diagnostics_timer,
                        'publishing the diagnostics')
        self._add_timer(WATCH_TICK_S, self._on_watch_tick, 'watching the daemon contact')
        if settings.republish_hz > 0:
            self._add_timer(1.0 / settings.republish_hz, self._on_keep_alive,
                            'publishing a keep-alive')

        self._log_start()
        self._forward_client_log()
        self._daemon = DaemonClient(
            ClientSettings(
                daemon_url=settings.daemon_url,
                token_file=settings.token_file,
                allow_replay=settings.allow_replay,
                refresh_hz=settings.refresh_hz,
                refresh_timeout_s=settings.refresh_timeout_s,
                reconnect_min_s=settings.reconnect_min_s,
                reconnect_max_s=settings.reconnect_max_s,
                http_timeout_s=settings.http_timeout_s),
            self._sink, self._targets, clock=self._mono)
        self._daemon.start()

    # ----- life cycle -------------------------------------------------------------------

    def close(self, timeout_s: float = 3.0) -> None:
        """
        Stop the timers and the daemon client; safe to call more than once.

        Returns within about timeout_s seconds whatever the daemon does. Watched I/O
        signals are given back at once; a motion watch runs out by itself.
        """
        if self._closed:
            return
        self._closed = True
        for timer in self._tick_timers:
            timer.cancel()
        if self._daemon is not None:
            self._daemon.stop(timeout_s)
        if self._forwarder is not None:
            client_logger = logging.getLogger(_CLIENT_LOGGER)
            client_logger.removeHandler(self._forwarder)
            client_logger.setLevel(self._forwarder_level)
            self._forwarder = None

    def destroy_node(self):
        """Close the bridge, then destroy the node."""
        self.close()
        return super().destroy_node()

    # ----- parameters -------------------------------------------------------------------

    def _read_parameters(self) -> parameters.Settings:
        """Declare every parameter of SPECS and return the checked settings."""
        values = {}
        refused = []
        for spec in parameters.SPECS:
            descriptor = ParameterDescriptor(description=spec.description, read_only=True)
            if spec.minimum is not None and spec.maximum is not None:
                descriptor.floating_point_range = [FloatingPointRange(
                    from_value=float(spec.minimum), to_value=float(spec.maximum), step=0.0)]
            try:
                if spec.kind == 'string_array':
                    # By type and without a default: an omitted key and an empty list
                    # both read as empty.
                    self.declare_parameter(spec.name, Parameter.Type.STRING_ARRAY, descriptor)
                    empty = Parameter(spec.name, Parameter.Type.STRING_ARRAY, [])
                    value = list(self.get_parameter_or(spec.name, empty).value or [])
                else:
                    value = self.declare_parameter(spec.name, spec.default, descriptor).value
            except InvalidParameterTypeException:
                refused.append(_refused_text(spec, self._given(spec.name), wrong_type=True))
                continue
            except InvalidParameterValueException:
                refused.append(_refused_text(spec, self._given(spec.name), wrong_type=False))
                continue
            values[spec.name] = value
        if refused:
            raise StartProblem(refused)
        values['use_sim_time'] = self.get_parameter('use_sim_time').value
        settings, problems = parameters.settings_from(values)
        if settings is None or problems:
            raise StartProblem(problems or ['the parameters cannot be used'])
        return settings

    def _given(self, name: str) -> object:
        """Return what was given for a parameter that could not be declared."""
        try:
            given = self.declare_parameter(
                name, None, ParameterDescriptor(dynamic_typing=True))
            self.undeclare_parameter(name)
        except Exception:  # the answer only words a message; the node ends either way
            return _NOT_KNOWN
        return given.value

    def _log_start(self) -> None:
        settings = self._settings
        if settings.controllers:
            which = 'controllers ' + ', '.join(settings.controllers)
        else:
            which = 'every controller the daemon serves'
        self.get_logger().info(
            f'bridging {which} from abb-rws-daemon at {self._host_port}; read-only; '
            f'stamps from the clock of the {self._stamp_source}')
        if (settings.allow_remote_daemon
                and settings.daemon_host not in parameters.TRUSTED_DAEMON_HOSTS):
            self.get_logger().warning(
                f'allow_remote_daemon is set and "{settings.daemon_host}" is not this '
                'machine: the access token of the daemon travels unencrypted to that '
                'host, and it also opens the setup, recording and shutdown routes of '
                'the daemon')

    def _forward_client_log(self) -> None:
        """
        Send what the daemon client logs to the logger of the node.

        The handler sits on the client's own logger, so it does not matter whether that
        logger hands its records on to a parent. Which lines show is then decided by
        the level of the node's logger alone.
        """
        client_logger = logging.getLogger(_CLIENT_LOGGER)
        self._forwarder = _LogForwarder(self.get_logger())
        self._forwarder_level = client_logger.level
        client_logger.addHandler(self._forwarder)
        client_logger.setLevel(logging.DEBUG)

    def _add_timer(self, period_s: float, callback, doing: str) -> None:
        def turn() -> None:
            try:
                callback()
            except Exception as error:  # one bad turn must not end the bridge
                self._fault(doing, error)

        self._tick_timers.append(self.create_timer(period_s, turn, clock=self._steady))

    def _fault(self, doing: str, error: Exception) -> None:
        """Count and report a fault of the bridge itself; the bridge goes on."""
        if not self.context.ok():
            # A signal ended the ROS context under the work: the shutdown, not a fault.
            return
        self.faults += 1
        self.get_logger().error(
            f'bridge fault while {doing}: {type(error).__name__}: {_shown(str(error))}',
            throttle_duration_sec=_THROTTLE_S)

    # ----- events of the daemon client --------------------------------------------------

    def _sink(self, event) -> None:
        """Take one event from a thread of the client; the executor handles it."""
        if self._closed:
            return
        self._events.put(event)
        try:
            self._wakeup.trigger()
        except Exception:  # the node is on its way out
            pass

    def _targets(self):
        """Return the controllers the client holds leases for (any thread)."""
        return self._wanted

    def _on_events(self) -> None:
        handled = 0
        while handled < _EVENTS_PER_TURN:
            try:
                event = self._events.get_nowait()
            except queue.Empty:
                break
            handled += 1
            try:
                self._dispatch(event)
            except Exception as error:  # one bad event must not end the bridge
                self._fault(f'handling {type(event).__name__}', error)
        if handled >= _EVENTS_PER_TURN:
            self._wakeup.trigger()
        try:
            self._diagnose(force=False)
        except Exception as error:  # the next period reports again
            self._fault('publishing the diagnostics', error)

    def _dispatch(self, event) -> None:
        if self._closed:
            return
        if isinstance(event, FrameReceived):
            self._on_frame(event)
        elif isinstance(event, Refreshed):
            self._on_refreshed(event)
        elif isinstance(event, LeaseResult):
            self._on_lease(event)
        elif isinstance(event, HealthRead):
            self._take_health(event.health)
            self._health_is_fresh = True
        elif isinstance(event, Connected):
            self._on_connected(event)
        elif isinstance(event, Disconnected):
            self._on_disconnected(event)

    def _take_health(self, answer) -> None:
        if not isinstance(answer, Mapping):
            return
        self._daemon_health = answer
        mode = answer.get('mode')
        self._mode = mode if isinstance(mode, str) else None

    def _on_connected(self, event: Connected) -> None:
        self._take_health(event.health)
        self._health_is_fresh = True
        self._connected = True
        self._connects += 1
        self._connected_mono = self._mono()
        self._reason = ''
        self._detail = ''
        self._retry_mono = None
        self._session_mode = self._mode
        self._contact_mono = None
        self._stall_ended = False
        self._clock_offsets.clear()
        if self._mode == _MODE_LIVE:
            prefix = _LIVE_PREFIX
        elif self._mode == _MODE_REPLAY and self._settings.allow_replay:
            prefix = names.REPLAY_TOKEN
        else:
            prefix = None
        if prefix is not None and any(out.prefix != prefix for out in self._outputs.values()):
            # The publishers of one mode go before those of the other are made.
            for controller_id in sorted(self._outputs):
                self._destroy_outputs(controller_id, 'the daemon changed its mode')
            if not self._settings.controllers:
                self._known = set()
                self._retokenize()
        self._prefix = prefix
        if prefix == names.REPLAY_TOKEN:
            self.get_logger().warning(
                f'connected to abb-rws-daemon at {self._host_port}, which REPLAYS A '
                f'RECORDING: published under {names.REPLAY_TOKEN}/ only, never on the '
                'live topic names')
        elif prefix is None:
            self.get_logger().error(
                f'connected to abb-rws-daemon at {self._host_port}, but nothing is '
                'published in its mode')
        else:
            self.get_logger().info(f'connected to abb-rws-daemon at {self._host_port}')

    def _on_disconnected(self, event: Disconnected) -> None:
        news = self._connected or event.reason != self._reason
        self._connected = False
        self._connected_mono = None
        self._reason = _shown(event.reason)
        self._detail = event.detail if isinstance(event.detail, str) else ''
        self._retry_mono = self._mono() + max(0.0, float(event.retry_in_s))
        if not self._health_is_fresh:
            # No health answer on this attempt: what was read before says nothing now.
            self._daemon_health = None
            self._mode = None
        self._health_is_fresh = False
        self._session_mode = None
        self._fleet.clear()
        self._contact_mono = None
        self._stall_ended = False
        self._clock_offsets.clear()
        self._leases.clear()
        self._io_accepted.clear()
        self._io_rejected.clear()
        self._conversion_problems.clear()
        self._silence()
        if event.reason == 'replay-restart':
            # The recording starts over, and with it its times.
            for gate in self._gates.values():
                gate.reset()
        if news:
            status = health.daemon_status(self._daemon_view(self._mono()))
            self.get_logger().warning(
                f'not connected to abb-rws-daemon at {self._host_port}; every data topic '
                f'is silent. {status.message}')
        # Said at once, before the events of the next connection in the same turn can
        # hide it: a reconnect after a short wait is quick, the executor may not be.
        self._diagnose(force=False)

    def _on_frame(self, event: FrameReceived) -> None:
        if not self._connected:
            return
        applied = self._fleet.apply(event.frame, event.at.wall_ns, event.at.mono)
        self._frames += 1
        self._contact_mono = event.at.mono
        self._note_clock_offset(event.frame, event.at.wall_ns)
        if applied.problem:
            self.get_logger().warning(
                f'a stream frame was not taken as it is: {_shown(applied.problem, 300)}',
                throttle_duration_sec=_THROTTLE_S)
        new = self._follow_membership(applied)
        if not self._live(self._mono()):
            return
        for controller_id in sorted(set(applied.changed) | new):
            out = self._outputs.get(controller_id)
            if out is None:
                continue
            self._publish_snapshot(controller_id, out)
            if controller_id in applied.motion_moved or controller_id in new:
                self._publish_joints(controller_id, out)
            self._publish_signals(controller_id, out, keep_alive=False)

    def _on_refreshed(self, event: Refreshed) -> None:
        if not self._connected:
            return
        applied = self._fleet.confirm(event.answer, event.at.wall_ns, event.at.mono)
        self._contact_mono = event.at.mono
        self._note_clock_offset(event.answer, event.at.wall_ns)
        if applied.problem:
            self.get_logger().warning(
                f'a controller list was not taken: {_shown(applied.problem, 300)}',
                throttle_duration_sec=_THROTTLE_S)
        if not self._live(self._mono()):
            return
        # The snapshot itself goes out with the next keep-alive, with its new stamp.
        for controller_id in sorted(self._outputs):
            out = self._outputs[controller_id]
            if controller_id in applied.motion_moved:
                self._publish_joints(controller_id, out)
            self._publish_signals(controller_id, out, keep_alive=False)

    def _on_lease(self, event: LeaseResult) -> None:
        if not self._connected:
            return
        controller_id = event.controller_id
        ok = event.ok is True
        self._leases.setdefault(controller_id, {})[event.kind] = (ok, _shown(event.code))
        if event.kind != 'io' or not ok or not isinstance(event.answer, Mapping):
            return
        accepted = event.answer.get('accepted')
        rejected = event.answer.get('rejected')
        self._io_accepted[controller_id] = tuple(
            name for name in accepted if isinstance(name, str)
        ) if isinstance(accepted, list) else ()
        self._io_rejected[controller_id] = tuple(
            (_shown(entry.get('name')), _shown(entry.get('reason'), 160))
            for entry in rejected if isinstance(entry, Mapping)
        ) if isinstance(rejected, list) else ()
        out = self._outputs.get(controller_id)
        if out is not None and self._live(self._mono()):
            self._publish_signals(controller_id, out, keep_alive=False)

    def _note_clock_offset(self, carrier, wall_ns: int) -> None:
        """Keep the ``observedAt`` of a frame or a list against this clock's arrival time."""
        try:
            sec, nanosec = timeconv.iso_to_sec_nanosec(carrier.get('observedAt'))
        except (ValueError, AttributeError):
            return
        daemon_ns = sec * timeconv.NANOSEC_PER_SEC + nanosec
        self._clock_offsets.append((daemon_ns - wall_ns) / 1e6)

    def _clock_offset_ms(self):
        """
        Return the daemon's clock minus this clock in milliseconds; None when not known.

        One sample is the daemon's time of a frame or a list minus its arrival here, so
        the time it took to arrive makes a sample smaller and never larger. The largest
        of the last few samples is therefore the best one.
        """
        return max(self._clock_offsets) if self._clock_offsets else None

    # ----- which controllers ------------------------------------------------------------

    def _selected(self, controller_id: str) -> bool:
        return not self._settings.controllers or controller_id in self._settings.controllers

    def _follow_membership(self, applied) -> set:
        """
        Follow which controllers the daemon serves: names, publishers and leases.

        Returns the controllers whose publishers were made just now.
        """
        served = set(self._fleet.ids())
        # After a refused snapshot nothing is known: no controller came or went.
        refused = applied.kind == 'snapshot' and not self._fleet.has_snapshot
        if not self._settings.controllers and not refused:
            if applied.kind == 'snapshot':
                known = served
            else:
                known = (self._known - set(applied.removed)) | (set(applied.changed) & served)
            if known != self._known:
                self._known = known
                self._retokenize()
        for controller_id in sorted(self._outputs):
            if controller_id not in served and not refused:
                self._destroy_outputs(controller_id, 'the daemon no longer serves it')
        for controller_id in applied.removed:
            self._leases.pop(controller_id, None)
            self._io_accepted.pop(controller_id, None)
            self._io_rejected.pop(controller_id, None)
            self._conversion_problems.pop(controller_id, None)
        # Publishers for every controller that is served, selected and has a name: on
        # first sight, and when a name it could not have before became free.
        new = {controller_id for controller_id in sorted(served)
               if controller_id not in self._outputs
               and self._outputs_for(controller_id) is not None}
        self._wanted = tuple(
            (controller_id, self._settings.watch_motion,
             tuple(self._settings.io_signals.get(controller_id, ())))
            for controller_id in sorted(served) if controller_id in self._outputs)
        return new

    def _retokenize(self) -> None:
        """Name the controllers again (only when every controller is bridged)."""
        usable = {controller_id for controller_id in self._known
                  if names.is_controller_id(controller_id)}
        tokens, problems = names.controller_tokens(usable, self._settings.primary_controller)
        for controller_id in self._known - usable:
            problems[controller_id] = ('the id the daemon gives it cannot be used in a '
                                       'topic name')
        for controller_id in sorted(self._outputs):
            if tokens.get(controller_id) != self._outputs[controller_id].token:
                self._destroy_outputs(controller_id, 'its topic names are not free any more')
        for controller_id in sorted(problems):
            if self._name_problems.get(controller_id) != problems[controller_id]:
                self.get_logger().error(f'controller "{_shown(controller_id)}" is not '
                                        f'bridged: {problems[controller_id]}')
        self._tokens = tokens
        self._name_problems = problems

    def _outputs_for(self, controller_id: str):
        """Return the publishers of a controller, made on first sight; None: not bridged."""
        out = self._outputs.get(controller_id)
        if out is not None:
            return out
        token = self._tokens.get(controller_id)
        if self._prefix is None or token is None or not self._selected(controller_id):
            return None
        out = _Outputs(token, self._prefix)
        try:
            out.snapshot = self._data_publisher(ControllerSnapshot, out.snapshot_topic)
            out.joints = self._data_publisher(JointState, out.joints_topic)
        except Exception as error:  # in practice a qos_overrides parameter that is wrong
            self._destroy_publishers(out)
            del self._tokens[controller_id]
            self._name_problems[controller_id] = (
                'its publishers could not be made; check the qos_overrides parameters '
                f'of its topics ({type(error).__name__})')
            self.get_logger().error(f'controller "{_shown(controller_id)}" is not bridged: '
                                    f'{self._name_problems[controller_id]}')
            return None
        self._outputs[controller_id] = out
        self.get_logger().info(
            f'controller "{_shown(controller_id)}" is published on {out.joints_topic} and '
            f'{out.snapshot_topic}')
        return out

    def _data_publisher(self, message_type, topic: str):
        return self.create_publisher(
            message_type, topic, _qos_profile(),
            qos_overriding_options=QoSOverridingOptions.with_default_policies())

    def _destroy_outputs(self, controller_id: str, why: str) -> None:
        out = self._outputs.pop(controller_id, None)
        if out is None:
            return
        self._destroy_publishers(out)
        self.get_logger().info(
            f'controller "{_shown(controller_id)}" is not published any more: {why}')

    def _destroy_publishers(self, out: _Outputs) -> None:
        publishers = [out.snapshot, out.joints]
        publishers += [signal.publisher for signal in out.signals.values()]
        for publisher in publishers:
            if publisher is not None:
                self.destroy_publisher(publisher)
        out.snapshot = None
        out.joints = None
        out.signals = {}

    # ----- current or not ---------------------------------------------------------------

    def _live(self, now: float) -> bool:
        """
        Return whether the daemon is live: the one rule for every data topic.

        Live: the stream is connected and has delivered its snapshot, the daemon's mode
        is the one this connection started with and one that is published, the last
        frame or controller list arrived less than stale_after_s ago, and the stream has
        not stalled (see _stream_behind_s).
        """
        return (self._connected
                and self._prefix is not None
                and self._mode == self._session_mode
                and self._fleet.has_snapshot
                and self._contact_mono is not None
                and now - self._contact_mono < self._settings.stale_after_s
                and not health.stream_stalled(self._stream_behind_s(now),
                                              self._settings.stale_after_s))

    def _stream_behind_s(self, now: float):
        """
        Return for how long the stream has owed a frame, in seconds; None: it owes none.

        A controller list answers over a connection of its own. So a stream that has
        stopped delivering (a socket that is open in name only) is seen here and
        nowhere else: the list keeps showing a value no frame brought. After
        stale_after_s of that, what the stream last delivered is not current any more.

        Not judged for a daemon that replays a recording: it moves the times inside
        every answer by the moment the answer is made, so there a list never equals a
        frame. A replay is watched by the silence rule of the client alone.
        """
        if not self._connected or self._session_mode != _MODE_LIVE:
            return None
        since = self._fleet.behind_since()
        return None if since is None else max(0.0, now - since)

    def _end_stalled_stream(self, behind_s: float) -> None:
        """End the connection whose stream stalled, once; the client then connects again."""
        self._stall_ended = True
        owing = '; '.join(f'"{_shown(controller_id, 40)}": {_shown(what, 60)}'
                          for controller_id, what in self._fleet.behind()[:3])
        self.get_logger().warning(
            'the stream of abb-rws-daemon has not delivered what its controller list has '
            f'shown for {behind_s:.1f} s ({owing}): every data topic is silent, and the '
            'connection is made again')
        self._daemon.end_stalled_stream(
            f'no frame for what the controller list has shown for {behind_s:.1f} s ({owing})')

    def _silence(self) -> None:
        """Forget what the I/O topics last said: the next current value goes out."""
        for out in self._outputs.values():
            for signal in out.signals.values():
                signal.last = None

    def _motion_age(self, controller_id: str, now: float):
        seen = self._fleet.seen(controller_id)
        if seen is None or seen.motion_mono is None:
            return None
        return max(0.0, now - seen.motion_mono)

    def _gate(self, topic: str) -> stamps.StampGate:
        """Return the stamp gate of a topic; it outlives the publisher."""
        gate = self._gates.get(topic)
        if gate is None:
            gate = self._gates[topic] = stamps.StampGate()
        return gate

    def _admit(self, topic: str, stamp, *, allow_equal: bool) -> bool:
        gate = self._gate(topic)
        before = gate.held_back
        if gate.admit(stamp, allow_equal=allow_equal):
            return True
        if gate.held_back != before:
            self._held_back_mono = self._mono()
            self.get_logger().warning(
                f'a message for {topic} was held back: its stamp is behind the last '
                'published one',
                throttle_duration_sec=_THROTTLE_S)
        return False

    # ----- publishing -------------------------------------------------------------------

    def _publish_snapshot(self, controller_id: str, out: _Outputs) -> None:
        controller = self._fleet.get(controller_id)
        seen = self._fleet.seen(controller_id)
        if controller is None or seen is None:
            return
        try:
            stamp = stamps.stamp_of(self._stamp_source, seen.observed_at, seen.wall_ns)
            message = self._converter.to_msg(_SNAPSHOT_MESSAGE, controller, stamp)
        except ConversionError as error:
            self._conversion_problems[controller_id] = _shown(str(error), 200)
            return
        except ValueError as error:
            self._conversion_problems[controller_id] = f'no stamp: {_shown(str(error), 200)}'
            return
        self._conversion_problems.pop(controller_id, None)
        if self._admit(out.snapshot_topic, stamp, allow_equal=True):
            out.snapshot.publish(message)

    def _publish_joints(self, controller_id: str, out: _Outputs) -> None:
        controller = self._fleet.get(controller_id)
        seen = self._fleet.seen(controller_id)
        if controller is None or seen is None:
            return
        current, _ = health.joints_current(
            controller, self._motion_age(controller_id, self._mono()),
            self._settings.stale_after_s)
        if not current:
            return
        data = joint_state.build(controller_id, _joint_token(controller_id),
                                 controller.get('motion'), self._namer)
        if data is None or data.problem is not None:
            return
        try:
            stamp = stamps.stamp_of(self._stamp_source, data.stamp, seen.motion_wall_ns)
        except ValueError:
            return
        if not self._admit(out.joints_topic, stamp, allow_equal=False):
            return
        message = JointState()
        message.header.stamp.sec, message.header.stamp.nanosec = stamp
        message.header.frame_id = self._settings.joint_states_frame_id
        message.name = list(data.names)
        message.position = list(data.positions)
        # Velocity and effort are not measured: they stay empty, never zeros.
        out.joints.publish(message)

    def _publish_signals(self, controller_id: str, out: _Outputs, *,
                         keep_alive: bool) -> None:
        configured = self._settings.io_signals.get(controller_id, ())
        controller = self._fleet.get(controller_id)
        if not configured or controller is None:
            return
        entries = _signal_entries(controller)
        granted = self._io_accepted.get(controller_id, ())
        for name in configured:
            signal = out.signals.get(name)
            if signal is None:
                topic = _topic(out.prefix, out.token, IO, names.io_topic_token(name))
                signal = out.signals[name] = _Signal(topic)
            entry = entries.get(name)
            if name in granted and signal.publisher is None:
                self._make_signal_publisher(signal, entry)
            reading, _ = self._reading(controller, signal.topic, name, entry, name in granted)
            if reading is None or signal.publisher is None:
                signal.last = None
                continue
            if keep_alive or reading != signal.last:
                message = _IO_MESSAGES[reading[0]]()
                message.data = reading[1]
                signal.publisher.publish(message)
                signal.last = reading

    def _make_signal_publisher(self, signal: _Signal, entry) -> None:
        """Make the publisher of a signal once its type is known; the type then stays."""
        kind = self._io_kinds.get(signal.topic)
        if kind is None:
            kind = io_values.kind_of(entry)
            if kind is None:
                return
            self._io_kinds[signal.topic] = kind
        try:
            signal.publisher = self._data_publisher(_IO_MESSAGES[kind], signal.topic)
        except Exception as error:  # in practice a qos_overrides parameter that is wrong
            self.get_logger().error(
                f'no publisher for {signal.topic}; check the qos_overrides parameters of '
                f'that topic ({type(error).__name__})',
                throttle_duration_sec=_THROTTLE_S)

    def _reading(self, controller, topic: str, name: str, entry, granted: bool):
        """
        Return ((kind, value), None) of a signal that may be published, or (None, why).

        A topic is never retyped: once a signal was seen as one kind, a report of
        another kind stops it, and the reason starts with health.TYPE_CHANGED.
        """
        current, why = health.signal_current(
            controller, entry if entry is not None else name, granted)
        if not current:
            return None, why
        first = self._io_kinds.get(topic)
        kind = io_values.kind_of(entry)
        if first is not None and kind is not None and kind != first:
            return None, (f'{health.TYPE_CHANGED}: first reported as {first}, now as {kind}; '
                          'the topic keeps its type, restart the bridge to publish the '
                          'new one')
        reading = io_values.typed(entry)
        if reading is None:
            return None, io_values.why_not(entry) or 'no value'
        return reading, None

    # ----- timers -----------------------------------------------------------------------

    def _on_keep_alive(self) -> None:
        if self._closed or not self._live(self._mono()):
            return
        for controller_id in sorted(self._outputs):
            out = self._outputs[controller_id]
            self._publish_snapshot(controller_id, out)
            self._publish_signals(controller_id, out, keep_alive=True)

    def _on_watch_tick(self) -> None:
        if self._closed:
            return
        now = self._mono()
        live = self._live(now)
        behind_s = self._stream_behind_s(now)
        stalled = health.stream_stalled(behind_s, self._settings.stale_after_s)
        if stalled and not self._stall_ended:
            self._end_stalled_stream(behind_s)
        if self._was_live and not live:
            self._silence()
            if self._connected and not stalled:
                self.get_logger().warning(
                    'no answer from abb-rws-daemon for '
                    f'{self._settings.stale_after_s:g} s: every data topic is silent')
        elif live and not self._was_live:
            self.get_logger().info('abb-rws-daemon is live: publishing')
        self._was_live = live
        self._diagnose(force=False)

    def _on_diagnostics_timer(self) -> None:
        if not self._closed:
            self._diagnose(force=True)

    # ----- diagnostics ------------------------------------------------------------------

    def _diagnose(self, *, force: bool) -> None:
        """Publish /diagnostics: every period, and at once when a level changes."""
        if self._closed:
            return
        statuses = self._statuses(self._mono())
        levels = tuple((status.name, status.level) for status in statuses)
        if not force and levels == self._last_levels:
            return
        self._last_levels = levels
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        for status in statuses:
            message = DiagnosticStatus()
            message.level = _LEVELS.get(status.level, DiagnosticStatus.ERROR)
            message.name = status.name
            message.message = status.message
            message.hardware_id = status.hardware_id
            message.values = [KeyValue(key=key, value=value) for key, value in status.values]
            array.status.append(message)
        self._diagnostics.publish(array)

    def _statuses(self, now: float) -> list:
        """Return the daemon status and one status per selected controller."""
        live = self._live(now)
        statuses = [health.daemon_status(self._daemon_view(now))]
        replayed = self._mode == _MODE_REPLAY and self._settings.allow_replay
        replay = self._replay_part()
        if self._settings.controllers:
            listed = sorted(self._settings.controllers)
        else:
            # Every controller the daemon serves, and every one a parameter names: a
            # name the daemon does not serve gets a status that says so, instead of a
            # topic that silently never appears.
            listed = sorted(self._known | set(self._fleet.incomplete()) | self._named)
        for controller_id in listed:
            status = health.controller_status(self._controller_view(controller_id, live, now))
            if replayed:
                status = health.replay_marked(status, replay)
            statuses.append(status)
        return statuses

    def _replay_part(self):
        answer = self._daemon_health
        replay = answer.get('replay') if isinstance(answer, Mapping) else None
        return replay if isinstance(replay, Mapping) else None

    def _daemon_view(self, now: float) -> health.DaemonView:
        answer = self._daemon_health if isinstance(self._daemon_health, Mapping) else {}
        version = answer.get('version')
        held_back = sum(gate.held_back for gate in self._gates.values())
        recent = max(_HELD_BACK_RECENT_S, 2.0 * self._settings.diagnostics_period_s)
        return health.DaemonView(
            node_name=self.get_name(),
            host_port=self._host_port,
            connected=self._connected,
            reason=self._reason,
            detail=self._detail,
            retry_in_s=(None if self._retry_mono is None
                        else max(0.0, self._retry_mono - now)),
            mode=self._mode,
            allow_replay=self._settings.allow_replay,
            replay=self._replay_part(),
            daemon_version=version if isinstance(version, str) else None,
            daemon_schema=self._fleet.schema_version,
            messages_schema=self._converter.schema_version,
            definitions_sha256=self._converter.definitions_sha256,
            connected_for_s=(None if self._connected_mono is None
                             else max(0.0, now - self._connected_mono)),
            reconnects=max(0, self._connects - 1),
            frames=self._frames,
            contact_age_s=(None if self._contact_mono is None
                           else max(0.0, now - self._contact_mono)),
            stale_after_s=self._settings.stale_after_s,
            clock_offset_ms=self._clock_offset_ms(),
            stamp_source=self._stamp_source,
            stamps_held_back=held_back,
            stamps_held_back_recently=(self._held_back_mono is not None
                                       and now - self._held_back_mono < recent),
            stream_behind_s=self._stream_behind_s(now))

    def _controller_view(self, controller_id: str, live: bool,
                         now: float) -> health.ControllerView:
        controller = self._fleet.get(controller_id)
        token = self._tokens.get(controller_id)
        conversion_problem = self._conversion_problems.get(controller_id)
        if controller is None and controller_id in self._fleet.incomplete():
            conversion_problem = 'the daemon reports it without all of its groups'
        joints = None
        if controller is not None and token is not None:
            joints = joint_state.build(controller_id, _joint_token(controller_id),
                                       controller.get('motion'), self._namer)
        return health.ControllerView(
            node_name=self.get_name(),
            controller_id=_shown(controller_id),
            controller=controller,
            daemon_live=live,
            name_problem=self._name_problems.get(controller_id),
            conversion_problem=conversion_problem,
            joints=joints,
            motion_age_s=self._motion_age(controller_id, now),
            stale_after_s=self._settings.stale_after_s,
            leases=dict(self._leases.get(controller_id, {})),
            io_accepted=self._io_accepted.get(controller_id, ()),
            io_rejected=self._io_rejected.get(controller_id, ()),
            io_unpublishable=self._unpublishable(controller_id, controller, token, live),
            watch_motion=self._settings.watch_motion)

    def _unpublishable(self, controller_id: str, controller, token, live: bool) -> tuple:
        """Return (name, why) for every accepted signal whose topic is silent."""
        if not live or controller is None or token is None or self._prefix is None:
            return ()
        if not health.controller_current(controller)[0]:
            return ()
        entries = _signal_entries(controller)
        silent = []
        for name in self._io_accepted.get(controller_id, ()):
            if name not in self._settings.io_signals.get(controller_id, ()):
                continue
            topic = _topic(self._prefix, token, IO, names.io_topic_token(name))
            reading, why = self._reading(controller, topic, name, entries.get(name), True)
            if reading is None:
                silent.append((_shown(name), _shown(why, 200)))
        return tuple(silent)


def _installed_converter() -> Converter:
    """Return the converter of the installed messages; StartProblem when unusable."""
    try:
        return Converter.from_installed()
    except (LookupError, OSError, ValueError, ImportError) as error:
        raise StartProblem([
            'the message package abb_rws_ros2_msgs cannot be used '
            f'({type(error).__name__}: {_shown(str(error), 200)})']) from None


def _qos_profile() -> QoSProfile:
    """Return a new profile: reliable, volatile, keep last 10 (nothing is latched)."""
    return QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=QOS_DEPTH,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.VOLATILE)


def _topic(*tokens: str) -> str:
    """Join the tokens of a relative topic name; empty ones are left out."""
    return '/'.join(token for token in tokens if token)


def _joint_token(controller_id: str) -> str:
    """
    Return what ``{controller}`` stands for in a joint name: the id as a name token.

    It is made from the id alone (``cell-1`` gives ``cell_1``), so the name of a joint
    does not change when its controller becomes the primary one or stops being it.
    """
    try:
        return names.ros_token(controller_id)
    except ValueError:
        return ''


def _signal_entries(controller) -> dict:
    """Return the watched signals of a controller by name."""
    io = controller.get('io') if isinstance(controller, Mapping) else None
    signals = io.get('signals') if isinstance(io, Mapping) else None
    if not isinstance(signals, (list, tuple)):
        return {}
    return {entry['name']: entry for entry in signals
            if isinstance(entry, Mapping) and isinstance(entry.get('name'), str)}


def _shown(text, limit: int = _SHOWN) -> str:
    """Return text for a log line or a status: printable ASCII and not too long."""
    if not isinstance(text, str):
        return ''
    return ''.join(character if ' ' <= character <= '~' else '?'
                   for character in text[:limit])


def _refused_text(spec: parameters.Spec, given, *, wrong_type: bool) -> str:
    """Return the line for a parameter that could not be declared."""
    if spec.kind == 'double':
        accepted = (f'a number from {spec.minimum:g} to {spec.maximum:g}, written with a '
                    'decimal point')
    else:
        accepted = _ACCEPTED.get(spec.kind, 'another value')
    hint = ''
    whole_number = isinstance(given, int) and not isinstance(given, bool)
    if wrong_type and spec.kind == 'double' and whole_number:
        hint = f' (write {given}.0, not {given})'
    elif wrong_type and spec.kind == 'string' and given is not _NOT_KNOWN:
        # A parameter file or a launch argument reads y, 08 or 1e3 as something else.
        hint = ' (put it in double quotes)'
    elif wrong_type and spec.kind == 'string_array' and isinstance(given, str):
        hint = f' (write ["{_shown(given, 40)}"])'
    elif wrong_type and spec.kind == 'string_array' and given is not _NOT_KNOWN:
        hint = ' (put every entry in double quotes)'
    return f'{spec.name}: {_given_text(given)} was given, but it must be {accepted}{hint}'


def _given_text(given) -> str:
    """Return what was given for a parameter, in plain words."""
    if given is _NOT_KNOWN or given is None:
        return 'a value that cannot be used'
    if isinstance(given, bool):
        return 'true' if given else 'false'
    if isinstance(given, int):
        return f'{given} (a whole number)'
    if isinstance(given, float):
        return f'{given!r}'
    if isinstance(given, str):
        return f'"{_shown(given, 40)}" (a single text)'
    try:
        entries = list(given)
    except TypeError:
        return 'a value that cannot be used'
    if entries and all(isinstance(entry, bool) for entry in entries):
        return 'a list of true or false values'
    if entries and all(isinstance(entry, (int, float)) for entry in entries):
        return 'a list of numbers'
    return 'a list'


def main(args=None) -> int:
    """
    Run the bridge until it is interrupted; return the exit code of the process.

    0 after SIGINT or SIGTERM. 2 when the node cannot start with its parameters: each
    problem is then one line of the log.
    """
    rclpy.init(args=args)
    node = None
    try:
        try:
            node = BridgeNode()
        except StartProblem as problem:
            logger = rclpy.logging.get_logger(problem.logger_name)
            for line in problem.problems:
                logger.fatal(f'cannot start: {line}')
            return EXIT_START_PROBLEM
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        except Exception:
            # A signal ends the ROS context under the executor, and whatever the
            # executor was doing at that moment fails. That is the shutdown, not a
            # fault; with the context still valid it is one.
            if rclpy.ok():
                raise
        return EXIT_OK
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())

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
The bridge node on a ROS graph, in the test process, against the daemon stand-in.

Every test starts its own stand-in daemon, its own bridge node (under a name and in a
namespace no other test uses) and a collector node, in the ROS domain the session took.
All daemon data is synthetic (``frames``). The scenario numbers in the test names group
the tests by what they show; most of scenario 15 (how the process starts and ends) is in
``test_node_process.py``.
"""

import ast
import inspect
import logging
import math
from pathlib import Path
import threading
import time

from abb_rws_bridge import bridge_node
from abb_rws_bridge import parameters
from abb_rws_bridge import timeconv
from abb_rws_bridge.bridge_node import BridgeNode, StartProblem
from abb_rws_bridge.state import COALESCE_WINDOW_S
from abb_rws_ros2_msgs.msg import ControllerSnapshot
from diagnostic_msgs.msg import DiagnosticStatus
import fake_daemon
from fake_daemon import wait_until
import frames
import pytest
from rcl_interfaces.msg import Log
import rclpy
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, UInt32

pytest_plugins = ('ros_helpers',)

OK = DiagnosticStatus.OK
WARN = DiagnosticStatus.WARN
ERROR = DiagnosticStatus.ERROR
STALE = DiagnosticStatus.STALE

# Short times, so that a test does not wait: every one is inside its parameter's range.
FAST = {
    'refresh_hz': 10.0,
    'stale_after_s': 0.5,
    'republish_hz': 10.0,
    'diagnostics_period_s': 0.2,
    'reconnect_min_s': 0.1,
    'reconnect_max_s': 1.0,
    'http_timeout_s': 1.0,
}
# What a loaded machine may add to a time the node keeps.
SLACK_S = 1.0
ROBOT_JOINTS = ['ROB_1_J1', 'ROB_1_J2', 'ROB_1_J3', 'ROB_1_J4', 'ROB_1_J5', 'ROB_1_J6']
SECRETS = (fake_daemon.TOKEN, fake_daemon.OTHER_TOKEN,
           fake_daemon.TOKEN[:16], fake_daemon.OTHER_TOKEN[:16])
ROSOUT = '/rosout'
DIAGNOSTICS_OVERRIDE = 'qos_overrides./diagnostics'


def values_of(status):
    """Return the values of a status as a dict."""
    return {entry.key: entry.value for entry in status.values}


def stamp_ns(stamp):
    """Return a ``builtin_interfaces/Time`` as nanoseconds."""
    return stamp.sec * timeconv.NANOSEC_PER_SEC + stamp.nanosec


def iso_ns(text):
    """Return a daemon time as nanoseconds."""
    sec, nanosec = timeconv.iso_to_sec_nanosec(text)
    return sec * timeconv.NANOSEC_PER_SEC + nanosec


class Times:
    """Daemon times only a test knows: T0 plus a base, one step later on every call."""

    def __init__(self, base_s, step_s=0.001):
        self._base_s = base_s
        self._step_s = step_s
        self._lock = threading.Lock()
        self.handed = []

    def __call__(self):
        with self._lock:
            text = frames.iso(self._base_s + len(self.handed) * self._step_s)
            self.handed.append(text)
            return text

    def handed_ns(self):
        """Return every time handed out so far, as nanoseconds."""
        with self._lock:
            return {iso_ns(text) for text in self.handed}


class Advance:
    """
    A robot that stands still while the daemon keeps reading it.

    Before every controller list the stand-in answers, ``motion.asOf`` of every
    controller moves on and nothing else changes; no stream frame is sent.
    """

    def __init__(self, fake, as_of=None):
        self.running = True
        self._as_of = as_of
        fake.on_controller_list = self

    def __call__(self, fake):
        if not self.running:
            return
        for snapshot in fake.controllers:
            motion = snapshot['motion']
            if motion['asOf'] is not None:
                motion['asOf'] = fake.now() if self._as_of is None else self._as_of()
                fake.update(snapshot['id'], motion=motion)

    def stop(self):
        """Stop moving the stamps: the daemon's sampler reads nothing any more."""
        self.running = False


class Bench:
    """One stand-in daemon, one bridge node and the collector of a test."""

    def __init__(self, context, collector, namespace, folder):
        self.context = context
        self.collector = collector
        self.namespace = namespace
        self.node_name = 'bridge_' + namespace[3:]
        self.token_file = folder / 'token'
        self.token_file.write_text(fake_daemon.TOKEN + '\n', encoding='ascii')
        self.fake = None
        self.node = None
        self._spinner = None
        collector.watch_diagnostics()

    def start_fake(self, controllers=None, **options):
        """Start the stand-in daemon with these controllers."""
        self.fake = fake_daemon.FakeDaemon(controllers, **options).start()
        return self.fake

    def overrides(self, **values):
        """Return the parameter overrides of a node of this bench."""
        merged = dict(FAST)
        if self.fake is not None:
            merged['daemon_url'] = self.fake.url
        merged['token_file'] = str(self.token_file)
        merged.update(values)
        return [Parameter(name, value=value) for name, value in merged.items()]

    def make_node(self, **values):
        """Make a bridge node of this bench without spinning it."""
        return BridgeNode(
            node_name=self.node_name, context=self.context, namespace=self.namespace,
            use_global_arguments=False, parameter_overrides=self.overrides(**values))

    def start_node(self, **values):
        """Start the bridge node; values are parameters that replace the defaults."""
        self.node = self.make_node(**values)
        self._spinner = self.collector.spin(self.node)
        return self.node

    def stop_node(self):
        """Stop and destroy the bridge node."""
        node, self.node = self.node, None
        if node is None:
            return
        self._spinner.stop()
        node.destroy_node()
        assert node.faults == 0, 'the node caught a fault while it handled an event'

    def close(self):
        """Stop the node, then the stand-in."""
        try:
            self.stop_node()
        finally:
            if self.fake is not None:
                self.fake.stop()

    def publisher(self, topic):
        """Return the node's publisher of a topic, or None."""
        name = self.collector.resolve(topic)
        for publisher in list(self.node.publishers):
            if publisher.topic_name == name:
                return publisher
        return None

    def wait_matched(self, topic):
        """Wait until the node's publisher of a topic and the collector know each other."""
        def matched():
            publisher = self.publisher(topic)
            return publisher is not None and publisher.get_subscription_count() >= 1

        assert wait_until(matched, 10.0), f'no subscription matched on {topic}'
        self.collector.wait_for_publishers(topic, 1)

    def daemon_status(self, where=None, **options):
        """Return the first daemon status that fits where."""
        return self.collector.wait_status(f'{self.node_name}: daemon', where, **options)

    def daemon_now(self):
        """Return the next daemon status the node publishes from now on."""
        return self.daemon_status(after=self.collector.now())

    def controller_status(self, controller_id, where=None, **options):
        """Return the first status of a controller that fits where."""
        return self.collector.wait_status(
            f'{self.node_name}: controller {controller_id}', where, **options)

    def data_topics(self):
        """Return the topics of the test's namespace that have a publisher now."""
        prefix = self.namespace + '/'
        return sorted(name for name in self.collector.topic_types()
                      if name.startswith(prefix) and self.collector.publisher_count(name))


@pytest.fixture
def bench(ros_context, collector, ros_namespace, tmp_path):
    made = Bench(ros_context, collector, ros_namespace, tmp_path)
    try:
        yield made
    finally:
        made.close()


class Collected(logging.Handler):

    def __init__(self):
        super().__init__(logging.DEBUG)
        self.records = []

    def emit(self, record):
        self.records.append(record)


@pytest.fixture
def debug_logs():
    """
    Collect what the loggers of this package and of the WebSocket library say at DEBUG.

    The handler sits on each logger itself, so it does not matter whether a logger hands
    its records on to the root logger (under the ROS test plugins it does not).
    """
    handler = Collected()
    loggers = [logging.root]
    loggers += [logger for logger in logging.root.manager.loggerDict.values()
                if isinstance(logger, logging.Logger)]
    for name in ('websocket', 'abb_rws_bridge', 'abb_rws_bridge.daemon_client'):
        if logging.getLogger(name) not in loggers:
            loggers.append(logging.getLogger(name))
    levels = [logger.level for logger in loggers]
    for logger in loggers:
        logger.addHandler(handler)
        logger.setLevel(logging.DEBUG)
    try:
        yield handler
    finally:
        for logger, level in zip(loggers, levels):
            logger.removeHandler(handler)
            logger.setLevel(level)


def moved_robot(controller_id, degrees):
    """Return a controller whose six robot joints stand at these angles."""
    joints = [frames.joint(f'J{axis}', value) for axis, value in enumerate(degrees, 1)]
    return frames.controller(
        controller_id, motion=frames.motion_group([frames.motion_unit(joints=joints)]))


def io_controller(controller_id='cell-1', signals=None):
    """Return a controller whose I/O group is watched with these signals."""
    if signals is None:
        signals = [frames.signal('do_gripper', 'DO', '1')]
    return frames.controller(controller_id, io=frames.io_group(signals))


# ----- scenario 1: the snapshot --------------------------------------------------------


@pytest.mark.parametrize('stamp_source', ['daemon', 'bridge'])
def test_scenario_1_snapshot_and_joint_states_per_controller(bench, stamp_source):
    degrees = {'cell-1': [10.0, -20.0, 30.0, 45.0, 90.0, -180.0],
               'cell-2': [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]}
    fake = bench.start_fake([moved_robot(name, degrees[name]) for name in degrees])
    # Daemon times nobody else has: early in 2026, far from the clock of this machine.
    observed = Times(2000.0)
    as_of = Times(1000.0, 0.01)
    fake.now = observed
    Advance(fake, as_of)
    collector = bench.collector
    for token in ('cell_1', 'cell_2'):
        collector.subscribe(f'{token}/joint_states', JointState)
        collector.subscribe(f'{token}/controller_snapshot', ControllerSnapshot)
    wall_before = time.time_ns()
    bench.start_node(stamp_source=stamp_source)

    for controller_id, token in (('cell-1', 'cell_1'), ('cell-2', 'cell_2')):
        states = collector.wait_for(f'{token}/joint_states', 3)
        snapshots = collector.wait_for(f'{token}/controller_snapshot', 3)
        wall_after = time.time_ns()
        for state in states:
            assert list(state.name) == ROBOT_JOINTS
            assert len(state.position) == 6
            for position, value in zip(state.position, degrees[controller_id]):
                assert abs(position - value * math.pi / 180.0) < 1e-12
            assert list(state.velocity) == [] and list(state.effort) == []
            assert state.header.frame_id == ''
        for snapshot in snapshots:
            assert snapshot.id == controller_id and snapshot.name == controller_id
            assert stamp_ns(snapshot.header.stamp) > 0
            assert snapshot.header.frame_id == ''
            # The snapshot stays in the daemon's units and times, whatever stamps it.
            joints = snapshot.motion.units[0].joints
            assert [joint.value for joint in joints] == degrees[controller_id]
            assert {joint.unit for joint in joints} == {'deg'}
            assert snapshot.motion.as_of_known
            assert stamp_ns(snapshot.motion.as_of) in as_of.handed_ns() | {iso_ns(frames.T0)}
        state_stamps = [stamp_ns(state.header.stamp) for state in states]
        snapshot_stamps = [stamp_ns(snapshot.header.stamp) for snapshot in snapshots]
        assert state_stamps == sorted(set(state_stamps))
        assert snapshot_stamps == sorted(snapshot_stamps)
        if stamp_source == 'daemon':
            # motion.asOf as the daemon read it, and the observedAt of a frame or of a
            # controller list. Never T0, the old stamp of the first snapshot: the daemon
            # sent it long after it read those joints, so they were not current.
            assert set(state_stamps) <= as_of.handed_ns()
            assert set(snapshot_stamps) <= observed.handed_ns()
        else:
            # The clock of this machine, inside the window of the test.
            for stamp in state_stamps + snapshot_stamps:
                assert wall_before <= stamp <= wall_after
    values = values_of(bench.daemon_status())
    assert values['stamp source'] == stamp_source


def test_scenario_1_stamp_source_auto_takes_the_daemon_clock_on_this_machine(bench):
    fake = bench.start_fake([frames.controller('cell-1')])
    as_of = Times(1000.0, 0.01)
    Advance(fake, as_of)
    bench.collector.subscribe('cell_1/joint_states', JointState)
    bench.start_node()
    states = bench.collector.wait_for('cell_1/joint_states', 3)
    assert {stamp_ns(state.header.stamp) for state in states} <= as_of.handed_ns()
    assert values_of(bench.daemon_status())['stamp source'] == 'daemon'


@pytest.mark.parametrize('stamp_source', ['daemon', 'bridge'])
def test_scenario_1_an_old_joint_stamp_in_the_first_snapshot_is_not_published(
        bench, stamp_source):
    # The daemon reports joints it read long before it sent the snapshot (T0 against a
    # real observedAt). Nothing says they were read since: no JointState, the status
    # says so, and with the bridge's clock no stamp is a time at which nothing was
    # confirmed. The first sample the daemon reads is published, with its own time.
    fake = bench.start_fake([frames.controller('cell-1')])
    reading = Advance(fake)
    reading.stop()
    collector = bench.collector
    collector.subscribe('joint_states', JointState)
    collector.subscribe('controller_snapshot', ControllerSnapshot)
    bench.start_node(primary_controller='cell-1', stamp_source=stamp_source)
    bench.daemon_status(lambda status: status.level == OK)
    waiting = bench.controller_status('cell-1', lambda status: status.level == STALE)
    assert 'joints not confirmed yet' in waiting.message
    assert values_of(waiting)['motion asOf age (s)'] == 'unknown'
    # The snapshot itself is current: the daemon is live and the link is up.
    collector.wait_for('controller_snapshot', 2)
    collector.assert_silent('joint_states', 0.6)
    assert collector.received('joint_states') == []

    resumed = time.time_ns()
    reading.running = True
    state = collector.wait_for('joint_states')[0]
    if stamp_source == 'daemon':
        assert stamp_ns(state.header.stamp) > iso_ns(frames.T0)
        assert stamp_ns(state.header.stamp) == iso_ns(fake.controllers[0]['motion']['asOf'])
    else:
        assert stamp_ns(state.header.stamp) >= resumed
    bench.controller_status('cell-1', lambda status: status.level == OK, after=collector.now())


def test_scenario_1_joint_names_follow_the_format_and_the_overrides(bench):
    fake = bench.start_fake([frames.controller('cell-1'), frames.controller('cell-2')])
    Advance(fake)
    bench.collector.subscribe('joint_states', JointState)
    bench.collector.subscribe('cell_2/joint_states', JointState)
    bench.start_node(
        primary_controller='cell-1', joint_name_format='{controller}_joint_{axis}',
        joint_name_overrides=['cell-2/ROB_1/J1=base_joint'],
        joint_states_frame_id='cell')
    primary = bench.collector.wait_for('joint_states')[0]
    other = bench.collector.wait_for('cell_2/joint_states')[0]
    # {controller} is made from the id alone: the primary controller keeps its name.
    assert list(primary.name) == [f'cell_1_joint_{axis}' for axis in range(1, 7)]
    assert list(other.name) == ['base_joint'] + [f'cell_2_joint_{axis}' for axis in range(2, 7)]
    assert primary.header.frame_id == 'cell' and other.header.frame_id == 'cell'


# ----- scenario 2: a motion delta ------------------------------------------------------


def test_scenario_2_a_motion_delta_arrives_as_one_new_joint_state(bench):
    fake = bench.start_fake([frames.controller('cell-1')])
    collector = bench.collector
    collector.subscribe('joint_states', JointState)
    bench.start_node(primary_controller='cell-1')
    bench.daemon_status(lambda status: status.level == OK)
    bench.wait_matched('joint_states')
    # The robot stood still and nothing confirmed its joints: the topic is silent.
    mark = collector.settled()
    joints = [frames.joint(f'J{axis}') for axis in range(1, 6)] + [frames.joint('J6', 5.0)]
    moved_at = fake.now()
    fake.push(frames.delta_frame(
        'cell-1',
        {'motion': frames.motion_group([frames.motion_unit(joints=joints)], as_of=moved_at)},
        observed_at=moved_at))
    state = collector.wait_for('joint_states', after=mark)[0]
    assert list(state.name) == ROBOT_JOINTS
    assert list(state.position[:5]) == [0.0] * 5
    assert abs(state.position[5] - 5.0 * math.pi / 180.0) < 1e-12
    assert stamp_ns(state.header.stamp) == iso_ns(moved_at)
    time.sleep(0.4)
    assert len(collector.received('joint_states', after=mark)) == 1


# ----- scenario 3: the edge cases ------------------------------------------------------


def test_scenario_3_edge_controller_units_and_joints_left_out(bench):
    fake = bench.start_fake([frames.edge_controller('edge-1')])
    Advance(fake)
    collector = bench.collector
    collector.subscribe('joint_states', JointState)
    collector.subscribe('controller_snapshot', ControllerSnapshot)
    bench.start_node(primary_controller='edge-1')

    state = collector.wait_for('joint_states')[0]
    # Millimetres became metres; the joints without a unit or a value are not there,
    # and neither is the joint of the unit that reports an error.
    assert list(state.name) == ['ROB_1_J1', 'ROB_1_J2', 'ROB_1_J5', 'ROB_1_J6', 'ROB_1_E1']
    expected = [10.0 * math.pi / 180.0, -20.5 * math.pi / 180.0, 45.0 * math.pi / 180.0,
                90.0 * math.pi / 180.0, 1.25]
    for position, value in zip(state.position, expected):
        assert abs(position - value) < 1e-12

    # A slow machine can miss the 0.5 s confirmation window first, and the status then
    # reads STALE; the joints left out are a warning once the joints are current.
    status = bench.controller_status(
        'edge-1', lambda found: 'left out' in found.message and found.level != STALE)
    assert status.level == WARN
    left_out = values_of(status)['joints left out']
    for text in ('ROB_1/J3: no-value', 'ROB_1/J4: unit-unknown', 'STN_1/J1: unit-error'):
        assert text in left_out and text in status.message
    assert values_of(status)['joints published'] == '5'
    assert 'link polling' in status.message

    snapshot = collector.wait_for('controller_snapshot')[0]
    robot, station = snapshot.motion.units
    joints = {joint.name: joint for joint in robot.joints}
    assert math.isnan(joints['J3'].value) and joints['J3'].unit == 'deg'
    assert joints['J4'].value == 5.0 and joints['J4'].unit == ''
    assert joints['E1'].value == 1250.0 and joints['E1'].unit == 'mm'
    assert joints['J1'].value == 10.0 and joints['J1'].unit == 'deg'
    assert station.joints[0].value == 30.0 and len(station.error) == 1
    assert list(snapshot.identity) == []


# ----- scenario 4: a robot that stands still -------------------------------------------


def test_scenario_4_stationary_robot_is_confirmed_then_goes_stale(bench):
    fake = bench.start_fake([frames.controller('cell-1')])
    reading = Advance(fake)
    collector = bench.collector
    collector.subscribe('joint_states', JointState)
    collector.subscribe('controller_snapshot', ControllerSnapshot)
    bench.start_node(primary_controller='cell-1')

    states = collector.wait_for('joint_states', 5)
    stamps = [stamp_ns(state.header.stamp) for state in states]
    assert stamps == sorted(set(stamps)), 'the stamp moves on with every confirmation'
    assert all(list(state.position) == [0.0] * 6 for state in states)
    current = bench.controller_status('cell-1', lambda status: status.level == OK)
    assert current.message == 'current'
    # Only the controller list moved the stamps: the stream sent its snapshot, no more.
    assert values_of(bench.daemon_now())['frames received'] == '1'

    reading.stop()
    mark = collector.now()
    stale = bench.controller_status('cell-1', lambda status: status.level == STALE,
                                    after=mark)
    assert 'joints not confirmed' in stale.message
    assert collector.now() - mark < FAST['stale_after_s'] + 0.25 + SLACK_S
    silent_from = collector.settled()
    collector.assert_silent('joint_states', 0.6, after=silent_from)
    # The daemon itself still answers: its status is fine and the snapshot keeps coming.
    assert collector.wait_for('controller_snapshot', 2, after=silent_from)
    assert bench.daemon_status(after=silent_from).level == OK

    reading.running = True
    assert collector.wait_for('joint_states', after=collector.now())
    bench.controller_status('cell-1', lambda status: status.level == OK,
                            after=collector.now())


def test_scenario_4_joints_nobody_watches_are_not_published(bench):
    fake = bench.start_fake([frames.controller(
        'cell-1', motion=frames.motion_group(watching=False))])
    Advance(fake)
    bench.collector.subscribe('joint_states', JointState)
    bench.start_node(primary_controller='cell-1', watch_motion=False)
    # With watch_motion false that is what was asked for: said in the status, no fault.
    status = bench.controller_status(
        'cell-1', lambda found: 'joints not watched' in found.message)
    assert status.level == OK
    assert status.message == 'current; joints not watched (watch_motion is false)'
    assert values_of(status)['motion lease'] == 'not asked'
    assert values_of(status)['joints published'] == '0'
    bench.collector.assert_silent('joint_states', 0.5)
    assert fake.requests_to('/v0/controllers/cell-1/motion/watch') == []


# ----- scenario 5: I/O -----------------------------------------------------------------


def test_scenario_5_io_values_typed_refused_and_never_retyped(bench):
    signals = [
        frames.signal('do_gripper', 'DO', '1'),
        frames.signal('ao_speed', 'AO', '-0.75', via='poll'),
        frames.signal('Local/DRV_1/go_program', 'GO', '42'),
        frames.signal('di_bad', 'DI', 'x'),
    ]
    fake = bench.start_fake([io_controller('cell-1', signals)])
    fake.known_signals = {'cell-1': {'do_gripper', 'ao_speed', 'go_program', 'di_bad'}}
    # The joints stay confirmed, so the level of the status is the signals' doing.
    Advance(fake)
    collector = bench.collector
    collector.subscribe('io/do_gripper', Bool)
    collector.subscribe('io/ao_speed', Float64)
    collector.subscribe('io/Local_DRV_1_go_program', UInt32)
    collector.subscribe('io/di_bad', Bool)
    bench.start_node(primary_controller='cell-1', io_signals=[
        'cell-1:do_gripper', 'cell-1:ao_speed', 'cell-1:Local/DRV_1/go_program',
        'cell-1:di_bad', 'cell-1:do_unknown'])

    assert collector.wait_for('io/do_gripper')[0].data is True
    assert collector.wait_for('io/ao_speed')[0].data == -0.75
    assert collector.wait_for('io/Local_DRV_1_go_program')[0].data == 42
    types = collector.topic_types()
    assert types[collector.resolve('io/do_gripper')] == ['std_msgs/msg/Bool']
    assert types[collector.resolve('io/ao_speed')] == ['std_msgs/msg/Float64']
    assert types[collector.resolve('io/Local_DRV_1_go_program')] == ['std_msgs/msg/UInt32']

    # A bad value and a refused name: no message, and the status names both.
    status = bench.controller_status('cell-1', lambda found: 'refused' in found.message)
    assert status.level == WARN
    values = values_of(status)
    assert 'do_unknown: no such signal on this controller' in values['signals refused']
    assert 'di_bad' in values['signals not publishable']
    assert 'neither "0" nor "1"' in values['signals not publishable']
    assert 'do_unknown' in status.message and 'di_bad' in status.message
    assert values['signals accepted'] == (
        'do_gripper, ao_speed, Local/DRV_1/go_program, di_bad')
    assert values['io lease'] == 'granted'
    assert collector.received('io/di_bad') == []
    assert collector.publisher_count('io/do_unknown') == 0

    # A change is published at once.
    mark = collector.now()
    opened = [dict(signals[0], value='0')] + signals[1:]
    fake.push(frames.delta_frame('cell-1', {'io': frames.io_group(opened)},
                                 observed_at=fake.now()))
    assert collector.wait_for('io/do_gripper', after=mark,
                              where=lambda message: message.data is False)

    # The controller now reports another kind for the same name: the signal stops, the
    # status is an error, and the topic keeps the type it had.
    retyped = [frames.signal('do_gripper', 'GO', '7')] + signals[1:]
    fake.push(frames.delta_frame('cell-1', {'io': frames.io_group(retyped)},
                                 observed_at=fake.now()))
    mark = collector.now()
    error = bench.controller_status('cell-1', lambda found: found.level == ERROR,
                                    after=mark)
    assert 'changed type and stopped: do_gripper: type changed' in error.message
    silent_from = collector.settled()
    collector.assert_silent('io/do_gripper', 0.5, after=silent_from)
    assert collector.topic_types()[collector.resolve('io/do_gripper')] == ['std_msgs/msg/Bool']
    assert collector.publisher_count('io/do_gripper') == 1
    assert collector.wait_for('io/ao_speed', after=silent_from)
    assert collector.wait_for('io/Local_DRV_1_go_program', after=silent_from)


def test_scenario_5_a_signal_is_silent_while_the_daemon_does_not_serve_it(bench):
    signals = [frames.signal('do_gripper', 'DO', '1'),
               frames.signal('do_missing', None, None, via=None, as_of=None,
                             error=frames.group_error())]
    fake = bench.start_fake([io_controller('cell-1', signals)])
    # The joints stay confirmed, so the level of the status is the signal's doing.
    Advance(fake)
    collector = bench.collector
    collector.subscribe('io/do_gripper', Bool)
    bench.start_node(primary_controller='cell-1',
                     io_signals=['cell-1:do_gripper', 'cell-1:do_missing'])
    assert collector.wait_for('io/do_gripper')[0].data is True
    status = bench.controller_status('cell-1', lambda found: 'do_missing' in found.message)
    assert status.level == WARN
    assert 'the last read failed (RESOURCE_NOT_FOUND)' in status.message
    # The type was never known, so the signal has no topic at all.
    assert collector.publisher_count('io/do_missing') == 0

    # The lease ran out at the daemon: it clears "via", and the value is not current.
    lapsed = [dict(signals[0], via=None), signals[1]]
    fake.push(frames.delta_frame('cell-1', {'io': frames.io_group(lapsed, watching=False)},
                                 observed_at=fake.now()))
    mark = collector.now()
    bench.controller_status('cell-1', lambda found: 'I/O not watched' in found.message,
                            after=mark)
    collector.assert_silent('io/do_gripper', 0.5, after=collector.settled())


# ----- scenario 6: the stream is lost --------------------------------------------------


def test_scenario_6_stream_loss_silence_and_recovery_with_the_same_lease(bench):
    fake = bench.start_fake([io_controller('cell-1')])
    Advance(fake)
    collector = bench.collector
    topics = {'joint_states': JointState, 'controller_snapshot': ControllerSnapshot,
              'io/do_gripper': Bool}
    for topic, message_type in topics.items():
        collector.subscribe(topic, message_type)
    bench.start_node(primary_controller='cell-1', io_signals=['cell-1:do_gripper'])
    for topic in topics:
        collector.wait_for(topic, 2)
    bench.daemon_status(lambda status: status.level == OK)
    motion_path = '/v0/controllers/cell-1/motion/watch'
    io_path = '/v0/controllers/cell-1/io/watch'
    assert fake.requests_to(motion_path, 'POST')[0].body == {'leaseId': 'new'}
    granted = next(iter(fake.leases[('cell-1', 'motion')]))
    assert granted and granted != 'new'

    # The daemon stops in good order and is gone.
    mark = collector.now()
    fake.close_stream(1001)
    fake.stop()
    down = bench.daemon_status(lambda status: status.level == ERROR, after=mark)
    assert down.message.startswith(('daemon-stopping', 'unreachable', 'closed'))
    assert values_of(down)['connected'] == 'false'
    silent_from = collector.settled()
    time.sleep(0.6)
    for topic in topics:
        assert collector.arrivals(topic, after=silent_from) == [], topic
    unreachable = bench.daemon_status(
        lambda status: status.message.startswith('unreachable'), after=mark)
    assert unreachable.level == ERROR
    waiting = bench.controller_status('cell-1', after=silent_from)
    assert waiting.level == STALE and 'the daemon is not live' in waiting.message
    # Nothing was torn down: the topics are there, silent.
    assert collector.publisher_count('joint_states') == 1

    # The daemon is back on the same port; it has forgotten every lease.
    fake.clear_requests()
    fake.start()
    back = collector.now()
    up = bench.daemon_status(lambda status: status.level == OK, after=back, timeout=20.0)
    assert int(values_of(up)['reconnects']) >= 1
    for topic in topics:
        collector.wait_for(topic, 2, after=back)
    assert wait_until(lambda: fake.requests_to(io_path, 'POST'), 5.0)
    bodies = [request.body for request in fake.requests_to(motion_path, 'POST')]
    bodies += [request.body for request in fake.requests_to(io_path, 'POST')]
    assert bodies and all(body['leaseId'] == granted for body in bodies)
    assert all(request.headers['host'] == f'127.0.0.1:{fake.port}'
               for request in fake.requests)


def test_scenario_6_a_stopped_node_gives_its_signals_back(bench):
    fake = bench.start_fake([io_controller('cell-1')])
    bench.collector.subscribe('io/do_gripper', Bool)
    bench.start_node(primary_controller='cell-1', io_signals=['cell-1:do_gripper'])
    bench.collector.wait_for('io/do_gripper')
    assert fake.io_watched('cell-1') == ['do_gripper']
    started = time.monotonic()
    bench.stop_node()
    assert time.monotonic() - started < 3.0 + SLACK_S
    assert fake.io_watched('cell-1') == []
    assert wait_until(lambda: fake.consumer_closes == [1000], 3.0), fake.consumer_closes
    assert fake.data_frames == 0


# ----- scenario 7: a token, a host or a recording that is refused ----------------------


def test_scenario_7_wrong_token_is_reported_and_nothing_is_published(bench):
    fake = bench.start_fake([frames.controller('cell-1')])
    bench.token_file.write_text(fake_daemon.OTHER_TOKEN + '\n', encoding='ascii')
    bench.start_node(primary_controller='cell-1')
    status = bench.daemon_status(lambda found: found.message.startswith('token-refused'))
    assert status.level == ERROR
    assert 'point token_file at the token file of this daemon' in status.message
    assert values_of(status)['reason'] == 'token-refused'
    time.sleep(0.5)
    assert bench.data_topics() == []
    assert fake.attached == 0
    assert all(request.token in (None, 'not matched') for request in fake.requests)

    # The right token arrives in the file: the next attempt reads it.
    bench.token_file.write_text(fake_daemon.TOKEN + '\r\n', encoding='ascii')
    bench.daemon_status(lambda found: found.level == OK, after=bench.collector.now())


def test_scenario_7_missing_token_file_is_reported_without_a_request(bench):
    fake = bench.start_fake([frames.controller('cell-1')])
    bench.token_file.unlink()
    bench.start_node()
    status = bench.daemon_status(lambda found: found.message.startswith('token-file'))
    assert status.level == ERROR
    assert 'start abb-rws-daemon first' in status.message
    assert all(request.path == '/v0/health' for request in fake.requests)


def test_scenario_7_refused_host_sends_no_token(bench):
    fake = bench.start_fake([frames.controller('cell-1')])
    fake.allowed_hosts = set()
    bench.start_node(primary_controller='cell-1')
    status = bench.daemon_status(lambda found: found.message.startswith('host-refused'))
    assert status.level == ERROR
    time.sleep(0.5)
    requests = fake.requests
    assert requests and all(request.status == 421 for request in requests)
    # No token header in any request, and never another Host than the URL's own.
    assert all(request.token is None for request in requests)
    assert all(fake_daemon.TOKEN_HEADER not in request.headers for request in requests)
    assert all(request.headers['host'] == f'127.0.0.1:{fake.port}' for request in requests)
    assert fake.requests_to('/v0/stream') == []
    assert bench.data_topics() == []


def test_scenario_7_replay_without_allow_replay_opens_no_stream(bench):
    fake = bench.start_fake([frames.controller('cell-1')], mode='replay')
    bench.start_node(primary_controller='cell-1')
    status = bench.daemon_status(lambda found: found.message.startswith('replay-refused'))
    assert status.level == ERROR
    assert 'set allow_replay to publish a recording under replay/' in status.message
    assert values_of(status)['mode'] == 'replay'
    time.sleep(0.5)
    assert fake.attached == 0
    assert all(request.path == '/v0/health' and request.token is None
               for request in fake.requests)
    assert bench.data_topics() == []


def test_scenario_7_replay_with_allow_replay_is_published_under_replay_only(bench):
    recorded = frames.iso(-86400.0)
    fake = bench.start_fake(
        [io_controller('cell-1'), frames.controller('cell-2')], mode='replay',
        replay=frames.replay_health(recorded_at=recorded, position_ms=1500))
    Advance(fake)
    collector = bench.collector
    replayed = {'replay/joint_states': JointState,
                'replay/controller_snapshot': ControllerSnapshot,
                'replay/io/do_gripper': Bool,
                'replay/cell_2/joint_states': JointState,
                'replay/cell_2/controller_snapshot': ControllerSnapshot}
    live = {'joint_states': JointState, 'controller_snapshot': ControllerSnapshot,
            'io/do_gripper': Bool, 'cell_2/joint_states': JointState,
            'cell_2/controller_snapshot': ControllerSnapshot}
    for topic, message_type in {**replayed, **live}.items():
        collector.subscribe(topic, message_type)
    bench.start_node(primary_controller='cell-1', allow_replay=True,
                     io_signals=['cell-1:do_gripper'])
    for topic in replayed:
        collector.wait_for(topic, 2)
    assert collector.wait_for('replay/controller_snapshot')[0].id == 'cell-1'
    assert collector.wait_for('replay/cell_2/controller_snapshot')[0].id == 'cell-2'

    # Nothing on the live topic names: no publisher, no message.
    for topic in live:
        assert collector.publisher_count(topic) == 0, topic
        assert collector.received(topic) == [], topic
    assert bench.data_topics() == sorted(collector.resolve(topic) for topic in replayed)

    daemon = bench.daemon_status(lambda found: found.level == WARN)
    assert daemon.message.startswith(f'REPLAY: recorded {recorded}')
    values = values_of(daemon)
    assert values['mode'] == 'replay' and values['recorded at'] == recorded
    assert values['replay state'] == 'playing' and values['replay position (ms)'] == '1500'
    for controller_id in ('cell-1', 'cell-2'):
        status = bench.controller_status(controller_id, lambda found: found.level == WARN)
        assert status.message.startswith(f'REPLAY: recorded {recorded}')


def test_scenario_7_a_replay_that_starts_over_may_stamp_in_the_past_again(bench):
    fake = bench.start_fake([frames.controller('cell-1')], mode='replay')
    recorded = Times(5000.0, 0.01)
    fake.now = recorded
    Advance(fake)
    collector = bench.collector
    collector.subscribe('replay/joint_states', JointState)
    bench.start_node(primary_controller='cell-1', allow_replay=True, stamp_source='daemon')
    collector.wait_for('replay/joint_states', 3)

    # The recording begins again: close 1012, and its times start in the past.
    again = Times(4000.0, 0.01)
    fake.now = again
    fake.update('cell-1', motion=frames.motion_group(as_of=again()))
    fake.close_stream(1012)
    collector.wait_for(
        'replay/joint_states', 3,
        where=lambda state: stamp_ns(state.header.stamp) in again.handed_ns())
    stamps = [stamp_ns(state.header.stamp)
              for state in collector.received('replay/joint_states')]
    restart = next(index for index, stamp in enumerate(stamps)
                   if stamp in again.handed_ns())
    first_run, second_run = stamps[:restart], stamps[restart:]
    assert len(first_run) >= 3 and len(second_run) >= 3
    # Time went back between the runs, and inside each run it only moves forward.
    assert max(second_run) < max(first_run)
    assert max(first_run) in recorded.handed_ns()
    assert first_run == sorted(first_run)
    assert second_run == sorted(second_run) and set(second_run) <= again.handed_ns()
    assert values_of(bench.daemon_now())['stamps held back'] == '0'


def test_scenario_7_a_daemon_that_changes_its_mode_gets_other_publishers(bench, capfd):
    fake = bench.start_fake([frames.controller('cell-1')])
    Advance(fake)
    collector = bench.collector
    collector.subscribe('joint_states', JointState)
    collector.subscribe('replay/joint_states', JointState)
    bench.start_node(primary_controller='cell-1', allow_replay=True)
    collector.wait_for('joint_states', 2)
    assert collector.publisher_count('replay/joint_states') == 0

    # The same address now serves a recording.
    fake.mode = 'replay'
    fake.close_stream(1001)
    collector.wait_for('replay/joint_states', 2)
    collector.wait_for_publishers('joint_states', 0)
    switched = collector.settled()
    replayed = bench.daemon_status(lambda status: status.level == WARN, after=switched)
    assert replayed.message.startswith('REPLAY: recorded')
    collector.assert_silent('joint_states', 0.4, after=switched)
    # The publishers of the live names went before those under replay/ were made.
    log = capfd.readouterr().err
    assert 0 < log.index('is not published any more: the daemon changed its mode') < (
        log.index('is published on replay/joint_states'))

    # And a robot again.
    fake.mode = 'live'
    fake.close_stream(1001)
    collector.wait_for_publishers('replay/joint_states', 0)
    back = collector.settled()
    collector.wait_for('joint_states', 2, after=back)
    collector.assert_silent('replay/joint_states', 0.4, after=back)
    assert bench.daemon_status(lambda status: status.level == OK, after=back)


def test_scenario_7_a_daemon_that_starts_to_replay_while_connected_is_silenced(bench):
    fake = bench.start_fake([frames.controller('cell-1')])
    Advance(fake)
    collector = bench.collector
    topics = {'joint_states': JointState, 'controller_snapshot': ControllerSnapshot}
    for topic, message_type in topics.items():
        collector.subscribe(topic, message_type)
    bench.start_node(primary_controller='cell-1')
    for topic in topics:
        collector.wait_for(topic, 2)

    # No close and no frame tells: only the health answer names the new mode (the
    # client reads it every 5 s).
    mark = collector.now()
    fake.mode = 'replay'
    refused = bench.daemon_status(
        lambda status: status.message.startswith('replay-refused'), after=mark,
        timeout=20.0)
    assert refused.level == ERROR
    silent_from = collector.settled()
    time.sleep(0.5)
    for topic in topics:
        assert collector.arrivals(topic, after=silent_from) == [], topic
    assert collector.publisher_count('replay/joint_states') == 0
    assert collector.publisher_count('replay/controller_snapshot') == 0


# ----- scenario 8: controllers come and go; names --------------------------------------


def test_scenario_8_controller_removed_and_added_primary_and_tokens(bench):
    fake = bench.start_fake([frames.controller('cell-1'), frames.controller('cell-2')])
    Advance(fake)
    collector = bench.collector
    for prefix in ('', 'cell_2/', 'cell_3/'):
        collector.subscribe(f'{prefix}joint_states', JointState)
        collector.subscribe(f'{prefix}controller_snapshot', ControllerSnapshot)
    bench.start_node(primary_controller='cell-1')

    # The primary controller sits in the node's namespace, the other under its token.
    assert collector.wait_for('controller_snapshot')[0].id == 'cell-1'
    assert collector.wait_for('cell_2/controller_snapshot')[0].id == 'cell-2'
    collector.wait_for('joint_states')
    collector.wait_for('cell_2/joint_states')
    assert collector.publisher_count('cell_1/joint_states') == 0
    assert bench.data_topics() == sorted(collector.resolve(topic) for topic in (
        'joint_states', 'controller_snapshot', 'cell_2/joint_states',
        'cell_2/controller_snapshot'))
    bench.controller_status('cell-2', lambda status: status.level == OK)

    # Removed: its publishers go, and so does its status.
    fake.push(frames.removed_frame('cell-2', observed_at=fake.now()))
    collector.wait_for_publishers('cell_2/joint_states', 0)
    collector.wait_for_publishers('cell_2/controller_snapshot', 0)
    gone = collector.settled()
    collector.assert_silent('cell_2/controller_snapshot', 0.5, after=gone)
    assert collector.statuses(f'{bench.node_name}: controller cell-2', after=gone) == []
    assert collector.wait_for('controller_snapshot', after=gone)

    # Added: a delta with every group for an id the bridge did not know.
    added = frames.controller('cell-3')
    fake.push(frames.delta_frame('cell-3', frames.groups_of(added), observed_at=fake.now()))
    assert collector.wait_for('cell_3/controller_snapshot')[0].id == 'cell-3'
    collector.wait_for('cell_3/joint_states')
    bench.controller_status('cell-3', lambda status: status.level == OK)
    assert wait_until(
        lambda: fake.requests_to('/v0/controllers/cell-3/motion/watch', 'POST'), 5.0)

    # And the one that was removed comes back under the name it had.
    back = collector.now()
    fake.push(frames.delta_frame('cell-2', frames.groups_of(frames.controller('cell-2')),
                                 observed_at=fake.now()))
    assert collector.wait_for('cell_2/controller_snapshot', after=back)[0].id == 'cell-2'
    collector.wait_for('cell_2/joint_states', after=back)


def test_scenario_8_a_name_that_is_taken_is_refused_not_renamed(bench):
    # Both ids give the token a_b, and "io" is a name the bridge uses itself.
    fake = bench.start_fake([frames.controller('a-b'), frames.controller('a_b'),
                             frames.controller('io')])
    Advance(fake)
    collector = bench.collector
    collector.subscribe('a_b/controller_snapshot', ControllerSnapshot)
    bench.start_node()
    snapshots = collector.wait_for('a_b/controller_snapshot', 5)
    assert {snapshot.id for snapshot in snapshots} == {'a-b'}
    assert collector.publisher_count('a_b/controller_snapshot') == 1
    refused = bench.controller_status('a_b')
    assert refused.level == ERROR
    assert 'not bridged' in refused.message and 'already taken by controller "a-b"' in (
        refused.message)
    reserved = bench.controller_status('io')
    assert reserved.level == ERROR and 'one the bridge uses itself' in reserved.message
    assert collector.publisher_count('io/controller_snapshot') == 0
    bench.controller_status('a-b', lambda status: status.level == OK)
    # A controller that is not bridged is not watched either.
    time.sleep(0.3)
    assert fake.requests_to('/v0/controllers/a_b/motion/watch') == []
    assert fake.requests_to('/v0/controllers/io/motion/watch') == []

    # The name becomes free: the controller that was refused takes it.
    fake.push(frames.removed_frame('a-b', observed_at=fake.now()))
    freed = collector.now()
    taken = collector.wait_for('a_b/controller_snapshot', after=freed,
                               where=lambda snapshot: snapshot.id == 'a_b')
    assert taken
    bench.controller_status('a_b', lambda status: status.level == OK, after=freed)
    assert wait_until(
        lambda: fake.requests_to('/v0/controllers/a_b/motion/watch', 'POST'), 5.0)


def test_scenario_8_a_controller_that_only_a_parameter_names_gets_a_status(bench):
    # Every controller the daemon serves is bridged; io_signals and the overrides name
    # one more. A topic that silently never appears would say nothing; a status does.
    fake = bench.start_fake([frames.controller('cell-1')])
    Advance(fake)
    collector = bench.collector
    collector.subscribe('cell_1/joint_states', JointState)
    bench.start_node(io_signals=['cell-9:do_x'], joint_name_overrides=['cell-8/ROB_1/J1=a'])
    collector.wait_for('cell_1/joint_states')
    for controller_id in ('cell-9', 'cell-8'):
        missing = bench.controller_status(controller_id, lambda status: status.level == ERROR)
        assert 'not served by the daemon' in missing.message
    bench.controller_status('cell-1', lambda status: status.level == OK)
    assert collector.publisher_count('cell_9/io/do_x') == 0
    # The daemon starts to serve it: the topic appears and the status follows.
    fake.push(frames.delta_frame('cell-9', frames.groups_of(io_controller(
        'cell-9', [frames.signal('do_x', 'DO', '1')])), observed_at=fake.now()))
    collector.subscribe('cell_9/io/do_x', Bool)
    assert collector.wait_for('cell_9/io/do_x')[0].data is True
    bench.controller_status('cell-9', lambda status: status.level == OK)


def test_scenario_8_only_the_selected_controllers_are_bridged(bench):
    fake = bench.start_fake([frames.controller('cell-1'), frames.controller('cell-2')])
    Advance(fake)
    collector = bench.collector
    collector.subscribe('cell_2/joint_states', JointState)
    bench.start_node(controllers=['cell-2', 'cell-9'])
    collector.wait_for('cell_2/joint_states')
    missing = bench.controller_status('cell-9', lambda status: status.level == ERROR)
    assert 'not served by the daemon' in missing.message
    bench.controller_status('cell-2', lambda status: status.level == OK)
    time.sleep(0.3)
    assert bench.data_topics() == sorted(collector.resolve(topic) for topic in (
        'cell_2/joint_states', 'cell_2/controller_snapshot'))
    assert collector.statuses(f'{bench.node_name}: controller cell-1') == []
    assert fake.requests_to('/v0/controllers/cell-1/motion/watch') == []
    assert fake.requests_to('/v0/controllers/cell-2/motion/watch', 'POST')


# ----- scenario 9: another schema version ----------------------------------------------


def test_scenario_9_newer_schema_with_unknown_content_works_with_a_warning(bench):
    controller = frames.controller('cell-1')
    controller['future'] = {'reading': 1, 'asOf': frames.T0}
    controller['motion']['futureField'] = 'x'
    controller['panel']['newThing'] = {'list': [1, 2]}
    fake = bench.start_fake([controller])
    fake.schema_version = frames.NEWER_SCHEMA_VERSION
    Advance(fake)
    collector = bench.collector
    collector.subscribe('joint_states', JointState)
    collector.subscribe('controller_snapshot', ControllerSnapshot)
    bench.start_node(primary_controller='cell-1')
    collector.wait_for('joint_states')
    collector.wait_for('controller_snapshot')

    mark = collector.now()
    fake.push({'type': 'future-frame', 'observedAt': fake.now(), 'payload': [1, 2, 3]})
    fake.push(frames.delta_frame(
        'cell-1', {'future': {'reading': 2, 'asOf': frames.T0},
                   'panel': dict(frames.panel_group(op_mode='MANR'), newThing=None)},
        observed_at=fake.now()))
    changed = collector.wait_for(
        'controller_snapshot', after=mark,
        where=lambda snapshot: snapshot.panel.op_mode == 'MANR')
    assert changed[0].id == 'cell-1'
    collector.wait_for('joint_states', 2, after=mark)

    status = bench.daemon_status(lambda found: found.level == WARN, after=mark)
    newer, ours = frames.NEWER_SCHEMA_VERSION, frames.SCHEMA_VERSION
    assert f'the daemon schema {newer} is newer than the messages ({ours})' in status.message
    values = values_of(status)
    assert values['daemon schema'] == newer and values['messages schema'] == ours
    assert bench.controller_status('cell-1', after=mark).level == OK


def test_scenario_9_older_schema_is_refused_and_nothing_is_published(bench):
    fake = bench.start_fake([frames.controller('cell-1')])
    fake.schema_version = frames.OLDER_SCHEMA_VERSION
    Advance(fake)
    bench.start_node(primary_controller='cell-1')
    status = bench.daemon_status(lambda found: found.message.startswith('schema too old'))
    assert status.level == ERROR
    older, ours = frames.OLDER_SCHEMA_VERSION, frames.SCHEMA_VERSION
    assert f'the daemon speaks {older}, the messages need {ours} or later' in status.message
    time.sleep(0.6)
    assert bench.data_topics() == []
    assert fake.attached >= 1
    # Nothing is asked of a daemon whose data cannot be used.
    assert fake.requests_to('/v0/controllers/cell-1/motion/watch') == []


# ----- scenario 10: built-ins and secrets ----------------------------------------------


def test_scenario_10_no_own_service_no_subscription_only_known_parameters(bench):
    fake = bench.start_fake([io_controller('cell-1')])
    Advance(fake)
    collector = bench.collector
    collector.subscribe('joint_states', JointState)
    collector.subscribe('io/do_gripper', Bool)
    node = bench.start_node(primary_controller='cell-1', io_signals=['cell-1:do_gripper'])
    collector.wait_for('joint_states')
    collector.wait_for('io/do_gripper')
    collector.wait_for_node(bench.node_name)

    # Services: only what every node has, judged by type and not by a list of names.
    services = collector.services_of(bench.node_name)
    assert services
    for name, types in services:
        for service_type in types:
            package = service_type.split('/')[0]
            assert package in ('rcl_interfaces', 'type_description_interfaces'), name
            assert package not in ('abb_rws_ros2_msgs', 'std_srvs'), name
    assert collector.subscriptions_of(bench.node_name) == []
    assert list(node.subscriptions) == []
    assert list(node.clients) == []

    # Parameters: the table, the two built-ins and one qos_overrides group per publisher.
    declared = set(node.get_parameters_by_prefix(''))
    specs = {spec.name: spec for spec in parameters.SPECS}
    assert set(specs) <= declared
    for name in declared - set(specs):
        assert (name in parameters.BUILTIN_PARAMETERS
                or name.startswith(parameters.QOS_OVERRIDE_PREFIX)), name
    assert parameters.BUILTIN_PARAMETERS <= declared
    for topic in ('joint_states', 'controller_snapshot', 'io/do_gripper'):
        prefix = f'{parameters.QOS_OVERRIDE_PREFIX}{collector.resolve(topic)}.publisher.'
        assert {prefix + 'depth', prefix + 'history', prefix + 'reliability'} <= declared
    assert not any(DIAGNOSTICS_OVERRIDE in name for name in declared)
    for spec in parameters.SPECS:
        descriptor = node.describe_parameter(spec.name)
        assert descriptor.read_only, spec.name
        assert descriptor.description == spec.description
        if spec.kind == 'double':
            limits = descriptor.floating_point_range[0]
            assert (limits.from_value, limits.to_value) == (spec.minimum, spec.maximum)
    results = node.set_parameters([Parameter('refresh_hz', value=5.0)])
    assert not results[0].successful
    assert node.get_parameter('refresh_hz').value == FAST['refresh_hz']
    assert node.get_parameter('io_signals').value == ['cell-1:do_gripper']


def test_scenario_10_the_token_is_in_no_log_line_and_no_diagnostic_value(
        bench, debug_logs, capfd):
    fake = bench.start_fake([io_controller('cell-1')])
    Advance(fake)
    collector = bench.collector
    collector.subscribe(ROSOUT, Log)
    collector.subscribe('joint_states', JointState)
    node = bench.start_node(primary_controller='cell-1',
                            io_signals=['cell-1:do_gripper', 'cell-1:bad name'])
    node.get_logger().set_level(LoggingSeverity.DEBUG)
    collector.wait_for('joint_states')
    bench.controller_status('cell-1', lambda status: 'refused' in status.message)

    # A token the daemon refuses, a token file without a token, then the new token.
    fake.token = fake_daemon.OTHER_TOKEN
    mark = collector.now()
    bench.daemon_status(lambda status: status.message.startswith('token-refused'),
                        after=mark)
    bench.token_file.write_text('no token at all', encoding='ascii')
    bench.daemon_status(lambda status: status.message.startswith('token-file'), after=mark)
    bench.token_file.write_text(fake_daemon.OTHER_TOKEN, encoding='ascii')
    bench.daemon_status(lambda status: status.level == OK, after=collector.now())
    # A close the daemon should never send, a redirect, and the way back.
    mark = collector.now()
    fake.redirect('http://127.0.0.1:9', routes=('stream',))
    fake.close_stream(1008, 'the stream accepts no messages')
    bench.daemon_status(lambda status: status.message.startswith('not-a-daemon'),
                        after=mark)
    fake.redirect(None)
    bench.daemon_status(lambda status: status.level == OK, after=collector.now())
    collector.wait_for('joint_states', after=collector.now())
    bench.stop_node()

    captured = capfd.readouterr()
    records = list(debug_logs.records)
    logged = ' | '.join(f'{record.name} {record.levelname} {record.getMessage()} '
                        f'{record.args!r}' for record in records)
    ros_log = ' | '.join(f'{entry.name} {entry.msg} {entry.file} {entry.function}'
                         for entry in collector.received(ROSOUT))
    diagnostics = []
    for array in collector.received('/diagnostics'):
        for status in array.status:
            diagnostics += [status.name, status.message, status.hardware_id]
            for entry in status.values:
                diagnostics += [entry.key, entry.value]
    text = '\n'.join([captured.out, captured.err, logged, ros_log, ' | '.join(diagnostics)])

    # The check is not vacuous: the client logged at DEBUG, the node wrote those lines
    # to its own log, and every refusal went through the log and the diagnostics.
    ours = [record for record in records if record.name == 'abb_rws_bridge.daemon_client']
    assert {record.levelno for record in ours} >= {logging.DEBUG, logging.INFO}
    assert '[DEBUG]' in captured.err + captured.out
    assert bench.node_name in ros_log
    for word in ('token-refused', 'token-file', 'not-a-daemon', 'policy'):
        assert word in text, word
    assert str(bench.token_file) in ' | '.join(diagnostics)
    for secret in SECRETS:
        assert secret not in text
    for name, parameter in node.get_parameters_by_prefix('').items():
        for secret in SECRETS:
            assert secret not in name and secret not in str(parameter.value)
    # The bridge never sent a data frame, an Origin or another Host than the URL's own.
    assert fake.data_frames == 0
    assert all('origin' not in request.headers for request in fake.requests)
    assert all(request.headers['host'] == f'127.0.0.1:{fake.port}'
               for request in fake.requests)


def test_scenario_10_only_the_node_module_imports_rclpy_and_it_offers_nothing():
    folder = Path(bridge_node.__file__).resolve().parent
    importers = []
    for path in sorted(folder.glob('*.py')):
        tree = ast.parse(path.read_text(encoding='utf-8'))
        for entry in ast.walk(tree):
            modules = []
            if isinstance(entry, ast.Import):
                modules = [alias.name for alias in entry.names]
            elif isinstance(entry, ast.ImportFrom):
                modules = [entry.module or '']
            if any(module == 'rclpy' or module.startswith('rclpy.') for module in modules):
                importers.append(path.name)
    assert sorted(set(importers)) == ['bridge_node.py']
    source = inspect.getsource(bridge_node)
    for word in ('create_service', 'create_subscription', 'create_client', 'ActionServer',
                 'ActionClient', 'enableTrace', 'debuglevel'):
        assert word not in source, word


def test_scenario_10_the_node_keeps_out_of_the_names_rclpy_uses(ros_context):
    # An attribute of BridgeNode with a name rclpy's Node uses would break either one.
    plain = rclpy.create_node('plain', context=ros_context, use_global_arguments=False,
                              start_parameter_services=False)
    try:
        taken = set(vars(plain)) | set(dir(Node))
    finally:
        plain.destroy_node()
    tree = ast.parse(inspect.getsource(BridgeNode))
    own = set()
    for entry in ast.walk(tree):
        if isinstance(entry, ast.FunctionDef):
            own.add(entry.name)
        elif (isinstance(entry, ast.Attribute) and isinstance(entry.ctx, ast.Store)
              and isinstance(entry.value, ast.Name) and entry.value.id == 'self'):
            own.add(entry.attr)
    assert len(own) > 40
    assert own & taken == {'__init__', 'destroy_node'}


# ----- scenario 11: a daemon that stops answering --------------------------------------


def test_scenario_11_frozen_daemon_silences_everything_then_recovers(bench):
    stale_after_s = 1.0
    fake = bench.start_fake([io_controller('cell-1')])
    Advance(fake)
    collector = bench.collector
    topics = {'joint_states': JointState, 'controller_snapshot': ControllerSnapshot,
              'io/do_gripper': Bool}
    for topic, message_type in topics.items():
        collector.subscribe(topic, message_type)
    bench.start_node(primary_controller='cell-1', io_signals=['cell-1:do_gripper'],
                     stale_after_s=stale_after_s)
    for topic in topics:
        collector.wait_for(topic, 3)
    bench.daemon_status(lambda status: status.level == OK)

    # The daemon keeps its sockets open and says nothing any more.
    frozen = collector.now()
    fake.freeze()
    stale = bench.daemon_status(lambda status: status.level == STALE, after=frozen)
    assert stale.message.startswith('no answer from the daemon for')
    assert values_of(stale)['connected'] == 'true'
    failed = bench.daemon_status(
        lambda status: status.message.startswith('refresh-failed'), after=frozen)
    assert failed.level == ERROR
    levels = [status.level for status in collector.statuses(
        f'{bench.node_name}: daemon', after=frozen)]
    assert levels.index(STALE) < levels.index(ERROR)
    waiting = bench.controller_status('cell-1', lambda status: status.level == STALE,
                                      after=frozen)
    assert 'the daemon is not live' in waiting.message
    # Every data topic fell silent within stale_after_s plus one tick, not after the
    # 75 s of the stream's own silence rule.
    time.sleep(0.5)
    limit = stale_after_s + bridge_node.WATCH_TICK_S + SLACK_S
    for topic in topics:
        late = [arrived - frozen for arrived in collector.arrivals(topic, after=frozen)]
        assert all(delay <= limit for delay in late), (topic, late)
    quiet_from = collector.now()
    time.sleep(0.5)
    for topic in topics:
        assert collector.arrivals(topic, after=quiet_from) == [], topic

    fake.unfreeze()
    thawed = collector.now()
    bench.daemon_status(lambda status: status.level == OK, after=thawed, timeout=20.0)
    for topic in topics:
        collector.wait_for(topic, 2, after=thawed)


def test_scenario_11_a_stalled_stream_with_an_answering_daemon_is_found_and_ended(bench):
    # The socket stays open and the daemon answers every request, but no frame arrives
    # any more (a connection that is open in name only). The controller list shows a
    # value that no frame delivered: the data topics fall silent, the daemon status is
    # STALE, the connection is ended with its own reason, and the next one brings the
    # value.
    stale_after_s = 1.0
    fake = bench.start_fake([io_controller('cell-1')])
    Advance(fake)
    collector = bench.collector
    topics = {'joint_states': JointState, 'controller_snapshot': ControllerSnapshot,
              'io/do_gripper': Bool}
    for topic, message_type in topics.items():
        collector.subscribe(topic, message_type)
    bench.start_node(primary_controller='cell-1', io_signals=['cell-1:do_gripper'],
                     stale_after_s=stale_after_s)
    for topic in topics:
        collector.wait_for(topic, 3)
    assert collector.wait_for('io/do_gripper')[0].data is True
    bench.daemon_status(lambda status: status.level == OK)

    # The signal falls in the daemon; the list shows it, the stream says nothing.
    changed = collector.now()
    fake.update('cell-1', io=frames.io_group([frames.signal('do_gripper', 'DO', '0')]))
    stale = bench.daemon_status(lambda status: status.level == STALE, after=changed)
    assert stale.message.startswith('the stream has not delivered what the controller list')
    assert values_of(stale)['connected'] == 'true'
    assert values_of(stale)['stream behind for (s)'] != 'not behind'
    ended = bench.daemon_status(
        lambda status: status.message.startswith('stream-stalled'), after=changed)
    assert ended.level == ERROR
    assert 'open in name only' in ended.message and 'io' in values_of(ended)['detail']
    levels = [status.level for status in collector.statuses(
        f'{bench.node_name}: daemon', after=changed)]
    assert levels.index(STALE) < levels.index(ERROR)
    # The old value went out as current for no longer than the list needed to show the
    # change, plus the window of the daemon, plus stale_after_s and one tick.
    limit = (1.0 / FAST['refresh_hz'] + COALESCE_WINDOW_S + stale_after_s
             + bridge_node.WATCH_TICK_S + SLACK_S)
    old = [arrived - changed for arrived, message in zip(
        collector.arrivals('io/do_gripper', after=changed),
        collector.received('io/do_gripper', after=changed)) if message.data is True]
    assert all(delay <= limit for delay in old), old
    for topic in ('joint_states', 'controller_snapshot'):
        late = [arrived - changed for arrived in collector.arrivals(topic, after=changed)]
        assert all(delay <= limit for delay in late), (topic, late)

    # The new connection brings the value the stream owed.
    recovered = bench.daemon_status(lambda status: status.level == OK, after=collector.now(),
                                    timeout=20.0)
    assert int(values_of(recovered)['reconnects']) >= 1
    assert collector.wait_for('io/do_gripper', after=changed,
                              where=lambda message: message.data is False)
    for topic in ('joint_states', 'controller_snapshot'):
        collector.wait_for(topic, 2, after=collector.now())
    assert fake.consumer_closes[:1] == [1000]


def test_scenario_11_the_reconnect_bookkeeping_of_a_link_is_no_stall(bench):
    # The daemon counts its attempts to reach a controller that is down without a
    # frame: the list differs from what the stream sent, and that is no stall.
    down = frames.link_group(connected=False, quality='disconnected',
                             quality_reason='connection refused', subscriptions=False,
                             reconnect={'failures': 1, 'nextAttemptAt': frames.T0})
    fake = bench.start_fake([frames.controller('cell-1', link=down),
                             frames.controller('cell-2')])
    Advance(fake)
    collector = bench.collector
    collector.subscribe('cell_2/joint_states', JointState)
    bench.start_node(stale_after_s=0.5)
    collector.wait_for('cell_2/joint_states', 2)
    mark = collector.now()
    for failures in range(2, 6):
        fake.update('cell-1', link=dict(down, reconnect={
            'failures': failures, 'nextAttemptAt': fake.now()}))
        time.sleep(0.3)
    assert collector.wait_for('cell_2/joint_states', 2, after=collector.now())
    statuses = collector.statuses(f'{bench.node_name}: daemon', after=mark)
    assert statuses and all(status.level == OK for status in statuses)
    assert all(values_of(status)['stream behind for (s)'] == 'not behind'
               for status in statuses)
    assert values_of(statuses[-1])['reconnects'] == '0'
    assert bench.controller_status('cell-1', after=mark).level == ERROR


# ----- scenario 12: stamps never go backwards ------------------------------------------


def test_scenario_12_older_stamps_after_a_reconnect_are_held_back(bench):
    fake = bench.start_fake([frames.controller('cell-1')])
    now = Times(9000.0, 0.01)
    fake.now = now
    Advance(fake)
    collector = bench.collector
    collector.subscribe('joint_states', JointState)
    collector.subscribe('controller_snapshot', ControllerSnapshot)
    bench.start_node(primary_controller='cell-1', stamp_source='daemon')
    collector.wait_for('joint_states', 3)
    collector.wait_for('controller_snapshot', 3)

    # The daemon comes back with a clock that stepped back an hour.
    earlier = Times(9000.0 - 3600.0, 0.01)
    fake.now = earlier
    mark = collector.now()
    fake.close_stream(1001)
    held = bench.daemon_status(
        lambda status: int(values_of(status)['stamps held back']) >= 2, after=mark,
        timeout=20.0)
    assert held.level == WARN and 'held back' in held.message
    reconnected = collector.settled()
    time.sleep(0.5)
    for topic in ('joint_states', 'controller_snapshot'):
        assert collector.arrivals(topic, after=reconnected) == [], topic
        stamps = [stamp_ns(message.header.stamp) for message in collector.received(topic)]
        assert stamps == sorted(stamps), topic
        assert not set(stamps) & earlier.handed_ns(), topic
    # The topics are silent, and the status says the data itself is fine.
    assert bench.controller_status('cell-1', after=reconnected).level == OK

    # Time has caught up: publishing goes on where the stamps left off.
    later = Times(9000.0 + 3600.0, 0.01)
    fake.now = later
    caught_up = collector.now()
    for topic in ('joint_states', 'controller_snapshot'):
        message = collector.wait_for(topic, after=caught_up)[0]
        assert stamp_ns(message.header.stamp) in later.handed_ns()


# ----- scenario 13: the diagnostics conventions ----------------------------------------


def test_scenario_13_diagnostics_conventions(bench):
    fake = bench.start_fake([frames.controller('cell-1'), frames.controller('cell-2')])
    Advance(fake)
    collector = bench.collector
    period = 0.2
    wall_before = time.time_ns()
    node = bench.start_node(diagnostics_period_s=period)
    bench.controller_status('cell-2', lambda status: status.level == OK)
    collector.wait_for_node(bench.node_name)

    # The topic is the absolute /diagnostics, whatever the namespace of the node.
    published = dict(collector.publishers_of(bench.node_name))
    assert published['/diagnostics'] == ['diagnostic_msgs/msg/DiagnosticArray']
    assert f'{bench.namespace}/diagnostics' not in published
    assert node.get_name() == bench.node_name != bridge_node.NODE_NAME

    mark = collector.now()
    time.sleep(8 * period)
    wall_after = time.time_ns()
    names = [f'{bench.node_name}: daemon', f'{bench.node_name}: controller cell-1',
             f'{bench.node_name}: controller cell-2']
    arrays = [array for array in collector.received('/diagnostics', after=mark)
              if any(status.name in names for status in array.status)]
    # One array per period, each with every status, in a fixed order.
    assert 5 <= len(arrays) <= 11, len(arrays)
    for array in arrays:
        assert [status.name for status in array.status] == names
        # Stamped by the clock of the node.
        assert wall_before <= stamp_ns(array.header.stamp) <= wall_after
        daemon, first, second = array.status
        assert daemon.hardware_id == 'abb-rws-daemon'
        assert (first.hardware_id, second.hardware_id) == ('cell-1', 'cell-2')
        for status in array.status:
            assert isinstance(status.level, bytes) and len(status.level) == 1
            assert status.level in (OK, WARN, ERROR, STALE)
            assert status.message
            assert status.values
            for entry in status.values:
                assert isinstance(entry.key, str) and entry.key
                assert isinstance(entry.value, str) and entry.value
    stamps = [stamp_ns(array.header.stamp) for array in arrays]
    assert stamps == sorted(stamps)


def test_scenario_13_a_level_change_is_published_at_once(bench):
    fake = bench.start_fake([frames.controller('cell-1')])
    Advance(fake)
    # With a long period only a change of level can explain a prompt array.
    bench.start_node(primary_controller='cell-1', diagnostics_period_s=10.0)
    connected = bench.daemon_status(lambda status: status.level == OK, timeout=5.0)
    assert connected.message == 'live'
    mark = bench.collector.now()
    fake.close_stream(1001)
    fake.stop()
    down = bench.daemon_status(lambda status: status.level == ERROR, after=mark,
                               timeout=5.0)
    assert bench.collector.now() - mark < 5.0
    assert values_of(down)['connected'] == 'false'


# ----- scenario 14: QoS overrides ------------------------------------------------------


def test_scenario_14_a_qos_override_is_the_depth_of_the_publisher(bench):
    fake = bench.start_fake([frames.controller('cell-1')])
    Advance(fake)
    collector = bench.collector
    collector.subscribe('joint_states', JointState)
    parameter = f'qos_overrides.{collector.resolve("joint_states")}.publisher.depth'
    node = bench.start_node(primary_controller='cell-1', **{parameter: 3})
    collector.wait_for('joint_states')
    assert bench.publisher('joint_states').qos_profile.depth == 3
    assert node.get_parameter(parameter).value == 3
    assert node.describe_parameter(parameter).read_only
    # Every other publisher keeps the default: reliable, volatile, keep last 10.
    for topic in ('controller_snapshot', '/diagnostics'):
        profile = bench.publisher(topic).qos_profile
        assert profile.depth == bridge_node.QOS_DEPTH == 10, topic
        assert profile.reliability == QoSReliabilityPolicy.RELIABLE, topic
        assert profile.durability == QoSDurabilityPolicy.VOLATILE, topic
    assert bench.publisher('joint_states').qos_profile.durability == (
        QoSDurabilityPolicy.VOLATILE)


def test_scenario_14_republish_hz_zero_publishes_on_change_only(bench):
    fake = bench.start_fake([io_controller('cell-1')])
    collector = bench.collector
    collector.subscribe('controller_snapshot', ControllerSnapshot)
    collector.subscribe('io/do_gripper', Bool)
    bench.start_node(primary_controller='cell-1', io_signals=['cell-1:do_gripper'],
                     republish_hz=0.0)
    bench.controller_status(
        'cell-1', lambda status: values_of(status)['io lease'] == 'granted')
    bench.wait_matched('controller_snapshot')
    bench.wait_matched('io/do_gripper')
    # Nothing changes: no keep-alive, although the daemon is live.
    quiet_from = collector.settled()
    time.sleep(0.5)
    assert collector.arrivals('controller_snapshot', after=quiet_from) == []
    assert collector.arrivals('io/do_gripper', after=quiet_from) == []
    assert bench.daemon_now().level == OK

    mark = collector.now()
    fake.push(frames.delta_frame('cell-1', {'panel': frames.panel_group(op_mode='MANR')},
                                 observed_at=fake.now()))
    assert collector.wait_for('controller_snapshot', after=mark)[0].panel.op_mode == 'MANR'
    released = [frames.signal('do_gripper', 'DO', '0')]
    fake.push(frames.delta_frame('cell-1', {'io': frames.io_group(released)},
                                 observed_at=fake.now()))
    assert collector.wait_for('io/do_gripper', after=mark)[0].data is False
    time.sleep(0.4)
    assert len(collector.received('controller_snapshot', after=mark)) == 2
    assert len(collector.received('io/do_gripper', after=mark)) == 1


# ----- a snapshot that does not fit the messages ---------------------------------------


def test_a_snapshot_that_does_not_fit_the_messages_is_an_error_not_a_cut(bench):
    fake = bench.start_fake([frames.controller('cell-1')])
    Advance(fake)
    collector = bench.collector
    collector.subscribe('controller_snapshot', ControllerSnapshot)
    collector.subscribe('joint_states', JointState)
    bench.start_node(primary_controller='cell-1')
    collector.wait_for('controller_snapshot')

    # 25 watched signals where the message holds at most 24.
    many = [frames.signal(f'do_{index}', 'DO', '0') for index in range(25)]
    fake.push(frames.delta_frame('cell-1', {'io': frames.io_group(many)},
                                 observed_at=fake.now()))
    mark = collector.now()
    status = bench.controller_status('cell-1', lambda found: found.level == ERROR,
                                     after=mark)
    assert 'the snapshot does not fit the messages: ControllerSnapshot.io.signals' in (
        status.message)
    silent_from = collector.settled()
    collector.assert_silent('controller_snapshot', 0.5, after=silent_from)
    # The joints are not part of what did not fit.
    collector.wait_for('joint_states', after=silent_from)

    fake.push(frames.delta_frame('cell-1', {'io': frames.io_group(many[:24])},
                                 observed_at=fake.now()))
    fits = collector.now()
    snapshot = collector.wait_for('controller_snapshot', after=fits)[0]
    assert len(snapshot.io.signals) == 24
    bench.controller_status('cell-1', lambda found: found.level == OK, after=fits)


# ----- a fault of the bridge itself ----------------------------------------------------


def test_a_fault_in_one_turn_is_counted_and_the_bridge_goes_on(bench, capfd, monkeypatch):
    fake = bench.start_fake([frames.controller('cell-1')])
    Advance(fake)
    collector = bench.collector
    collector.subscribe('joint_states', JointState)
    node = bench.start_node(primary_controller='cell-1')
    collector.wait_for('joint_states')
    bench.daemon_status(lambda status: status.level == OK)

    def broken(view):
        raise RuntimeError('synthetic fault')

    with monkeypatch.context() as patch:
        patch.setattr(bridge_node.health, 'daemon_status', broken)
        assert wait_until(lambda: node.faults >= 2, 5.0), node.faults
        # What failed is the report; the data topics do not depend on it.
        collector.wait_for('joint_states', 2, after=collector.now())
    mended = collector.now()
    assert bench.daemon_status(after=mended).level == OK
    collector.wait_for('joint_states', after=mended)
    log = capfd.readouterr().err
    assert 'bridge fault while' in log and 'RuntimeError: synthetic fault' in log
    assert 'Traceback' not in log
    # The fault was the test's own doing: the bench checks for any other.
    node.faults = 0


# ----- parameters that cannot be used (the process test checks the exit code) ----------


@pytest.mark.parametrize('values, expected', [
    ({'refresh_hz': 50.0},
     ['refresh_hz: 50.0 was given', 'a number from 0.2 to 10']),
    ({'refresh_hz': 2},
     ['refresh_hz: 2 (a whole number) was given', 'write 2.0, not 2']),
    ({'stale_after_s': 0.5, 'refresh_hz': 2.0},
     ['stale_after_s: 0.5 is below 2 / refresh_hz']),
    ({'daemon_url': 'http://robot-cell.example:7810'},
     ['daemon_url: the host "robot-cell.example" is not this machine',
      'allow_remote_daemon']),
    ({'use_sim_time': True},
     ['use_sim_time: simulated time is not supported']),
    ({'allow_replay': 'yes'},
     ['allow_replay: "yes" (a single text) was given, but it must be true or false']),
    ({'controllers': 'cell-1'},
     ['controllers: "cell-1" (a single text) was given, but it must be a list of texts '
      '(write ["cell-1"])']),
    # What a parameter file or a launch argument makes of an id such as y, or of 1e3.
    ({'controllers': [True]},
     ['controllers: a list of true or false values was given, but it must be a list of '
      'texts (put every entry in double quotes)']),
    ({'io_signals': [8.0]},
     ['io_signals: a list of numbers was given, but it must be a list of texts '
      '(put every entry in double quotes)']),
    ({'token_file': 1000.0},
     ['token_file: 1000.0 was given, but it must be text (put it in double quotes)']),
    ({'daemon_url': 7810},
     ['daemon_url: 7810 (a whole number) was given, but it must be text '
      '(put it in double quotes)']),
    ({'daemon_url': 'https://127.0.0.1:7810'}, ['daemon_url: https is not possible']),
    ({'stamp_source': 'robot'}, ['stamp_source: "robot" is not one of']),
    ({'reconnect_min_s': 5.0, 'reconnect_max_s': 2.0},
     ['reconnect_min_s: 5 is above reconnect_max_s']),
    ({'primary_controller': 'cell-1', 'controllers': ['cell-2']},
     ['primary_controller: "cell-1" is not in controllers']),
    ({'joint_name_format': '{robot}_{joint}'}, ['joint_name_format:', '{robot}']),
    ({'io_signals': ['cell-1']}, ['io_signals: entry "cell-1" is not of the form']),
    ({'controllers': ['cell-1'], 'io_signals': ['cell-2:do_x']},
     ['io_signals: an entry names the controller "cell-2", which is not in controllers']),
    ({'controllers': ['cell-1'], 'joint_name_overrides': ['cell-2/ROB_1/J1=a']},
     ['joint_name_overrides: an entry names the controller "cell-2"']),
])
def test_scenario_15_a_parameter_that_cannot_be_used_stops_the_node(bench, values, expected):
    with pytest.raises(StartProblem) as raised:
        bench.make_node(**values)
    problems = raised.value.problems
    assert len(problems) == 1, problems
    for text in expected:
        assert text in problems[0]
    assert '\n' not in problems[0]
    # No half-made node is left on the graph.
    time.sleep(0.2)
    assert (bench.node_name, bench.namespace) not in (
        bench.collector.node.get_node_names_and_namespaces())


def test_scenario_15_every_problem_is_reported_not_only_the_first(bench):
    with pytest.raises(StartProblem) as raised:
        bench.make_node(refresh_hz=50.0, http_timeout_s=3, allow_replay='yes')
    assert [problem.split(':')[0] for problem in raised.value.problems] == [
        'allow_replay', 'refresh_hz', 'http_timeout_s']
    with pytest.raises(StartProblem) as raised:
        bench.make_node(stamp_source='robot', stale_after_s=0.5, refresh_hz=1.0)
    assert sorted(problem.split(':')[0] for problem in raised.value.problems) == [
        'stale_after_s', 'stamp_source']


def test_scenario_15_a_remote_daemon_is_accepted_only_when_asked_for(bench, capfd):
    # Nothing listens there: the node starts, warns once and reports "unreachable".
    bench.start_node(daemon_url='http://localhost:9', allow_remote_daemon=True)
    bench.daemon_status(lambda status: status.message.startswith('unreachable'))
    bench.stop_node()
    assert 'allow_remote_daemon' not in capfd.readouterr().err
    # A loopback address that is not one of the trusted names: nothing leaves this host.
    bench.make_node(daemon_url='http://127.0.0.2:9', allow_remote_daemon=True).destroy_node()
    lines = [line for line in capfd.readouterr().err.splitlines()
             if 'allow_remote_daemon' in line]
    assert len(lines) == 1 and '[WARN]' in lines[0]
    assert 'travels unencrypted' in lines[0]

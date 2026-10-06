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
ROS graph helpers of the node tests: a collector node and the fixtures around it.

A test module loads this file as a pytest plugin (``pytest_plugins = ('ros_helpers',)``)
and then asks for the fixtures by name:

- ``ros_context``: ROS started once for the session, in the domain the ``ros_domain``
  fixture took for it;
- ``ros_namespace``: a namespace no other test uses;
- ``collector``: a ``Collector`` node in that namespace.

So a test is hermetic by itself: its own ROS domain for the session and its own
namespace per test. Nothing here relies on how the container was started.

A ``Collector`` subscribes, waits for messages with a timeout and reads the graph: the
topics with their types, and the publishers, subscriptions and services of a node. It
spins in a thread of its own, as ``Spinner`` does for any other node of a test.
"""

import threading
import time
import uuid

from diagnostic_msgs.msg import DiagnosticArray
import pytest
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.signals import SignalHandlerOptions

DIAGNOSTICS_TOPIC = '/diagnostics'
ROSOUT_TOPIC = '/rosout'
DEFAULT_TIMEOUT_S = 10.0

_STEP_S = 0.005
_SPIN_S = 0.05


@pytest.fixture(scope='session')
def ros_context(ros_domain):
    """Start ROS once for the test session, in the session's own domain."""
    context = rclpy.Context()
    rclpy.init(args=[], context=context, domain_id=ros_domain,
               signal_handler_options=SignalHandlerOptions.NO)
    try:
        yield context
    finally:
        rclpy.try_shutdown(context=context)


@pytest.fixture
def ros_namespace():
    """Return a namespace no other test uses."""
    return '/t_' + uuid.uuid4().hex[:12]


@pytest.fixture
def collector(ros_context, ros_namespace):
    """Give the test a collector node in its own namespace."""
    made = Collector(ros_context, ros_namespace)
    try:
        yield made
    finally:
        made.close()


def wait_until(predicate, timeout=DEFAULT_TIMEOUT_S, step=_STEP_S):
    """Poll predicate until it is true; return whether it became true in time."""
    deadline = time.monotonic() + timeout
    while True:
        if predicate():
            return True
        if time.monotonic() >= deadline:
            return False
        time.sleep(step)


def stamp_ns(stamp):
    """Return a ``builtin_interfaces/Time`` as nanoseconds."""
    return stamp.sec * 1000000000 + stamp.nanosec


class Spinner:
    """Spins one node in a thread of its own until ``stop()``."""

    def __init__(self, node, context):
        self.node = node
        self._context = context
        self._stopping = threading.Event()
        self._executor = SingleThreadedExecutor(context=context)
        self._executor.add_node(node)
        self.failure = None
        self._thread = threading.Thread(target=self._run, name='test-spinner', daemon=True)
        self._thread.start()

    def _run(self):
        try:
            while not self._stopping.is_set() and self._context.ok():
                self._executor.spin_once(timeout_sec=_SPIN_S)
        except ExternalShutdownException:
            pass
        except Exception as error:  # shown by the test that owns the node
            self.failure = error

    def stop(self):
        """Stop spinning; the node is then safe to destroy."""
        self._stopping.set()
        self._thread.join(5.0)
        self._executor.remove_node(self.node)
        self._executor.shutdown(timeout_sec=1.0)
        assert not self._thread.is_alive(), 'the spinner thread did not end'
        assert self.failure is None, f'a callback of the node failed: {self.failure!r}'


class Collector:
    """
    A node that records what it is sent, and reads the ROS graph.

    Topic names are relative to the test's namespace unless they start with ``/``.
    Every message is kept with the monotonic time it arrived at, so a test can ask for
    what came after a moment (``now()``) and for how long a topic was silent.
    """

    def __init__(self, context, namespace):
        self.context = context
        self.namespace = namespace
        self.node = rclpy.create_node(
            'collector', context=context, namespace=namespace, use_global_arguments=False,
            start_parameter_services=False)
        self._condition = threading.Condition()
        self._received = {}
        self._subscriptions = {}
        self._spinner = Spinner(self.node, context)

    def close(self):
        """Stop spinning and destroy the node."""
        self._spinner.stop()
        self.node.destroy_node()

    def spin(self, node):
        """Spin another node of the test in a thread of its own; return its Spinner."""
        return Spinner(node, self.context)

    @staticmethod
    def now():
        """Return the monotonic time, the clock arrivals are measured on."""
        return time.monotonic()

    @staticmethod
    def settled(seconds=0.3):
        """
        Let what is already on its way arrive; return the moment after that.

        A message that was published just before something ended can still be
        delivered a moment later. Silence is judged from the moment this returns.
        """
        time.sleep(seconds)
        return time.monotonic()

    # ----- topics -----------------------------------------------------------------------

    def resolve(self, topic):
        """Return the full name of a topic of this test."""
        return self.node.resolve_topic_name(topic)

    def subscribe(self, topic, message_type, depth=200):
        """Start recording a topic (reliable, volatile); return its full name."""
        name = self.resolve(topic)
        if name in self._subscriptions:
            return name
        with self._condition:
            self._received[name] = []
        qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=depth,
                         reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.VOLATILE)

        def record(message):
            with self._condition:
                self._received[name].append((time.monotonic(), message))
                self._condition.notify_all()

        self._subscriptions[name] = self.node.create_subscription(
            message_type, name, record, qos)
        return name

    def received(self, topic, after=None, where=None):
        """Return the messages of a topic that arrived after a moment and fit where."""
        name = self.resolve(topic)
        with self._condition:
            entries = list(self._received[name])
        return [message for arrived, message in entries
                if (after is None or arrived > after) and (where is None or where(message))]

    def arrivals(self, topic, after=None):
        """Return the monotonic arrival times of the messages of a topic."""
        name = self.resolve(topic)
        with self._condition:
            return [arrived for arrived, _ in self._received[name]
                    if after is None or arrived > after]

    def wait_for(self, topic, count=1, *, after=None, where=None,
                 timeout=DEFAULT_TIMEOUT_S):
        """
        Return the first count messages that arrived after a moment and fit where.

        Fails the test when fewer than count arrive within timeout seconds.
        """
        name = self.resolve(topic)
        deadline = time.monotonic() + timeout
        with self._condition:
            while True:
                found = [message for arrived, message in self._received[name]
                         if (after is None or arrived > after)
                         and (where is None or where(message))]
                if len(found) >= count:
                    return found[:count]
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    total = len(self._received[name])
                    raise AssertionError(
                        f'{name}: {len(found)} of {count} expected messages within '
                        f'{timeout:g} s ({total} received on the topic in all)')
                self._condition.wait(min(remaining, 0.1))

    def assert_silent(self, topic, duration, after=None):
        """Wait duration seconds and fail when the topic said anything after a moment."""
        start = time.monotonic() if after is None else after
        time.sleep(duration)
        late = self.arrivals(topic, after=start)
        assert not late, f'{self.resolve(topic)}: {len(late)} messages where none may come'

    # ----- diagnostics ------------------------------------------------------------------

    def watch_diagnostics(self):
        """Start recording ``/diagnostics``."""
        return self.subscribe(DIAGNOSTICS_TOPIC, DiagnosticArray)

    def statuses(self, name, after=None):
        """Return every status of this name on ``/diagnostics`` since a moment, in order."""
        return [status for array in self.received(DIAGNOSTICS_TOPIC, after=after)
                for status in array.status if status.name == name]

    def wait_status(self, name, where=None, *, after=None, timeout=DEFAULT_TIMEOUT_S):
        """Return the first status of this name, since a moment, that fits where."""
        found = []

        def look():
            found[:] = [status for status in self.statuses(name, after=after)
                        if where is None or where(status)]
            return bool(found)

        if not wait_until(look, timeout):
            seen = [(status.level, status.message) for status in self.statuses(name)][-3:]
            raise AssertionError(
                f'no status "{name}" as expected within {timeout:g} s; the last ones '
                f'were {seen}')
        return found[0]

    # ----- the graph --------------------------------------------------------------------

    def topic_types(self):
        """Return {full topic name: its types} as the graph has them now."""
        return {name: list(types) for name, types in self.node.get_topic_names_and_types()}

    def publisher_count(self, topic):
        """Return how many publishers a topic has."""
        return self.node.count_publishers(self.resolve(topic))

    def wait_for_publishers(self, topic, count=1, timeout=DEFAULT_TIMEOUT_S):
        """Wait until a topic has exactly count publishers; fail the test otherwise."""
        if not wait_until(lambda: self.publisher_count(topic) == count, timeout):
            raise AssertionError(
                f'{self.resolve(topic)}: {self.publisher_count(topic)} publishers, not '
                f'{count}, after {timeout:g} s')

    def wait_for_node(self, node_name, namespace=None, timeout=DEFAULT_TIMEOUT_S):
        """Wait until a node is on the graph; fail the test otherwise."""
        wanted = (node_name, self.namespace if namespace is None else namespace)
        if not wait_until(lambda: wanted in self.node.get_node_names_and_namespaces(),
                          timeout):
            raise AssertionError(f'node {wanted} is not on the graph after {timeout:g} s')

    def services_of(self, node_name, namespace=None):
        """Return [(service name, its types)] of a node."""
        return self.node.get_service_names_and_types_by_node(
            node_name, self.namespace if namespace is None else namespace)

    def subscriptions_of(self, node_name, namespace=None):
        """Return [(topic name, its types)] a node subscribes to."""
        return self.node.get_subscriber_names_and_types_by_node(
            node_name, self.namespace if namespace is None else namespace)

    def publishers_of(self, node_name, namespace=None):
        """Return [(topic name, its types)] a node publishes."""
        return self.node.get_publisher_names_and_types_by_node(
            node_name, self.namespace if namespace is None else namespace)

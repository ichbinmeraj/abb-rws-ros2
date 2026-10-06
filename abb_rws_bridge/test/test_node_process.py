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
The installed ``bridge`` executable, run as a child process (scenario 15 of the node tests).

These tests start what a user starts: the executable of the installed package, with a
parameter file. They cover how the process begins and ends: a parameter that cannot be
used gives exit code 2 and one readable line, and SIGINT or SIGTERM end a running bridge
with exit code 0. The package must be built and its install sourced; the tests of the
node's behaviour (``test_node_integration.py``) need no install.

Every child gets a node name and a namespace of its own and inherits the ROS domain of
the session. The daemon is the stand-in of the tests, on 127.0.0.1.
"""

import json
import os
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import PackageNotFoundError
from diagnostic_msgs.msg import DiagnosticStatus
import fake_daemon
from fake_daemon import wait_until
import frames
import pytest
from sensor_msgs.msg import JointState

pytest_plugins = ('ros_helpers',)

PACKAGE = 'abb_rws_bridge'
EXECUTABLE = 'bridge'
# How long a child may take to start and to say why it cannot (a loaded machine).
START_TIMEOUT_S = 30.0
# A running bridge ends within this long after SIGINT or SIGTERM.
STOP_LIMIT_S = 5.0
SECRETS = (fake_daemon.TOKEN, fake_daemon.TOKEN[:16])


def bridge_executable():
    """Return the installed executable; fail the test when the package is not installed."""
    try:
        prefix = get_package_prefix(PACKAGE)
    except PackageNotFoundError:
        pytest.fail(f'{PACKAGE} is not installed: build it and source its install first')
    path = Path(prefix) / 'lib' / PACKAGE / EXECUTABLE
    if not path.is_file():
        pytest.fail(f'{path} does not exist: build {PACKAGE} first')
    return str(path)


class Child:
    """One run of the installed executable, with its output kept in files."""

    def __init__(self, folder, namespace, values, extra=()):
        self.node_name = 'bridge_' + namespace[3:]
        self.namespace = namespace
        self._folder = folder
        params_file = folder / 'bridge.yaml'
        lines = ['/**:', '  ros__parameters:']
        # JSON scalars and lists are YAML too: text in quotes, 2 an integer, 2.0 a double.
        lines += [f'    {name}: {json.dumps(value)}' for name, value in values.items()]
        params_file.write_text('\n'.join(lines) + '\n', encoding='utf-8')
        command = [bridge_executable(), '--ros-args', '--params-file', str(params_file),
                   '-r', f'__ns:={namespace}', '-r', f'__node:={self.node_name}', *extra]
        environment = dict(os.environ)
        self.log_folder = folder / 'log'
        environment['ROS_LOG_DIR'] = str(self.log_folder)
        self._out = open(folder / 'out.txt', 'wb')
        self._err = open(folder / 'err.txt', 'wb')
        self.process = subprocess.Popen(
            command, cwd=folder, env=environment, stdin=subprocess.DEVNULL,
            stdout=self._out, stderr=self._err)

    def wait(self, timeout):
        """Return the exit code; fail the test when the child is still running."""
        try:
            return self.process.wait(timeout)
        except subprocess.TimeoutExpired:
            self.close()
            pytest.fail(f'the bridge did not end within {timeout:g} s:\n{self.output()}')

    def output(self):
        """Return everything the child wrote so far (stdout, then stderr)."""
        self._out.flush()
        self._err.flush()
        return ''.join((self._folder / name).read_text(encoding='utf-8', errors='replace')
                       for name in ('out.txt', 'err.txt'))

    def close(self):
        """End the child if it still runs and close its output files."""
        if self.process.poll() is None:
            self.process.kill()
            self.process.wait(10.0)
        self._out.close()
        self._err.close()


@pytest.fixture
def run_bridge(ros_domain, ros_namespace, tmp_path):
    """Return a function that starts the executable; every child is ended afterwards."""
    children = []

    def start(values, extra=()):
        child = Child(tmp_path, ros_namespace, values, extra)
        children.append(child)
        return child

    try:
        yield start
    finally:
        for child in children:
            child.close()


@pytest.fixture
def daemon():
    """Give the test a stand-in daemon with one robot."""
    with fake_daemon.FakeDaemon([frames.controller('cell-1')]) as fake:
        # A robot that stands still while the daemon keeps reading it.
        def keep_reading(stand_in):
            stand_in.update('cell-1', motion=frames.motion_group(as_of=stand_in.now()))

        fake.on_controller_list = keep_reading
        yield fake


@pytest.fixture
def daemon_parameters(daemon, tmp_path):
    """Return the parameters of a bridge for the stand-in daemon, with short times."""
    token_file = tmp_path / 'token'
    token_file.write_text(fake_daemon.TOKEN + '\n', encoding='ascii')
    return {'daemon_url': daemon.url, 'token_file': str(token_file), 'refresh_hz': 10.0,
            'stale_after_s': 0.5, 'republish_hz': 10.0, 'diagnostics_period_s': 0.2,
            'reconnect_min_s': 0.1, 'reconnect_max_s': 1.0}


def start_problem_lines(child):
    """Return the lines with which a child said that it cannot start."""
    return [line for line in child.output().splitlines() if 'cannot start' in line]


@pytest.mark.parametrize('values, expected', [
    ({'refresh_hz': 50.0},
     ['refresh_hz: 50.0 was given', 'a number from 0.2 to 10']),
    ({'refresh_hz': 2},
     ['refresh_hz: 2 (a whole number) was given', 'write 2.0, not 2']),
    ({'stale_after_s': 0.5, 'refresh_hz': 2.0},
     ['stale_after_s: 0.5 is below 2 / refresh_hz (1 with refresh_hz 2)']),
    ({'daemon_url': 'http://robot-cell.example:7810'},
     ['daemon_url: the host "robot-cell.example" is not this machine',
      'allow_remote_daemon']),
    ({'use_sim_time': True},
     ['use_sim_time: simulated time is not supported']),
], ids=['out-of-range', 'integer-for-double', 'stale-below-two-refreshes', 'remote-host',
        'sim-time'])
def test_scenario_15_a_start_problem_is_one_readable_line_and_exit_code_2(
        run_bridge, values, expected):
    child = run_bridge(values)
    assert child.wait(START_TIMEOUT_S) == 2
    output = child.output()
    lines = start_problem_lines(child)
    assert len(lines) == 1, output
    # The one line is all the bridge says, under the name the node was given.
    assert [line for line in output.splitlines() if line.strip()] == lines
    assert lines[0].startswith('[FATAL]')
    assert f'[{child.namespace[1:]}.{child.node_name}]: cannot start: ' in lines[0]
    for text in expected:
        assert text in lines[0]
    assert 'Traceback' not in output


def test_scenario_15_every_start_problem_gets_its_own_line(run_bridge):
    child = run_bridge({'refresh_hz': 2, 'allow_replay': 'yes',
                        'controllers': 'cell-1'})
    assert child.wait(START_TIMEOUT_S) == 2
    lines = start_problem_lines(child)
    assert len(lines) == 3, child.output()
    assert 'controllers: "cell-1" (a single text) was given' in lines[0]
    assert 'allow_replay: "yes" (a single text) was given' in lines[1]
    assert 'refresh_hz: 2 (a whole number) was given' in lines[2]
    assert 'Traceback' not in child.output()


@pytest.mark.parametrize('stop_signal', [signal.SIGINT, signal.SIGTERM],
                         ids=['SIGINT', 'SIGTERM'])
def test_scenario_15_an_empty_controllers_list_starts_and_a_signal_ends_it_with_0(
        run_bridge, daemon, daemon_parameters, collector, stop_signal):
    collector.watch_diagnostics()
    collector.subscribe('cell_1/joint_states', JointState)
    # "controllers: []" in a parameter file: every controller the daemon serves.
    logger = f'{collector.namespace[1:]}.bridge_{collector.namespace[3:]}'
    child = run_bridge(dict(daemon_parameters, controllers=[], io_signals=[],
                            joint_name_overrides=[]),
                       extra=['--log-level', f'{logger}:=debug'])
    name = f'{child.node_name}: daemon'
    status = collector.wait_status(
        name, lambda found: found.level == DiagnosticStatus.OK, timeout=START_TIMEOUT_S)
    assert status.message == 'live'
    state = collector.wait_for('cell_1/joint_states', timeout=START_TIMEOUT_S)[0]
    assert list(state.name) == [f'ROB_1_J{axis}' for axis in range(1, 7)]
    collector.wait_for_node(child.node_name)
    for _, types in collector.services_of(child.node_name):
        assert all(service_type.split('/')[0] in (
            'rcl_interfaces', 'type_description_interfaces') for service_type in types)
    assert collector.subscriptions_of(child.node_name) == []
    assert child.process.poll() is None

    sent = time.monotonic()
    child.process.send_signal(stop_signal)
    assert child.wait(STOP_LIMIT_S) == 0, child.output()
    assert time.monotonic() - sent < STOP_LIMIT_S
    # The stream was closed in good order, by the bridge.
    assert wait_until(lambda: daemon.consumer_closes == [1000], 3.0), daemon.consumer_closes
    output = child.output()
    assert 'Traceback' not in output and 'cannot start' not in output
    # At DEBUG the lines of the daemon client are in the log of the node, and the token
    # is in none of them.
    assert '[DEBUG]' in output and 'connected to the daemon stream' in output
    for secret in SECRETS:
        assert secret not in output
    for path in child.log_folder.rglob('*'):
        if path.is_file():
            text = path.read_text(encoding='utf-8', errors='replace')
            assert not any(secret in text for secret in SECRETS), path.name


def test_scenario_15_a_daemon_that_is_not_there_is_no_start_problem(
        run_bridge, daemon, daemon_parameters, collector):
    collector.watch_diagnostics()
    daemon.stop()
    child = run_bridge(daemon_parameters)
    status = collector.wait_status(
        f'{child.node_name}: daemon',
        lambda found: found.message.startswith('unreachable'), timeout=START_TIMEOUT_S)
    assert status.level == DiagnosticStatus.ERROR
    assert child.process.poll() is None
    child.process.send_signal(signal.SIGINT)
    assert child.wait(STOP_LIMIT_S) == 0
    assert 'Traceback' not in child.output()

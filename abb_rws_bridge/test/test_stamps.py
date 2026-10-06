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

"""The stamp source by daemon host, the stamp of a value, and the gate of a publisher."""

from abb_rws_bridge import stamps
from abb_rws_bridge import timeconv
import frames
import pytest

T0_SEC = 1767225600


def test_the_local_hosts_are_the_three_loopback_names():
    assert stamps.LOCAL_HOSTS == frozenset({'127.0.0.1', 'localhost', '::1'})
    assert isinstance(stamps.LOCAL_HOSTS, frozenset)


@pytest.mark.parametrize('host', ['127.0.0.1', 'localhost', '::1'])
def test_auto_uses_the_daemon_clock_on_the_same_machine(host):
    assert stamps.resolve_source('auto', host) == 'daemon'


@pytest.mark.parametrize('host', ['LOCALHOST', 'LocalHost', '[::1]', ' localhost '])
def test_auto_reads_a_local_host_whatever_its_spelling(host):
    assert stamps.resolve_source('auto', host) == 'daemon'


@pytest.mark.parametrize('host', [
    'host.docker.internal',
    'host.containers.internal',
    'daemon.example.com',
    'localhost.example.com',
    'localhost.',
    'localhost:7810',
    'ip6-localhost',
    '127.1',
    '',
])
def test_auto_uses_the_bridge_clock_for_every_other_host(host):
    assert stamps.resolve_source('auto', host) == 'bridge'


@pytest.mark.parametrize('source', ['daemon', 'bridge'])
@pytest.mark.parametrize('host', ['127.0.0.1', 'host.docker.internal', 'daemon.example.com', ''])
def test_an_explicit_source_is_kept_for_any_host(source, host):
    assert stamps.resolve_source(source, host) == source


@pytest.mark.parametrize('source', ['', 'Auto', 'DAEMON', 'Bridge', 'ros', 'sim', 'system', None,
                                    0, True])
def test_another_stamp_source_is_a_value_error(source):
    with pytest.raises(ValueError):
        stamps.resolve_source(source, '127.0.0.1')


@pytest.mark.parametrize('host', [None, 7810, b'localhost'])
def test_auto_needs_a_host_that_is_text(host):
    with pytest.raises(ValueError):
        stamps.resolve_source('auto', host)


def test_the_daemon_source_stamps_with_the_daemon_time():
    assert stamps.stamp_of('daemon', frames.T0, 5) == (T0_SEC, 0)
    assert stamps.stamp_of('daemon', frames.iso(1.25), 5) == (T0_SEC + 1, 250000000)
    assert stamps.stamp_of('daemon', frames.iso(0.5), None) == (T0_SEC, 500000000)


def test_the_bridge_source_stamps_with_the_arrival_time():
    arrival = (T0_SEC + 7) * timeconv.NANOSEC_PER_SEC + 123456789
    assert stamps.stamp_of('bridge', frames.T0, arrival) == (T0_SEC + 7, 123456789)
    assert stamps.stamp_of('bridge', None, arrival) == (T0_SEC + 7, 123456789)
    assert stamps.stamp_of('bridge', 'not a time', 0) == (0, 0)
    assert stamps.stamp_of('bridge', None, 999999999) == (0, 999999999)
    last = timeconv.MAX_SEC * timeconv.NANOSEC_PER_SEC + 999999999
    assert stamps.stamp_of('bridge', None, last) == (timeconv.MAX_SEC, 999999999)


@pytest.mark.parametrize('daemon_iso', [None, '', 'not a time', '2026-01-01', 1767225600,
                                        '1969-12-31T23:59:59Z', '2040-01-01T00:00:00Z'])
def test_a_daemon_time_that_cannot_be_read_gives_no_stamp(daemon_iso):
    with pytest.raises(ValueError):
        stamps.stamp_of('daemon', daemon_iso, T0_SEC * timeconv.NANOSEC_PER_SEC)


@pytest.mark.parametrize('arrival', [
    None, -1, 1.5, 1767225600.0, '1767225600000000000', True,
    (timeconv.MAX_SEC + 1) * timeconv.NANOSEC_PER_SEC,
])
def test_an_arrival_time_that_is_no_ros_time_gives_no_stamp(arrival):
    with pytest.raises(ValueError):
        stamps.stamp_of('bridge', frames.T0, arrival)


@pytest.mark.parametrize('source', ['auto', '', 'Daemon', 'BRIDGE', None])
def test_stamp_of_needs_a_resolved_source(source):
    with pytest.raises(ValueError):
        stamps.stamp_of(source, frames.T0, T0_SEC * timeconv.NANOSEC_PER_SEC)


def test_a_new_gate_admits_the_first_stamp():
    gate = stamps.StampGate()
    assert gate.held_back == 0
    assert gate.last is None
    assert gate.admit((10, 0), allow_equal=False) is True
    assert gate.last == (10, 0)
    assert gate.held_back == 0


def test_a_gate_admits_newer_stamps():
    gate = stamps.StampGate()
    for stamp in [(10, 0), (10, 1), (10, 999999999), (11, 0), (2000, 5)]:
        assert gate.admit(stamp, allow_equal=False) is True
        assert gate.last == stamp
    assert gate.held_back == 0


def test_a_gate_holds_back_an_older_stamp_and_counts_it():
    gate = stamps.StampGate()
    assert gate.admit((10, 500), allow_equal=True) is True
    assert gate.admit((10, 499), allow_equal=True) is False
    assert gate.held_back == 1
    assert gate.admit((9, 999999999), allow_equal=False) is False
    assert gate.held_back == 2
    assert gate.admit((0, 0), allow_equal=True) is False
    assert gate.held_back == 3
    # What was held back did not move the gate.
    assert gate.last == (10, 500)
    assert gate.admit((10, 501), allow_equal=False) is True
    assert gate.held_back == 3


def test_seconds_are_compared_before_nanoseconds():
    gate = stamps.StampGate()
    assert gate.admit((1, 999999999), allow_equal=False) is True
    assert gate.admit((2, 0), allow_equal=False) is True
    assert gate.admit((1, 999999999), allow_equal=False) is False
    assert gate.held_back == 1


def test_an_equal_stamp_passes_only_when_allowed_and_is_never_counted():
    gate = stamps.StampGate()
    assert gate.admit((10, 0), allow_equal=False) is True
    # A snapshot keep-alive repeats the stamp of the state it repeats.
    assert gate.admit((10, 0), allow_equal=True) is True
    assert gate.admit((10, 0), allow_equal=True) is True
    # A joint state with the same stamp is simply not sent again.
    assert gate.admit((10, 0), allow_equal=False) is False
    assert gate.held_back == 0
    assert gate.last == (10, 0)


def test_reset_lets_time_start_again_and_keeps_the_count():
    gate = stamps.StampGate()
    assert gate.admit((100, 0), allow_equal=False) is True
    assert gate.admit((50, 0), allow_equal=False) is False
    assert gate.held_back == 1
    gate.reset()
    assert gate.last is None
    assert gate.held_back == 1
    assert gate.admit((50, 0), allow_equal=False) is True
    assert gate.admit((49, 0), allow_equal=False) is False
    assert gate.held_back == 2


def test_gates_are_independent():
    first, second = stamps.StampGate(), stamps.StampGate()
    assert first.admit((100, 0), allow_equal=False) is True
    assert second.admit((1, 0), allow_equal=False) is True
    assert first.admit((1, 0), allow_equal=False) is False
    assert (first.held_back, second.held_back) == (1, 0)


def test_a_gate_takes_a_list_as_well_as_a_tuple():
    gate = stamps.StampGate()
    assert gate.admit([10, 0], allow_equal=False) is True
    assert gate.last == (10, 0)
    assert gate.admit((10, 0), allow_equal=True) is True


@pytest.mark.parametrize('stamp', [
    None, 10, (10,), (10, 0, 0), ('10', '0'), (10.0, 0), (10, 0.5), (True, 0), (-1, 0),
    (10, -1), (10, 1000000000), '10',
])
def test_a_gate_refuses_what_is_not_a_stamp(stamp):
    gate = stamps.StampGate()
    with pytest.raises(ValueError):
        gate.admit(stamp, allow_equal=True)
    assert gate.held_back == 0
    assert gate.last is None


def test_allow_equal_must_be_named():
    gate = stamps.StampGate()
    with pytest.raises(TypeError):
        gate.admit((10, 0), True)
    with pytest.raises(TypeError):
        gate.admit((10, 0))


def test_stamps_from_stamp_of_pass_a_gate_in_daemon_order():
    gate = stamps.StampGate()
    admitted = [gate.admit(stamps.stamp_of('daemon', frames.iso(offset), 0), allow_equal=False)
                for offset in (0.0, 0.125, 0.25, 0.125, 0.25, 0.375)]
    assert admitted == [True, True, True, False, False, True]
    assert gate.held_back == 1

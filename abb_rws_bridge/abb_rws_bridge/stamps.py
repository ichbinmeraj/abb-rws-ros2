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
The one place that decides a ``header.stamp``.

Two clocks can stamp what the bridge publishes: the daemon's (the times it reports,
``asOf`` and ``observedAt``) and the bridge's own (the system clock when a frame or an
answer arrived). ``resolve_source`` picks one for the whole run, ``stamp_of`` turns the
pick into seconds and nanoseconds, and one ``StampGate`` per publisher keeps the stamps
of a topic from going backwards. Pure: nothing from ROS is imported.
"""

from abb_rws_bridge import timeconv

# Daemon hosts that are this machine, so the daemon's clock is the bridge's clock.
LOCAL_HOSTS = frozenset({'127.0.0.1', 'localhost', '::1'})

AUTO = 'auto'
DAEMON = 'daemon'
BRIDGE = 'bridge'


def resolve_source(stamp_source: str, daemon_host: str) -> str:
    """
    Return the clock that stamps messages: ``'daemon'`` or ``'bridge'``.

    ``'daemon'`` and ``'bridge'`` are returned as given. ``'auto'`` gives ``'daemon'``
    when the daemon host is one of LOCAL_HOSTS (the same machine, the same clock) and
    ``'bridge'`` for every other host. The host is compared without regard to letter
    case, and ``[::1]`` is read as ``::1``.

    Raises ValueError for any other stamp_source, and for ``'auto'`` with a host that
    is not text.
    """
    if stamp_source in (DAEMON, BRIDGE):
        return stamp_source
    if stamp_source != AUTO:
        raise ValueError('stamp_source must be auto, daemon or bridge')
    if not isinstance(daemon_host, str):
        raise ValueError('the daemon host must be text')
    host = daemon_host.strip().lower()
    if host.startswith('[') and host.endswith(']'):
        host = host[1:-1]
    return DAEMON if host in LOCAL_HOSTS else BRIDGE


def stamp_of(source: str, daemon_iso: str | None,
             arrival_wall_ns: int | None) -> tuple[int, int]:
    """
    Return the stamp (seconds, nanoseconds) of one value under the resolved source.

    ``'daemon'``: the daemon's own time of the value, daemon_iso (``motion.asOf`` for
    joint states, the ``observedAt`` kept for a controller for its snapshot).
    ``'bridge'``: arrival_wall_ns, the system clock in nanoseconds when the frame or
    answer that carried the value arrived. Only the argument of the chosen source is
    read; the other may be None.

    Raises ValueError when the source is not a resolved one, when the daemon time
    cannot be read, or when the arrival time is not a whole number of nanoseconds that
    a ROS time can hold. The caller then publishes nothing: there is never a stamp for
    a time at which nothing was confirmed.
    """
    if source == DAEMON:
        return timeconv.iso_to_sec_nanosec(daemon_iso)
    if source != BRIDGE:
        raise ValueError('the stamp source must be daemon or bridge')
    if isinstance(arrival_wall_ns, bool) or not isinstance(arrival_wall_ns, int):
        raise ValueError('the arrival time must be a whole number of nanoseconds')
    sec, nanosec = divmod(arrival_wall_ns, timeconv.NANOSEC_PER_SEC)
    if sec < 0 or sec > timeconv.MAX_SEC:
        raise ValueError('the arrival time does not fit a ROS time')
    return sec, nanosec


class StampGate:
    """
    The gate of one publisher: its stamps never go backwards.

    A stamp behind the last admitted one is refused and counted in ``held_back``. A
    stamp equal to the last one is admitted only when the caller allows it (a snapshot
    keep-alive repeats its stamp; a joint state with the same stamp is simply not sent
    again) and is never counted.
    """

    held_back: int

    def __init__(self) -> None:
        self.held_back = 0
        self._last = None

    @property
    def last(self) -> tuple[int, int] | None:
        """Return the last admitted stamp, or None when none was admitted since a reset."""
        return self._last

    def admit(self, stamp: tuple[int, int], *, allow_equal: bool) -> bool:
        """
        Return whether a message with this stamp may be published now.

        stamp is (seconds, nanoseconds). An admitted stamp becomes the one the next is
        compared with. Raises ValueError for anything that is not such a pair.
        """
        stamp = _pair(stamp)
        if self._last is None or stamp > self._last:
            self._last = stamp
            return True
        if stamp == self._last:
            return bool(allow_equal)
        self.held_back += 1
        return False

    def reset(self) -> None:
        """
        Forget the last admitted stamp, so that time may start again in the past.

        For a replay that starts over. ``held_back`` keeps counting: it is the count of
        the gate's whole life.
        """
        self._last = None


def _pair(stamp):
    if not isinstance(stamp, (tuple, list)) or len(stamp) != 2:
        raise ValueError('a stamp is (seconds, nanoseconds)')
    sec, nanosec = stamp
    for part in (sec, nanosec):
        if isinstance(part, bool) or not isinstance(part, int):
            raise ValueError('a stamp is (seconds, nanoseconds), both whole numbers')
    if sec < 0 or not 0 <= nanosec < timeconv.NANOSEC_PER_SEC:
        raise ValueError('a stamp is (seconds, nanoseconds), nanoseconds below one second')
    return sec, nanosec

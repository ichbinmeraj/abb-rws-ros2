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
The recorded fixtures through the stream: stand-in daemon, daemon client, state.

The stand-in daemon plays a recording to the daemon client. Every frame must arrive as
it was recorded, in order, and applying what arrived must leave the same state as
applying the recording directly. The controller list the stand-in answers after its last
frame must then confirm every controller and change nothing.

The fixture test reads every ``*.jsonl`` in the fixtures folder: the folder named by
ABB_RWS_TEST_FIXTURES when that is set, otherwise ``fixtures`` beside this file. With no
recording there it is skipped and says so. The other tests run the same check on a
recording written by hand from the ``frames`` builders, also while that folder is empty.
"""

import json
import threading
import time

from abb_rws_bridge import state
from abb_rws_bridge.daemon_client import (
    ClientSettings, DaemonClient, Disconnected, FrameReceived, Refreshed)
import fake_daemon
import frames
import pytest
import recording

# A recording is played in about this long, whatever time it spans (seconds).
PLAY_S = 0.5
# How long one step may take on a busy machine before the check gives up (seconds).
WAIT_S = 20.0
WALL_NS = 1_700_000_000_000_000_000
CONTROLLER_LIST = '/v0/controllers'


def refuse(path, text):
    """Fail the check: name the recording and say what went wrong."""
    raise AssertionError(f'{path.name}: {text}')


class Inbox:
    """The sink of the client under test: keeps every event and lets a test wait."""

    def __init__(self):
        self._events = []
        self._changed = threading.Condition()

    def __call__(self, event):
        with self._changed:
            self._events.append(event)
            self._changed.notify_all()

    def of(self, kind):
        """Return the events of a kind so far, oldest first."""
        with self._changed:
            return [event for event in self._events if isinstance(event, kind)]

    def take(self, kind, count, wait_s):
        """
        Return the first count events of a kind, waiting for them if need be.

        Fails when the connection ends first or they do not arrive within wait_s.
        """
        deadline = time.monotonic() + wait_s
        with self._changed:
            while True:
                found = [event for event in self._events if isinstance(event, kind)]
                if len(found) >= count:
                    return found[:count]
                for event in self._events:
                    if isinstance(event, Disconnected):
                        raise AssertionError(
                            f'the connection ended: {event.reason} ({event.detail})')
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise AssertionError(f'only {len(found)} of {count} {kind.__name__} '
                                         f'event(s) within {wait_s} s')
                self._changed.wait(remaining)


def relay(timed, token_path, frames_wait_s=WAIT_S):
    """
    Play (seconds, frame) pairs through the stand-in daemon to a daemon client.

    Returns the FrameReceived events (the stand-in's own empty snapshot first, then one
    per frame) and the Refreshed event of a controller list the stand-in answered after
    its last frame. The stand-in's clock stands at the last frame, so that list is as
    old as the newest thing in the recording, whenever the recording was made.
    frames_wait_s is how long the played frames may take to arrive.
    """
    fake = fake_daemon.FakeDaemon()
    last = timed[-1][1].get('observedAt')
    if isinstance(last, str):
        fake.now = lambda: last
    inbox = Inbox()
    with fake:
        token_path.write_text(fake.token + '\n', encoding='ascii')
        settings = ClientSettings(
            daemon_url=fake.url, token_file=str(token_path), allow_replay=False,
            refresh_hz=10.0, refresh_timeout_s=WAIT_S, reconnect_min_s=0.5,
            reconnect_max_s=15.0, http_timeout_s=WAIT_S)
        # No target: the client holds no watch lease, it only listens and refreshes.
        client = DaemonClient(settings, inbox, tuple)
        client.start()
        try:
            inbox.take(FrameReceived, 1, WAIT_S)
            player = fake.play(timed, speed=max(1.0, timed[-1][0] / PLAY_S))
            player.join(WAIT_S)
            if player.is_alive():
                raise AssertionError(f'the stand-in did not play the frames within {WAIT_S} s')
            # A list asked for from here on is answered from the state after the last frame.
            asked = len(fake.requests_to(CONTROLLER_LIST))
            received = inbox.take(FrameReceived, 1 + len(timed), frames_wait_s)
            refreshed = inbox.take(Refreshed, asked + 1, WAIT_S)[asked]
        finally:
            client.stop()
        more = len(inbox.of(FrameReceived)) - len(received)
        if more:
            raise AssertionError(f'{more} more frame(s) arrived than were played')
        if inbox.of(Disconnected):
            raise AssertionError('the connection ended while the recording was played')
        if fake.data_frames:
            raise AssertionError('the client sent a data frame to the stream')
    return received, refreshed


def differences(got, want, *, times):
    """Return where two fleet states differ, one short text each; nothing when the same."""
    found = []
    if got.ids() != want.ids():
        found.append('other controllers')
    if got.incomplete() != want.incomplete():
        found.append('other controllers held back')
    if (got.schema_version, got.unknown_frames) != (want.schema_version, want.unknown_frames):
        found.append('another schema version or count of unknown frames')
    for controller_id in want.ids():
        mine, theirs = got.get(controller_id), want.get(controller_id)
        if mine is None:
            continue
        found.extend(f'{controller_id}: {key}' for key in sorted(set(mine) | set(theirs))
                     if key not in mine or key not in theirs or mine[key] != theirs[key])
        seen, wanted = got.seen(controller_id), want.seen(controller_id)
        if times and ((seen.observed_at, seen.motion_as_of)
                      != (wanted.observed_at, wanted.motion_as_of)):
            found.append(f'{controller_id}: the times seen')
    return found


def check_stream(path, tmp_path, messages_schema, frames_wait_s=WAIT_S):
    """Refuse a recording that the stream does not hand on as direct application has it."""
    timed = list(recording.read_frames(path))
    if not timed:
        refuse(path, 'holds no frame')
    direct = state.FleetState(messages_schema)
    outcomes = [direct.apply(frame, WALL_NS, seconds) for seconds, frame in timed]

    received, refreshed = relay(timed, tmp_path / 'token', frames_wait_s)
    first = received[0].frame
    if first.get('type') != 'snapshot' or first.get('controllers') != []:
        refuse(path, 'the stream did not start with the empty snapshot of the stand-in')
    for number, (event, (_, frame)) in enumerate(zip(received[1:], timed, strict=True), 1):
        if event.frame != frame:
            refuse(path, f'frame {number} did not arrive as it was recorded')

    via = state.FleetState(messages_schema)
    via.apply(first, received[0].at.wall_ns, received[0].at.mono)
    for number, (event, outcome) in enumerate(zip(received[1:], outcomes, strict=True), 1):
        if via.apply(event.frame, event.at.wall_ns, event.at.mono) != outcome:
            refuse(path, f'frame {number} did something else to the state after the stream '
                         'than applied directly')
    found = differences(via, direct, times=True)
    if found:
        refuse(path, 'the state after the stream is not the state of direct application: '
                     + '; '.join(found[:8]))

    done = via.confirm(refreshed.answer, refreshed.at.wall_ns, refreshed.at.mono)
    if done.kind != state.CONFIRM or done.problem is not None:
        refuse(path, f'the controller list after the last frame was not taken: {done.problem}')
    if sorted(done.confirmed) != list(direct.ids()):
        refuse(path, 'the controller list after the last frame does not confirm every '
                     'controller')
    found = sorted(done.changed) + differences(via, direct, times=False)
    if found:
        refuse(path, 'the controller list after the last frame changed the state: '
                     + '; '.join(found[:8]))


@pytest.fixture(scope='module')
def recordings(fixtures_dir):
    found = recording.find(fixtures_dir)
    if not found:
        pytest.skip(recording.skip_reason(fixtures_dir))
    return found


def test_every_fixture_played_to_the_client_leaves_the_state_of_direct_application(
        recordings, tmp_path):
    version = frames.load_schema()['x-schema-version']
    for path in recordings:
        check_stream(path, tmp_path, version)


# ----- a recording written by hand ------------------------------------------------------

def timed_frames():
    """
    Return a short recording as (seconds, frame) pairs: synthetic, from the builders.

    Its frames have every size a WebSocket frame can have: a snapshot above 64 KiB
    (four controllers with twenty long event texts each, not all of them ASCII), motion
    deltas of about 1 KiB and a removed frame below 126 bytes.
    """
    text = '\u00e5\u00e4\u00f6 \u65e5\u672c\u8a9e ' + 'x' * 900
    events = frames.events_group(
        [frames.event(seq, description=text, causes=text) for seq in range(1, 21)])
    controllers = [frames.controller(f'cell-{number}', events=events) for number in (1, 2, 3)]
    controllers.append(frames.edge_controller('edge-1'))
    controllers[-1]['events'] = events
    timed = [(0.0, frames.snapshot_frame(controllers))]
    for step in range(1, 6):
        at = step * 0.25
        joints = [frames.joint(f'J{axis}', 10.0 * axis + at) for axis in range(1, 7)]
        motion = frames.motion_group([frames.motion_unit(joints=joints)], as_of=frames.iso(at))
        timed.append((at, frames.delta_frame(f'cell-{1 + step % 3}', {'motion': motion},
                                             observed_at=frames.iso(at))))
    added = frames.controller('cell-9', name='Added later')
    timed.append((1.5, frames.delta_frame('cell-9', frames.groups_of(added), name='Added later',
                                          observed_at=frames.iso(1.5))))
    timed.append((1.75, frames.removed_frame('edge-1', observed_at=frames.iso(1.75))))
    return timed


@pytest.fixture
def made(tmp_path):
    return recording.write(tmp_path / 'made.jsonl', timed_frames(), frames.SCHEMA_VERSION)


def test_a_recording_written_by_hand_arrives_as_it_was_recorded(made, tmp_path):
    sizes = [len(json.dumps(frame)) for _, frame in recording.read_frames(made)]
    assert max(sizes) > 65535 and min(sizes) < 126
    assert any(126 <= size <= 65535 for size in sizes)
    check_stream(made, tmp_path, frames.SCHEMA_VERSION)


def test_a_frame_changed_on_its_way_is_caught(made, tmp_path, monkeypatch):
    push = fake_daemon.FakeDaemon.push

    def bent(self, frame):
        if frame['type'] == 'removed':
            frame = dict(frame, id='cell-1')
        push(self, frame)

    monkeypatch.setattr(fake_daemon.FakeDaemon, 'push', bent)
    with pytest.raises(AssertionError, match='made.jsonl: frame 8 did not arrive as it was'):
        check_stream(made, tmp_path, frames.SCHEMA_VERSION)


def test_a_frame_lost_on_its_way_is_caught(made, tmp_path, monkeypatch):
    push = fake_daemon.FakeDaemon.push

    def lossy(self, frame):
        if frame['type'] != 'removed':
            push(self, frame)

    monkeypatch.setattr(fake_daemon.FakeDaemon, 'push', lossy)
    with pytest.raises(AssertionError, match='only [0-8] of 9 FrameReceived'):
        check_stream(made, tmp_path, frames.SCHEMA_VERSION, frames_wait_s=1.0)

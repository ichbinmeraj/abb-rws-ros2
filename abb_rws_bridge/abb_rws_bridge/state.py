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
What abb-rws-daemon last said about every controller, kept from its stream.

``FleetState`` applies the frames of ``WS /v0/stream`` (one ``snapshot``, then ``delta``
and ``removed`` frames) and the answers of ``GET /v0/controllers``. The stream is the only
writer of values. An answer can only confirm them: the stream sends a group when its
content changes, so while a robot stands still no frame arrives although the daemon
keeps reading, and the answer is what moves the freshness stamps (``asOf``) forward.

An answer also shows when the stream has fallen behind: it holds another value than the
state although it is newer than everything the stream delivered. The state says since
when that has been so (``behind_since``); what to do about it is the caller's decision.

Pure: no ROS import and no lock. The caller applies everything from one thread. The
state keeps references into the frames and answers it is handed, so they must not be
changed afterwards; in turn it never edits a group in place, it replaces it, so a reader
may keep a reference to a group or to a controller it was given.
"""

from collections.abc import Mapping
from dataclasses import dataclass, field
import json
import re

from abb_rws_bridge import timeconv

# The groups of one controller, in the order of the daemon's schema.
GROUPS = ('identity', 'link', 'panel', 'rapid', 'motion', 'io', 'events', 'access',
          'capabilities', 'freshness')

SNAPSHOT = 'snapshot'
DELTA = 'delta'
REMOVED = 'removed'
CONFIRM = 'confirm'
IGNORED = 'ignored'

# The daemon sends at most one frame for a controller in a window of this many seconds.
# A controller list that is newer than the last frame by less may simply be ahead of a
# frame that is on its way.
COALESCE_WINDOW_S = 0.25
# What the stream owes for a controller that a controller list no longer holds.
NOT_LISTED = 'no longer listed'

_ALL_GROUPS = frozenset(GROUPS)
_NANOSEC_PER_SEC = 1_000_000_000
_NOT_GROUPS = frozenset({'id', 'name'})
_VERSION = re.compile(r'([0-9]+)\.([0-9]+)')
_LABEL_LENGTH = 64


@dataclass(frozen=True)
class Seen:
    """
    What the bridge knows about the times of one controller.

    ``observed_at`` is the daemon time (``observedAt``) of the last frame applied to the
    controller, or of the last answer that confirmed all of it; ``wall_ns`` and ``mono``
    are the system clock (nanoseconds) and the monotonic clock (seconds) when that frame
    or answer arrived here. ``motion_as_of`` is ``motion.asOf`` as it is now;
    ``motion_wall_ns`` and ``motion_mono`` are the arrival of the frame or answer in
    which ``motion.asOf`` last moved forward, None when it never did. A stamp that was
    there when the controller was first seen moved forward only when it was fresh then
    (see ``FleetState``).
    """

    observed_at: str
    wall_ns: int
    mono: float
    motion_as_of: str | None
    motion_wall_ns: int | None
    motion_mono: float | None


@dataclass(frozen=True)
class Applied:
    """
    What one frame or one answer did to the state.

    ``kind`` is ``'snapshot'``, ``'delta'``, ``'removed'``, ``'confirm'`` or
    ``'ignored'``. ``changed`` maps each controller the frame touched to the names of
    the groups it replaced: every group after a snapshot or for a controller a delta
    added, the groups a delta named otherwise (a group name this version does not know
    included). After a confirm it holds the groups in which a stamp or the measured
    rate moved; no value ever changes in a confirm. ``removed`` lists the controllers
    that are gone, ``confirmed`` those whose whole state an answer confirmed,
    ``motion_moved`` those whose ``motion.asOf`` moved forward. ``problem`` says what was
    refused or held back, None when nothing was.
    """

    kind: str
    changed: Mapping[str, frozenset[str]] = field(default_factory=dict)
    removed: tuple[str, ...] = ()
    confirmed: tuple[str, ...] = ()
    motion_moved: tuple[str, ...] = ()
    problem: str | None = None


def comparable(group_name: str, group) -> str:
    """
    Return a group as JSON text without its freshness stamps.

    Every ``asOf`` key is left out, at any depth, and for the ``motion`` group also its
    ``rate`` object, whose measured rate wobbles. Two groups with the same text hold the
    same values. Keys are sorted, so the order in which they arrived does not matter.
    """
    stripped = _without_as_of(group)
    if group_name == 'motion' and isinstance(stripped, dict):
        stripped.pop('rate', None)
    return json.dumps(stripped, sort_keys=True, separators=(',', ':'))


def owed(group_name: str, group) -> str:
    """
    Return a group as JSON text without what the daemon's stream does not announce.

    This is ``comparable`` and, for the ``link`` group, also without its ``reconnect``
    object: the daemon counts its attempts to reach a controller that is down, and
    books the next one, without sending a frame for it. A controller list that
    differs from the state in that object alone therefore shows nothing the stream
    owes. Two groups with the same text differ in nothing a frame must deliver.
    """
    if group_name == 'link' and isinstance(group, Mapping):
        group = {key: value for key, value in group.items() if key != 'reconnect'}
    return comparable(group_name, group)


class FleetState:
    """
    Every controller the daemon serves, as its stream last reported it.

    ``schema_version`` is the ``schemaVersion`` of the last snapshot frame, accepted or
    refused; None before the first one and after ``clear()``. ``messages_schema`` is the
    version it is held against. ``unknown_frames`` counts the frames of a type this
    version does not know (they are ignored). ``first_sight_max_age_s`` is the limit for
    a joint stamp that nothing came before, None for no limit.
    """

    schema_version: str | None
    messages_schema: str
    unknown_frames: int
    first_sight_max_age_s: float | None

    def __init__(self, messages_schema: str, *,
                 first_sight_max_age_s: float | None = None) -> None:
        """
        Start empty.

        messages_schema is the schema version the messages were generated from
        (``Converter.schema_version``, ``'0.12'``). A snapshot of another major line, or
        of a lower minor, is refused. Raises ValueError when it is not ``<major>.<minor>``.

        first_sight_max_age_s is for a ``motion.asOf`` that has no stamp before it: the
        first sight of a controller (the snapshot of a connection, a controller a delta
        adds) and joints that had not been read before. Nothing but the stamp itself
        says when such a sample was taken, so it counts as having moved forward only
        when it lies less than this many seconds from the ``observedAt`` of the frame
        or answer that carries it, both on the daemon's clock. Otherwise the joints
        wait for the next stamp that is later. None (the default) takes every such
        stamp as fresh; a node passes its staleness limit. Raises ValueError when it is
        neither None nor a number above zero.
        """
        version = _version(messages_schema)
        if version is None:
            raise ValueError('the schema version of the messages must be <major>.<minor>')
        if first_sight_max_age_s is not None:
            usable = (isinstance(first_sight_max_age_s, (int, float))
                      and not isinstance(first_sight_max_age_s, bool)
                      and 0 < first_sight_max_age_s < float('inf'))
            if not usable:
                raise ValueError('first_sight_max_age_s must be None or a number above zero')
        self.first_sight_max_age_s = first_sight_max_age_s
        self.messages_schema = messages_schema
        self.schema_version = None
        self.unknown_frames = 0
        self._messages_version = version
        self._accepting = False
        self._fleet = {}
        self._seen = {}
        self._incomplete = {}
        self._texts = {}
        self._behind = {}

    def apply(self, frame: Mapping, wall_ns: int, mono: float) -> Applied:
        """
        Apply one stream frame that arrived at wall_ns (system clock) and mono (seconds).

        A snapshot replaces the whole fleet; the controllers it no longer lists are
        reported as removed. A delta replaces each group it names, whole, and the name.
        A delta for an unknown controller adds it once every group is there;
        until then the controller is held back: not served by ``get``, and reported. A
        removed frame drops a controller. A frame of an unknown type is ignored and
        counted; a group with an unknown name is kept and nothing else.

        Refused, with ``problem`` set: a snapshot whose schema version does not fit the
        messages (the fleet is then empty and stays empty until a snapshot is
        accepted), a frame before the first accepted snapshot, and a frame that is not
        shaped as the schema says.
        """
        if not isinstance(frame, Mapping):
            return Applied(IGNORED, problem='a frame that is not an object')
        kind = frame.get('type')
        if kind == SNAPSHOT:
            return self._snapshot(frame, wall_ns, mono)
        if kind == DELTA:
            return self._delta(frame, wall_ns, mono)
        if kind == REMOVED:
            return self._removed(frame)
        self.unknown_frames += 1
        return Applied(IGNORED)

    def confirm(self, answer: Mapping, wall_ns: int, mono: float) -> Applied:
        """
        Take the freshness stamps of one ``GET /v0/controllers`` answer.

        For every controller the state holds and every group of it in the answer: when
        the answer is not older than what the stream last delivered for that controller
        and the group equals the stored one but for its stamps (see ``comparable``),
        the stored group is replaced by the answer's. Anything else is left for the
        stream. So a confirm can move a stamp or a measured rate forward and can never
        change a value, add a controller or remove one.

        A controller whose groups were all taken is confirmed: its
        ``Seen.observed_at`` becomes the answer's ``observedAt``.

        The answer also shows whether the stream is behind (see ``behind_since``): a
        controller whose answer is more than COALESCE_WINDOW_S newer than what the
        stream delivered for it, and which holds another value in a group or is not
        listed any more, is owed a frame from the arrival of this answer (mono) on. An
        answer that shows no other value settles it.

        Refused, with ``problem`` set: an answer that is not shaped as a controller
        list, and one that names another schema version than the stream's snapshot.
        """
        if not isinstance(answer, Mapping):
            return Applied(IGNORED, problem='a controller list that is not an object')
        observed_at = answer.get('observedAt')
        answered = _time(observed_at)
        if answered is None:
            return Applied(IGNORED,
                           problem='a controller list without a readable observedAt')
        controllers = answer.get('controllers')
        if not isinstance(controllers, list):
            return Applied(IGNORED, problem='a controller list without controllers')
        if not self._fleet:
            return Applied(CONFIRM)
        version = answer.get('schemaVersion')
        if isinstance(version, str) and version != self.schema_version:
            return Applied(IGNORED, problem='a controller list of another schema version '
                                            'than the stream')
        changed = {}
        confirmed = []
        moved = []
        handled = set()
        listed = set()
        for entry in controllers:
            controller_id = entry.get('id') if isinstance(entry, Mapping) else None
            if not isinstance(controller_id, str) or controller_id not in self._fleet:
                continue
            listed.add(controller_id)
            seen = self._seen[controller_id]
            if answered < _time(seen.observed_at) or controller_id in handled:
                continue
            handled.add(controller_id)
            taken, replaced, differing = self._take(controller_id, entry)
            if not differing:
                self._behind.pop(controller_id, None)
            elif _seconds_between(_time(seen.observed_at), answered) > COALESCE_WINDOW_S:
                self._owe(controller_id, mono, ', '.join(differing))
            motion_as_of = seen.motion_as_of
            motion_wall_ns, motion_mono = seen.motion_wall_ns, seen.motion_mono
            if 'motion' in replaced:
                motion_as_of = _as_of(replaced['motion'])
                if _motion_moved(motion_as_of, seen.motion_as_of, observed_at,
                                 self.first_sight_max_age_s):
                    motion_wall_ns, motion_mono = wall_ns, mono
                    moved.append(controller_id)
            if taken == len(GROUPS):
                confirmed.append(controller_id)
                seen = Seen(observed_at, wall_ns, mono,
                            motion_as_of, motion_wall_ns, motion_mono)
            else:
                seen = Seen(seen.observed_at, seen.wall_ns, seen.mono,
                            motion_as_of, motion_wall_ns, motion_mono)
            self._seen[controller_id] = seen
            if replaced:
                changed[controller_id] = frozenset(replaced)
        for controller_id in self._fleet:
            # A controller the daemon no longer lists: the stream owes its removal.
            if controller_id in listed:
                continue
            last = _time(self._seen[controller_id].observed_at)
            if _seconds_between(last, answered) > COALESCE_WINDOW_S:
                self._owe(controller_id, mono, NOT_LISTED)
        return Applied(CONFIRM, changed=changed, confirmed=tuple(confirmed),
                       motion_moved=tuple(moved))

    def behind(self) -> tuple[tuple[str, str], ...]:
        """
        Return the open debts of the stream as (controller id, what it owes), sorted.

        What it owes is the names of the groups in which the last controller list
        showed another value, or NOT_LISTED for a controller the list no longer holds.
        """
        return tuple((controller_id, self._behind[controller_id][1])
                     for controller_id in sorted(self._behind))

    def behind_since(self) -> float | None:
        """
        Return since when the stream owes a frame (monotonic seconds); None: it owes none.

        The stream owes a frame for a controller from the moment a controller list
        arrives that is newer than everything the stream delivered for that controller
        (by more than COALESCE_WINDOW_S, on the daemon's clock) and shows another value
        in one of its groups, or no longer lists it. What the daemon does not announce
        on its stream is no value here: the stamps (``asOf``), the measured motion rate
        and the reconnect bookkeeping of a link (see ``owed``). The debt is settled by
        the next frame applied to the controller, and by an answer that shows no other
        value. This is the earliest of the open debts.

        A healthy stream settles a debt within the daemon's window. One that stays open
        means that frames no longer arrive although the daemon answers: a connection
        that is open in name only.
        """
        return min((since for since, _ in self._behind.values()), default=None)

    def _owe(self, controller_id, mono, what):
        """Open a debt for a controller, or note what an open one is about now."""
        since = self._behind[controller_id][0] if controller_id in self._behind else mono
        self._behind[controller_id] = (since, what)

    @property
    def has_snapshot(self) -> bool:
        """Return whether a snapshot was accepted since the last clear() or refusal."""
        return self._accepting

    def get(self, controller_id: str) -> Mapping | None:
        """Return one controller as the daemon's ControllerSnapshot, or None."""
        return self._fleet.get(controller_id)

    def seen(self, controller_id: str) -> Seen | None:
        """Return the Seen of one controller, or None when the state does not hold it."""
        return self._seen.get(controller_id)

    def ids(self) -> tuple[str, ...]:
        """Return the ids of the controllers the state holds, sorted."""
        return tuple(sorted(self._fleet))

    def incomplete(self) -> tuple[str, ...]:
        """Return the ids of the controllers that are held back as incomplete, sorted."""
        return tuple(sorted(self._incomplete))

    def clear(self) -> None:
        """
        Forget every controller: the stream is gone.

        Nothing is taken again before the next accepted snapshot. ``unknown_frames``
        keeps counting.
        """
        self.schema_version = None
        self._accepting = False
        self._fleet = {}
        self._seen = {}
        self._incomplete = {}
        self._texts = {}
        self._behind = {}

    def _snapshot(self, frame, wall_ns, mono):
        before = self.ids()
        previous = self._seen
        version = frame.get('schemaVersion')
        observed_at = frame.get('observedAt')
        controllers = frame.get('controllers')
        problem = self._version_problem(version)
        if problem is None and _time(observed_at) is None:
            problem = 'a snapshot without a readable observedAt'
        if problem is None and not isinstance(controllers, list):
            problem = 'a snapshot without controllers'
        self.clear()
        self.schema_version = version if _version(version) is not None else None
        if problem is not None:
            return Applied(SNAPSHOT, removed=before, problem=problem)
        self._accepting = True
        notes = []
        moved = []
        nameless = 0
        for entry in controllers:
            controller_id = entry.get('id') if isinstance(entry, Mapping) else None
            if not isinstance(controller_id, str) or not controller_id:
                nameless += 1
                continue
            if controller_id in self._fleet or controller_id in self._incomplete:
                notes.append(f'controller {_label(controller_id)} is listed more than once; '
                             'the first entry is kept')
                continue
            controller = dict(entry)
            lacking = _lacking(controller)
            if lacking:
                self._incomplete[controller_id] = controller
                notes.append(_held_back(controller_id, lacking))
                continue
            self._fleet[controller_id] = controller
            self._seen[controller_id], did_move = _seen_after(
                previous.get(controller_id), controller, observed_at, wall_ns, mono,
                self.first_sight_max_age_s)
            if did_move:
                moved.append(controller_id)
        if nameless:
            notes.append(f'{nameless} controller(s) without an id were left out')
        return Applied(
            SNAPSHOT,
            changed={controller_id: _ALL_GROUPS for controller_id in self._fleet},
            removed=tuple(name for name in before if name not in self._fleet),
            motion_moved=tuple(moved),
            problem='; '.join(notes) or None)

    def _delta(self, frame, wall_ns, mono):
        if not self._accepting:
            return Applied(IGNORED, problem='a delta before an accepted snapshot')
        controller_id = frame.get('id')
        groups = frame.get('groups')
        observed_at = frame.get('observedAt')
        if not isinstance(controller_id, str) or not controller_id:
            return Applied(IGNORED, problem='a delta without a controller id')
        label = _label(controller_id)
        if not isinstance(groups, Mapping):
            return Applied(IGNORED, problem=f'a delta for {label} without groups')
        if _time(observed_at) is None:
            return Applied(IGNORED,
                           problem=f'a delta for {label} without a readable observedAt')
        groups = {name: value for name, value in groups.items()
                  if isinstance(name, str) and name not in _NOT_GROUPS}
        unfit = [name for name in GROUPS
                 if name in groups and not _group_fits(name, groups[name])]
        if unfit:
            return Applied(IGNORED, problem=f'a delta for {label} in which '
                                            f'{", ".join(unfit)} is not an object')
        name = frame.get('name')
        stored = self._fleet.get(controller_id)
        if stored is None:
            controller = dict(self._incomplete.pop(controller_id, None)
                              or {'id': controller_id, 'name': controller_id})
        else:
            controller = dict(stored)
        controller.update(groups)
        if isinstance(name, str):
            controller['name'] = name
        if stored is None:
            lacking = _lacking(controller)
            if lacking:
                self._incomplete[controller_id] = controller
                return Applied(DELTA, problem=_held_back(controller_id, lacking))
            names = _ALL_GROUPS | frozenset(groups)
        else:
            names = frozenset(groups)
        self._fleet[controller_id] = controller
        texts = self._texts.get(controller_id)
        if texts:
            for group_name in groups:
                texts.pop(group_name, None)
        self._behind.pop(controller_id, None)
        self._seen[controller_id], did_move = _seen_after(
            self._seen.get(controller_id), controller, observed_at, wall_ns, mono,
            self.first_sight_max_age_s)
        return Applied(DELTA, changed={controller_id: names},
                       motion_moved=(controller_id,) if did_move else ())

    def _removed(self, frame):
        if not self._accepting:
            return Applied(IGNORED, problem='a removed frame before an accepted snapshot')
        controller_id = frame.get('id')
        if not isinstance(controller_id, str) or not controller_id:
            return Applied(IGNORED, problem='a removed frame without a controller id')
        self._incomplete.pop(controller_id, None)
        if controller_id not in self._fleet:
            return Applied(REMOVED)
        del self._fleet[controller_id]
        del self._seen[controller_id]
        self._texts.pop(controller_id, None)
        self._behind.pop(controller_id, None)
        return Applied(REMOVED, removed=(controller_id,))

    def _take(self, controller_id, entry):
        """
        Replace the stored groups the answer's entry confirms.

        Returns (how many groups were taken, the groups that were replaced, the names
        of the groups in which the entry shows something the stream owes: see ``owed``).
        """
        stored = self._fleet[controller_id]
        texts = self._texts.setdefault(controller_id, {})
        taken = 0
        replaced = {}
        differing = []
        for group_name in GROUPS:
            if group_name not in entry:
                continue
            offered = entry[group_name]
            kept = stored[group_name]
            if offered is kept or offered == kept:
                taken += 1
                continue
            if not _group_fits(group_name, offered):
                continue
            if group_name not in texts:
                texts[group_name] = comparable(group_name, kept)
            if comparable(group_name, offered) != texts[group_name]:
                if group_name != 'link' or owed(group_name, offered) != owed(group_name, kept):
                    differing.append(group_name)
                continue
            taken += 1
            replaced[group_name] = offered
        if replaced:
            controller = dict(stored)
            controller.update(replaced)
            self._fleet[controller_id] = controller
        return taken, replaced, differing

    def _version_problem(self, version):
        found = _version(version)
        if found is None:
            return 'a snapshot without a readable schemaVersion'
        if found[0] != self._messages_version[0]:
            return (f'the daemon speaks schema {version}, another major line than the '
                    f'messages ({self.messages_schema})')
        if found[1] < self._messages_version[1]:
            return (f'the daemon speaks schema {version}, older than the messages '
                    f'({self.messages_schema})')
        return None


def _without_as_of(value):
    if isinstance(value, Mapping):
        return {key: _without_as_of(item) for key, item in value.items() if key != 'asOf'}
    if isinstance(value, (list, tuple)):
        return [_without_as_of(item) for item in value]
    return value


def _version(text):
    """Return (major, minor) of a schema version text, or None."""
    match = _VERSION.fullmatch(text) if isinstance(text, str) else None
    return (int(match.group(1)), int(match.group(2))) if match else None


def _time(text):
    """Return a daemon time as (seconds, nanoseconds), or None when it cannot be read."""
    try:
        return timeconv.iso_to_sec_nanosec(text)
    except ValueError:
        return None


def _as_of(group):
    value = group.get('asOf') if isinstance(group, Mapping) else None
    return value if isinstance(value, str) else None


def _seconds_between(earlier, later):
    """Return the seconds from one daemon time to another; both as (seconds, nanoseconds)."""
    return (later[0] - earlier[0]) + (later[1] - earlier[1]) / _NANOSEC_PER_SEC


def _motion_moved(new_iso, old_iso, observed_at, first_sight_max_age_s):
    """
    Tell whether motion.asOf moved forward in the frame or answer that carries it.

    With a readable stamp before it: when it is readable and later. Without one (the
    first sight of a controller, or joints that had not been read before) nothing but
    the stamp itself says when the sample was taken: it counts only when it lies less
    than first_sight_max_age_s from observed_at, the daemon's time of its carrier (None:
    always).
    """
    new = _time(new_iso)
    if new is None:
        return False
    old = _time(old_iso)
    if old is not None:
        return new > old
    if first_sight_max_age_s is None:
        return True
    carried = _time(observed_at)
    return carried is not None and abs(_seconds_between(new, carried)) < first_sight_max_age_s


def _seen_after(previous, controller, observed_at, wall_ns, mono, first_sight_max_age_s):
    """Return the Seen of a controller after a frame, and whether motion.asOf moved."""
    motion_as_of = _as_of(controller.get('motion'))
    moved = _motion_moved(motion_as_of, previous.motion_as_of if previous else None,
                          observed_at, first_sight_max_age_s)
    if moved:
        motion_wall_ns, motion_mono = wall_ns, mono
    elif previous is not None:
        motion_wall_ns, motion_mono = previous.motion_wall_ns, previous.motion_mono
    else:
        motion_wall_ns, motion_mono = None, None
    return Seen(observed_at, wall_ns, mono, motion_as_of, motion_wall_ns, motion_mono), moved


def _group_fits(group_name, value):
    """Tell whether a value can be a group: an object, or null for identity."""
    return isinstance(value, Mapping) or (value is None and group_name == 'identity')


def _lacking(controller):
    """
    Return what keeps a controller from being whole.

    The missing groups, then the ones that are not objects: each kind as one text that
    names them all, so the line stays short whatever the number of groups.
    """
    missing = [name for name in GROUPS if name not in controller]
    unfit = [name for name in GROUPS
             if name in controller and not _group_fits(name, controller[name])]
    lacking = []
    if missing:
        lacking.append(f'{", ".join(missing)} {"is" if len(missing) == 1 else "are"} missing')
    if unfit:
        lacking.append(f'{", ".join(unfit)} '
                       + ('is not an object' if len(unfit) == 1 else 'are not objects'))
    return lacking


def _held_back(controller_id, lacking):
    return f'controller {_label(controller_id)} is held back: {"; ".join(lacking)}'


def _label(text):
    """Return an id that is safe in one line of a status: printable and not too long."""
    shown = ''.join(char if char.isprintable() else '?' for char in text[:_LABEL_LENGTH])
    return shown + '...' if len(text) > _LABEL_LENGTH else shown

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
The content of one ``sensor_msgs/JointState``, from a controller's motion group.

The daemon reports joints in degrees or millimetres and names the unit per joint. Here
they become radians and metres. A joint that cannot be converted honestly is left out
and listed with the reason; it is never guessed and never passed through.
"""

from collections.abc import Mapping
from dataclasses import dataclass

from abb_rws_bridge import units

UNIT_ERROR = 'unit-error'
UNIT_UNKNOWN = 'unit-unknown'
UNIT_UNSUPPORTED = 'unit-unsupported'
NO_VALUE = 'no-value'

NO_JOINT = 'no joint to publish'


@dataclass(frozen=True)
class Skipped:
    """
    One joint that is left out of the message, and why.

    The reason is ``unit-unknown`` (the unit is null), ``unit-unsupported`` (a unit
    other than deg and mm), ``no-value`` (the value is null or not finite) or
    ``unit-error`` (the mechanical unit reports an error, so its joints hold last good
    values).
    """

    unit: str
    joint: str
    reason: str


@dataclass(frozen=True)
class JointStateData:
    """
    What goes into one JointState: the daemon's time, joint names and positions.

    ``stamp`` is ``motion.asOf`` as the daemon wrote it. ``positions`` are radians for a
    revolute joint and metres for a prismatic one, one per name. Velocity and effort are
    not measured and have no field here.

    ``problem`` set means: publish nothing. It is NO_JOINT when simply no joint is left
    to publish, which is no error by itself (the joints left out are in ``skipped``).
    Any other text is an error: two joints with one name, or a joint that cannot be
    named. ``names`` and ``positions`` are then still filled in, for the report only.
    """

    stamp: str
    names: tuple[str, ...]
    positions: tuple[float, ...]
    skipped: tuple[Skipped, ...]
    problem: str | None


def build(controller_id: str, controller_token: str, motion: Mapping | None,
          namer) -> JointStateData | None:
    """
    Return the JointState content of one controller; None while ``motion.asOf`` is null.

    All mechanical units go into one message, units and joints in the daemon's order.
    ``namer`` gives the names: ``namer.name(controller_id, controller_token, unit,
    joint)``, as names.JointNamer does. Only joints that are published get a name, and
    those names must be unique: a duplicate sets ``problem`` and nothing is renamed.
    """
    if not isinstance(motion, Mapping):
        return None
    stamp = motion.get('asOf')
    if stamp is None:
        return None
    if not isinstance(stamp, str):
        return JointStateData('', (), (), (), 'motion.asOf is not a time written as text')

    names: list[str] = []
    positions: list[float] = []
    sources: list[str] = []
    skipped: list[Skipped] = []
    problems: list[str] = []
    for unit in _mappings(motion.get('units')):
        unit_name = unit.get('name')
        in_error = unit.get('error') is not None
        for joint in _mappings(unit.get('joints')):
            joint_name = joint.get('name')
            source = f'{unit_name}/{joint_name}'
            if not isinstance(unit_name, str) or not isinstance(joint_name, str):
                problems.append(f'the joint {source} has no name written as text')
                continue
            reason, position = _position(joint, in_error)
            if reason is not None:
                skipped.append(Skipped(unit_name, joint_name, reason))
                continue
            try:
                name = namer.name(controller_id, controller_token, unit_name, joint_name)
            except ValueError as error:
                problems.append(str(error))
                continue
            if not isinstance(name, str) or not name:
                problems.append(f'the joint {source} would get an empty name')
                continue
            names.append(name)
            positions.append(position)
            sources.append(source)

    problems.extend(_duplicates(names, sources))
    if problems:
        problem = '; '.join(problems)
    elif not names:
        problem = NO_JOINT
    else:
        problem = None
    return JointStateData(stamp, tuple(names), tuple(positions), tuple(skipped), problem)


def _position(joint: Mapping, in_error: bool) -> tuple[str | None, float]:
    """Return (why the joint is left out or None, its position in radians or metres)."""
    if in_error:
        return UNIT_ERROR, 0.0
    unit = joint.get('unit')
    if unit is None:
        return UNIT_UNKNOWN, 0.0
    if units.to_si(0.0, unit) is None:
        return UNIT_UNSUPPORTED, 0.0
    position = units.to_si(joint.get('value'), unit)
    if position is None:
        return NO_VALUE, 0.0
    return None, position


def _duplicates(names: list[str], sources: list[str]) -> list[str]:
    """Return one sentence for every name that more than one joint would get."""
    owners: dict[str, list[str]] = {}
    for name, source in zip(names, sources):
        owners.setdefault(name, []).append(source)
    return [f'the joint name "{name}" is given to ' + ', '.join(joints)
            for name, joints in owners.items() if len(joints) > 1]


def _mappings(items: object) -> list[Mapping]:
    """Return the mappings of a list; anything else is not a unit or a joint."""
    if not isinstance(items, (list, tuple)):
        return []
    return [item for item in items if isinstance(item, Mapping)]

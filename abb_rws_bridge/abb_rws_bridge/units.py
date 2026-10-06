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
Unit conversions for ``sensor_msgs/JointState``: degrees to radians, millimetres to metres.

The daemon reports every joint in the unit the controller uses and names that unit per
joint. Only a value whose unit is named ``deg`` or ``mm`` is converted. A unit that is
null or any other text is never guessed, and such a value is never handed on as if it
were SI: the answer is None.
"""

import math

_DEGREES = 'deg'
_MILLIMETRES = 'mm'


def deg_to_rad(deg: float) -> float:
    """Return an angle given in degrees as radians."""
    return math.radians(deg)


def mm_to_m(mm: float) -> float:
    """Return a length given in millimetres as metres."""
    return mm / 1000.0


def to_si(value: float | None, unit: str | None) -> float | None:
    """
    Return value in radians (unit ``deg``) or metres (unit ``mm``), or None.

    None is the answer when the unit is null or any other text, and when the value is
    null, not a number or not finite. A boolean is not a number here.
    """
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    try:
        number = float(value)
    except OverflowError:
        return None
    if not math.isfinite(number):
        return None
    if not isinstance(unit, str):
        return None
    if unit == _DEGREES:
        return deg_to_rad(number)
    if unit == _MILLIMETRES:
        return mm_to_m(number)
    return None

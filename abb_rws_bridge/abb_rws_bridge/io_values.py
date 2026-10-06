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
Typed values of watched I/O signals.

The daemon passes a signal's value through as the text the controller reports, with the
signal's type beside it. The type decides the ROS message: ``DI`` and ``DO`` are
``std_msgs/Bool``, ``AI`` and ``AO`` are ``std_msgs/Float64``, ``GI`` and ``GO`` are
``std_msgs/UInt32``. A value is read strictly; one that does not fit its type gives no
message and a reason, never a nearby value.
"""

from collections.abc import Mapping
import math
import re

BOOL = 'bool'
FLOAT64 = 'float64'
UINT32 = 'uint32'
UINT32_MAX = 4294967295

_KINDS = {'DI': BOOL, 'DO': BOOL, 'AI': FLOAT64, 'AO': FLOAT64, 'GI': UINT32, 'GO': UINT32}
_DIGITAL = {'0': False, '1': True}
_NUMBER = re.compile(r'[+-]?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+)(?:[eE][+-]?[0-9]+)?')
_WHOLE_NUMBER = re.compile(r'[0-9]+')
_SHOWN = 24


def typed(signal: Mapping) -> tuple[str, bool | float | int] | None:
    """
    Return (kind, value) of a watched signal, or None when it gives no message.

    kind is ``'bool'`` (a bool), ``'float64'`` (a finite float) or ``'uint32'`` (an int
    from 0 to 4294967295); signal is one entry of ``io.signals``. why_not says why the
    answer is None.
    """
    return _read(signal)[0]


def why_not(signal: Mapping) -> str | None:
    """Return why the signal gives no message; None when it gives one."""
    return _read(signal)[1]


def kind_of(signal: Mapping) -> str | None:
    """
    Return the kind the signal's reported type asks for, whatever its value is.

    None while the type is null or is not one of DI, DO, AI, AO, GI and GO. A kind
    other than the one a signal was first published with means the signal changed its
    type.
    """
    signal_type = signal.get('type') if isinstance(signal, Mapping) else None
    return _KINDS.get(signal_type) if isinstance(signal_type, str) else None


def _read(signal: object) -> tuple[tuple[str, bool | float | int] | None, str | None]:
    """Return (typed value, None) or (None, the reason)."""
    if not isinstance(signal, Mapping):
        return None, 'not a signal of the daemon'
    signal_type = signal.get('type')
    if signal_type is None:
        return None, 'the type is not known yet'
    kind = _KINDS.get(signal_type) if isinstance(signal_type, str) else None
    if kind is None:
        return None, f'the type {_shown(signal_type)} is not one of DI, DO, AI, AO, GI, GO'
    value = signal.get('value')
    if value is None:
        return None, 'no value yet'
    if not isinstance(value, str):
        return None, f'the value {_shown(value)} is not text'
    if kind == BOOL:
        if value not in _DIGITAL:
            return None, f'the value {_shown(value)} of a {signal_type} is neither "0" nor "1"'
        return (BOOL, _DIGITAL[value]), None
    if kind == FLOAT64:
        number = float(value) if _NUMBER.fullmatch(value) else math.nan
        if not math.isfinite(number):
            return None, f'the value {_shown(value)} of an {signal_type} is not a finite number'
        return (FLOAT64, number), None
    whole = _whole_number(value)
    if whole is None:
        return None, (f'the value {_shown(value)} of a {signal_type} is not a whole number '
                      f'from 0 to {UINT32_MAX}')
    return (UINT32, whole), None


def _whole_number(text: str) -> int | None:
    """Return text as an int from 0 to UINT32_MAX; None for anything else."""
    if not _WHOLE_NUMBER.fullmatch(text):
        return None
    digits = text.lstrip('0') or '0'
    if len(digits) > len(str(UINT32_MAX)):
        return None
    whole = int(digits)
    return whole if whole <= UINT32_MAX else None


def _shown(value: object) -> str:
    """Return a short, printable form of a value for a reason."""
    text = value if isinstance(value, str) else repr(value)
    text = ''.join(character if ' ' <= character <= '~' else '?' for character in text)
    if len(text) > _SHOWN:
        text = text[:_SHOWN] + '...'
    return f'"{text}"'

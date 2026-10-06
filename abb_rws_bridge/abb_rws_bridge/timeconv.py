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
Daemon times as ROS times: ISO 8601 text to whole seconds and nanoseconds.

abb-rws-daemon writes every time as ``YYYY-MM-DDTHH:MM:SS[.fraction]Z`` (UTC). This
module turns such a text into the pair a ``builtin_interfaces/Time`` holds. It uses
integer arithmetic only, so no digit is lost to a float, and it imports nothing from ROS.
"""

import datetime
import re

NANOSEC_PER_SEC = 1000000000

# The last second a builtin_interfaces/Time can hold: its sec field is an int32.
MAX_SEC = 2**31 - 1

_ISO = re.compile(
    r'([0-9]{4})-([0-9]{2})-([0-9]{2})T([0-9]{2}):([0-9]{2}):([0-9]{2})(?:\.([0-9]+))?Z')
_EPOCH_DAY = datetime.date(1970, 1, 1).toordinal()


def iso_to_sec_nanosec(text: str) -> tuple[int, int]:
    """
    Return (seconds, nanoseconds) since 1970-01-01T00:00:00Z for a daemon time.

    The text must be ``YYYY-MM-DDTHH:MM:SS[.fraction]Z``, exactly: upper-case ``T`` and
    ``Z``, ASCII digits, a real calendar date, a time of day up to 23:59:59. A fraction
    may have any number of digits; what lies beyond the ninth is cut off, never rounded.

    Raises ValueError for anything else (a value that is not text included), for a time
    before 1970 and for a time after 2038-01-19T03:14:07Z, which the int32 seconds of
    ``builtin_interfaces/Time`` cannot hold. The message never repeats the text.
    """
    if not isinstance(text, str):
        raise ValueError(f'a time must be text, not {type(text).__name__}')
    match = _ISO.fullmatch(text)
    if match is None:
        raise ValueError('not a time written as YYYY-MM-DDTHH:MM:SS[.fraction]Z')
    year, month, day, hour, minute, second = (int(part) for part in match.groups()[:6])
    if hour > 23 or minute > 59 or second > 59:
        raise ValueError('not a time of day')
    try:
        days = datetime.date(year, month, day).toordinal() - _EPOCH_DAY
    except ValueError:
        raise ValueError('not a calendar date') from None
    if days < 0:
        raise ValueError('a time before 1970')
    sec = days * 86400 + hour * 3600 + minute * 60 + second
    if sec > MAX_SEC:
        raise ValueError('a time after 2038-01-19T03:14:07Z does not fit a ROS time')
    fraction = match.group(7) or ''
    return sec, int((fraction + '000000000')[:9])

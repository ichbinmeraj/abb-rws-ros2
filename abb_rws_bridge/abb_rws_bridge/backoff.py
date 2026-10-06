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

"""Waits between reconnect attempts: doubling between two bounds, with jitter."""

import math
import random

JITTER_LOW = 0.8
JITTER_HIGH = 1.2


class Backoff:
    """
    The waits between reconnect attempts.

    The first wait is based on min_s, each later one on twice the base before it, up to
    max_s. Every wait is its base multiplied by a random factor between 0.8 and 1.2, so
    that several bridges that lost the same daemon do not come back in step. A wait
    therefore lies between 0.8 * min_s and 1.2 * max_s.

    rng is anything with a ``uniform(low, high)`` method, such as a ``random.Random``;
    without one a new ``random.Random()`` is used. One thread owns an instance.
    """

    def __init__(self, min_s: float, max_s: float, rng=None) -> None:
        for name, value in (('min_s', min_s), ('max_s', max_s)):
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f'{name} must be a number of seconds')
            if not math.isfinite(value):
                raise ValueError(f'{name} must be finite')
        if min_s <= 0:
            raise ValueError('min_s must be above zero')
        if max_s < min_s:
            raise ValueError('max_s must not be below min_s')
        self._min_s = float(min_s)
        self._max_s = float(max_s)
        self._rng = random.Random() if rng is None else rng
        self._base_s = self._min_s

    def next(self) -> float:  # noqa: A003
        """Return the wait before the next attempt, in seconds, and double the base."""
        base_s = self._base_s
        self._base_s = min(self._max_s, base_s * 2.0)
        return base_s * self._rng.uniform(JITTER_LOW, JITTER_HIGH)

    def reset(self) -> None:
        """Start again from min_s: the connection held long enough to count as good."""
        self._base_s = self._min_s

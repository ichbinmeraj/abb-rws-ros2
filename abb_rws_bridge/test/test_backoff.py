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

"""The reconnect waits: doubling between the bounds, jitter, reset."""

import random

from abb_rws_bridge.backoff import Backoff, JITTER_HIGH, JITTER_LOW
import pytest


class FixedRng:
    """A random source that always answers the same factor and records what was asked."""

    def __init__(self, factor=1.0):
        self.factor = factor
        self.asked = []

    def uniform(self, low, high):
        self.asked.append((low, high))
        return self.factor


def test_waits_double_from_min_to_max():
    backoff = Backoff(0.5, 15.0, FixedRng())
    assert [backoff.next() for _ in range(8)] == [0.5, 1.0, 2.0, 4.0, 8.0, 15.0, 15.0, 15.0]


def test_reset_starts_again_from_min():
    backoff = Backoff(0.5, 15.0, FixedRng())
    for _ in range(4):
        backoff.next()
    backoff.reset()
    assert [backoff.next() for _ in range(3)] == [0.5, 1.0, 2.0]
    backoff.reset()
    backoff.reset()
    assert backoff.next() == 0.5


def test_every_wait_is_its_base_times_a_factor_between_the_jitter_bounds():
    rng = FixedRng(1.2)
    backoff = Backoff(1.0, 4.0, rng)
    assert [backoff.next() for _ in range(4)] == pytest.approx([1.2, 2.4, 4.8, 4.8])
    assert rng.asked == [(0.8, 1.2)] * 4
    assert (JITTER_LOW, JITTER_HIGH) == (0.8, 1.2)
    low = Backoff(1.0, 4.0, FixedRng(0.8))
    assert [low.next() for _ in range(3)] == pytest.approx([0.8, 1.6, 3.2])


def test_a_seeded_random_gives_the_same_sequence_as_its_twin():
    backoff = Backoff(0.5, 15.0, random.Random(7))
    twin = random.Random(7)
    bases = [0.5, 1.0, 2.0, 4.0, 8.0, 15.0, 15.0]
    waits = [backoff.next() for _ in bases]
    assert waits == pytest.approx([base * twin.uniform(0.8, 1.2) for base in bases])
    assert len(set(waits)) == len(waits)


def test_without_a_random_source_the_waits_stay_inside_the_bounds():
    backoff = Backoff(0.5, 2.0)
    bases = [0.5, 1.0, 2.0, 2.0, 2.0]
    for _ in range(40):
        backoff.reset()
        for base in bases:
            wait = backoff.next()
            assert 0.8 * base <= wait <= 1.2 * base
    assert 0.8 * 0.5 <= Backoff(0.5, 2.0).next() <= 1.2 * 0.5


def test_two_instances_do_not_share_their_place():
    first = Backoff(1.0, 8.0, FixedRng())
    second = Backoff(1.0, 8.0, FixedRng())
    assert [first.next(), first.next(), first.next()] == [1.0, 2.0, 4.0]
    assert second.next() == 1.0


def test_equal_bounds_give_one_base():
    backoff = Backoff(3.0, 3.0, FixedRng())
    assert [backoff.next() for _ in range(3)] == [3.0, 3.0, 3.0]


def test_whole_numbers_are_accepted_and_waits_are_floats():
    backoff = Backoff(1, 5, FixedRng())
    waits = [backoff.next() for _ in range(4)]
    assert waits == [1.0, 2.0, 4.0, 5.0]
    assert all(isinstance(wait, float) for wait in waits)


@pytest.mark.parametrize('min_s, max_s', [
    (0.0, 1.0), (-1.0, 1.0), (2.0, 1.0), (float('nan'), 1.0), (1.0, float('nan')),
    (1.0, float('inf')), (float('inf'), float('inf')), ('1', 2.0), (1.0, None), (True, 2.0),
])
def test_bounds_that_make_no_sense_are_refused(min_s, max_s):
    with pytest.raises(ValueError):
        Backoff(min_s, max_s)

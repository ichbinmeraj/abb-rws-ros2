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

"""Daemon times become whole seconds and nanoseconds, and nothing else is accepted."""

import datetime

from abb_rws_bridge import timeconv
import frames
import pytest

UTC = datetime.timezone.utc


def _seconds(*parts):
    return int(datetime.datetime(*parts, tzinfo=UTC).timestamp())


def test_the_epoch_is_zero():
    assert timeconv.iso_to_sec_nanosec('1970-01-01T00:00:00Z') == (0, 0)
    assert timeconv.iso_to_sec_nanosec('1970-01-01T00:00:00.000Z') == (0, 0)


def test_the_time_the_test_frames_start_at():
    assert frames.T0 == '2026-01-01T00:00:00.000Z'
    assert timeconv.iso_to_sec_nanosec(frames.T0) == (1767225600, 0)


@pytest.mark.parametrize('parts', [
    (1970, 1, 1, 0, 0, 1),
    (1999, 12, 31, 23, 59, 59),
    (2000, 2, 29, 12, 0, 0),
    (2024, 2, 29, 23, 59, 59),
    (2026, 10, 5, 17, 4, 33),
    (2028, 2, 29, 0, 0, 0),
    (2037, 12, 31, 23, 59, 59),
])
def test_whole_seconds_equal_the_calendar(parts):
    text = '{:04d}-{:02d}-{:02d}T{:02d}:{:02d}:{:02d}Z'.format(*parts)
    assert timeconv.iso_to_sec_nanosec(text) == (_seconds(*parts), 0)


@pytest.mark.parametrize('fraction, nanosec', [
    ('0', 0),
    ('5', 500000000),
    ('05', 50000000),
    ('123', 123000000),
    ('999', 999000000),
    ('123456', 123456000),
    ('000000001', 1),
    ('123456789', 123456789),
    ('999999999', 999999999),
])
def test_a_fraction_is_read_to_the_nanosecond(fraction, nanosec):
    sec = _seconds(2026, 3, 4, 5, 6, 7)
    assert timeconv.iso_to_sec_nanosec(f'2026-03-04T05:06:07.{fraction}Z') == (sec, nanosec)


def test_digits_beyond_the_ninth_are_cut_off_not_rounded():
    sec = _seconds(2026, 3, 4, 5, 6, 7)
    assert timeconv.iso_to_sec_nanosec('2026-03-04T05:06:07.1234567899Z') == (sec, 123456789)
    # Rounding would carry into the next second.
    assert timeconv.iso_to_sec_nanosec('2026-03-04T05:06:07.9999999999Z') == (sec, 999999999)


def test_the_result_is_two_whole_numbers():
    sec, nanosec = timeconv.iso_to_sec_nanosec('2026-03-04T05:06:07.250Z')
    assert type(sec) is int
    assert type(nanosec) is int
    assert 0 <= nanosec < timeconv.NANOSEC_PER_SEC


def test_one_nanosecond_survives_where_a_float_would_lose_it():
    sec, nanosec = timeconv.iso_to_sec_nanosec('2026-03-04T05:06:07.000000001Z')
    assert nanosec == 1
    assert float(sec) + 1e-9 == float(sec), 'a float of seconds cannot hold this digit'


@pytest.mark.parametrize('offset_s', [0.0, 0.001, 0.5, 1.0, 59.999, 3600.125, 86400.0, 1e6])
def test_the_times_the_frame_builders_write_are_read(offset_s):
    sec, nanosec = timeconv.iso_to_sec_nanosec(frames.iso(offset_s))
    total_ms = round(offset_s * 1000)
    assert sec == 1767225600 + total_ms // 1000
    assert nanosec == (total_ms % 1000) * 1000000


def test_a_later_time_gives_a_larger_pair():
    texts = ['2026-01-01T00:00:00.000Z', '2026-01-01T00:00:00.001Z', '2026-01-01T00:00:00.5Z',
             '2026-01-01T00:00:01.000Z', '2026-01-02T00:00:00.000Z', '2027-01-01T00:00:00.000Z']
    pairs = [timeconv.iso_to_sec_nanosec(text) for text in texts]
    assert pairs == sorted(pairs)
    assert len(set(pairs)) == len(pairs)


def test_the_last_second_a_ros_time_holds():
    assert timeconv.MAX_SEC == 2**31 - 1
    assert timeconv.iso_to_sec_nanosec('2038-01-19T03:14:07Z') == (timeconv.MAX_SEC, 0)
    assert timeconv.iso_to_sec_nanosec('2038-01-19T03:14:07.999999999Z') == (
        timeconv.MAX_SEC, 999999999)


@pytest.mark.parametrize('text', [
    '2038-01-19T03:14:08Z',
    '2038-01-20T00:00:00.000Z',
    '2100-01-01T00:00:00Z',
    '9999-12-31T23:59:59.999Z',
])
def test_a_time_a_ros_time_cannot_hold_is_refused(text):
    with pytest.raises(ValueError):
        timeconv.iso_to_sec_nanosec(text)


@pytest.mark.parametrize('text', [
    '1969-12-31T23:59:59Z',
    '1969-12-31T23:59:59.999Z',
    '1900-01-01T00:00:00Z',
    '0001-01-01T00:00:00Z',
    '0000-01-01T00:00:00Z',
])
def test_a_time_before_1970_is_refused(text):
    with pytest.raises(ValueError):
        timeconv.iso_to_sec_nanosec(text)


@pytest.mark.parametrize('text', [
    '',
    'now',
    '2026-01-01',
    '2026-01-01T00:00:00',
    '2026-01-01T00:00Z',
    '2026-01-01T00:00:00+00:00',
    '2026-01-01T00:00:00.000+01:00',
    '2026-01-01 00:00:00Z',
    '2026-01-01 T 00:00:00',
    '2026-01-01t00:00:00Z',
    '2026-01-01T00:00:00z',
    '2026-01-01T00:00:00.Z',
    '2026-01-01T00:00:00,5Z',
    '2026-01-01T00:00:00.5 Z',
    ' 2026-01-01T00:00:00Z',
    '2026-01-01T00:00:00Z ',
    '2026-01-01T00:00:00Z\n',
    '2026-1-1T00:00:00Z',
    '26-01-01T00:00:00Z',
    '2026-01-01T0:00:00Z',
    '+2026-01-01T00:00:00Z',
    '-2026-01-01T00:00:00Z',
    '2026-01-01T00:00:00.-5Z',
    '2026-01-01T00:00:00.5e3Z',
    '2026-00-10T00:00:00Z',
    '2026-13-01T00:00:00Z',
    '2026-01-00T00:00:00Z',
    '2026-01-32T00:00:00Z',
    '2026-04-31T00:00:00Z',
    '2026-02-29T00:00:00Z',
    '2100-02-29T00:00:00Z',
    '2026-01-01T24:00:00Z',
    '2026-01-01T00:60:00Z',
    '2026-01-01T00:00:60Z',
    '2026-12-31T23:59:60Z',
    '\uff12\uff10\uff12\uff16-01-01T00:00:00Z',
    '2026-01-01T00:00:0\u0663Z',
    '1767225600',
])
def test_anything_but_a_daemon_time_is_refused(text):
    with pytest.raises(ValueError):
        timeconv.iso_to_sec_nanosec(text)


@pytest.mark.parametrize('value', [
    None, 0, 1767225600, 1767225600.5, True, b'2026-01-01T00:00:00Z',
    ['2026-01-01T00:00:00Z'], {'asOf': '2026-01-01T00:00:00Z'},
    datetime.datetime(2026, 1, 1, tzinfo=UTC),
])
def test_a_value_that_is_not_text_is_a_value_error(value):
    with pytest.raises(ValueError):
        timeconv.iso_to_sec_nanosec(value)


def test_the_error_never_repeats_what_was_given():
    for text in ['not-a-time-SENTINEL', '1969-12-31T23:59:59.555Z', '2026-13-41T00:00:00.555Z',
                 '2100-01-01T00:00:00.555Z']:
        with pytest.raises(ValueError) as caught:
            timeconv.iso_to_sec_nanosec(text)
        assert text not in str(caught.value)
        assert 'SENTINEL' not in str(caught.value)
        assert '555' not in str(caught.value)

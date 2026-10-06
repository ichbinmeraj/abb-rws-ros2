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

import copy
import math

from abb_rws_bridge import io_values
import frames
import pytest


def read(signal_type, value):
    signal = frames.signal('sig', signal_type, value)
    answer = io_values.typed(signal)
    reason = io_values.why_not(signal)
    # Exactly one of the two speaks.
    assert (answer is None) != (reason is None)
    return answer, reason


@pytest.mark.parametrize('signal_type', ['DI', 'DO'])
def test_digital_signals_are_bool(signal_type):
    assert read(signal_type, '0') == (('bool', False), None)
    assert read(signal_type, '1') == (('bool', True), None)
    assert io_values.typed(frames.signal('s', signal_type, '1'))[1] is True
    assert io_values.typed(frames.signal('s', signal_type, '0'))[1] is False


@pytest.mark.parametrize('signal_type', ['DI', 'DO'])
@pytest.mark.parametrize('value', [
    '', '2', '-1', '01', '00', '1.0', '0.0', 'true', 'false', 'True', 'on', 'off', ' 1',
    '1 ', '1\n', 'high', '+1', '\u0661',
])
def test_a_digital_value_other_than_0_and_1_gives_no_message(signal_type, value):
    answer, reason = read(signal_type, value)
    assert answer is None
    assert 'neither "0" nor "1"' in reason
    assert signal_type in reason


@pytest.mark.parametrize('signal_type', ['AI', 'AO'])
@pytest.mark.parametrize('value, number', [
    ('2.5', 2.5), ('-0.75', -0.75), ('0', 0.0), ('10', 10.0), ('+3.25', 3.25), ('5.', 5.0),
    ('.5', 0.5), ('1e3', 1000.0), ('1.5E-2', 0.015), ('-2.50000E+00', -2.5),
    ('0.000000', 0.0), ('-0', 0.0), ('007.5', 7.5),
])
def test_analog_signals_are_float64(signal_type, value, number):
    answer, reason = read(signal_type, value)
    assert reason is None
    assert answer == ('float64', number)
    assert isinstance(answer[1], float)


@pytest.mark.parametrize('signal_type', ['AI', 'AO'])
@pytest.mark.parametrize('value', [
    '', 'nan', 'NaN', 'inf', '-inf', 'Infinity', '1e999', '-1e999', '2,5', '1_000', ' 2.5',
    '2.5 ', '2.5\n', '0x10', '1.2.3', 'e5', '.', '-', '+', '1e', '1e+', '--1', 'abc', '2.5V',
    '\u0662.5',
])
def test_an_analog_value_that_is_not_a_finite_number_gives_no_message(signal_type, value):
    answer, reason = read(signal_type, value)
    assert answer is None
    assert 'not a finite number' in reason
    assert signal_type in reason


@pytest.mark.parametrize('signal_type', ['GI', 'GO'])
@pytest.mark.parametrize('value, number', [
    ('0', 0), ('1', 1), ('42', 42), ('65535', 65535), ('4294967295', 4294967295),
    ('007', 7), ('0000000000000004294967295', 4294967295), ('0' * 6000, 0),
])
def test_group_signals_are_uint32(signal_type, value, number):
    answer, reason = read(signal_type, value)
    assert reason is None
    assert answer == ('uint32', number)
    assert type(answer[1]) is int


@pytest.mark.parametrize('signal_type', ['GI', 'GO'])
@pytest.mark.parametrize('value', [
    '', '-1', '4294967296', '99999999999', '9' * 6000, '42.0', '4.2e1', '+42', ' 42',
    '42 ', '0x2A', '1_000', 'forty-two', '\u0664\u0662',
])
def test_a_group_value_that_is_not_a_uint32_gives_no_message(signal_type, value):
    answer, reason = read(signal_type, value)
    assert answer is None
    assert 'whole number from 0 to 4294967295' in reason
    assert signal_type in reason


@pytest.mark.parametrize('signal_type', ['DI', 'DO', 'AI', 'AO', 'GI', 'GO'])
def test_a_null_value_gives_no_message(signal_type):
    answer, reason = read(signal_type, None)
    assert answer is None
    assert reason == 'no value yet'


@pytest.mark.parametrize('signal_type', ['DI', 'AO', 'GO'])
@pytest.mark.parametrize('value', [1, 0, 1.5, True, False, ['1'], {'v': 1}])
def test_a_value_that_is_not_text_gives_no_message(signal_type, value):
    answer, reason = read(signal_type, value)
    assert answer is None
    assert 'is not text' in reason


def test_a_null_type_gives_no_message():
    for value in (None, '1', '2.5', '42'):
        answer, reason = read(None, value)
        assert answer is None
        assert reason == 'the type is not known yet'


@pytest.mark.parametrize('signal_type', [
    '', 'di', 'do', 'Di', 'DIO', 'XX', 'BOOL', 'num', 'D', ' DI', 'DI ', 5, True, ['DI'],
])
def test_any_other_type_gives_no_message(signal_type):
    answer, reason = read(signal_type, '1')
    assert answer is None
    assert 'is not one of DI, DO, AI, AO, GI, GO' in reason


@pytest.mark.parametrize('signal', [None, 'do_gripper', 5, ['DO', '1'], ('DO', '1')])
def test_something_that_is_not_a_signal_gives_no_message(signal):
    assert io_values.typed(signal) is None
    assert io_values.why_not(signal) == 'not a signal of the daemon'


def test_missing_keys_read_as_null():
    assert io_values.typed({}) is None
    assert io_values.why_not({}) == 'the type is not known yet'
    assert io_values.why_not({'type': 'DO'}) == 'no value yet'
    assert io_values.typed({'type': 'DO', 'value': '1'}) == ('bool', True)


def test_the_edge_controller_has_one_signal_of_each_type():
    expected = {
        'di_ready': ('bool', True),
        'do_gripper': ('bool', False),
        'ai_pressure': ('float64', 2.5),
        'ao_speed': ('float64', -0.75),
        'gi_code': ('uint32', 42),
        'Local/DRV_1/go_program': ('uint32', 4294967295),
        'do_missing': None,
    }
    signals = frames.edge_controller()['io']['signals']
    before = copy.deepcopy(signals)
    assert {signal['name']: io_values.typed(signal) for signal in signals} == expected
    reasons = {signal['name']: io_values.why_not(signal) for signal in signals}
    assert reasons['do_missing'] == 'the type is not known yet'
    assert all(reason is None for name, reason in reasons.items() if name != 'do_missing')
    assert signals == before


def test_every_signal_type_of_the_schema_has_a_kind():
    samples = {'DI': '1', 'DO': '0', 'AI': '1.5', 'AO': '2', 'GI': '3', 'GO': '4'}
    assert set(samples) == set(frames.SIGNAL_TYPES)
    kinds = {signal_type: io_values.typed(frames.signal('s', signal_type, value))[0]
             for signal_type, value in samples.items()}
    assert kinds == {'DI': 'bool', 'DO': 'bool', 'AI': 'float64', 'AO': 'float64',
                     'GI': 'uint32', 'GO': 'uint32'}
    assert {io_values.BOOL, io_values.FLOAT64, io_values.UINT32} == set(kinds.values())
    assert io_values.UINT32_MAX == 2 ** 32 - 1


def test_kind_of_follows_the_type_whatever_the_value_is():
    for signal_type, kind in (('DI', 'bool'), ('DO', 'bool'), ('AI', 'float64'),
                              ('AO', 'float64'), ('GI', 'uint32'), ('GO', 'uint32')):
        for value in ('1', 'not a value', None, 5):
            assert io_values.kind_of(frames.signal('s', signal_type, value)) == kind
    for signal_type in (None, '', 'di', 'XX', 5, ['DI']):
        assert io_values.kind_of(frames.signal('s', signal_type, '1')) is None
    for signal in (None, 'DO', 5, {}):
        assert io_values.kind_of(signal) is None
    # A signal that was a DO and is reported as an AO has changed its kind.
    was = io_values.typed(frames.signal('s', 'DO', '1'))[0]
    assert io_values.kind_of(frames.signal('s', 'AO', 'bad')) not in (None, was)


def test_the_reason_shows_a_short_printable_form_of_the_value():
    reason = io_values.why_not(frames.signal('s', 'DO', 'x' * 500))
    assert len(reason) < 120
    assert 'x' * 24 + '...' in reason
    reason = io_values.why_not(frames.signal('s', 'AO', 'a\x00b\nc\u00e9'))
    assert '"a?b?c?"' in reason
    assert all(' ' <= character <= '~' for character in reason)


def test_the_value_is_what_decides_not_the_other_fields():
    signal = frames.signal('do_x', 'DO', '1', via=None, as_of=None,
                           error=frames.group_error())
    assert io_values.typed(signal) == ('bool', True)
    assert io_values.why_not(signal) is None


def test_no_value_is_ever_nan_or_infinite():
    for value in ('nan', 'inf', '-inf', '1e400'):
        assert io_values.typed(frames.signal('s', 'AI', value)) is None
    answer = io_values.typed(frames.signal('s', 'AI', '1e308'))
    assert answer == ('float64', 1e308)
    assert math.isfinite(answer[1])

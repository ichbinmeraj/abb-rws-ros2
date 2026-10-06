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

import dataclasses

from abb_rws_bridge import parameters
from abb_rws_bridge.parameters import settings_from
import pytest

# The parameter table of the bridge: name, kind, default, minimum, maximum.
TABLE = (
    ('daemon_url', 'string', 'http://127.0.0.1:7810', None, None),
    ('allow_remote_daemon', 'bool', False, None, None),
    ('token_file', 'string', '', None, None),
    ('controllers', 'string_array', (), None, None),
    ('primary_controller', 'string', '', None, None),
    ('joint_name_format', 'string', '{unit}_{joint}', None, None),
    ('joint_name_overrides', 'string_array', (), None, None),
    ('joint_states_frame_id', 'string', '', None, None),
    ('stamp_source', 'string', 'auto', None, None),
    ('watch_motion', 'bool', True, None, None),
    ('io_signals', 'string_array', (), None, None),
    ('allow_replay', 'bool', False, None, None),
    ('refresh_hz', 'double', 2.0, 0.2, 10.0),
    ('republish_hz', 'double', 1.0, 0.0, 10.0),
    ('stale_after_s', 'double', 2.0, 0.5, 60.0),
    ('diagnostics_period_s', 'double', 1.0, 0.2, 10.0),
    ('reconnect_min_s', 'double', 0.5, 0.1, 60.0),
    ('reconnect_max_s', 'double', 15.0, 1.0, 300.0),
    ('http_timeout_s', 'double', 5.0, 0.5, 30.0),
)


def good(**changes):
    settings, problems = settings_from(changes)
    assert problems == []
    assert settings is not None
    return settings


def bad(**changes):
    settings, problems = settings_from(changes)
    assert settings is None
    assert problems
    assert all(isinstance(problem, str) and problem for problem in problems)
    return problems


def one(**changes):
    problems = bad(**changes)
    assert len(problems) == 1, problems
    return problems[0]


def test_specs_are_exactly_the_table():
    assert isinstance(parameters.SPECS, tuple)
    listed = tuple((spec.name, spec.kind, spec.default, spec.minimum, spec.maximum)
                   for spec in parameters.SPECS)
    assert listed == TABLE


def test_every_spec_is_complete():
    names_seen = [spec.name for spec in parameters.SPECS]
    assert len(set(names_seen)) == len(names_seen) == 19
    for spec in parameters.SPECS:
        assert isinstance(spec, parameters.Spec)
        assert spec.kind in ('string', 'bool', 'double', 'string_array')
        assert isinstance(spec.description, str)
        assert len(spec.description) > 20
        assert spec.description.endswith('.')
        assert spec.description == ' '.join(spec.description.split())
        if spec.kind == 'string':
            assert isinstance(spec.default, str)
        elif spec.kind == 'bool':
            assert isinstance(spec.default, bool)
        elif spec.kind == 'string_array':
            assert spec.default == ()
        if spec.kind == 'double':
            assert type(spec.default) is float
            assert type(spec.minimum) is float and type(spec.maximum) is float
            assert spec.minimum <= spec.default <= spec.maximum
        else:
            assert spec.minimum is None and spec.maximum is None


def test_a_spec_cannot_be_changed():
    with pytest.raises(dataclasses.FrozenInstanceError):
        parameters.SPECS[0].default = 'http://example.com:1'


def test_no_description_names_a_secret_or_a_place_on_one_machine():
    for spec in parameters.SPECS:
        lowered = spec.description.lower()
        assert 'password' not in lowered
        assert 'c:\\' not in lowered
        assert '/home/' not in lowered


def test_the_other_constants():
    assert parameters.BUILTIN_PARAMETERS == frozenset(
        {'use_sim_time', 'start_type_description_service'})
    assert parameters.QOS_OVERRIDE_PREFIX == 'qos_overrides.'
    assert parameters.STAMP_SOURCES == ('auto', 'daemon', 'bridge')
    assert parameters.TRUSTED_DAEMON_HOSTS == frozenset({
        '127.0.0.1', 'localhost', '::1', 'host.docker.internal', 'host.containers.internal'})
    assert not parameters.BUILTIN_PARAMETERS & {spec.name for spec in parameters.SPECS}


def test_the_defaults_are_settings():
    settings = good()
    for spec in parameters.SPECS:
        if spec.name in ('io_signals', 'joint_name_overrides'):
            assert dict(getattr(settings, spec.name)) == {}
        else:
            assert getattr(settings, spec.name) == spec.default
    assert settings.daemon_host == '127.0.0.1'
    assert settings.daemon_port == 7810
    assert settings.ws_url == 'ws://127.0.0.1:7810/v0/stream'
    assert settings.refresh_timeout_s == 2.0
    by_default = settings_from({spec.name: spec.default for spec in parameters.SPECS})
    assert by_default == (settings, [])


def test_settings_have_one_field_per_parameter_and_the_derived_ones():
    fields = [field.name for field in dataclasses.fields(parameters.Settings)]
    assert fields == [spec.name for spec in parameters.SPECS] + [
        'daemon_host', 'daemon_port', 'ws_url', 'refresh_timeout_s']
    with pytest.raises(dataclasses.FrozenInstanceError):
        good().allow_replay = True


def test_every_value_is_taken_and_typed():
    settings = good(
        daemon_url='http://localhost:7830', allow_remote_daemon=False,
        token_file='/run/secrets/abb_rws_daemon_token', controllers=['cell-1', 'cell-2'],
        primary_controller='cell-1', joint_name_format='{controller}_{unit}_{joint}',
        joint_name_overrides=['cell-1/ROB_1/J1=joint_1'], joint_states_frame_id='base',
        stamp_source='bridge', watch_motion=False,
        io_signals=['cell-1:do_gripper', 'cell-2:Local/DRV_1/di_x', 'cell-1:ai_p'],
        allow_replay=True, refresh_hz=4.0, republish_hz=0.0, stale_after_s=1.0,
        diagnostics_period_s=0.5, reconnect_min_s=1.0, reconnect_max_s=30.0,
        http_timeout_s=0.5)
    assert settings.daemon_url == 'http://localhost:7830'
    assert settings.daemon_host == 'localhost'
    assert settings.daemon_port == 7830
    assert settings.ws_url == 'ws://localhost:7830/v0/stream'
    assert settings.token_file == '/run/secrets/abb_rws_daemon_token'
    assert settings.controllers == ('cell-1', 'cell-2')
    assert settings.primary_controller == 'cell-1'
    assert settings.joint_name_format == '{controller}_{unit}_{joint}'
    assert dict(settings.joint_name_overrides) == {('cell-1', 'ROB_1', 'J1'): 'joint_1'}
    assert settings.joint_states_frame_id == 'base'
    assert settings.stamp_source == 'bridge'
    assert settings.watch_motion is False
    assert dict(settings.io_signals) == {
        'cell-1': ('do_gripper', 'ai_p'), 'cell-2': ('Local/DRV_1/di_x',)}
    assert settings.allow_replay is True
    assert (settings.refresh_hz, settings.republish_hz, settings.stale_after_s) == (4.0, 0.0, 1.0)
    assert settings.diagnostics_period_s == 0.5
    assert (settings.reconnect_min_s, settings.reconnect_max_s) == (1.0, 30.0)
    assert settings.http_timeout_s == 0.5
    assert settings.refresh_timeout_s == 0.5


def test_parsed_entries_are_read_only():
    settings = good(io_signals=['cell-1:do_x'], joint_name_overrides=['cell-1/ROB_1/J1=a'])
    with pytest.raises(TypeError):
        settings.io_signals['cell-2'] = ('do_y',)
    with pytest.raises(TypeError):
        settings.joint_name_overrides[('cell-1', 'ROB_1', 'J2')] = 'b'


def test_string_arrays_may_be_lists_tuples_or_missing():
    assert good(controllers=[]).controllers == ()
    assert good(controllers=()).controllers == ()
    assert good(controllers=None).controllers == ()
    assert good(controllers=('a', 'b')).controllers == ('a', 'b')
    assert dict(good(io_signals=None).io_signals) == {}


def test_names_that_are_not_parameters_of_the_bridge_are_not_looked_at():
    settings = good(**{
        'use_sim_time': False, 'start_type_description_service': True,
        'qos_overrides./joint_states.publisher.depth': 5, 'something_else': object()})
    assert settings == good()


def test_refresh_timeout_is_the_smaller_of_two():
    assert good(http_timeout_s=5.0, stale_after_s=2.0).refresh_timeout_s == 2.0
    assert good(http_timeout_s=1.0, stale_after_s=2.0).refresh_timeout_s == 1.0
    assert good(http_timeout_s=30.0, stale_after_s=60.0).refresh_timeout_s == 30.0


@pytest.mark.parametrize('refresh_hz, stale_after_s', [
    (2.0, 1.0), (2.0, 2.0), (1.0, 2.0), (4.0, 0.5), (10.0, 0.5), (0.2, 10.0), (0.2, 60.0),
    (3.0, 2.0 / 3.0),
])
def test_stale_after_gives_two_chances_to_confirm(refresh_hz, stale_after_s):
    settings = good(refresh_hz=refresh_hz, stale_after_s=stale_after_s)
    assert settings.stale_after_s >= 2.0 / settings.refresh_hz - 1e-9


@pytest.mark.parametrize('refresh_hz, stale_after_s', [
    (2.0, 0.5), (2.0, 0.99), (1.0, 1.9), (0.2, 9.9), (0.5, 2.0), (3.0, 0.6),
])
def test_stale_after_below_two_refreshes_is_a_problem(refresh_hz, stale_after_s):
    problem = one(refresh_hz=refresh_hz, stale_after_s=stale_after_s)
    assert problem.startswith('stale_after_s: ')
    assert '2 / refresh_hz' in problem


def test_the_backoff_bounds():
    assert good(reconnect_min_s=1.0, reconnect_max_s=1.0).reconnect_min_s == 1.0
    assert good(reconnect_min_s=0.1, reconnect_max_s=300.0).reconnect_max_s == 300.0
    problem = one(reconnect_min_s=20.0, reconnect_max_s=15.0)
    assert problem.startswith('reconnect_min_s: ')
    assert 'reconnect_max_s' in problem
    assert one(reconnect_min_s=2.0, reconnect_max_s=1.0).startswith('reconnect_min_s: ')


@pytest.mark.parametrize('name, value', [
    ('refresh_hz', 0.0), ('refresh_hz', 0.19), ('refresh_hz', 10.5), ('refresh_hz', 50.0),
    ('refresh_hz', -1.0), ('republish_hz', -0.1), ('republish_hz', 10.1),
    ('stale_after_s', 0.4), ('stale_after_s', 61.0), ('diagnostics_period_s', 0.1),
    ('diagnostics_period_s', 11.0), ('reconnect_min_s', 0.05), ('reconnect_min_s', 61.0),
    ('reconnect_max_s', 0.9), ('reconnect_max_s', 301.0), ('http_timeout_s', 0.4),
    ('http_timeout_s', 31.0),
])
def test_a_double_outside_its_range_is_a_problem(name, value):
    problem = one(**{name: value})
    assert problem.startswith(f'{name}: ')
    assert 'outside the range' in problem


@pytest.mark.parametrize('name, value', [
    ('refresh_hz', 0.2), ('refresh_hz', 10.0), ('republish_hz', 0.0), ('republish_hz', 10.0),
    ('stale_after_s', 60.0), ('diagnostics_period_s', 0.2), ('diagnostics_period_s', 10.0),
    ('reconnect_max_s', 300.0), ('http_timeout_s', 0.5), ('http_timeout_s', 30.0),
])
def test_the_ends_of_a_range_are_inside(name, value):
    extra = {'stale_after_s': 10.0} if name == 'refresh_hz' else {}
    extra[name] = value
    assert getattr(good(**extra), name) == value


@pytest.mark.parametrize('name', [
    spec.name for spec in parameters.SPECS if spec.kind == 'double'])
def test_a_whole_number_for_a_double_is_a_problem_with_a_hint(name):
    problem = one(**{name: 2})
    assert problem.startswith(f'{name}: ')
    assert 'write 2.0, not 2' in problem


@pytest.mark.parametrize('value', ['2.0', None, True, [2.0], float('nan'), float('inf')])
def test_a_double_of_another_kind_is_a_problem(value):
    assert one(refresh_hz=value).startswith('refresh_hz: ')


@pytest.mark.parametrize('name, value', [
    ('daemon_url', 7810), ('daemon_url', None), ('token_file', ['a']), ('token_file', 5),
    ('primary_controller', True), ('joint_name_format', None), ('stamp_source', 1),
    ('joint_states_frame_id', 0.5), ('allow_remote_daemon', 'true'), ('allow_remote_daemon', 1),
    ('watch_motion', 0), ('watch_motion', None), ('allow_replay', 'false'),
    ('controllers', 'cell-1'), ('controllers', [1, 2]), ('controllers', ['a', None]),
    ('io_signals', 'cell-1:do_x'), ('io_signals', 5), ('joint_name_overrides', {'a': 'b'}),
    ('joint_name_overrides', 'cell-1/ROB_1/J1=a'),
])
def test_a_value_of_the_wrong_type_is_one_problem(name, value):
    problem = one(**{name: value})
    assert problem.startswith(f'{name}: ')


@pytest.mark.parametrize('url, host, port, ws_url', [
    ('http://127.0.0.1:7810', '127.0.0.1', 7810, 'ws://127.0.0.1:7810/v0/stream'),
    ('http://127.0.0.1:7810/', '127.0.0.1', 7810, 'ws://127.0.0.1:7810/v0/stream'),
    ('http://localhost:7830', 'localhost', 7830, 'ws://localhost:7830/v0/stream'),
    ('http://LOCALHOST:7830', 'localhost', 7830, 'ws://localhost:7830/v0/stream'),
    ('HTTP://localhost:1', 'localhost', 1, 'ws://localhost:1/v0/stream'),
    ('http://[::1]:7810', '::1', 7810, 'ws://[::1]:7810/v0/stream'),
    ('http://host.docker.internal:7810', 'host.docker.internal', 7810,
     'ws://host.docker.internal:7810/v0/stream'),
    ('http://host.containers.internal:65535', 'host.containers.internal', 65535,
     'ws://host.containers.internal:65535/v0/stream'),
])
def test_the_trusted_daemon_urls(url, host, port, ws_url):
    settings = good(daemon_url=url)
    assert settings.daemon_host == host
    assert settings.daemon_port == port
    assert settings.ws_url == ws_url
    assert settings.daemon_url == ws_url.replace('ws://', 'http://').replace('/v0/stream', '')
    assert settings.allow_remote_daemon is False


@pytest.mark.parametrize('url', [
    'http://192.0.2.10:7810', 'http://example.com:7810', 'http://robot-pc:7810',
    'http://127.0.0.2:7810', 'http://127.1:7810', 'http://0.0.0.0:7810',
    'http://[::ffff:127.0.0.1]:7810', 'http://[0:0:0:0:0:0:0:1]:7810',
    'http://localhost.example.com:7810', 'http://2130706433:7810',
    'http://docker.internal:7810', 'http://[2001:db8::1]:7810',
])
def test_a_remote_host_is_refused_unless_allowed(url):
    problem = one(daemon_url=url)
    assert problem.startswith('daemon_url: ')
    assert 'allow_remote_daemon' in problem
    assert 'token' in problem
    settings = good(daemon_url=url, allow_remote_daemon=True)
    assert settings.allow_remote_daemon is True
    assert settings.daemon_url == url
    assert settings.ws_url == url.replace('http://', 'ws://') + '/v0/stream'


def test_allow_remote_daemon_changes_nothing_for_a_local_daemon():
    assert good(allow_remote_daemon=True).daemon_host == '127.0.0.1'


@pytest.mark.parametrize('url, said', [
    ('https://127.0.0.1:7810', 'https'),
    ('HTTPS://127.0.0.1:7810', 'https'),
    ('http://127.0.0.1', 'port'),
    ('http://127.0.0.1/', 'port'),
    ('http://127.0.0.1:', 'port'),
    ('http://localhost', 'port'),
    ('http://[::1]', 'port'),
    ('http://127.0.0.1:0', 'port'),
    ('http://127.0.0.1:65536', 'port'),
    ('http://127.0.0.1:port', 'port'),
    ('http://127.0.0.1:-1', 'port'),
    ('http://user@127.0.0.1:7810', 'user info'),
    ('http://user:secret-word@127.0.0.1:7810', 'user info'),
    ('http://@127.0.0.1:7810', 'user info'),
    ('http://127.0.0.1:7810/v0', 'path'),
    ('http://127.0.0.1:7810/v0/stream', 'path'),
    ('http://127.0.0.1:7810//', 'path'),
    ('http://127.0.0.1:7810/?x=1', 'query'),
    ('http://127.0.0.1:7810?x=1', 'query'),
    ('http://127.0.0.1:7810/?', 'query'),
    ('http://127.0.0.1:7810/#top', 'fragment'),
    ('http://127.0.0.1:7810#', 'fragment'),
    ('http://:7810', 'host'),
    ('http://exa_mple:7810', 'host'),
    ('http://-bad-:7810', 'host'),
    ('http://localhost.:7810', 'host'),
    ('http://%6c%6f%63alhost:7810', 'host'),
])
def test_a_daemon_url_of_another_form_is_refused(url, said):
    for allow in (False, True):
        problems = bad(daemon_url=url, allow_remote_daemon=allow)
        assert all(problem.startswith('daemon_url: ') for problem in problems)
        assert any(said in problem for problem in problems), problems


@pytest.mark.parametrize('url', [
    '', ' ', '127.0.0.1:7810', 'localhost', 'ws://127.0.0.1:7810', 'ftp://127.0.0.1:7810',
    'http:/127.0.0.1:7810', 'http//127.0.0.1:7810', '//127.0.0.1:7810',
    'http://127.0.0.1:7810 ', ' http://127.0.0.1:7810', 'http://127.0.0.1 :7810',
    'http://127.0.0.1:7810\n', 'http://127.0.0.1:78\t10', 'http://[::1:7810',
    'http://::1:7810', 'http://[::1%eth0]:7810', 'http://[not-an-address]:7810',
    'http://127.0.0.1:7810\\@example.com', 'http://localhost\u3002example:7810',
    'http://127.0.0.1:\uff17810',
])
def test_a_daemon_url_that_cannot_be_read_is_refused(url):
    for allow in (False, True):
        problems = bad(daemon_url=url, allow_remote_daemon=allow)
        assert all(problem.startswith('daemon_url: ') for problem in problems)


def test_a_problem_never_repeats_the_daemon_url():
    for url in ('http://user:secret-word@127.0.0.1:7810', 'https://user:secret-word@h:1',
                'http://user:secret-word@example.com:7810/path?x#y',
                'http://secret-word@[::1', 'secret-word', 'http://h:1/secret-word'):
        problems = bad(daemon_url=url)
        assert not any('secret-word' in problem for problem in problems), problems


@pytest.mark.parametrize('source', ['auto', 'daemon', 'bridge'])
def test_the_stamp_sources(source):
    assert good(stamp_source=source).stamp_source == source


@pytest.mark.parametrize('source', ['', 'Auto', 'DAEMON', 'ros', 'system', 'bridge ', 'node'])
def test_another_stamp_source_is_a_problem(source):
    problem = one(stamp_source=source)
    assert problem.startswith('stamp_source: ')
    assert 'auto, daemon, bridge' in problem


@pytest.mark.parametrize('fmt', ['{unit}_{joint}', 'joint_{axis}', '{controller}/{unit}/{joint}',
                                 'fixed', '{{{joint}}}'])
def test_the_placeholders_of_the_joint_name_format(fmt):
    assert good(joint_name_format=fmt).joint_name_format == fmt


@pytest.mark.parametrize('fmt', ['', '{robot}_{joint}', '{}', '{unit', '{axis:02d}', '{joint!r}',
                                 '{unit.name}'])
def test_a_wrong_joint_name_format_is_a_problem(fmt):
    assert one(joint_name_format=fmt).startswith('joint_name_format: ')


def test_signal_entries():
    problems = bad(io_signals=['cell-1:do_x', 'do_y', 'cell-1:', 'cell-1:do-x', 'cell-1:do_x',
                               'cell 2:do_z', 'cell-1:do.x'])
    assert len(problems) == 6
    assert all(problem.startswith('io_signals: ') for problem in problems)
    assert '<controller id>:<signal>' in problems[0]
    assert '<controller id>:<signal>' in problems[1]
    assert '"io/do_x"' in problems[2]
    assert 'listed twice' in problems[3]
    assert 'cannot be a controller id' in problems[4]
    assert '"io/do_x"' in problems[5]


def test_an_entry_for_a_controller_outside_the_controllers_list_is_a_problem():
    # Such an entry would have no effect, and nobody would be told: no topic, no status.
    problem = one(controllers=['cell-1'], io_signals=['cell-1:do_x', 'cell-2:do_x'])
    assert problem == ('io_signals: an entry names the controller "cell-2", which is not in '
                       'controllers (cell-1)')
    problem = one(controllers=['cell-1', 'cell-9'], joint_name_overrides=['cell-3/ROB_1/J1=a'])
    assert problem == ('joint_name_overrides: an entry names the controller "cell-3", which '
                       'is not in controllers (cell-1, cell-9)')
    problems = bad(controllers=['cell-1'], io_signals=['cell-2:do_x', 'cell-3:do_x'],
                   joint_name_overrides=['cell-2/ROB_1/J1=a', 'cell-1/ROB_1/J1=b'])
    assert [problem.split(':')[0] for problem in problems] == [
        'joint_name_overrides', 'io_signals', 'io_signals']
    assert '"cell-2"' in problems[0] and '"cell-2"' in problems[1] and '"cell-3"' in problems[2]
    # One problem per controller, not per entry.
    assert len(bad(controllers=['cell-1'], io_signals=['cell-2:do_x', 'cell-2:do_y'])) == 1
    # A bad entry is reported for what it is, not a second time for its controller.
    problems = bad(controllers=['cell-1'], io_signals=['cell 2:do_x'])
    assert len(problems) == 1 and 'cannot be a controller id' in problems[0]
    # A controllers list that is itself wrong, or an entry list that is, gives no such problem.
    problems = bad(controllers='cell-1', io_signals=['cell-2:do_x'])
    assert len(problems) == 1 and problems[0].startswith('controllers: ')
    problems = bad(controllers=['cell-1'], io_signals='cell-2:do_x')
    assert len(problems) == 1 and problems[0].startswith('io_signals: ')


def test_with_every_controller_bridged_an_entry_for_any_controller_is_taken():
    # The daemon decides which controllers there are; a name it does not serve gets a
    # status of its own from the node.
    settings = good(io_signals=['cell-2:do_x'], joint_name_overrides=['cell-3/ROB_1/J1=a'])
    assert dict(settings.io_signals) == {'cell-2': ('do_x',)}
    assert dict(settings.joint_name_overrides) == {('cell-3', 'ROB_1', 'J1'): 'a'}
    settings = good(controllers=['cell-1', 'cell-2'], primary_controller='cell-1',
                    io_signals=['cell-2:do_x', 'cell-1:do_y'],
                    joint_name_overrides=['cell-1/ROB_1/J1=a', 'cell-2/ROB_1/J1=b'])
    assert set(settings.io_signals) == {'cell-1', 'cell-2'}


def test_override_entries():
    problems = bad(joint_name_overrides=[
        'cell-1/ROB_1/J1=joint_1', 'cell-1/ROB_1=joint_2', 'cell-1/ROB_1/J1=other', '=x'])
    assert len(problems) == 3
    assert all(problem.startswith('joint_name_overrides: ') for problem in problems)
    assert '<controller id>/<unit>/<joint>=<name>' in problems[0]
    assert 'already named "joint_1"' in problems[1]


def test_controllers():
    assert good(controllers=['cell-1', 'irb1600']).controllers == ('cell-1', 'irb1600')
    problem = one(controllers=['cell-1', 'cell-1'])
    assert problem.startswith('controllers: ') and 'listed twice' in problem
    for entry in ('', 'a b', 'a/b', 'a:b', '../x', 'caf\u00e9'):
        problem = one(controllers=['cell-1', entry])
        assert problem.startswith('controllers: ')
        assert 'cannot be a controller id' in problem


def test_the_primary_controller():
    assert good(primary_controller='cell-1').primary_controller == 'cell-1'
    assert good(controllers=['cell-1', 'cell-2'],
                primary_controller='cell-2').primary_controller == 'cell-2'
    problem = one(controllers=['cell-1', 'cell-2'], primary_controller='cell-3')
    assert problem.startswith('primary_controller: ')
    assert 'cell-1, cell-2' in problem
    problem = one(primary_controller='a/b')
    assert problem.startswith('primary_controller: ')
    assert 'cannot be a controller id' in problem


def test_simulated_time_is_a_problem():
    problem = one(use_sim_time=True)
    assert problem.startswith('use_sim_time: ')
    assert 'not supported' in problem
    assert good(use_sim_time=False) == good()
    assert good(use_sim_time=None) == good()


def test_every_problem_is_listed_at_once():
    problems = bad(
        daemon_url='https://example.com/x', refresh_hz=2, stale_after_s=0.1,
        reconnect_min_s=30.0, reconnect_max_s=15.0, stamp_source='gps',
        joint_name_format='{robot}', io_signals=['x'], joint_name_overrides=['y'],
        controllers=['a', 'a'], primary_controller='b', use_sim_time=True,
        allow_replay='yes')
    starts = [problem.split(':')[0] for problem in problems]
    assert sorted(starts) == sorted([
        'daemon_url', 'refresh_hz', 'stale_after_s', 'reconnect_min_s', 'stamp_source',
        'joint_name_format', 'io_signals', 'joint_name_overrides', 'controllers',
        'primary_controller', 'use_sim_time', 'allow_replay'])


def test_a_wrong_value_does_not_cause_a_second_problem():
    # refresh_hz is wrong by type; the rule against stale_after_s is then not judged.
    assert len(bad(refresh_hz='fast', stale_after_s=0.5)) == 1
    assert len(bad(reconnect_min_s=None, reconnect_max_s=1.0)) == 1
    assert len(bad(controllers='cell-1', primary_controller='cell-2')) == 1
    assert len(bad(daemon_url=5, allow_remote_daemon=False)) == 1


def test_the_values_given_are_not_changed():
    values = {'controllers': ['cell-1'], 'io_signals': ['cell-1:do_x'],
              'joint_name_overrides': ['cell-1/ROB_1/J1=a'], 'refresh_hz': 2.0}
    before = {key: list(value) if isinstance(value, list) else value
              for key, value in values.items()}
    settings_from(values)
    assert values == before

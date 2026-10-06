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
The launch file of the bridge: its arguments, its overrides and its node action.

The launch file is loaded from the source folder, as it is now. No process is started
and no ROS graph is touched: the description is visited with a launch context, and the
node action that comes out is asked what it would hand to the node.
"""

import os
from pathlib import Path
from types import MappingProxyType
from types import SimpleNamespace

from abb_rws_bridge import parameters
from launch import LaunchContext
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events.process import ProcessExited
from launch.launch_description_sources import get_launch_description_from_python_launch_file
from launch.launch_description_sources import load_python_launch_file_as_module
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
from launch_ros.substitutions import ExecutableInPackage
from launch_ros.utilities import evaluate_parameters
import pytest
import yaml

PACKAGE_FOLDER = Path(__file__).resolve().parent.parent
LAUNCH_FILE = PACKAGE_FOLDER / 'launch' / 'bridge.launch.py'
SOURCE_CONFIG = PACKAGE_FOLDER / 'config' / 'bridge.yaml'

ARGUMENT_NAMES = [
    'namespace', 'params_file', 'daemon_url', 'token_file', 'controller', 'allow_replay',
    'log_level',
]
SPECS = {spec.name: spec for spec in parameters.SPECS}

NOT_TRUE_OR_FALSE = [
    'True', 'TRUE', 'False', 'FALSE', 'yes', 'no', 'on', 'off', '1', '0', 'y', 'n',
    ' true', 'true ', 'true\n', 'truefalse', 'maybe', '[true]', '"true"',
]


class _WrittenFileLoader(yaml.SafeLoader):
    """Reads the parameter file launch_ros writes, which tags a Python tuple."""


_WrittenFileLoader.add_constructor(
    'tag:yaml.org,2002:python/tuple',
    lambda loader, node: tuple(loader.construct_sequence(node)))


@pytest.fixture(scope='module')
def bridge_launch():
    """Return the launch file as a module, loaded the way ros2 launch loads it."""
    return load_python_launch_file_as_module(str(LAUNCH_FILE))


def _visit(bridge_launch, arguments):
    """
    Visit the description as a launch run with these arguments would.

    Returns the context and every action the description produced. Arguments that are
    not given take their declared default, exactly as on the command line.
    """
    description = bridge_launch.generate_launch_description()
    context = LaunchContext()
    context.launch_configurations.update(arguments)
    produced = []
    for entity in description.entities:
        produced.extend(entity.visit(context) or [])
    return context, produced


def _node(bridge_launch, arguments):
    """Return (context, the one node action) for these launch arguments."""
    context, produced = _visit(bridge_launch, arguments)
    # The node, and the handler that makes launch fail when the node failed.
    assert [type(action) for action in produced] == [Node, RegisterEventHandler], produced
    return context, produced[0]


def _stored_parameters(node):
    """Return the parameters as the node action keeps them, normalized by launch_ros."""
    name = '_Node__parameters'
    assert hasattr(node, name), (
        'launch_ros.actions.Node no longer keeps its parameters under this name; '
        'this helper needs another way to read them')
    return getattr(node, name)


def _evaluated(bridge_launch, arguments):
    """Return what launch_ros evaluates the parameters of the node action to."""
    context, node = _node(bridge_launch, arguments)
    return evaluate_parameters(context, _stored_parameters(node))


def _command_line(context, node):
    """
    Return the arguments the node would be started with, without the executable.

    Performs the substitutions of the action the way its execution does. A part that
    only the running launch service can resolve reads as None.
    """
    node._perform_substitutions(context)
    parts = []
    for part in node.cmd[1:]:
        try:
            parts.append(perform_substitutions(context, part))
        except Exception:
            parts.append(None)
    return parts


def _parameter_files(command_line):
    """Return the paths that follow --params-file, in order."""
    return [
        command_line[index + 1] for index, part in enumerate(command_line)
        if part == '--params-file'
    ]


def _started(bridge_launch, arguments):
    """
    Return (node, command line, parameter files, written overrides or None).

    The file launch_ros writes for a parameter dictionary is read and removed.
    """
    context, node = _node(bridge_launch, arguments)
    command_line = _command_line(context, node)
    files = _parameter_files(command_line)
    written = None
    for path in files:
        if os.path.basename(path).startswith('launch_params_'):
            with open(path, encoding='utf-8') as handle:
                written = yaml.load(handle, Loader=_WrittenFileLoader)
            os.unlink(path)
    return node, command_line, files, written


# The description and its arguments.


def test_launch_file_is_plain_ascii_with_lf_line_endings():
    data = LAUNCH_FILE.read_bytes()
    assert b'\r' not in data, 'bridge.launch.py has a carriage return; it must use LF'
    assert b'\t' not in data
    assert max(data) < 127
    assert data.endswith(b'\n')


def test_description_builds(bridge_launch):
    assert isinstance(bridge_launch.generate_launch_description(), LaunchDescription)


def test_description_loads_the_way_ros2_launch_loads_it():
    description = get_launch_description_from_python_launch_file(str(LAUNCH_FILE))
    assert isinstance(description, LaunchDescription)


def test_description_declares_the_seven_arguments(bridge_launch):
    declared = bridge_launch.generate_launch_description().get_launch_arguments()
    assert [argument.name for argument in declared] == ARGUMENT_NAMES


def test_every_argument_is_empty_by_default(bridge_launch):
    context = LaunchContext()
    for argument in bridge_launch.generate_launch_description().get_launch_arguments():
        assert argument.default_value is not None, f'{argument.name} has no default'
        assert perform_substitutions(context, argument.default_value) == '', argument.name
        assert argument.choices is None, argument.name


def test_every_argument_has_a_description(bridge_launch):
    for argument in bridge_launch.generate_launch_description().get_launch_arguments():
        assert len(argument.description) >= 30, argument.name
        assert 'Empty: ' in argument.description, (
            f'{argument.name} does not say what an empty argument means')
        assert argument.description.isascii()


def test_description_is_the_arguments_and_one_opaque_function(bridge_launch):
    entities = bridge_launch.generate_launch_description().entities
    kinds = [type(entity) for entity in entities]
    assert kinds == [DeclareLaunchArgument] * 7 + [OpaqueFunction]


def test_argument_table_of_the_module_is_the_declared_one(bridge_launch):
    assert [name for name, _ in bridge_launch.ARGUMENTS] == ARGUMENT_NAMES


# parameter_overrides: a plain dictionary with real types.


def test_overrides_of_nothing_are_empty(bridge_launch):
    assert bridge_launch.parameter_overrides({}) == {}
    assert bridge_launch.parameter_overrides(dict.fromkeys(ARGUMENT_NAMES, '')) == {}


def test_overrides_are_a_plain_dict(bridge_launch):
    assert type(bridge_launch.parameter_overrides({})) is dict
    assert type(bridge_launch.parameter_overrides({'controller': 'x'})) is dict


def test_controller_gives_a_list_of_str_and_the_primary_controller(bridge_launch):
    overrides = bridge_launch.parameter_overrides({'controller': 'cell-1'})
    assert overrides == {'controllers': ['cell-1'], 'primary_controller': 'cell-1'}
    assert type(overrides['controllers']) is list
    assert [type(entry) for entry in overrides['controllers']] == [str]
    assert type(overrides['primary_controller']) is str


@pytest.mark.parametrize('text', ['1', '08', 'true', 'y', '1e3', 'null', '~', '[a]'])
def test_controller_stays_text_whatever_it_looks_like(bridge_launch, text):
    overrides = bridge_launch.parameter_overrides({'controller': text})
    assert overrides == {'controllers': [text], 'primary_controller': text}
    assert type(overrides['controllers'][0]) is str
    assert type(overrides['primary_controller']) is str


@pytest.mark.parametrize(('text', 'value'), [('true', True), ('false', False)])
def test_allow_replay_gives_a_bool(bridge_launch, text, value):
    overrides = bridge_launch.parameter_overrides({'allow_replay': text})
    assert overrides == {'allow_replay': value}
    assert type(overrides['allow_replay']) is bool


@pytest.mark.parametrize('text', NOT_TRUE_OR_FALSE)
def test_allow_replay_other_than_true_or_false_is_a_readable_error(bridge_launch, text):
    with pytest.raises(ValueError) as raised:
        bridge_launch.parameter_overrides({'allow_replay': text})
    message = str(raised.value)
    assert message.startswith('allow_replay:=')
    assert 'allow_replay:=true' in message and 'allow_replay:=false' in message
    assert '\n' not in message and message.isascii() and message.isprintable()


def test_error_shows_a_long_or_odd_value_cut_and_printable(bridge_launch):
    odd = 'x' + chr(27) + '[31m' + chr(233) + 'y' * 500
    with pytest.raises(ValueError) as raised:
        bridge_launch.parameter_overrides({'allow_replay': odd})
    message = str(raised.value)
    assert len(message) < 200
    assert message.isascii() and message.isprintable()


def test_daemon_url_and_token_file_pass_as_text(bridge_launch):
    overrides = bridge_launch.parameter_overrides({
        'daemon_url': 'http://host.docker.internal:7810',
        'token_file': '/run/secrets/abb_rws_daemon_token',
    })
    assert overrides == {
        'daemon_url': 'http://host.docker.internal:7810',
        'token_file': '/run/secrets/abb_rws_daemon_token',
    }
    assert all(type(value) is str for value in overrides.values())


def test_empty_argument_gives_no_key(bridge_launch):
    overrides = bridge_launch.parameter_overrides({
        'daemon_url': '', 'token_file': '', 'controller': 'x', 'allow_replay': '',
    })
    assert sorted(overrides) == ['controllers', 'primary_controller']


def test_arguments_that_are_not_parameters_give_no_key(bridge_launch):
    overrides = bridge_launch.parameter_overrides({
        'namespace': 'cell', 'params_file': '/tmp/other.yaml', 'log_level': 'debug',
        'something_else': 'x',
    })
    assert overrides == {}


def test_all_four_together(bridge_launch):
    given = MappingProxyType({
        'namespace': 'cell',
        'daemon_url': 'http://127.0.0.1:7811',
        'token_file': '/tmp/token',
        'controller': 'cell-1',
        'allow_replay': 'false',
    })
    assert bridge_launch.parameter_overrides(given) == {
        'daemon_url': 'http://127.0.0.1:7811',
        'token_file': '/tmp/token',
        'controllers': ['cell-1'],
        'primary_controller': 'cell-1',
        'allow_replay': False,
    }


def test_every_override_is_a_parameter_of_the_node_with_the_type_of_its_spec(bridge_launch):
    overrides = bridge_launch.parameter_overrides({
        'daemon_url': 'http://127.0.0.1:7811', 'token_file': '/tmp/token',
        'controller': 'cell-1', 'allow_replay': 'true',
    })
    assert len(overrides) == 5
    python_type = {'string': str, 'bool': bool, 'double': float, 'string_array': list}
    for name, value in overrides.items():
        assert name in SPECS, f'{name} is not a parameter of the node'
        assert type(value) is python_type[SPECS[name].kind], name


def test_overrides_pass_the_rules_of_the_node(bridge_launch):
    overrides = bridge_launch.parameter_overrides({
        'daemon_url': 'http://host.docker.internal:7810', 'token_file': '/tmp/token',
        'controller': 'cell-1', 'allow_replay': 'true',
    })
    settings, problems = parameters.settings_from(overrides)
    assert problems == []
    assert settings.controllers == ('cell-1',)
    assert settings.primary_controller == 'cell-1'
    assert settings.allow_replay is True
    assert settings.daemon_host == 'host.docker.internal'


# The node action, as launch_ros evaluates it.


def test_node_action_names_the_bridge_executable(bridge_launch):
    _, node = _node(bridge_launch, {'params_file': str(SOURCE_CONFIG)})
    assert node.node_package == 'abb_rws_bridge'
    assert node.node_executable == 'bridge'


def test_bridge_package_installs_that_executable(bridge_launch):
    context, node = _node(bridge_launch, {'params_file': str(SOURCE_CONFIG)})
    executable = ExecutableInPackage(
        package=node.node_package, executable=node.node_executable).perform(context)
    assert os.path.basename(executable) == 'bridge'
    assert os.path.isfile(executable)


def test_controller_and_allow_replay_evaluate_to_a_tuple_of_str_and_a_bool(bridge_launch):
    evaluated = _evaluated(bridge_launch, {
        'params_file': str(SOURCE_CONFIG), 'controller': 'x', 'allow_replay': 'true',
    })
    assert len(evaluated) == 2
    assert evaluated[0] == SOURCE_CONFIG
    overrides = evaluated[1]
    assert overrides == {
        'controllers': ('x',), 'primary_controller': 'x', 'allow_replay': True,
    }
    assert type(overrides['controllers']) is tuple
    assert [type(entry) for entry in overrides['controllers']] == [str]
    assert type(overrides['primary_controller']) is str
    assert type(overrides['allow_replay']) is bool


@pytest.mark.parametrize('text', ['1', '08', 'true', 'null', '1.5', 'cell-1'])
def test_controller_that_looks_like_another_type_evaluates_to_text(bridge_launch, text):
    overrides = _evaluated(bridge_launch, {
        'params_file': str(SOURCE_CONFIG), 'controller': text,
    })[1]
    assert overrides == {'controllers': (text,), 'primary_controller': text}
    assert type(overrides['controllers'][0]) is str
    assert type(overrides['primary_controller']) is str


def test_allow_replay_false_evaluates_to_false(bridge_launch):
    overrides = _evaluated(bridge_launch, {
        'params_file': str(SOURCE_CONFIG), 'allow_replay': 'false',
    })[1]
    assert overrides == {'allow_replay': False}
    assert overrides['allow_replay'] is False


def test_evaluated_overrides_pass_the_rules_of_the_node(bridge_launch):
    overrides = _evaluated(bridge_launch, {
        'params_file': str(SOURCE_CONFIG), 'controller': 'x', 'allow_replay': 'true',
        'daemon_url': 'http://127.0.0.1:7811', 'token_file': '/tmp/token',
    })[1]
    settings, problems = parameters.settings_from(overrides)
    assert problems == []
    assert settings.controllers == ('x',) and settings.allow_replay is True
    assert settings.daemon_url == 'http://127.0.0.1:7811'
    assert settings.token_file == '/tmp/token'


def test_no_argument_gives_the_parameter_file_and_nothing_else(bridge_launch):
    evaluated = _evaluated(bridge_launch, {'params_file': str(SOURCE_CONFIG)})
    assert list(evaluated) == [SOURCE_CONFIG]


def test_empty_params_file_is_the_installed_bridge_yaml(bridge_launch):
    evaluated = _evaluated(bridge_launch, {})
    assert len(evaluated) == 1
    installed = evaluated[0]
    assert installed.parts[-4:] == ('share', 'abb_rws_ros2', 'config', 'bridge.yaml')
    assert installed.is_file()
    assert list(yaml.safe_load(installed.read_text(encoding='ascii'))) == ['/**']


def test_node_is_started_with_the_file_first_and_the_overrides_after_it(bridge_launch):
    node, command_line, files, written = _started(bridge_launch, {
        'params_file': str(SOURCE_CONFIG), 'controller': 'x', 'allow_replay': 'true',
    })
    assert len(files) == 2
    assert files[0] == str(SOURCE_CONFIG)
    assert os.path.basename(files[1]).startswith('launch_params_')
    assert list(written) == ['/**'], 'the overrides must apply under any node name'
    values = written['/**']['ros__parameters']
    assert sorted(values) == ['allow_replay', 'controllers', 'primary_controller']
    assert values['allow_replay'] is True
    assert not isinstance(values['controllers'], str)
    assert list(values['controllers']) == ['x']
    assert type(values['controllers'][0]) is str
    assert values['primary_controller'] == 'x'
    assert command_line[0] == '--ros-args'
    assert '--log-level' not in command_line
    assert node.expanded_node_namespace == Node.UNSPECIFIED_NODE_NAMESPACE


def test_node_is_started_with_the_file_only_when_nothing_is_overridden(bridge_launch):
    _, command_line, files, written = _started(
        bridge_launch, {'params_file': str(SOURCE_CONFIG)})
    assert files == [str(SOURCE_CONFIG)]
    assert written is None
    assert command_line == ['--ros-args', '--params-file', str(SOURCE_CONFIG)]


def test_node_is_given_no_name_so_it_keeps_its_own(bridge_launch):
    _, command_line, _, _ = _started(bridge_launch, {'params_file': str(SOURCE_CONFIG)})
    assert not any(part and part.startswith('__node:=') for part in command_line)
    assert '-r' not in command_line


def test_namespace_argument_is_the_namespace_of_the_node(bridge_launch):
    node, _, _, _ = _started(
        bridge_launch, {'params_file': str(SOURCE_CONFIG), 'namespace': 'cell'})
    assert node.expanded_node_namespace == '/cell'


def test_empty_namespace_keeps_the_one_an_including_launch_file_pushed(bridge_launch):
    context, node = _node(bridge_launch, {'params_file': str(SOURCE_CONFIG)})
    context.launch_configurations['ros_namespace'] = '/outer'
    _command_line(context, node)
    assert node.expanded_node_namespace == '/outer'


@pytest.mark.parametrize('level', ['debug', 'info', 'warn', 'error', 'fatal', 'DEBUG'])
def test_log_level_is_passed_to_the_node(bridge_launch, level):
    _, command_line, _, _ = _started(
        bridge_launch, {'params_file': str(SOURCE_CONFIG), 'log_level': level})
    assert command_line[:3] == ['--ros-args', '--log-level', level]


# Launch errors: a wrong argument stops the launch with one readable line.


@pytest.mark.parametrize('text', ['yes', 'True', '1'])
def test_wrong_allow_replay_stops_the_launch(bridge_launch, text):
    with pytest.raises(ValueError, match='allow_replay:=true or allow_replay:=false'):
        _visit(bridge_launch, {'params_file': str(SOURCE_CONFIG), 'allow_replay': text})


@pytest.mark.parametrize('level', ['loud', 'warning', 'debug,info', 'abb_rws_bridge:=debug'])
def test_wrong_log_level_stops_the_launch(bridge_launch, level):
    with pytest.raises(ValueError, match='log_level:=') as raised:
        _visit(bridge_launch, {'params_file': str(SOURCE_CONFIG), 'log_level': level})
    assert 'debug, info, warn, error, fatal' in str(raised.value)


def test_missing_parameter_file_stops_the_launch(bridge_launch, tmp_path):
    missing = tmp_path / 'no-such.yaml'
    with pytest.raises(ValueError, match='is not a file') as raised:
        _visit(bridge_launch, {'params_file': str(missing)})
    assert str(raised.value).startswith('params_file:=')


def test_folder_as_parameter_file_stops_the_launch(bridge_launch, tmp_path):
    with pytest.raises(ValueError, match='is not a file'):
        _visit(bridge_launch, {'params_file': str(tmp_path)})


# The end of the node: launch fails when the node failed.


def _exited(action, returncode):
    """Return the event launch emits when the process of an action has ended."""
    return ProcessExited(action=action, name='bridge', cmd=['bridge'], cwd=None, env=None,
                         pid=1, returncode=returncode)


def test_the_launch_watches_the_end_of_its_node(bridge_launch):
    context, produced = _visit(bridge_launch, {'params_file': str(SOURCE_CONFIG)})
    node, registration = produced
    handler = registration.event_handler
    assert isinstance(handler, OnProcessExit)
    # Bound to this node action: the end of another process is not its business.
    assert handler.matches(_exited(node, 2))
    other, _ = _visit(bridge_launch, {'params_file': str(SOURCE_CONFIG)})[1]
    assert not handler.matches(_exited(other, 2))
    # A node that failed gives the action that fails the launch; a clean end nothing.
    failing = handler.handle(_exited(node, 2), context)
    assert [type(action) for action in failing] == [OpaqueFunction]
    with pytest.raises(RuntimeError, match='ended with exit code 2;'):
        failing[0].visit(context)
    assert not handler.handle(_exited(node, 0), context)


@pytest.mark.parametrize('returncode', [2, 1, -11, 255])
def test_a_node_that_ended_with_an_error_makes_the_launch_fail(bridge_launch, returncode):
    running = SimpleNamespace(is_shutdown=False)
    actions = bridge_launch.bridge_ended(SimpleNamespace(returncode=returncode), running)
    assert [type(action) for action in actions] == [OpaqueFunction]
    # Launch ends with exit code 1 on an action that raises, and prints the message.
    with pytest.raises(RuntimeError, match=f'ended with exit code {returncode};') as raised:
        actions[0].visit(LaunchContext())
    assert len(str(raised.value)) < 120 and str(raised.value).isascii()


def test_a_node_that_ended_cleanly_changes_nothing(bridge_launch):
    running = SimpleNamespace(is_shutdown=False)
    assert bridge_launch.bridge_ended(SimpleNamespace(returncode=0), running) is None


@pytest.mark.parametrize('returncode', [0, 2, -2, -15, -9])
def test_a_node_that_launch_itself_stopped_changes_nothing(bridge_launch, returncode):
    # Ctrl+C: launch is shutting down and hands the signal on; however the node ends
    # then, the exit code of launch stays its own.
    stopping = SimpleNamespace(is_shutdown=True)
    assert bridge_launch.bridge_ended(SimpleNamespace(returncode=returncode), stopping) is None

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
Launch the ABB RWS supervisory bridge: one read-only node that reads abb-rws-daemon.

Every argument is empty by default, and an empty argument changes nothing: the
parameter file decides. An argument that is given wins over the parameter file.

    ros2 launch abb_rws_ros2 bridge.launch.py
    ros2 launch abb_rws_ros2 bridge.launch.py controller:=cell-1 namespace:=cell

``ros2 launch abb_rws_ros2 bridge.launch.py --show-args`` lists the arguments.

When the node ends with an error (exit code 2: it could not start with its
parameters), the launch ends with exit code 1, so a container or a service manager
does not take it for a clean stop. Stop a running launch with SIGINT (Ctrl+C): launch
hands that on to the node, which then closes its connection to the daemon in order.
"""

from collections.abc import Mapping
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

BRINGUP_PACKAGE = 'abb_rws_ros2'
BRIDGE_PACKAGE = 'abb_rws_bridge'
BRIDGE_EXECUTABLE = 'bridge'
LOG_LEVELS = ('debug', 'info', 'warn', 'error', 'fatal')

_FILE_DECIDES = 'Empty: the parameter file decides.'

# The launch arguments, in the order --show-args lists them. All default to empty.
ARGUMENTS = (
    ('namespace',
     'ROS namespace of the node and of its data topics. Empty: none.'),
    ('params_file',
     'Parameter file of the node. Empty: config/bridge.yaml as installed with this '
     'package.'),
    ('daemon_url',
     'Where abb-rws-daemon listens, http://<host>:<port> (the parameter daemon_url). '
     + _FILE_DECIDES),
    ('token_file',
     'File that holds the access token of the daemon: the path, never the token itself '
     '(the parameter token_file). ' + _FILE_DECIDES),
    ('controller',
     'Bridge this one controller of the daemon, with its topics directly in the '
     'namespace (sets the parameters controllers and primary_controller). '
     + _FILE_DECIDES),
    ('allow_replay',
     'true or false: publish what a daemon in replay mode serves, under replay/ names '
     'only (the parameter allow_replay). ' + _FILE_DECIDES),
    ('log_level',
     'Log level of the node: debug, info, warn, error or fatal. Empty: the node keeps '
     'its own.'),
)


def parameter_overrides(values: Mapping[str, str]) -> dict:
    """
    Return the node parameters that the launch arguments override, with real types.

    values maps launch argument names to their resolved texts. An argument that is
    empty or missing gives no key, so the parameter file keeps its say. ``controller``
    gives ``controllers`` (a list of one str) and ``primary_controller``;
    ``allow_replay`` gives a bool and must be the text ``true`` or ``false``, anything
    else raises ValueError with a message for the person who typed it. Arguments that
    are not parameters (``namespace``, ``params_file``, ``log_level``) are not looked
    at.
    """
    overrides = {}
    daemon_url = values.get('daemon_url', '')
    if daemon_url:
        overrides['daemon_url'] = daemon_url
    token_file = values.get('token_file', '')
    if token_file:
        overrides['token_file'] = token_file
    controller = values.get('controller', '')
    if controller:
        overrides['controllers'] = [controller]
        overrides['primary_controller'] = controller
    allow_replay = values.get('allow_replay', '')
    if allow_replay:
        if allow_replay not in ('true', 'false'):
            raise ValueError(
                f'allow_replay:={_shown(allow_replay)} is not possible; '
                'write allow_replay:=true or allow_replay:=false')
        overrides['allow_replay'] = allow_replay == 'true'
    return overrides


def launch_setup(context, *args, **kwargs):
    """Return the bridge node for the launch arguments as they are resolved now."""
    values = {
        name: context.perform_substitution(LaunchConfiguration(name))
        for name, _ in ARGUMENTS
    }

    params_file = values['params_file']
    if not params_file:
        params_file = context.perform_substitution(
            PathJoinSubstitution([FindPackageShare(BRINGUP_PACKAGE), 'config', 'bridge.yaml']))
    if not os.path.isfile(params_file):
        # Without this the node would start on its defaults, with only a warning.
        raise ValueError(f'params_file:={_shown(params_file)} is not a file')
    parameters = [params_file]
    overrides = parameter_overrides(values)
    if overrides:
        parameters.append(overrides)

    ros_arguments = None
    log_level = values['log_level']
    if log_level:
        if log_level.lower() not in LOG_LEVELS:
            raise ValueError(
                f'log_level:={_shown(log_level)} is not possible; '
                f'write one of {", ".join(LOG_LEVELS)}')
        ros_arguments = ['--log-level', log_level]

    node = Node(
        package=BRIDGE_PACKAGE,
        executable=BRIDGE_EXECUTABLE,
        # None, not '': for '' launch_ros passes __ns:=/ to the node, for None it
        # passes nothing, so an empty argument changes nothing.
        namespace=values['namespace'] or None,
        parameters=parameters,
        ros_arguments=ros_arguments,
        output='screen',
    )
    return [node, RegisterEventHandler(OnProcessExit(target_action=node, on_exit=bridge_ended))]


def bridge_ended(event, context):
    """
    Return what launch does once the bridge node has ended: fail when the node failed.

    ``ros2 launch`` ends with exit code 0 whatever the exit code of its node was. So
    when the node ended with an error and launch is not shutting down itself, an
    action that raises is handed back: launch reports it and ends with exit code 1.
    A node that ended with 0, or that was stopped by launch (Ctrl+C), changes nothing.
    """
    if event.returncode == 0 or context.is_shutdown:
        return None
    return [OpaqueFunction(function=_fail, args=[event.returncode])]


def _fail(context, returncode):
    """Raise: the one way a launch file makes ``ros2 launch`` end with an error."""
    raise RuntimeError(
        f'the bridge node ended with exit code {returncode}; its own lines above say why')


def generate_launch_description():
    """Return the launch description: the arguments, then the bridge node."""
    declared = [
        DeclareLaunchArgument(name, default_value='', description=description)
        for name, description in ARGUMENTS
    ]
    return LaunchDescription([*declared, OpaqueFunction(function=launch_setup)])


def _shown(text: str) -> str:
    """Return a short, printable form of an argument for an error message."""
    cut = str(text)[:80]
    return ''.join(character if ' ' <= character <= '~' else '?' for character in cut)

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
Shared fixtures of the bridge tests.

This folder has no ``__init__.py``: pytest puts it on the import path, so the helpers
beside this file are imported as top-level modules (``import frames``).

Both fixtures are lazy. A test that asks for neither needs no ROS installation.
"""

import os
from pathlib import Path

import pytest

FIXTURES_VARIABLE = 'ABB_RWS_TEST_FIXTURES'
_ROS_VARIABLES = ('ROS_DOMAIN_ID', 'ROS_AUTOMATIC_DISCOVERY_RANGE')


@pytest.fixture(scope='session')
def ros_domain():
    """
    Give the test session a ROS domain of its own and keep its graph on this host.

    Takes a free domain id for the whole session and sets ROS_DOMAIN_ID and
    ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST in the environment, so that every
    rclpy.init that follows and every child process use them. Ask for it before the
    first rclpy.init. Both variables are restored when the session ends.
    """
    import domain_coordinator

    before = {name: os.environ.get(name) for name in _ROS_VARIABLES}
    with domain_coordinator.domain_id() as domain_id:
        os.environ['ROS_DOMAIN_ID'] = str(domain_id)
        os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
        try:
            yield int(domain_id)
        finally:
            for name, value in before.items():
                if value is None:
                    os.environ.pop(name, None)
                else:
                    os.environ[name] = value


@pytest.fixture(scope='session')
def fixtures_dir():
    """
    Return the folder that holds the recorded fixtures.

    It is the folder named by ABB_RWS_TEST_FIXTURES when that variable is set, otherwise
    ``fixtures`` beside this file. The folder may not exist or may hold no recording: a
    test that needs one skips with the reason.
    """
    named = os.environ.get(FIXTURES_VARIABLE)
    if named:
        return Path(named)
    return Path(__file__).resolve().parent / 'fixtures'

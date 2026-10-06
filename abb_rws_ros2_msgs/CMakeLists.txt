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

cmake_minimum_required(VERSION 3.8)
project(abb_rws_ros2_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Sets MSG_FILES: the generated list of the messages. No glob, so a stray file
# in msg/ never becomes a message.
include(msg/interfaces.cmake)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  DEPENDENCIES builtin_interfaces std_msgs
)

# The vendored schema, where it came from, and the JSON-to-ROS field mapping.
install(DIRECTORY schema/ DESTINATION share/${PROJECT_NAME}/schema)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_generated test/test_generated.py)
  ament_add_pytest_test(test_generator_rules test/test_generator_rules.py)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()

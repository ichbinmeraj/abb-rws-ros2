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

# Runtime image: the three packages, built and installed under /opt/abb_rws.
# It holds no access token and no recording; the daemon's token file is
# mounted when the container starts (see compose.yaml).
#
#   docker build -f docker/Dockerfile -t abb-rws-ros2:jazzy .

ARG ROS_DISTRO=jazzy

# Build stage: sources, every dependency, one package at a time.
FROM ros:${ROS_DISTRO}-ros-base AS build

WORKDIR /ws

# The manifests go in first, so the dependency layer is made again only when
# one of them changes, not on every change of a source file.
COPY abb_rws_ros2_msgs/package.xml src/abb_rws_ros2_msgs/package.xml
COPY abb_rws_bridge/package.xml src/abb_rws_bridge/package.xml
COPY abb_rws_ros2/package.xml src/abb_rws_ros2/package.xml

# rosdep update reads the rosdistro index (which key maps to which package);
# every package itself comes from the Ubuntu and ROS apt repositories.
RUN apt-get update \
    && rosdep update --rosdistro "${ROS_DISTRO}" \
    && rosdep install --from-paths src --ignore-src -y --rosdistro "${ROS_DISTRO}" \
    && rm -rf /var/lib/apt/lists/*

COPY abb_rws_ros2_msgs src/abb_rws_ros2_msgs
COPY abb_rws_bridge src/abb_rws_bridge
COPY abb_rws_ros2 src/abb_rws_ros2

RUN . "/opt/ros/${ROS_DISTRO}/setup.sh" \
    && MAKEFLAGS=-j2 colcon build \
        --install-base /opt/abb_rws \
        --executor sequential \
        --cmake-args -DBUILD_TESTING=OFF

# Runtime stage: what the packages need to run, and the install tree.
FROM ros:${ROS_DISTRO}-ros-base

COPY abb_rws_ros2_msgs/package.xml /tmp/deps/abb_rws_ros2_msgs/package.xml
COPY abb_rws_bridge/package.xml /tmp/deps/abb_rws_bridge/package.xml
COPY abb_rws_ros2/package.xml /tmp/deps/abb_rws_ros2/package.xml

RUN apt-get update \
    && rosdep update --rosdistro "${ROS_DISTRO}" \
    && rosdep install --from-paths /tmp/deps --ignore-src -y --rosdistro "${ROS_DISTRO}" \
        --dependency-types exec \
    && rm -rf /var/lib/apt/lists/* /tmp/deps

COPY --from=build /opt/abb_rws /opt/abb_rws

# The mode is set here, so it does not depend on the mode the file has in git.
COPY --chmod=0755 docker/entrypoint.sh /entrypoint.sh

# Logs go to /tmp, so the container also runs on a read-only file system.
ENV ROS_LOG_DIR=/tmp/ros-log \
    PYTHONDONTWRITEBYTECODE=1

USER ubuntu

# "docker stop" and "docker compose down" send this signal. ros2 launch hands
# SIGINT on to the node, which then closes its connection to the daemon in
# order; on SIGTERM launch would end at once and the node would be killed.
STOPSIGNAL SIGINT

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "abb_rws_ros2", "bridge.launch.py"]

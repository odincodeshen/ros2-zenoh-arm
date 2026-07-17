#!/bin/bash
# Shared package layer for the ros2-zenoh-arm image family.
# Installs rmw_zenoh (official apt binary) + common networking/dev tools.
set -e
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
apt-get update
apt-get install -y --no-install-recommends \
    git curl wget vim nano iputils-ping iproute2 net-tools just iftop \
    ros-${ROS_DISTRO}-rmw-zenoh-cpp
apt-get clean
rm -rf /var/lib/apt/lists/*

#!/usr/bin/env bash
set -e

# Setup script for running natively on Ubuntu 22.04 with ROS 2 Humble.
# Assumes ROS 2 Humble is already installed.
# Usage: bash setup.bash

if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo "ERROR: ROS 2 Humble not found at /opt/ros/humble/setup.bash"
    echo "Install it first: https://docs.ros.org/en/humble/Installation.html"
    exit 1
fi

source /opt/ros/humble/setup.bash

echo "==> Installing system dependencies via rosdep..."
sudo apt-get update
rosdep update --rosdistro=humble
rosdep install --from-paths ros2_ws/src --ignore-src -r -y

echo "==> Installing additional tools..."
sudo apt-get install -y python3-colcon-common-extensions

echo "==> Building workspace..."
cd ros2_ws
colcon build --symlink-install
source install/setup.bash

echo ""
echo "Done. Setup completed successfully"
#!/bin/bash
set -e

WORKSPACE=/home/ros/ws
EAGLEYE_DIR=/workspaces/eagleye

# Create ROS workspace
mkdir -p ${WORKSPACE}/src

# Symlink the eagleye repo into the workspace
ln -sf ${EAGLEYE_DIR} ${WORKSPACE}/src/eagleye

# Import dependency repos
cd ${WORKSPACE}/src
vcs import < ${EAGLEYE_DIR}/eagleye.repos

# Install ROS dependencies
cd ${WORKSPACE}
sudo apt-get update
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# Add workspace to bashrc
echo "source ${WORKSPACE}/install/setup.bash" >> ~/.bashrc

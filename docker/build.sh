#!/bin/bash
cd ~/moveit_ws/src
vcs import --recursive < ../moveit2_tutorials.repos
cd ..
apt update && rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
colcon build --mixin release
source install/setup.bash

cd ~/fanuc_ws
git config --global --add safe.directory '*'
colcon build --symlink-install --cmake-args -DBUILD_TESTING=1 -DBUILD_EXAMPLES=1
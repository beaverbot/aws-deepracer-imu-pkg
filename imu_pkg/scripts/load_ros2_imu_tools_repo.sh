#!/bin/bash

source /opt/ros/$ROS_DISTRO/setup.bash;
echo 'ros2 imu tools'
git clone https://gitlab.com/boldhearts/ros2_imu_tools.git
rosdep install --from-paths . --ignore-src -r -y;

# after that run following commands to build workspace
# colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
# . install/setup.bash
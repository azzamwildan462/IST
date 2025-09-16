#!/bin/bash

# colcon build --symlink-install --executor parallel --parallel $(nproc) --cmake-args -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble -DCROSS_COMPILE=x86_64-linux-gnu -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
# colcon build --symlink-install --executor parallel --parallel 2 --cmake-args -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble -DCROSS_COMPILE=x86_64-linux-gnu -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
colcon build --symlink-install --cmake-args -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble -DCROSS_COMPILE=x86_64-linux-gnu -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
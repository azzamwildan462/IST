#!/bin/bash

colcon build --symlink-install --executor parallel --parallel $(nproc) --cmake-args -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble -DCROSS_COMPILE=x86_64-linux-gnu
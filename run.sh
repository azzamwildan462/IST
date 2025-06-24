#!/bin/bash

export ROS_LOCALHOST_ONLY=0
. install/setup.bash 
ros2 launch ros2_utils all.launch.py 
# ros2 launch ros2_utils tes_rs_rtabmap.launch.py 
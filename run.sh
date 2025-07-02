#!/bin/bash

# rm /home/ist/.ros/rtabmap.db
# cp /root/.ros/map_baru_bagus.db /root/.ros/rtabmap.db

export ROS_LOCALHOST_ONLY=0
. install/setup.bash 
ros2 launch ros2_utils all.launch.py 
# ros2 launch ros2_utils tes_rs_rtabmap.launch.py 
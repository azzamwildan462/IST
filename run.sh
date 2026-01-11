#!/bin/bash

# rm /home/ist/.ros/rtabmap.db
# cp /root/.ros/map_baru_bagus.db /root/.ros/rtabmap.db

# HIDUPKAN HW 
./tools/hidupkan_hw

sleep 3

# KALO LAGI NGECAS, MATIKAN dan poweroff 
./tools/a.out

sleep 3

# RUN PROGRAM NORMAL
export ROS_LOCALHOST_ONLY=1
. install/setup.bash 
ros2 launch ros2_utils all.launch.py 
# ros2 launch ros2_utils tes_rs_rtabmap.launch.py 

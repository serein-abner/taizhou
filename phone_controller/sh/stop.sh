#!/bin/bash


rosnode kill /order_location /order_navication /order_cruise /order_pause_cruise /order_gohome /order_stop

pkill -f roslaunch 

sleep 3

roslaunch phone_controller run.launch

# sleep 2

# roslaunch lslidar_driver lslidar_c16.launch

# sleep 2

# roslaunch imu_launch imu_spec_msg.launch

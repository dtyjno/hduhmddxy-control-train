#!/bin/bash

source /opt/ros/jazzy/setup.bash
ros2 launch mavros apm.launch fcu_url:=udp://127.0.0.1:14550@14555
#ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM0:921600;



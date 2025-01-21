#!/bin/bash

gnome-terminal -t "gazebo" -x bash -c "
#export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/gz_ws/src/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH;
#export GZ_SIM_RESOURCE_PATH=$HOME/gz_ws/src/ardupilot_gazebo/models:$HOME/gz_ws/src/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH; 
#gz sim -v4 -r iris_runway.sdf;
##gz sim -v4 -r gimbal.sdf;

cd ~/ardupilot_ws;
source install/local_setup.bash;
source /opt/ros/humble/setup.bash;
source ~/ws/install/local_setup.bash;
export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_ws/src/ros_gz_sim_ardupilot/models:$HOME/ardupilot_ws/src/ros_gz_sim_ardupilot/worlds:${GZ_SIM_SYSTEM_PLUGIN_PATH};
ros2 launch ros_gz_sim_ardupilot iris_runway.launch.py;
"


#gnome-terminal -t "gazebo" -x bash -c "
#source ~/ros2_ws/install/setup.sh;
#cd ~/ros2_ws/;
#ros2 launch ardupilot_gz_bringup iris_runway.launch.py;"


sleep 1s
gnome-terminal -t "SITL" -x bash -c "sim_vehicle.py -v ArduCopter -L HDU -f gazebo-iris --model JSON --map --console;
"
sleep 11s
gnome-terminal -t "mavros" -x bash -c "
source /opt/ros/humble/setup.bash;
ros2 launch mavros apm.launch fcu_url:=udp://127.0.0.1:14550@14555;
#ros2 launch mavros apm.launch fcu_url:=tcp:127.0.0.1:5760;

"
#gnome-terminal -t "mavproxy" -x bash -c "mavproxy.py --console --map --aircraft test --master=:14550"

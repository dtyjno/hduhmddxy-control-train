#!/bin/bash

# sim_vehicle.py -v ArduCopter -L HDU -f gazebo-iris --model JSON --map --console;
source ~/.bashrc
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON -L HDU --out 127.0.0.1:14550
# sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON -L HDU --map --console --out 127.0.0.1:14550
# sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console --no-mavproxy
cd ~/scripts;

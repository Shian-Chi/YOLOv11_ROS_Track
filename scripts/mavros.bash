#!/bin/bash

cd /home/ubuntu/track/track2/
source ./install/setup.bash

echo "123456789" | sudo -S chmod 777 /dev/ttyPixhawk 

ros2 run mavros mavros_node --ros-args --param fcu_url:=serial:///dev/ttyPixhawk &


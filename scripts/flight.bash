#!/bin/bash

cd /home/ubuntu/track/track2/
source ./install/setup.bash

echo "123456789" | sudo -S chmod 777 /dev/ttyPixhawk 

echo "run Flight"
python3 /home/ubuntu/torch_v2/yolo_tracking_v2/flight/drone_landing_ROS2.py &

sleep infinity

#!/bin/bash

cd /home/ubuntu/track/track2/
source ./install/setup.bash

echo "123456789" | sudo -S chmod 777 /dev/ttyPixhawk 

cleanup() {
    echo "Ctrl+C detected! Cleaning up..."
    pkill -f "drone_landing_ROS2.py"
    exit 0
}

trap cleanup SIGINT

echo "run Flight"
python3 /home/ubuntu/track/track2/flight/drone_pitch_landing3.py &

wait
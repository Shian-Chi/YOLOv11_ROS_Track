#!/bin/bash

cd /home/ubuntu/track/track2/
source ./install/setup.bash

echo "123456789" | sudo -S chmod 777 "/dev/ttyTHS0"

python3 /home/ubuntu/track/track2/detect/trackDetect.py &

sleep infinity

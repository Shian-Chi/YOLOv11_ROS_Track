#!/bin/bash

cd /home/ubuntu/track/track2/ || exit
source ./install/setup.bash

# 給裝置設權限
echo "123456789" | sudo -S chmod 777 /dev/ttyPixhawk

# 定義結束時要執行的函數
cleanup() {
    echo "Ctrl+C detected! Cleaning up..."
    if [[ -n "$MAVROS_PID" ]]; then
        kill "$MAVROS_PID"
        wait "$MAVROS_PID"
        echo "mavros_node terminated."
    fi
    exit 0
}

# 設定中斷處理 (例如 Ctrl+C)
trap cleanup SIGINT

# 啟動 mavros_node 並背景執行
ros2 run mavros mavros_node --ros-args --param fcu_url:=serial:///dev/ttyPixhawk &
MAVROS_PID=$!

# 等待子程序結束（或被 Ctrl+C 中斷）
wait $MAVROS_PID
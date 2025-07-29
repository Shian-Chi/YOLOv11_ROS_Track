#!/bin/bash

# 設定 cleanup 函數來處理終止
cleanup() {
    echo -e "\nCtrl+C detected! Cleaning up..."

    # 終止所有已啟動的 Python 程式
    pkill -f "/home/ubuntu/track/track2/detect_task/pub_img.py"
    pkill -f "/home/ubuntu/track/track2/detect_task/gimbal_track.py"
    pkill -f "/home/ubuntu/track/track2/detect_task/trackDetect_1.py"
    pkill -f "/home/ubuntu/track/track2/detect_task/csvLog.py"
    pkill -f "/home/ubuntu/track/track2/detect_task/VisionAnglePositioning.py"
    
    # 終止log監控程序
    pkill -f "tail -f"
    
    # 清理命名管道
    rm -f /tmp/python_logs_*
    
    echo "Cleanup completed."
    exit 0
}

# 設定監控 Ctrl+C 的信號
trap cleanup SIGINT

# 進入工作目錄
cd /home/ubuntu/track/track2/ || exit

# 設置必要的權限
source ./install/setup.bash
echo "123456789" | sudo -S chmod 777 "/dev/ttyTHS0"

# 創建logs目錄和命名管道
mkdir -p logs
mkfifo /tmp/python_logs_pub_img 2>/dev/null || true
mkfifo /tmp/python_logs_gimbal 2>/dev/null || true
mkfifo /tmp/python_logs_track 2>/dev/null || true
mkfifo /tmp/python_logs_csv 2>/dev/null || true
mkfifo /tmp/python_logs_vision_pos 2>/dev/null || true

echo "========================================="
echo "Starting all Python programs..."
echo "========================================="

# 啟動log監控器（在背景）
(while read line; do echo "[$(date '+%H:%M:%S')] [PUB_IMG] $line"; done < /tmp/python_logs_pub_img) &
(while read line; do echo "[$(date '+%H:%M:%S')] [GIMBAL] $line"; done < /tmp/python_logs_gimbal) &
(while read line; do echo "[$(date '+%H:%M:%S')] [TRACK] $line"; done < /tmp/python_logs_track) &
(while read line; do echo "[$(date '+%H:%M:%S')] [CSV_LOG] $line"; done < /tmp/python_logs_csv) &
(while read line; do echo "[$(date '+%H:%M:%S')] [POSITION_LOG] $line"; done < /tmp/python_logs_vision_pos) &

# 啟動Python程式並將輸出同時發送到文件和命名管道
python3 /home/ubuntu/track/track2/detect_task/pub_img.py 2>&1 | tee logs/pub_img.log > /tmp/python_logs_pub_img &
PID_PUB_IMG=$!

python3 /home/ubuntu/track/track2/detect_task/gimbal_track.py 2>&1 | tee logs/gimbal_track.log > /tmp/python_logs_gimbal &
PID_GIMBAL_TRACK=$!

python3 /home/ubuntu/track/track2/detect_task/trackDetect_1.py 2>&1 | tee logs/trackDetect_1.log > /tmp/python_logs_track &
PID_TRACK_DETECT=$!

python3 /home/ubuntu/track/track2/detect_task/csvLog.py 2>&1 | tee logs/csvLog.log > /tmp/python_logs_csv &
PID_AI_LOG=$!

python3 /home/ubuntu/track/track2/detect_task/VisionAnglePositioning.py 2>&1 | tee logs/position.log > /tmp/python_logs_vision_pos &
PID_POS_LOG=$!

echo "All programs started with PIDs:"
echo "  pub_img.py: $PID_PUB_IMG"
echo "  gimbal_track.py: $PID_GIMBAL_TRACK"
echo "  trackDetect_1.py: $PID_TRACK_DETECT"
echo "  csvLog.py: $PID_AI_LOG"
echo "  position.py $PID_POS_LOG"
echo ""
echo "Log files are being saved to logs/ directory"
echo "Press Ctrl+C to stop all processes"
echo ""
echo "========================================="
echo "Real-time logs (all programs combined):"
echo "========================================="

# 檢查程式狀態的函數
check_processes() {
    local running=0
    if kill -0 $PID_PUB_IMG 2>/dev/null; then ((running++)); fi
    if kill -0 $PID_GIMBAL_TRACK 2>/dev/null; then ((running++)); fi
    if kill -0 $PID_TRACK_DETECT 2>/dev/null; then ((running++)); fi
    if kill -0 $PID_AI_LOG 2>/dev/null; then ((running++)); fi
    if kill -0 $PID_POS_LOG 2>/dev/null; then ((running++)); fi
    echo $running
}

# 主循環：等待程式結束或用戶中斷
while true; do
    sleep 5
    running=$(check_processes)
    if [ $running -eq 0 ]; then
        echo -e "\nAll Python programs have exited."
        break
    fi
done

# 清理
cleanup
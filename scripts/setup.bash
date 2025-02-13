#!/bin/bash
set -e  # 發生錯誤時立即退出

# 讀取 sudo 密碼
read -s -p "Enter sudo password: " SUDO_PASS
echo

# **批量執行 sudo 命令**
echo "$SUDO_PASS" | sudo -S bash -c '
for dev in /dev/ttyTHS0 /dev/ttyUSB0 /dev/ttyRTK /dev/ttyPixhawk; do
    if [ -e "$dev" ]; then
        echo "設定 $dev 權限..."
        chmod 777 "$dev"
    else
        echo "警告：$dev 不存在，跳過..."
    fi
done

# 掃描 I2C 設備
i2cdetect -y 8

# 檢查是否具有 root 權限
if [ "$EUID" -ne 0 ]; then
    echo "請使用 sudo 權限執行此腳本"
    exit 1
fi

# 確保 NTP 已安裝
echo "檢查 NTP 是否已安裝..."
if ! command -v ntpq &> /dev/null; then
    echo "NTP 未安裝，正在安裝..."
    apt update
    apt install ntp -y
else
    echo "NTP 已安裝"
fi

# 啟動並啟用 NTP 服務
echo "啟動 NTP 服務..."
systemctl enable ntp
systemctl restart ntp

# 等待 NTP 服務啟動
sleep 5

# 顯示當前 NTP 同步狀態
echo "NTP 伺服器同步狀態："
ntpq -p

# 強制同步時間（與台灣 NTP 伺服器）
NTP_SERVER="time.stdtime.gov.tw"
echo "正在與 $NTP_SERVER 進行時間同步..."
ntpdate -u $NTP_SERVER

# 顯示同步後的時間
echo "同步後的系統時間："
date +"%Y-%m-%d %H:%M:%S %Z"

# 將系統時間寫入硬體時鐘
echo "更新硬體時鐘..."
hwclock --systohc

# 確認同步成功
echo "系統時間已成功同步並更新到硬體時鐘。"
'

exit 0

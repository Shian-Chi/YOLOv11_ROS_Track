#!/bin/bash
# 以明文密碼提升權限（僅限測試環境）
echo "123456789" | sudo -S bash -c '
# ===== 1) /dev/ttyTHS0 ==================================================
if [ -e /dev/ttyTHS0 ]; then
    echo "設定 /dev/ttyTHS0 權限..."
    chmod 777 /dev/ttyTHS0
else
    echo "警告：/dev/ttyTHS0 不存在，跳過..."
fi

# ===== 2) /dev/ttyUSB0 ==================================================
if [ -e /dev/ttyUSB0 ]; then
    echo "設定 /dev/ttyUSB0 權限..."
    chmod 777 /dev/ttyUSB0
else
    echo "警告：/dev/ttyUSB0 不存在，跳過..."
fi

# ===== 3) /dev/ttyRTK ===================================================
if [ -e /dev/ttyRTK ]; then
    echo "設定 /dev/ttyRTK 權限..."
    chmod 777 /dev/ttyRTK
else
    echo "警告：/dev/ttyRTK 不存在，跳過..."
fi

# ===== 4) /dev/ttyPixhawk ==============================================
if [ -e /dev/ttyPixhawk ]; then
    echo "設定 /dev/ttyPixhawk 權限..."
    chmod 777 /dev/ttyPixhawk
else
    echo "警告：/dev/ttyPixhawk 不存在，跳過..."
fi

# ===== 5) 掃描 I²C Bus 8 ===============================================
echo "掃描 I²C Bus 8 裝置..."
# 若未安裝 i2c-tools，請先：apt install -y i2c-tools
i2cdetect -y 8
'
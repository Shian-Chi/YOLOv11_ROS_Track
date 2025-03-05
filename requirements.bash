#!/bin/bash

set -e  # 遇到錯誤時終止腳本
trap 'echo "❌ 安裝過程發生錯誤，請檢查日誌！"' ERR

# 確保腳本有 sudo 權限
if [ "$(id -u)" -ne 0 ]; then
    echo "❌ 請使用 sudo 執行此腳本！"
    exit 1
fi

# 檢查是否為 Jetson 硬體
IS_JETSON=false
if grep -q "nvidia" /proc/device-tree/compatible 2>/dev/null; then
    echo "✅ 這台設備是 NVIDIA Jetson，將跳過 OpenCV 安裝。"
    IS_JETSON=true
else
    echo "❌ 這台設備不是 Jetson，將安裝 OpenCV..."
fi

# 檢查 Python 版本
if command -v python3 &> /dev/null; then
    PYTHON_CMD="python3"
else
    echo "❌ 未找到 Python3，請先安裝 Python3。"
    exit 1
fi

# 檢查並安裝 pip
if ! command -v pip3 &> /dev/null; then
    echo "📦 pip3 未安裝，正在安裝..."
    apt update && apt install -y python3-pip
fi

# 定義函數來安裝 Python 套件並驗證
install_python_package() {
    PACKAGE_NAME=$1
    echo "📦 正在安裝 $PACKAGE_NAME..."
    $PYTHON_CMD -m pip install --upgrade "$PACKAGE_NAME"
    if $PYTHON_CMD -c "import ${PACKAGE_NAME%%=*}" &> /dev/null; then
        echo "✅ $PACKAGE_NAME 安裝成功！"
    else
        echo "❌ $PACKAGE_NAME 安裝失敗，請檢查錯誤信息。"
        exit 1
    fi
}

# 安裝 Python 套件（OpenCV 只在非 Jetson 設備上安裝）
install_python_package "pip"
if [ "$IS_JETSON" = false ]; then
    install_python_package "opencv-python"
fi
install_python_package "numpy"
install_python_package "pyserial"
install_python_package "ultralytics"

# 安裝 Torch
echo "📥 下載 Torch 安裝包..."
wget --progress=bar:force https://developer.download.nvidia.cn/compute/redist/jp/v512/pytorch/torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl -O torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl

echo "📦 安裝 Torch 及相關依賴..."
apt-get install -y libopenblas-base libopenmpi-dev libomp-dev
$PYTHON_CMD -m pip install 'Cython<3'
$PYTHON_CMD -m pip install torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl

# 安裝 TorchVision
echo "📥 下載並安裝 TorchVision..."
apt-get install -y libjpeg-dev zlib1g-dev libpython3-dev libopenblas-dev libavcodec-dev libavformat-dev libswscale-dev

git clone --branch v0.16.1 https://github.com/pytorch/vision torchvision
cd torchvision
export BUILD_VERSION=0.16.1
$PYTHON_CMD setup.py install --user
cd ../

echo "✅ 所有依賴安裝完成！"
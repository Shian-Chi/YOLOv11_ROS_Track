# **ROS_YOLOv11_Tracking**

## 1. 專案簡介
本專案使用 **YOLOv11** 進行物件追蹤，並透過 **ROS2** 進行訊息傳遞與控制。該系統可與 **MAVROS** 進行整合，以便在無人機導航與降落時提供高準確度的目標追蹤。

---

## 2. 使用環境
- **Jetson Xavier NX**
- **Ubuntu 20.04**

---

## 3. 安裝與依賴
請確保您的環境已安裝以下軟體與庫：
- **ROS2** ([安裝指南](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html))
- **MAVROS**
- **YOLOv11**
- **pyserial**
- **colcon (ROS2 編譯工具)**
- **Python 3.8+**

### **安裝步驟**
1. 安裝 **ROS2** (根據您的系統選擇適合的版本)
   ```sh
   sudo apt update
   sudo apt install ros-foxy-desktop
   ```
2. 安裝 MAVROS 套件：
   ```sh
   sudo apt install ros-foxy-mavros ros-foxy-mavros-extras
   ```
3. 安裝 Python 相關依賴：
   ```sh
   sudo bash requirements.bash
   ```
4. 編譯專案：
   ```sh
   colcon build
   ```

---

## 4. 使用方式

### **執行 YOLO 物件追蹤**
```sh
bash scripts/setup.bash
source install/setup.bash
python3 detect/trackDetect.py
```

### **運行無人機控制**
```sh
source install/setup.bash
python3 flight/drone_ROS2.py
```

### **4G網路無人機通訊**
```sh
source install/setup.bash
python3 flight/drone_ROS2.py
```

---

## 5. 專案架構

以下是專案的主要目錄結構：

```sh
$ tree -L 2
.
├── build                        # ROS2
│   ├── COLCON_IGNORE
│   └── tutorial_interfaces
├── config.json                  # Tracking configuration
├── ctrl                         # Gimbal configuration
│   ├── gimbal_ctrl.py
│   └── pid
├── detect                       # Detect
│   ├── camera_function.py
│   ├── detect.py                # YOLOv11 detection
│   ├── landpadv11.pt            # YOLOv11 landpad module
│   ├── trackDetect copy.py
│   ├── trackDetect.py
│   └── yolo11n.pt               # YOLOv11 COCO module
├── flight                       # Flight Code
│   ├── drone_landing_ROS2.py    # After identifying the target, head to the landing point and land
│   ├── drone_ROS2.py
│   └── drone_tan_landing.py
├── install
│   ├── setup.bash
│   └── ...
├── log                            # ROS2
│   └── ...
├── publish.json
├── PWCL
│   ├── drone_4g_v2.py
│   ├── drone_PWCL_new.py
│   ├── drone_PWCL.py
│   ├── __pycache__
│   ├── rtk_4g_recv.py
│   └── rtk_xbee_recv.py
├── __pycache__
│   └── utils.cpython-38.pyc
├── README.md
├── requirements.bash
├── runs
│   ├── exp1
│       ├── log.csv
│       └── output.avi
│   └── ...
├── scripts
│   └── setup.bash
├── src
│   └── tutorial_interfaces
├── temp
│   └── shareParameter.json
├── utils.py
└── yolo11n.pt

```

---

## 6. 設定與配置

本專案的主要設定檔為 **config.json**，用於調整 **YOLOv11 參數**、**無人機控制選項** 以及 **追蹤模式**。

---

## 7. 示範與測試

### **測試 YOLOv11 物件偵測**
```sh
python3 detect/detect.py
```
此指令將開啟攝影機，並執行 YOLOv11 進行即時物件偵測。

### **測試 MAVROS 與無人機控制**
1. 啟動 MAVROS：
   ```sh
   # Actual test
   ros2 run mavros mavros_node --ros-args --param fcu_url:=serial:///dev/ttyPixhawk &
   ```
2. 啟動通訊：
   ```sh
   python3 PWCL/drone_PWCL_new.py
   ```
3. 啟動追蹤
    ```sh
    python3 detect/trackDetect.py
    ```
---

## 8. 開發與貢獻

如果你希望貢獻本專案，請依照以下方式進行：
1. Fork 本專案
2. 建立新分支 (`git checkout -b feature-xxx`)
3. 提交你的修改 (`git commit -m "新增 xxx 功能"`)
4. 推送到你的 Fork (`git push origin feature-xxx`)
5. 提交 Pull Request

---

### **日誌記錄**

| 日期       | 更新內容                                                      | 分支 (Branch) |
| ---------- | ------------------------------------------------------------- | ------------- |
| 2025/02/28 | 飛行測試，目標追蹤情況良好                                     | main          |
| 2025/03/11 | 加入 CSV log                                               | v2.0          |
| 2025/03/13 | 雲台 Pitch 自穩                                             | v2.0          |
| 2025/03/19 | 錄影及 log.csv 加入新資料，修正追蹤邏輯錯誤                     | v2.0          |
| 2025/03/19 | 加入由雲台仰俯判斷降落的程式碼                                  | v2.1          |
| 2025/03/20 | 修正雲台角度偏移，補償 UAV 晃動影響；由基站設定飛行高度            | v2.1          |
| 2025/03/23 | drone_pitch_landing 修正機頭轉向 (未測試成功)                 | v2.1          |
| 2025/03/24 | 修正 AI 辨識到目標返回起飛點                                  | v2.1          |
| 2025/03/26 | 飛行高度從地面基站中取得                                      | v2.1          |
| 2025/03/31 | 修改移動參數及儲存相對高度                                     | v2.1          |
| 2025/04/01 | 優化程式碼並合併 Node                                        | v2.1.1        |
| 2025/04/09 | 檔名修改：stream/http_stream.py → stream/http_Cam_stream.py | v2.1.1        |
| 2025/04/10 | 移除Pitch自穩                                               | v2.1.1        |
| 2025/04/10 | 新增自動偵測欄位並在檔案不存在時寫入表頭                         | v2.1.1        |




---

### **改進目標**
 - 2025/02/28 -> 加入角度控制計算結果及記錄無人機相關資訊
   

---
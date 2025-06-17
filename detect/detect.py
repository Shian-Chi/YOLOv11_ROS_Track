from ultralytics import YOLO
import cv2
import numpy as np
from pathlib import Path

def isdocker() -> bool:
    """
    檢測是否在 Docker 容器中運行。
    """
    return Path('/workspace').exists() or Path('/.dockerenv').exists()

def check_imshow() -> bool:
    """
    檢查是否支持顯示圖像功能。
    """
    try:
        assert not isdocker(), 'cv2.imshow() is disabled in Docker environments'
        return True
    except Exception as e:
        print(f'WARNING: Environment does not support cv2.imshow() or PIL Image.show() image displays\n{e}')
        return False

def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1280,
    capture_height=720,
    display_width=1280,
    display_height=720,
    framerate=30,
    flip_method=0,
):
    """
    Generate GStreamer pipeline for camera input.
    """
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )

# 載入 YOLOv8 模型（使用官方預訓練權重或自定義模型權重）
model = YOLO('detect/landpadv11.pt')  
# 設定影片或影像來源（攝影機或檔案）
video_source = 0  # 0 表示攝影機，或提供影片路徑，如 'video.mp4'

# 開啟影像來源
cap = cv2.VideoCapture(gstreamer_pipeline(capture_width=1280,
                                            capture_height=720,
                                            flip_method=0
                                         ) 
                       , cv2.CAP_GSTREAMER)

show = check_imshow()

# 設定影片錄製（可選擇錄製格式和檔案名稱）
fourcc = cv2.VideoWriter_fourcc(*'XVID')  # 設定影片編碼格式
output = cv2.VideoWriter('record_output.avi', fourcc, 30.0, (1920, 1080))

# 影像推論
count = 1
while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    # 使用模型進行推論
    frame = cv2.resize(frame,(1920, 1080))
    results = model(frame, conf=0.3, half=True, device=0, nms=True, iou=0.5, agnostic_nms=False, max_det=300, retina_masks=False)

    # 遍歷所有檢測到的物件
    for result in results:
        boxes = result.boxes.xyxy.cpu().numpy()
        confidences = result.boxes.conf.cpu().numpy()
        class_ids = result.boxes.cls.cpu().numpy()

        for bbox, conf, cls_id in zip(boxes, confidences, class_ids):
            x0, y0, x1, y1 = map(int, bbox)
            label = f"{model.names[int(cls_id)]} {conf:.2f}"

            # 繪製矩形框與標籤
            cv2.rectangle(frame, (x0, y0), (x1, y1), (0, 255, 0), 2)
            cv2.putText(frame, label, (x0, y0 - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            print(f"Bbox: {x0}, {y0}, {x1}, {y1}")
    # 顯示偵測影像
    if show:
        frame = cv2.resize(frame,(1280, 720))
        cv2.imshow("YOLOv11 Detection", frame)

    # # 錄製當前影像
    # frame = cv2.resize(frame,(1920, 1080))
    # output.write(frame)

    # cv2.imwrite(f"/home/ubuntu/track/track2/detect/output{count}.jpg", frame)  # Save image as output.jpg
    # count += 1

    # 按下 'q' 鍵退出
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 釋放資源
cap.release()
output.release()
cv2.destroyAllWindows()

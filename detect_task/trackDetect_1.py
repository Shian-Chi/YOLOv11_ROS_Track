import signal
import sys, os, time, datetime
from pathlib import Path
from typing import Union
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../')))
import threading, queue
import cv2, math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Int32, Bool, Int32MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from ultralytics import YOLO
from detect_task.camera_function import Camera, gstreamer_pipeline_1
from parameter import Parameter
from utils import system_time, increment_path

# ---------- 全域變數 ----------
YOLO_FPS = None
stop_threads = False
video_processor = None

# ---------- ROS2 初始化 ----------
rclpy.init()

# ---------- 參數設定 ----------
para = Parameter()
VIDEO_WIDTH = para.VIDEO_WIDTH
VIDEO_HEIGHT = para.VIDEO_HEIGHT
SHOW = para.image_show_enabled
IMGSZ = para.yolo_image_size
CONF_THRESHOLD = para.yolo_confidence
SAVEIMG = para.image_save_enabled
save_data = para.data_save_enabled
is_Stream = para.img_send_enabled


# ---------- 共用清理流程 ----------
def cleanup(cap: Camera, ROS_thrd:threading.Thread, detection_thread):
    global video_processor, stop_threads
    stop_threads = True

    if ROS_thrd and ROS_thrd.is_alive():
        rclpy.shutdown()
        ROS_thrd.join(timeout=2)
    
    try:
        detection_thread and detection_thread.is_alive()
        print("[cleanup] Joining detection_thread...")
        detection_thread.join(timeout=2)
    except Exception as e:
        print("[cleanup] join detection_thread:", e)

    try:
        if cap:
            cap.stop()
            cap = None
    except Exception as e:
        print("[cleanup] cap.stop:", e)

    if SHOW:
        cv2.destroyAllWindows()

    try:
        if SAVEIMG and video_processor:
            video_processor.stop_recording()
            video_processor = None
    except Exception as e:
        print("[cleanup] video_processor:", e)
    
    print("\033[31m[cleanup] 程式安全結束\033[0m")

# ---------- 訊號處理 ----------
def signal_handler(sig, frame, cap, *thrd):
    print(f"\n[signal_handler] Received signal {sig}, cleaning up...")
    cleanup(cap, *thrd)
    # 讓主執行緒退出（避免繼續執行 while）
    sys.exit(0)

# ---------- 建立資料夾 ----------
def create_exp_dir(root: Union[str, Path] = "runs",
                   date_fmt: str = "%Y-%m-%d") -> Path:
    """
    在 root/YYYY-MM-DD/expN 底下建立遞增資料夾並回傳路徑。

    Args:
        root     (str | Path):  最上層資料夾 (預設 'runs')
        date_fmt (str)       :  日期字串格式 (strftime)，預設 'YYYY-MM-DD'

    Returns:
        Path:  已建立好的 exp、exp1、exp2… 資料夾路徑
    """
    date_str  = datetime.datetime.now().strftime(date_fmt)
    date_dir  = Path(root) / date_str
    date_dir.mkdir(parents=True, exist_ok=True)

    exp_dir   = increment_path(date_dir / "exp", exist_ok=False)
    exp_dir.mkdir(parents=True, exist_ok=True)
    return exp_dir

# ---------- ROS ----------
class YOLO_Node(Node):
    def __init__(self):
        super().__init__('yolo_node')
        self.publisher_xyxy = self.create_publisher(Int32MultiArray, '/target_xyxy', 1)
        self.publisher_detected = self.create_publisher(Bool, '/target_detected', 1)
        self.publisher_YOLO_FPS = self.create_publisher(Int32, '/YOLO_FPS', 1)
        
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.GlobalPositionSub = self.create_subscription(NavSatFix, 'mavros/global_position/global', self.GPcb, qos)
        self.latitude = 0.0
        self.longitude = 0.0
        self.gps_altitude = 0.0
        
    def publish_detected(self, detected: bool):
        """
        發布目標是否被檢測到的訊息到 ROS 主題。
        
        Args:
            detected (bool): 是否檢測到目標。
        """
        msg = Bool(data=detected)
        self.publisher_detected.publish(msg)    
    
    def publish_target_xyxy(self, xyxy):
        data = Int32MultiArray(data=xyxy)
        self.publisher_xyxy.publish(data)

    def publish_yolo_fps(self, fps: float):
        """
        發布 YOLO FPS 到 ROS 主題。
        
        Args:
            fps (float): YOLO 的幀率。
        """
        msg = Int32(data=int(fps))
        self.publisher_YOLO_FPS.publish(msg)
    
    def GPcb(self, msg:NavSatFix):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.gps_altitude = msg.altitude
    
    def get_gps_data(self):
        return self.latitude, self.longitude, self.gps_altitude
        
# ---------- 視訊處理器 ----------
class VideoProcessor:
    def __init__(self, isSave:Bool=True, name:str="video.avi", video_width:int=1280, video_height:int=720, record_fps:int=20):
        self.name = name
        self.video_width = video_width
        self.video_height = video_height
        self.record_fps = record_fps
        
        if isSave:
            gst = self.record_gst_pipeline(self.name, video_width, video_height, record_fps)
            self.video_save = cv2.VideoWriter(gst, cv2.CAP_GSTREAMER, 0, self.record_fps, (VIDEO_WIDTH, VIDEO_HEIGHT))

            if not self.video_save.isOpened():
                raise RuntimeError(f"VideoWriter open failed! pipeline = {gst}")
        
        self._init_display_params()
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.image_center_x = video_width / 2
        self.image_center_y = video_height / 2
        
    def record_gst_pipeline(self, path: str, w: int, h: int, fps: int) -> str:
        return (
            f"appsrc is-live=true block=true format=GST_FORMAT_TIME "
            f"caps=video/x-raw,format=BGR,width={w},height={h},framerate={fps}/1 "
            "! queue leaky=2 "
            "! videoconvert "
            "! nvvidconv "
            "! video/x-raw(memory:NVMM),format=NV12 "
            "! nvv4l2h264enc bitrate=4000000 insert-sps-pps=true idrinterval=30 "
            "! h264parse "
            "! mp4mux streamable=true fragment-duration=1000 "
            f"! filesink location={path} sync=false"
        )
    
    def _init_display_params(self):
        self.font_scale, self.thickness, self.line_spacing, self.margin = 0.7, 2, 35, 10

    def put_text_line(self, frame, text, x, y, color=(0, 255, 0)):
        cv2.putText(frame, text, (x, y), self.font, self.font_scale, color, self.thickness)

    def put_info(self, node:YOLO_Node, frame):
        if frame is None:
            return frame
        h, w = frame.shape[:2]
        current_date = datetime.date.today().strftime("%Y-%m-%d")
        current_time = datetime.datetime.now().strftime("%H:%M:%S")

        for i, text in enumerate([current_date, current_time]):
            text_size, _ = cv2.getTextSize(text, self.font, self.font_scale, self.thickness)
            text_x = (w - text_size[0]) // 2
            text_y = self.margin + (i * self.line_spacing) + text_size[1]
            self.put_text_line(frame, text, text_x, text_y)

        if YOLO_FPS is not None:
            fps_text = f"FPS: {YOLO_FPS:.0f}"
            fps_size, _ = cv2.getTextSize(fps_text, self.font, self.font_scale, self.thickness)
            self.put_text_line(frame, fps_text, w - fps_size[0] - self.margin, self.margin + fps_size[1])

        lat, lon, alt = node.get_gps_data()
        gps_lines = [f"Lat: {lat:.6f}, Lon: {lon:.6f}, alt: {alt:.1f}"]

        lines = gps_lines 
        for i, text in enumerate(lines):
            self.put_text_line(frame, text, self.margin, self.margin + 20 + (i * 20), (2, 128, 20))
        
        return frame

    def record(self, node:YOLO_Node):
        while not stop_threads:
            if hasattr(self, 'last_frame') and self.last_frame is not None:
                frame = self.put_info(node, self.last_frame)
                self.video_save.write(frame)
            time.sleep(1 / self.record_fps)  # 控制錄影FPS
        print("[record] thread exit...")

    def update_frame(self, frame):
        self.last_frame = frame

    def start_recording(self, node):
        self.recording_thread = threading.Thread(target=self.record, args=(node,),daemon=True)
        self.recording_thread.start()
        print("\033[32mStart recording\033[0m")

    def stop_recording(self):
        global stop_threads
        stop_threads = True
        if hasattr(self, "recording_thread"):
            self.recording_thread.join()
            self.video_save.release()
        print("\033[31mRecording end\033[0m")

    @staticmethod
    def calculate_distance_to_center(bbox_xyxy, image_width=1280, image_height=720):
        x1, y1, x2, y2 = bbox_xyxy
        image_center_x = image_width / 2
        image_center_y = image_height / 2
        bbox_center_x = (x1 + x2) / 2
        bbox_center_y = (y1 + y2) / 2
        distance = math.sqrt((bbox_center_x - image_center_x) ** 2 + (bbox_center_y - image_center_y) ** 2)
        return distance

# ---------- YOLO檢測線程 ----------
def yolo_detection_thread(model: YOLO, cap: Camera, ros_node: YOLO_Node):
    global YOLO_FPS, stop_threads, video_processor
    not_read_count = 0

    def draw_bboxInfo(frame, bbox, conf, color=(0, 255, 0), thickness=2):
        if bbox is None:
            return frame
        x1, y1, x2, y2 = map(int, bbox)
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)
        label = f"{conf:.1f}"
        cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        return frame

    while not stop_threads:
        try:
            frame = cap.frame_queue.get(timeout=0.1)
        except queue.Empty:
            continue

        if frame is None:
            not_read_count += 1
            if not_read_count >= 3:
                break
            continue

        frame = cv2.resize(frame, (1280, 720))
        detect_STATUS = False
        t1 = time.time()
        results = model.predict(frame, imgsz=IMGSZ, conf=CONF_THRESHOLD, iou=0.45, max_det=1)
        t2 = time.time()
        YOLO_FPS = 1.0 / (t2 - t1)
        print(f"[YOLO] FPS: {YOLO_FPS:.2f}, Time: {t2 - t1:.3f}s")
        pred = results[0].boxes.data
        target_xyxy = None
        target_conf = -1

        if len(pred):
            detect_STATUS = True
            for box in results[0].boxes:
                conf = box.conf[0].item()
                if conf > target_conf:
                    target_conf = conf
                    target_xyxy = box.xyxy[0]
            
            x0, y0, x1, y1 = map(int, target_xyxy.cpu())
            ros_node.publish_target_xyxy([x0, y0, x1, y1])

        # Publish detection status
        ros_node.publish_detected(detect_STATUS)
        ros_node.publish_yolo_fps(YOLO_FPS)
        
        if SHOW or SAVEIMG:
            frame = video_processor.put_info(ros_node, frame)
            frame = draw_bboxInfo(frame, target_xyxy, target_conf)

        if SHOW:
            cv2.imshow("YOLO Detection", frame)
            cv2.waitKey(1)

        if SAVEIMG:
            video_processor.update_frame(frame)

    print("[YOLO] Detection thread exited.")

# ---------- 載入 YOLO 模型 ----------
def load_yolo_model(path):
    print("Loading YOLO model...")
    model = YOLO(path)
    print("YOLO model loaded.")
    return model

# ---------- 儲存設定 ----------
def save(imgsave:bool, datasave:bool) -> Path:
    """
    建立儲存資料夾並回傳路徑。
    """
    video_path = data_path = None
    save_dir   = create_exp_dir("/home/ubuntu/track/track2/runs")
    if imgsave:
        video_path = save_dir / "video.mp4"
        
    if datasave:
        data_path = save_dir / "data.csv"
    return [video_path, data_path]

def main():
    if SAVEIMG or save_data:
        path = save(SAVEIMG, save_data)
        with open("/home/ubuntu/track/track2/detect_task/.temp.txt", "w") as file:
            current_time = datetime.date.today().strftime("%Y-%m-%d") + " " + \
                            datetime.datetime.now().strftime("%H:%M:%S")
            file.write(f"{current_time}\nvideo: {path[0]}\ndata: {path[1]}")
        print(f"\033[32m影片儲存路徑：{path[0]}\033[0m")
        print(f"\033[32m數據儲存路徑：{path[1]}\033[0m")
    
    global video_processor, stop_threads
    cap = Camera(gstreamer_pipeline_1(), 1280, 720, 30)
    cap.start()
    
    ros_node = YOLO_Node()
    ros_thread = threading.Thread(target=rclpy.spin, args=(ros_node, ), daemon=True)
    ros_thread.start()
    
    
    if SHOW or SAVEIMG:
        video_processor = VideoProcessor(SAVEIMG, name=path[0], video_width=VIDEO_WIDTH, video_height=VIDEO_HEIGHT)
    
    if SAVEIMG:
        video_processor.start_recording(ros_node)

    model = load_yolo_model("/home/ubuntu/track/track2/detect_task/landpadv11.pt")
    detection_thread = threading.Thread(target=yolo_detection_thread, args=(model, cap, ros_node), daemon=True)
    detection_thread.start()
    
    signal.signal(signal.SIGTERM, lambda s,f: signal_handler(s, f, cap, detection_thread, ros_thread))
    signal.signal(signal.SIGTERM, lambda s,f: signal_handler(s, f, cap, detection_thread, ros_thread))
    
    try:
        # 主執行緒只負責「存活」並定期睡一下
        while not stop_threads:
            time.sleep(1)
    except KeyboardInterrupt:
        stop_threads = True
        detection_thread.join()
    finally:
        ros_node.destroy_node()
        cleanup(cap, ros_thread, detection_thread)
        print("Main thread exited.")

if __name__ == "__main__":
    main()

from pathlib import Path
import signal
import sys, os, time, datetime
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../')))
import threading, queue
import cv2, math

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import ReliabilityPolicy, QoSProfile
from sensor_msgs.msg import NavSatFix, Imu
from transforms3d import euler
from tutorial_interfaces.msg import Img, Bbox

from ultralytics import YOLO
from camera_function import gstreamer_pipeline
from utils import JsonHandler, check_imshow, increment_path, check_file
from ctrl.gimbal_ctrl import GimbalTimerTask

# ----------------------------------
# 全域變數
# ----------------------------------
YOLO_FPS = None
executor = None

# ----------------------------------
# ROS2初始化
# ----------------------------------
if not rclpy.ok():
    rclpy.init()

# ----------------------------------
# 讀取設定
# ----------------------------------
json_file = check_file("config.json")
json_config = JsonHandler(json_file)
VIDEO_WIDTH = json_config.get(["video_resolutions", "default", "width"])
VIDEO_HEIGHT = json_config.get(["video_resolutions", "default", "height"])
SHOW = json_config.get(["image_show", "switch"]) if check_imshow else False
IMGSZ = json_config.get(["yolo", "imgsz"])
CONF_THRESHOLD = json_config.get(["yolo", "conf"])
SAVE = json_config.get(["img_save", "default"])
trackMode = json_config.get(["trackMode", "default"])

# 用來做 Bounding Box 資訊互通的全域 dict
json_file = check_file("publish.json")
json_pub = JsonHandler(json_file)
pub_bbox = json_pub.get(["pub_bbox"], default={})
pub_img = json_pub.get(["pub_img"], default={})

# Helper function - 初始化 BBox
def bbox_init():
    """
    重置 bounding box 相關參數。
    """
    pub_img["detect"] = False
    pub_bbox["detect"] = False
    pub_bbox["id"] = -1
    pub_bbox["conf"] = 0.0
    pub_bbox["x0"] = pub_bbox["y0"] = 0
    pub_bbox["x1"] = pub_bbox["y1"] = 0
    pub_bbox["name"] = ""

# AppState: 紀錄應用程式整體狀態
class AppState():
    def __init__(self, save_img=False):
        self.save_img = save_img
        self.stop_threads = False
        self.frame_lock = threading.Lock()
        self.executor = MultiThreadedExecutor()
        
        self.ROS_stop_event = threading.Event()
        self.ROS_spin = threading.Thread()
        
        # 初始化攝影機 (Jetson 可能需要依照實際pipeline做修改)
        self.cap = cv2.VideoCapture(
            gstreamer_pipeline(capture_width=VIDEO_WIDTH,
                               capture_height=VIDEO_HEIGHT,
                               flip_method=0), 
            cv2.CAP_GSTREAMER
        )
        
        # 若要存檔，建立路徑、VideoWriter
        if save_img:
            self.video_name = "output.avi"
            self.save_path = "/home/ubuntu/track/track2/runs"
            self.save_path = increment_path(Path(self.save_path) / "exp", exist_ok=False)
            self.save_path.mkdir(parents=True, exist_ok=True)
            
            # 完整檔案路徑
            self.name = str(self.save_path / self.video_name)
            print(f"**** Video file path: {self.name} ****")
            
            # 建立 VideoWriter
            self.record_fps = 13
            self.vid_writer = cv2.VideoWriter(
                self.name, 
                cv2.VideoWriter_fourcc(*'XVID'), 
                self.record_fps, 
                (VIDEO_WIDTH, VIDEO_HEIGHT)
            )
            
        # 用來做錄影時的 Frame buffer
        self.frame_queue = queue.Queue(maxsize=10)
   
# MinimalSubscriber: 訂閱 GPS、IMU、Bbox 等資訊
class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__("minimal_subscriber")
        self.GlobalPositionSub = self.create_subscription(
            NavSatFix, 
            "mavros/global_position/global", 
            self.GPcb, 
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        self.imuSub = self.create_subscription(
            Imu, 
            "mavros/imu/data", 
            self.IMUcb, 
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        self.holdSub = self.create_subscription(
            Img, 
            "img", 
            self.holdcb, 
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        self.bboxSub = self.create_subscription(
            Bbox, 
            'bbox', 
            self.bbox_callback, 
            1
        )
        
        # 初始化變數
        self.detect = False
        self.ID = -1
        self.conf = -1
        self.x0 = self.y0 = self.x1 = self.y1 = 0
        
        self.latitude = 0.0
        self.longitude = 0.0
        self.gps_altitude = 0.0
        self.drone_pitch = 0.0
        self.drone_roll = 0.0
        self.drone_yaw = 0.0
        self.hold = False

    def holdcb(self, msg):
        self.hold = pub_img["hold_status"] = msg.hold_status

    def GPcb(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.gps_altitude = msg.altitude

    def IMUcb(self, msg):
        ned_euler_data = euler.quat2euler([
            msg.orientation.w,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z
        ])
        self.drone_pitch = math.degrees(ned_euler_data[0])
        self.drone_roll = math.degrees(ned_euler_data[1])
        self.drone_yaw = math.degrees(ned_euler_data[2])

    def bbox_callback(self, msg):
        self.detect = msg.detect
        self.ID = msg.class_id
        self.conf = msg.confidence
        self.x0 = msg.x0
        self.y0 = msg.y0
        self.x1 = msg.x1
        self.y1 = msg.y1
    
    def get_bbox(self):
        return self.x0, self.y0, self.x1, self.y1

    def get_gps_data(self):
        """
        回傳經緯度，給外部(例如 VideoProcessor)使用
        """
        return (self.latitude, self.longitude)

# MinimalPublisher: 發布 Img、Bbox 等資訊
class MinimalPublisher(Node):
    def __init__(self):
        super().__init__("minimal_publisher")
        # Img publish
        self.imgPublish = self.create_publisher(Img, "img", 1)
        img_timer_period = 1/10
        self.img_timer = self.create_timer(img_timer_period, self.img_callback)
        
        # Bbox publish
        # self.bboxPublish = self.create_publisher(Bbox, "bbox", 1)
        # bbox_timer_period = 1/20
        # self.bbox_timer = self.create_timer(bbox_timer_period, self.bbox_callback)
        
        self.img = Img()
        # self.bbox = Bbox()
        
    def img_callback(self):
        pub_img['camera_center'] = gimbalTask.center_status
        
        yA, pA = gimbalTask.get_angle()
        pub_img['motor_pitch'] = pA + ROS_Sub.drone_pitch
        pub_img['motor_yaw'] = yA
        
        (self.img.detect, 
         self.img.camera_center, 
         self.img.motor_pitch, 
         self.img.motor_yaw, 
         self.img.target_latitude, 
         self.img.target_longitude, 
         self.img.hold_status, 
         self.img.send_info) = pub_img.values()
        
        self.imgPublish.publish(self.img)
    
    # def bbox_callback(self):
    #     self.bbox.detect = pub_bbox['detect']
    #     self.bbox.class_id = pub_bbox['id']
    #     self.bbox.confidence = pub_bbox['conf']
    #     self.bbox.x0 = pub_bbox['x0']
    #     self.bbox.y0 = pub_bbox['y0']
    #     self.bbox.x1 = pub_bbox['x1']
    #     self.bbox.y1 = pub_bbox['y1']
    #     self.bboxPublish.publish(self.bbox)

# VideoProcessor: 在影像上繪製文字、並執行錄影
class VideoProcessor:
    def __init__(self, app_state, gps_node, video_width=1920, video_height=1080):
        """
        這裡多加一個 gps_node: MinimalSubscriber，來取得經緯度
        """
        self.app_state = app_state
        self.gps_node = gps_node
        self.video_width = video_width
        self.video_height = video_height
        
        # 設定字體大小、厚度與位置參數
        if video_width == 1920 and video_height == 1080:  # 1080p
            self.font_scale = 1
            self.thickness = 3
            self.line_spacing = 50
            self.margin = 20
        elif video_width == 1280 and video_height == 720:  # 720p
            self.font_scale = 0.7
            self.thickness = 2
            self.line_spacing = 35
            self.margin = 10
        else:  # 其他解析度
            self.font_scale = 0.5
            self.thickness = 2
            self.line_spacing = 30
            self.margin = 10

        self.font = cv2.FONT_HERSHEY_SIMPLEX

    def put_info(self, frame):
        """
        在影像上繪製日期、時間、FPS 和 GPS。
        """
        global YOLO_FPS
        h, w = frame.shape[:2]

        current_date = datetime.date.today().strftime("%Y-%m-%d")
        current_time = datetime.datetime.now().strftime("%H:%M:%S")

        # 繪製日期、時間 (置中顯示)
        date_size, _ = cv2.getTextSize(current_date, self.font, self.font_scale, self.thickness)
        time_size, _ = cv2.getTextSize(current_time, self.font, self.font_scale, self.thickness)
        
        date_x = (w - date_size[0]) // 2
        date_y = self.margin + date_size[1]
        time_x = (w - time_size[0]) // 2
        time_y = date_y + self.line_spacing

        cv2.putText(frame, current_date, (date_x, date_y), 
                    self.font, self.font_scale, (0, 255, 0), self.thickness)
        cv2.putText(frame, current_time, (time_x, time_y), 
                    self.font, self.font_scale, (0, 255, 0), self.thickness)

        # 繪製 GPS 經緯度
        lat, lon = self.gps_node.get_gps_data()
        gps_text = f"Lat: {lat:.6f}, Lon: {lon:.6f}"
        cv2.putText(frame, gps_text, (self.margin, self.margin+20), 
                    self.font, self.font_scale, (255, 255, 0), self.thickness)

        # 繪製移動角度
        yaw_text, pitch_text = f"Yaw: {gimbalTask.output_deg[0]:.1f}", f"Pitch: {gimbalTask.output_deg[1]:.1f}"
        cv2.putText(frame, yaw_text, (self.margin, self.margin+45), 
                    self.font, self.font_scale, (255, 128, 0), self.thickness)
        cv2.putText(frame, pitch_text, (self.margin, self.margin+65), 
                    self.font, self.font_scale, (255, 128, 0), self .thickness)

        # visual_ranging
        
        # 繪製圖像框資訊 (置中顯示在底部)
        bbox_text = f'bbox: {pub_bbox["x0"]}, {pub_bbox["y0"]}, {pub_bbox["x1"]}, {pub_bbox["y1"]}'
        bbox_size, _ = cv2.getTextSize(bbox_text, self.font, self.font_scale, self.thickness)
        bbox_x = (w - bbox_size[0]) // 2  # 置中對齊
        bbox_y = h - self.margin  # 靠近底部

        cv2.putText(frame, bbox_text, (bbox_x, bbox_y), 
                    self.font, self.font_scale, (0, 255, 255), self.thickness)

        if gimbalTask.D_c is not None:
            cv2.putText(frame, f'D_c:{gimbalTask.D_c}, D_obj:{gimbalTask.D_obj},', 
                        (360,10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), self.thickness)
        
        # 繪製 FPS
        if YOLO_FPS is not None:
            fps_str = f"FPS: {YOLO_FPS:.0f}"
            fps_size, _ = cv2.getTextSize(fps_str, self.font, self.font_scale, self.thickness)
            fps_x = w - fps_size[0] - self.margin
            fps_y = self.margin + fps_size[1]
            cv2.putText(frame, fps_str, (fps_x, fps_y), self.font, self.font_scale, (0, 255, 0), self.thickness)

        return frame

    def record(self):
        """
        持續從 Queue 中取出畫面並寫到檔案 (app_state.vid_writer)。
        """
        last_frame = None
        while not self.app_state.stop_threads:
            try:
                frame = self.app_state.frame_queue.get(timeout=1)
                last_frame = frame
            except queue.Empty:
                if last_frame is None:
                    time.sleep(0.1)
                    continue
                frame = last_frame
            except Exception as e:
                print(f"錄影錯誤: {e}")
                self.app_state.stop_threads = True
                break

            # 在畫面上疊加資訊再寫檔
            frame = self.put_info(frame)
            frame = cv2.resize(frame, (self.video_width, self.video_height))
            self.app_state.vid_writer.write(frame)

        print("[record] thread exit...")

    def start_recording(self):
        """啟動錄影執行緒 (建議在 main 裡呼叫)"""
        self.recording_thread = threading.Thread(target=self.record, daemon=True)
        self.recording_thread.start()
        print("錄影開始")

    def stop_recording(self):
        """停止錄影執行緒"""
        self.app_state.stop_threads = True
        if hasattr(self, "recording_thread"):
            self.recording_thread.join()
        print("錄影結束")

def cale_record_fps(app_state:AppState, model):
    aver_fps = 0
    aver_fps_stat = False

    for i in range(11):
        if not aver_fps_stat:
            # 讀取影格
            ret, frame = app_state.cap.read()
            t1 = time.time()
            results = model.predict(frame, imgsz=IMGSZ, conf=CONF_THRESHOLD)
            t2 = time.time()

            YOLO_FPS = 1.0 / (t2 - t1)

            # ------- 關鍵：跳過第一幀 (i == 0) --------
            if i == 0:
                # 第一幀的 YOLO_FPS 不要加入 aver_fps
                continue
            
            aver_fps += YOLO_FPS
            if i == 10 and not aver_fps_stat:
                app_state.record_fps = aver_fps / 10  
                
                # 開始錄影
                video_processor.start_recording()
                aver_fps_stat = True

                print("aver_fps sum:", aver_fps)              # 總和
                print("Final average FPS:", app_state.record_fps)  # 平均幀率

        else:
            break

# GimbalTimerTask
if trackMode == 'pid':
    gimbalTask = GimbalTimerTask(trackMode)
else:
    h_fov = json_config.get(["video_resolutions", "default", "Horizontal_FOV"])
    v_fov = json_config.get(["video_resolutions", "default", "Vertical_FOV"])
    gimbalTask = GimbalTimerTask(trackMode, 11, h_fov, v_fov)

# 清理資源與訊號處理
def cleanup_resources(app_state:AppState, gimbal_task:GimbalTimerTask):
    print("[cleanup_resources] start...")
    # 1) 停止自定義執行緒
    app_state.stop_threads = True

    # 2) 關閉 gimbalTask
    if gimbal_task:
        gimbal_task.close()

    # 3) 關閉 executor
    if hasattr(app_state, "executor") and app_state.executor is not None:
        app_state.executor.shutdown()

    if hasattr(app_state, "ROS_spin") and app_state.ROS_spin.is_alive():
        app_state.ROS_spin.join()

    # 4) 銷毀 Node
    try:
        if hasattr(gimbal_task, "destroy_node"):
            gimbal_task.destroy_node()
        if hasattr(ROS_Pub, "destroy_node"):
            ROS_Pub.destroy_node()
        if hasattr(ROS_Sub, "destroy_node"):
            ROS_Sub.destroy_node()
    except Exception as e:
        print(f"Error destroying nodes: {e}")

    # 5) shutdown ROS
    if rclpy.ok():
        rclpy.shutdown()

    # 6) 釋放其他非 ROS 資源
    if hasattr(app_state, "vid_writer") and app_state.vid_writer is not None:
        app_state.vid_writer.release()
    if hasattr(app_state, "cap") and app_state.cap.isOpened():
        app_state.cap.release()

    # 關閉任何 OpenCV 視窗
    cv2.destroyAllWindows()

    print("[cleanup_resources] Done cleaning up.")

def signal_handler(sig, frame, app_state:AppState):
    print(f"Received signal {sig}, exiting...")
    cleanup_resources(app_state, gimbalTask)
    os._exit(0)

# ROS 執行緒 (多執行緒 Executor)
def _spinThread(*args):
    global executor
    executor = MultiThreadedExecutor()
    for task in args:
        executor.add_node(task)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        rclpy.shutdown()

# 目標模型載入
pt_file = check_file(r'landpadv11.pt')
MODEL = YOLO(pt_file)

# 主要偵測函數：擔任主執行緒
def detect_loop(app_state: AppState, model: YOLO, obj_class:int, video_processor: VideoProcessor):
    global YOLO_FPS
          
    if SAVE:
        cale_record_fps(app_state, model)
    
    while not app_state.stop_threads:
        ret, frame = app_state.cap.read()
        if not ret or frame is None:
            time.sleep(0.05)
            continue

        # 1) 執行 YOLO 推論
        t1 = time.time()
        results = model.predict(frame, imgsz=IMGSZ, conf=CONF_THRESHOLD)
        t2 = time.time()
        YOLO_FPS = 1.0 / (t2 - t1)  # 更新全域 FPS
                
        pred = results[0].boxes.data  # tensor
        # 2) 解析結果
        x0 = y0 = x1 = y1 = 0
        max_xyxy = None
        max_conf = -1
        found_target = False

        if len(pred):
            for box in results[0].boxes:
                cls_id = int(box.cls[0])
                conf = box.conf[0].item()
                # 只找特定 class
                if cls_id == obj_class:
                    if conf > max_conf:
                        max_conf = conf
                        max_xyxy = box.xyxy[0]
                    found_target = True

            if found_target and max_xyxy is not None:
                pub_bbox["detect"] = pub_img["detect"] = True
                pub_bbox["conf"] = max_conf
                pub_bbox["id"] = obj_class
                pub_bbox["name"] = model.names[obj_class]
                x0, y0, x1, y1 = map(int, max_xyxy.cpu())
                pub_bbox["x0"], pub_bbox["y0"] = x0, y0
                pub_bbox["x1"], pub_bbox["y1"] = x1, y1
            else:
                bbox_init()
        else:
            bbox_init()

        # 3) 更新雲台
        gimbalTask.xyxy_update(pub_bbox["detect"], x0, y0, x1, y1)
            
        # 4) 顯示/錄影
        if SHOW or app_state.save_img:
            # 畫框
            if pub_bbox["detect"]:
                label = f"{pub_bbox['name']} {pub_bbox['conf']:.2f}"
                cv2.rectangle(frame, (x0, y0), (x1, y1), (0, 255, 0), 2)
                cv2.putText(frame, label, (x0, y0 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

            # 如果要錄影，先把原圖放到 queue，之後再由 record thread 做 put_info
            if app_state.save_img:
                with app_state.frame_lock:
                    if not app_state.frame_queue.full():
                        app_state.frame_queue.put(frame)

            # 如果要即時顯示，這裡就直接疊加資訊
            if SHOW:
                show_frame = video_processor.put_info(frame.copy())
                cv2.imshow('YOLOv11', show_frame)
                if cv2.waitKey(1) == ord('q'):
                    break

    # 離開迴圈後，清理資源
    cleanup_resources(app_state, gimbalTask)

# Global ROS Node (先在 global 建立節點)
ROS_Pub = MinimalPublisher()
ROS_Sub = MinimalSubscriber()

# Main
def main():
    app_state = AppState(save_img=SAVE)  # 是否要開啟錄影

    # 註冊訊號 (Ctrl+C / kill SIGTERM)
    signal.signal(signal.SIGINT,  lambda s,f: signal_handler(s,f,app_state))
    signal.signal(signal.SIGTERM, lambda s,f: signal_handler(s,f,app_state))

    # 建立 VideoProcessor
    global video_processor
    video_processor = VideoProcessor(app_state, ROS_Sub, VIDEO_WIDTH, VIDEO_HEIGHT)

    # 啟動錄影執行緒
    # if app_state.save_img:
    #     video_processor.start_recording()

    # 開 ROS spin 執行緒
    app_state.ROS_spin = threading.Thread(target=_spinThread, args=(ROS_Pub, ROS_Sub, gimbalTask), daemon=True)
    app_state.ROS_spin.start()

    # 執行偵測 (主線程)
    try:
        detect_loop(app_state, MODEL, 0, video_processor)
    finally:
        # 停止錄影
        video_processor.stop_recording()

if __name__ == "__main__":
    main()

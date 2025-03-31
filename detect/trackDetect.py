from pathlib import Path
import signal
import sys, os, time, datetime
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../')))
import threading, queue
import cv2, math, csv

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import ReliabilityPolicy, QoSProfile
from sensor_msgs.msg import NavSatFix, Imu
from mavros_msgs.msg import Altitude
from transforms3d import euler
from std_msgs.msg import Float64
from tutorial_interfaces.msg import Img, Bbox

from ultralytics import YOLO
from camera_function import gstreamer_pipeline
from utils import JsonHandler, CSVHandler, check_imshow, increment_path, check_file, system_time
from ctrl.gimbal_ctrl import GimbalTimerTask
from ctrl.pid.motor import normalize_angle_180
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
VIDEO_WIDTH = json_config.get(["video_resolutions", "HD", "width"])
VIDEO_HEIGHT = json_config.get(["video_resolutions", "HD", "height"])
SHOW = json_config.get(["image_show", "switch"]) if check_imshow else False
IMGSZ = json_config.get(["yolo", "imgsz"])
CONF_THRESHOLD = json_config.get(["yolo", "conf"])
SAVE = json_config.get(["img_save", "default"])
trackMode = json_config.get(["trackMode", "default"])
save_data = json_config.get(["data_save", "default"])

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

class AppState():
    def __init__(self, save_img=False):
        self.save_img = save_img
        self.stop_threads = False
        self.frame_lock = threading.Lock()
        self.executor = MultiThreadedExecutor()
        
        self.ROS_stop_event = threading.Event()
        self.ROS_spin = threading.Thread()
        self.cap = cv2.VideoCapture(
            gstreamer_pipeline(capture_width=VIDEO_WIDTH,
                               capture_height=VIDEO_HEIGHT,
                               flip_method=0), 
            cv2.CAP_GSTREAMER
        )
        self.total_detect_count = 0
        
        # 若要存檔，建立路徑、VideoWriter
        if save_img or save_data:
            self.video_name = "output.avi"
            self.save_path = "/home/ubuntu/track/track2/runs_test"
            self.save_path = increment_path(Path(self.save_path) / "exp", exist_ok=False)
            self.save_path.mkdir(parents=True, exist_ok=True)
            
            # 完整檔案路徑
            self.name = str(self.save_path / self.video_name)
            print(f"**** Video file path: {self.name} ****")
            if save_img:
                # 建立 VideoWriter
                self.record_fps = 13
                self.vid_writer = cv2.VideoWriter(self.name, cv2.VideoWriter_fourcc(*'XVID'), self.record_fps, (VIDEO_WIDTH, VIDEO_HEIGHT))
                
            if save_data:
                self.csv_data = {
                    "time": system_time(),
                    "GPS/RTK" : "None",
                    "latitude": 0.0,
                    "longitude": 0.0,
                    "altitude(m)": 0.0,
                    "H": 0.0,
                    "detectionCount": 0,
                    "gimbalYawDeg(°)": 0.0,
                    "gimbalPitchDeg(°)": 0.0,
                    "gimbalYawMove(°)": f"{gimbalTask.output_deg[0]:.2f}",
                    "gimbalPitchMove(°)": f"{gimbalTask.output_deg[1]:.2f}",
                    "gimbalCemter": False,
                    "FPS": 0.0,
                    "centerDistance": None,
                    "Bbox_x1": 0, "Bbox_x2": 0,
                    "Bbox_y1":0, "Bbox_y2":0,
                    "distanceVisual": 0.0,
                    "distanceActual": 0.0,
                    "thetaDeg(°)" : 0.0, "phiDeg(°)" : 0.0
                    }                
                # 寫入資料到CSV
                log_name = str(self.save_path) + "/log.csv" 
                self.log = CSVHandler(log_name, self.csv_data)
                
        # 用來做錄影時的 Frame buffer
        self.frame_queue = queue.Queue(maxsize=10)
    
    def csv_init_val(self, data):
        self.log.write_row(data)

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__("minimal_subscriber")
        self.GlobalPositionRTK_Sub = self.create_subscription(NavSatFix, "mavros/global_position/global", self.RTcb, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.GlobalPositionGPS_Sub = self.create_subscription(NavSatFix, "mavros/global_position/raw/fix", self.GPcb, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.AltitudeSub = self.create_subscription(Altitude, 'mavros/altitude', self.Altcb, 10)        
        self.imuSub = self.create_subscription(Imu, "mavros/imu/data", self.IMUcb, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.holdSub = self.create_subscription(Img, "img", self.holdcb, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.globalPosition = self.create_timer(1/15, self.postion)
        # 初始化變數
        self.detect = False
        self.ID = -1
        self.conf = -1
        self.x0 = self.y0 = self.x1 = self.y1 = 0
        
        self.gps_stat = "None"
        self.rtk_latitude = 0.0
        self.rtk_longitude = 0.0
        self.rtk_altitude = 0.0
        self.gps_latitude = 0.0
        self.gps_longitude = 0.0
        self.gps_altitude = 0.0
        self.latitude = 0.0
        self.longitude = 0.0
        self.altitude = 0.0
        self.relative_altitude = 0.0
        self.drone_pitch = 0.0
        self.drone_roll = 0.0
        self.drone_yaw = 0.0
        self.hold = False
        self.initialHeight = self.__initialHeight__()
        
    def __initialHeight__(self):
        while self.altitude != 0.0:
            self.initialHeight = self.altitude
            break
    
    def holdcb(self, msg):
        self.hold = pub_img["hold_status"] = msg.hold_status

    def RTcb(self, msg):
        self.rtk_latitude = msg.latitude
        self.rtk_longitude = msg.longitude
        self.rtk_altitude = msg.altitude

    def GPcb(self, msg):
        self.gps_latitude = msg.latitude
        self.gps_longitude = msg.longitude
        self.gps_altitude = msg.altitude

    def Altcb(self, msg): 
        self.altitude = msg.relative
        
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

    def postion(self):
        if self.rtk_latitude == 0.0 and self.rtk_longitude == 0.0:
            # RTK 無數據
            self.gps_stat = "GPS"
            self.latitude = self.gps_latitude
            self.longitude = self.gps_longitude
            self.altitude = self.gps_altitude
        elif self.rtk_latitude != 0.0 and self.rtk_longitude != 0.0:
            # RTK 數據
            self.gps_stat = "RTK"
            self.latitude = self.rtk_latitude
            self.longitude = self.rtk_longitude
            self.altitude = self.rtk_altitude
        else:
            # 無數據
            self.gps_stat = "None"

    def get_gps_data(self):
        return self.latitude, self.longitude, self.gps_altitude

    def relative_height(self):
        if self.initialHeight is None:
            return 0.0
        return self.altitude - self.initialHeight
        
class MinimalPublisher(Node):
    def __init__(self):
        super().__init__("minimal_publisher")
        # Img publish
        self.imgPublish = self.create_publisher(Img, "img", 1)
        img_timer_period = 1/10
        self.img_timer = self.create_timer(img_timer_period, self.img_callback)
                
        self.img = Img()
        
    def img_callback(self):
        pub_img['camera_center'] = gimbalTask.center_status
        
        yA, pA = gimbalTask.get_angle()
        pub_img['motor_pitch'] = pA + ROS_Sub.drone_pitch
        pub_img['motor_yaw'] = normalize_angle_180(yA) * -1
        
        (self.img.detect, 
         self.img.camera_center, 
         self.img.motor_pitch, 
         self.img.motor_yaw, 
         self.img.target_latitude, 
         self.img.target_longitude, 
         self.img.hold_status, 
         self.img.send_info) = pub_img.values()
        # print(f"pubData: detect:{pub_img['detect']}, center: {pub_img['camera_center']}")
        self.imgPublish.publish(self.img)

# 在影像上繪製文字、並執行錄影
class VideoProcessor:
    def __init__(self, app_state, sub, video_width=1920, video_height=1080):
        self.app_state = app_state
        self.sub = sub
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
        lat, lon, alt= self.sub.get_gps_data()
        gps_text = f"Lat: {lat:.6f}, Lon: {lon:.6f}, alt: {alt:.1f}"
        cv2.putText(frame, gps_text, (self.margin, self.margin+20), 
                    self.font, self.font_scale, (255, 255, 0), self.thickness)
        
        # 繪製相對高度
        h_text = f"H: {self.sub.relative_height()}"
        cv2.putText(frame, h_text, (self.margin, self.margin+45), 
                    self.font, self.font_scale, (0, 255, 0), self.thickness)
        
        # 繪製移動角度
        yaw_text, pitch_text = f"ctrlYaw: {gimbalTask.output_deg[0]:.1f}", f"ctrlPitch: {gimbalTask.output_deg[1]:.1f}"
        cv2.putText(frame, yaw_text, (self.margin, self.margin+65), 
                    self.font, self.font_scale, (2, 128, 20), self.thickness)
        cv2.putText(frame, pitch_text, (self.margin, self.margin+85), 
                    self.font, self.font_scale, (2, 128, 20), self .thickness)
        
        yawAngle, pitchAngle = f"Yaw: {gimbalTask.yaw.info.angle:.1f}", f"Pitch: {gimbalTask.pitch.info.angle:.1f}" 
        cv2.putText(frame, yawAngle, (self.margin, self.margin+105), 
                    self.font, self.font_scale, (2, 128, 20), self .thickness)
        cv2.putText(frame, pitchAngle, (self.margin, self.margin+125), 
                    self.font, self.font_scale, (2, 128, 20), self .thickness)
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
h_fov = json_config.get(["video_resolutions", "default", "Horizontal_FOV"])
v_fov = json_config.get(["video_resolutions", "default", "Vertical_FOV"])
gimbalTask = GimbalTimerTask(trackMode, 160, h_fov, v_fov)

# 飛行日誌
class Log(object):
    def __init__(self, app_state:AppState, sub:MinimalSubscriber):
        self.app_state = app_state
        self.data = app_state.csv_data
        self.log = app_state.log
        self.sub = sub
    
    def get_data(self):
        _, sys_time = system_time()
        self.data = {
            "time": sys_time,
            "GPS/RTK" : self.sub.gps_stat,
            "latitude": self.sub.latitude,
            "longitude": self.sub.longitude,
            "altitude(m)": self.sub.altitude,
            "H": f"{self.sub.relative_height():.1f}",
            "detectionCount": self.app_state.total_detect_count,
            "gimbalYawDeg(°)": f"{pub_img['motor_yaw']:.3f}",
            "gimbalPitchDeg(°)": f"{pub_img['motor_pitch']:.3f}",
            "gimbalYawMove(°)": f"{gimbalTask.output_deg[0]:.2f}",
            "gimbalPitchMove(°)": f"{gimbalTask.output_deg[1]:.2f}",
            "gimbalCemter": f"{gimbalTask.center_status}",
            "FPS": f"{YOLO_FPS:.0f}",
            "centerDistance": gimbalTask.centerDistance,
            "Bbox_x1": pub_bbox["x0"], "Bbox_x2": pub_bbox["x1"],
            "Bbox_y1": pub_bbox["y0"], "Bbox_y2": pub_bbox["y1"],
            "distanceVisual": f"{gimbalTask.threeD_data['distance_visual']:.3f}" if gimbalTask.threeD_data['distance_visual'] is not None else "0.000",
            "distanceActual": f"{gimbalTask.threeD_data['distance_actual']:.3f}" if gimbalTask.threeD_data['distance_actual'] is not None else "0.000",
            "thetaDeg(°)": f"{gimbalTask.threeD_data['theta_deg']:.3f}" if gimbalTask.threeD_data['theta_deg'] is not None else "0.000",
            "phiDeg(°)": f"{gimbalTask.threeD_data['phi_deg']:.3f}" if gimbalTask.threeD_data['phi_deg'] is not None else "0.000"
        }
        return self.data
        
    def write(self):
        self.log.write_row(self.get_data())

# 清理資源與訊號處理
def cleanup_resources(app_state:AppState, gimbal_task:GimbalTimerTask):
    print("[cleanup_resources] start...")
    # 停止自定義執行緒
    app_state.stop_threads = True

    # 關閉 gimbalTask
    if gimbal_task:
        gimbal_task.close()

    # 關閉 executor
    if hasattr(app_state, "executor") and app_state.executor is not None:
        app_state.executor.shutdown()

    if hasattr(app_state, "ROS_spin") and app_state.ROS_spin.is_alive():
        app_state.ROS_spin.join()

    # 銷毀 Node
    try:
        if hasattr(gimbal_task, "destroy_node"):
            gimbal_task.destroy_node()
        if hasattr(ROS_Pub, "destroy_node"):
            ROS_Pub.destroy_node()
        if hasattr(ROS_Sub, "destroy_node"):
            ROS_Sub.destroy_node()
    except Exception as e:
        print(f"Error destroying nodes: {e}")

    # shutdown ROS
    if rclpy.ok():
        rclpy.shutdown()

    # 釋放其他非 ROS 資源
    if hasattr(app_state, "vid_writer") and app_state.vid_writer is not None:
        app_state.vid_writer.release()
    if hasattr(app_state, "cap") and app_state.cap.isOpened():
        app_state.cap.release()

    # 關閉所有 OpenCV 視窗
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
    detect_counters = gimbalTask.detect_countuers
    if SAVE:
        cale_record_fps(app_state, model)
    not_read_count = 0
    while not app_state.stop_threads:
        ret, frame = app_state.cap.read()
        if not ret or frame is None:
            not_read_count += 1
            if not_read_count >= 3:
                break 
            time.sleep(0.05)
            continue

        # 執行 YOLO 推論
        t1 = time.time()
        results = model.predict(frame, imgsz=IMGSZ, conf=CONF_THRESHOLD)
        t2 = time.time()
        YOLO_FPS = 1.0 / (t2 - t1)  # 更新全域 FPS
                
        pred = results[0].boxes.data  # tensor
        # 解析結果
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

        # 更新CSV
        log.write()
        
        # 更新雲台
        gimbalTask.xyxy_update(pub_bbox["detect"], x0, y0, x1, y1)
            
        # 顯示/錄影
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

            if SHOW:
                show_frame = video_processor.put_info(frame.copy())
                cv2.imshow('YOLOv11', show_frame)
                if cv2.waitKey(1) == ord('q'):
                    break
                
        # 檢測次數計數
        app_state.total_detect_count += 1
    # 離開迴圈後，清理資源
    cleanup_resources(app_state, gimbalTask)


# Main
def main():
    app_state = AppState(save_img=SAVE)  # 是否要開啟錄影
    
    # Global ROS Node
    global ROS_Pub, ROS_Sub
    ROS_Pub = MinimalPublisher()
    ROS_Sub = MinimalSubscriber()
    
    # csv log
    global log
    log = Log(app_state, ROS_Sub)
    
    # 註冊訊號 (Ctrl+C / kill SIGTERM)
    signal.signal(signal.SIGINT,  lambda s,f: signal_handler(s, f, app_state))
    signal.signal(signal.SIGTERM, lambda s,f: signal_handler(s, f, app_state))

    # 建立 VideoProcessor
    global video_processor
    video_processor = VideoProcessor(app_state, ROS_Sub, VIDEO_WIDTH, VIDEO_HEIGHT)

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

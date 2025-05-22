from pathlib import Path
import signal
import sys, os, time, datetime
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../')))
import threading, queue
import cv2, math, csv

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from tutorial_interfaces.msg import Img, Bbox

from ultralytics import YOLO
from camera_function import gstreamer_pipeline
from utils import JsonHandler, CSVHandler, check_imshow, increment_path, check_file, system_time
from ctrl.gimbal_ctrl import GimbalTimerTask
from ctrl.pid.motor import normalize_angle_180
from flyLog.csvLog import LogWrite, LogData
from ros.ros_topic import MinimalSubscriber, MinimalPublisher
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
SAVEIMG = json_config.get(["img_save", "default"])
trackMode = json_config.get(["trackMode", "default"])
save_data = json_config.get(["data_save", "default"])

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
            self.save_path = "/home/ubuntu/track/track2/runs/runs_test/0430"
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
                self.log_name = str(self.save_path) + "/log.csv" 
                self.log = LogWrite(self.log_name)
                
        # 用來做錄影時的 Frame buffer
        self.frame_queue = queue.Queue(maxsize=10)
    

# 在影像上繪製文字、並執行錄影
class VideoProcessor:
    def __init__(self, app_state, sub, video_width=1920, video_height=1080):
        self.app_state = app_state
        self.sub = sub
        self.video_width = video_width
        self.video_height = video_height

        self._init_display_params(video_width, video_height)
        self.font = cv2.FONT_HERSHEY_SIMPLEX

    def _init_display_params(self, w, h):
        # if w == 1920 and h == 1080:
        #     self.font_scale, self.thickness, self.line_spacing, self.margin = 1, 3, 50, 20
        # elif w == 1280 and h == 720:
        #     self.font_scale, self.thickness, self.line_spacing, self.margin = 0.7, 2, 35, 10
        self.font_scale, self.thickness, self.line_spacing, self.margin = 0.7, 2, 35, 10

    def put_text_line(self, frame, text, x, y, color=(0, 255, 0)):
        cv2.putText(frame, text, (x, y), self.font, self.font_scale, color, self.thickness)

    def put_info(self, frame):
        h, w = frame.shape[:2]
        current_date = datetime.date.today().strftime("%Y-%m-%d")
        current_time = datetime.datetime.now().strftime("%H:%M:%S")

        # 中央顯示日期時間
        for i, text in enumerate([current_date, current_time]):
            text_size, _ = cv2.getTextSize(text, self.font, self.font_scale, self.thickness)
            text_x = (w - text_size[0]) // 2
            text_y = self.margin + (i * self.line_spacing) + text_size[1]
            self.put_text_line(frame, text, text_x, text_y)

        # GPS、相對高度
        lat, lon, alt = self.sub.get_gps_data()
        gps_lines = [
            f"Lat: {lat:.6f}, Lon: {lon:.6f}, alt: {alt:.1f}",
            f"H: {self.sub.relative_height()}"
        ]
        # 雲台資料
        gimbal_lines = [
            f"ctrlYaw: {gimbalTask.output_deg[0]:.1f}",
            f"ctrlPitch: {gimbalTask.output_deg[1]:.1f}",
            f"Yaw: {gimbalTask.yaw.info.angle:.1f}",
            f"Pitch: {gimbalTask.pitch.info.angle:.1f}"
        ]
        lines = gps_lines + gimbal_lines
        for i, text in enumerate(lines):
            self.put_text_line(frame, text, self.margin, self.margin + 20 + (i * 20), (2, 128, 20))

        # 圖像框資訊
        bbox = ROS_Pub.pub_bbox
        bbox_text = f'bbox: {bbox["x0"]}, {bbox["y0"]}, {bbox["x1"]}, {bbox["y1"]}'
        bbox_size, _ = cv2.getTextSize(bbox_text, self.font, self.font_scale, self.thickness)
        self.put_text_line(frame, bbox_text, (w - bbox_size[0]) // 2, h - self.margin, (0, 255, 255))

        # D_c、D_obj
        if getattr(gimbalTask, "D_c", None) is not None:
            dc_text = f'D_c:{gimbalTask.D_c}, D_obj:{gimbalTask.D_obj}'
            self.put_text_line(frame, dc_text, 360, 30, (0, 255, 255))

        # FPS 顯示
        if YOLO_FPS is not None:
            fps_text = f"FPS: {YOLO_FPS:.0f}"
            fps_size, _ = cv2.getTextSize(fps_text, self.font, self.font_scale, self.thickness)
            self.put_text_line(frame, fps_text, w - fps_size[0] - self.margin, self.margin + fps_size[1])

        return frame

    def record(self):
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

            frame = self.put_info(frame)
            frame = cv2.resize(frame, (self.video_width, self.video_height))
            self.app_state.vid_writer.write(frame)

        print("[record] thread exit...")
        
    @staticmethod
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
        
    def start_recording(self):
        self.recording_thread = threading.Thread(target=self.record, daemon=True)
        self.recording_thread.start()
        print("錄影開始")

    def stop_recording(self):
        self.app_state.stop_threads = True
        if hasattr(self, "recording_thread"):
            self.recording_thread.join()
        print("錄影結束")

# 飛行日誌
class Log(object):
    def __init__(self, app_state:AppState, sub:MinimalSubscriber):
        self.app_state = app_state
        self.data = LogData()
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
            "H": f"{self.sub.relative_height() - self.sub.altitude:.1f}",
            "detectionCount": self.app_state.total_detect_count,
            "gimbalYawDeg(°)": f"{ROS_Pub.pub_img['motor_yaw']:.3f}",
            "gimbalPitchDeg(°)": f"{ROS_Pub.pub_img['motor_pitch']:.3f}",
            "gimbalYawMove(°)": f"{gimbalTask.output_deg[0]:.2f}",
            "gimbalPitchMove(°)": f"{gimbalTask.output_deg[1]:.2f}",
            "gimbalCemter": f"{gimbalTask.center_status}",
            "FPS": f"{YOLO_FPS:.0f}" if YOLO_FPS is not None else "N/A",
            "centerDistance": gimbalTask.centerDistance,
            "Bbox_x1": ROS_Pub.pub_bbox["x0"], "Bbox_x2": ROS_Pub.pub_bbox["x1"],
            "Bbox_y1": ROS_Pub.pub_bbox["y0"], "Bbox_y2": ROS_Pub.pub_bbox["y1"],
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
    if SAVEIMG:
        video_processor.cale_record_fps(app_state, model)
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
                ROS_Pub.pub_bbox["detect"] = ROS_Pub.pub_img["detect"] = True
                ROS_Pub.pub_bbox["conf"] = max_conf
                ROS_Pub.pub_bbox["id"] = obj_class
                ROS_Pub.pub_bbox["name"] = model.names[obj_class]
                x0, y0, x1, y1 = map(int, max_xyxy.cpu())
                ROS_Pub.pub_bbox["x0"], ROS_Pub.pub_bbox["y0"] = x0, y0
                ROS_Pub.pub_bbox["x1"], ROS_Pub.pub_bbox["y1"] = x1, y1
            else:
                ROS_Pub.bbox_init()
        else:
            ROS_Pub.bbox_init()

        # 更新CSV
        if save_data:
            log.write()
        
        # 更新雲台
        gimbalTask.xyxy_update(ROS_Pub.pub_bbox["detect"], x0, y0, x1, y1)
            
        # 顯示/錄影
        if SHOW or app_state.save_img:
            # 畫框
            if ROS_Pub.pub_bbox["detect"]:
                label = f"{ROS_Pub.pub_bbox['name']} {ROS_Pub.pub_bbox['conf']:.2f}"
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

def gimbalTaskInit(sub):
    h_fov = json_config.get(["video_resolutions", "default", "Horizontal_FOV"])
    v_fov = json_config.get(["video_resolutions", "default", "Vertical_FOV"])
    return GimbalTimerTask(sub, trackMode, 160, h_fov, v_fov)

# Main
def main():
    # Global ROS Node
    global ROS_Pub, ROS_Sub, gimbalTask
    ROS_Sub = MinimalSubscriber()
    gimbalTask = gimbalTaskInit(ROS_Sub)
    ROS_Pub = MinimalPublisher(ROS_Sub, gimbalTask)    
    
    app_state = AppState(save_img=SAVEIMG)  # 是否要開啟錄影
    
    # csv log
    global log
    if save_data:
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
        if SAVEIMG:
            video_processor.stop_recording()

if __name__ == "__main__":
    main()

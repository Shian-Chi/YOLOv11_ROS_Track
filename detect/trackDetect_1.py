from pathlib import Path
import signal
import sys, os, time, datetime
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../')))
import threading, queue
import cv2 ,math
import numpy as np

import rclpy
from rclpy.executors import MultiThreadedExecutor

from ultralytics import YOLO
from camera_function import gstreamer_pipeline
from utils import JsonHandler, check_imshow, increment_path, check_file, system_time
from ctrl.gimbal_ctrl import GimbalTimerTask
from flyLog.csvLog import LogWrite, LogData
from lidar.lidar_alt import LidarPublisher
from position.two_pt_pos import targetPositioning
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
trackMode = json_config.get(["track", "default"])
track_obj_size = json_config.get(["track", "obj_size"])
save_data = json_config.get(["data_save", "default"])
is_Stream = json_config.get(["img_send", "switch"])

class AppState():
    def __init__(self, sub:MinimalSubscriber, save_img=False, save_data=False):
        self.sub = sub
        self.save_img = save_img
        self.save_data = save_data
        self.stop_threads = False
        self.executor = MultiThreadedExecutor()
        self.ROS_spin = threading.Thread()
        self.cap = self._cap_()
        self.total_detect_count = 0
        self.stream_on = is_Stream  # 動態控制 flag
        self.out = None           # GStreamer stream writer
        
        # 若要存檔，建立路徑、VideoWriter
        if save_img or save_data:
            self._save_()
            if save_img:
                self.frame_lock = threading.Lock()
                self.video_name = "output.avi"
            
                self.name = str(self.save_path / self.video_name)
                print(f"**** \033[31m Video file path: {self.name} \033[0m ****")
                self.record_fps = 10.0
                self.video_save = cv2.VideoWriter(self.name, cv2.VideoWriter_fourcc(*'H264'), self.record_fps, (VIDEO_WIDTH, VIDEO_HEIGHT))
            if save_data:
                self.log_name = str(self.save_path) + "/log.csv" 
                self.log = LogWrite(self.log_name)
                
        # 用來做錄影時的 Frame buffer
        self.frame_queue = queue.Queue(maxsize=1)

        self.positioning = targetPositioning(sub)
        
    def _cap_(self):
        return cv2.VideoCapture(
            gstreamer_pipeline(capture_width=VIDEO_WIDTH,
                               capture_height=VIDEO_HEIGHT,
                               flip_method=0), 
            cv2.CAP_GSTREAMER
        )

    def _save_(self):
        self.save_path = "/home/ubuntu/track/track2/runs/test"
        self.save_path = increment_path(Path(self.save_path) / "exp", exist_ok=False)
        self.save_path.mkdir(parents=True, exist_ok=True)
    
    def _stream_GStreamer_(self, width, height, fps, bitrate, ip, port):
        return (
            f"appsrc ! videoconvert ! nvvidconv ! video/x-raw(memory:NVMM), format=NV12, width={width}, height={height}, framerate={fps}/1 ! "
            f"nvv4l2h264enc bitrate={bitrate} ! rtph264pay config-interval=1 pt=96 ! "
            f"udpsink host={ip} port={port}"
        )
    
# 在影像上繪製文字、並執行錄影
class VideoProcessor:
    def __init__(self, app_state:AppState, sub:MinimalSubscriber, gimbal:GimbalTimerTask, video_width=1280, video_height=720):
        self.app_state = app_state
        self.sub = sub
        self.gimbal = gimbal
        self.video_width = video_width
        self.video_height = video_height

        self._init_display_params()
        self.font = cv2.FONT_HERSHEY_SIMPLEX

        self.frame = None
        self.last_frame = None
        self.image_center_x = VIDEO_WIDTH / 2
        self.image_center_y = VIDEO_HEIGHT / 2
    def _init_display_params(self):
        self.font_scale, self.thickness, self.line_spacing, self.margin = 0.7, 2, 35, 10

    def put_text_line(self, frame, text, x, y, color=(0, 255, 0)):
        cv2.putText(frame, text, (x, y), self.font, self.font_scale, color, self.thickness)

    def put_info(self, frame):
        if frame is None:
            return frame
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
            f"H: {self.sub.relative_altitude}"
        ]
        # 雲台資料
        gimbal_lines = [
            f"ctrlYaw: {self.gimbal.output_deg[0]:.1f}",
            f"ctrlPitch: {self.gimbal.output_deg[1]:.1f}",
            f"Yaw: {self.gimbal.yaw.info.angle:.1f}",
            f"Pitch: {self.gimbal.pitch.info.angle:.1f}"
        ]
        lines = gps_lines + gimbal_lines
        for i, text in enumerate(lines):
            self.put_text_line(frame, text, self.margin, self.margin + 20 + (i * 20), (2, 128, 20))
            
        if ROS_Pub.pub_img["detect"]:
            # 圖像框資訊
            bbox = ROS_Pub.pub_bbox
            bbox_text = f'bbox: {bbox["x0"]}, {bbox["y0"]}, {bbox["x1"]}, {bbox["y1"]}'
            bbox_size, _ = cv2.getTextSize(bbox_text, self.font, self.font_scale, self.thickness)       
            self.put_text_line(frame, bbox_text, (w - bbox_size[0]) // 2, h - self.margin, (0, 255, 255))

            # 計算距離到中心點 (直接計算，不使用額外方法)
            bbox_center_x = (bbox["x0"] + bbox["x1"]) / 2
            bbox_center_y = (bbox["y0"] + bbox["y1"]) / 2
            dis = math.sqrt((bbox_center_x - self.image_center_x)**2 + (bbox_center_y - self.image_center_y)**2)

            # 顯示距離資訊
            dis_text = f'Distance: {dis:.1f}px'
            dis_size, _ = cv2.getTextSize(dis_text, self.font, self.font_scale, self.thickness)
            self.put_text_line(frame, dis_text, (w - dis_size[0]) // 2, h - self.margin - 30, (255, 0, 255))

            # D_c、D_obj
            # if getattr(self.gimbal, "D_c", None) is not None:
            dc_text = f"Dc:{self.gimbal.threeD_data['distance_visual']}, Dobj:{self.gimbal.threeD_data['distance_actual']}"
            self.put_text_line(frame, dc_text, 10, 360, (0, 255, 255))

        # FPS 顯示
        if YOLO_FPS is not None:
            fps_text = f"FPS: {YOLO_FPS:.0f}"
            fps_size, _ = cv2.getTextSize(fps_text, self.font, self.font_scale, self.thickness)
            self.put_text_line(frame, fps_text, w - fps_size[0] - self.margin, self.margin + fps_size[1])

        return frame

    def record(self):
        while not self.app_state.stop_threads:
            self.app_state.video_save.write(self._putText_frame_())

        print("[record] thread exit...")
    
    def stream(self):
        fps = 1/30
        while not self.app_state.stop_threads:
            print("\033[32mRun Stream\033[0m")
            self.app_state.video_stream.write(self._putText_frame_())
            
            time.sleep(fps)
    
    def _get_last_frame_(self):
        try:
            self.frame = self.app_state.frame_queue.get_nowait()
            self.frame = cv2.resize(self.frame, (VIDEO_WIDTH, VIDEO_HEIGHT))
        except queue.Empty:
            pass  # 保持現有的 frame
        return self.frame
    
    def _putText_frame_(self):
        frame = self._get_last_frame_()
        if frame is None:
            frame = np.zeros((1280, 720, 3), dtype=np.uint8)  # 全黑的圖像
        self.last_frame = self.put_info(frame)
        return self.last_frame

    @staticmethod
    def _cale_record_fps_(app_state:AppState, model):
        global YOLO_FPS
        aver_fps = 0
        aver_fps_stat = False

        for i in range(11):
            if not aver_fps_stat:
                # 讀取影格
                ret, frame = app_state.cap.read()
                frame = cv2.resize(frame, (1280, 720))
                t1 = time.time()
                model.predict(frame, imgsz=IMGSZ, conf=CONF_THRESHOLD)
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
    
    def calculate_distance_to_center(bbox_xyxy, image_width=1280, image_height=720):
        x1, y1, x2, y2 = bbox_xyxy
        
        # 圖像中心
        image_center_x = image_width / 2  # 640
        image_center_y = image_height / 2  # 360
        
        # bbox中心
        bbox_center_x = (x1 + x2) / 2
        bbox_center_y = (y1 + y2) / 2
        
        # 計算直線距離
        distance = math.sqrt((bbox_center_x - image_center_x)**2 + 
                            (bbox_center_y - image_center_y)**2)
        
        return distance
        
    def start_recording(self):
        self.recording_thread = threading.Thread(target=self.record, daemon=True)
        self.recording_thread.start()
        print("\033[32mStart recording\033[0m")

    def stop_recording(self):
        self.app_state.stop_threads = True
        if hasattr(self, "recording_thread"):
            self.recording_thread.join()
        print("\033[31mRecording end\033[0m")

# 飛行日誌
class Log(object):
    def __init__(self, app_state:AppState, sub:MinimalSubscriber, gimbal:GimbalTimerTask):
        self.app_state = app_state
        self.data = LogData()
        self.log = app_state.log
        self.sub = sub
        self.gimbal = gimbal
    def get_data(self):
        _, sys_time = system_time()
        self.data = {
            "time": sys_time,
            "GPS/RTK" : self.sub.gps_stat,
            "latitude": self.sub.latitude,
            "longitude": self.sub.longitude,
            "altitude(m)": self.sub.altitude,
            "H": f"{self.sub.relative_altitude:.1f}",
            "detectionCount": self.app_state.total_detect_count,
            "FPS": f"{YOLO_FPS:.0f}" if YOLO_FPS is not None else "N/A",
            "gimbalYawDeg(°)": f"{ROS_Pub.pub_img['motor_yaw']:.3f}",
            "gimbalPitchDeg(°)": f"{ROS_Pub.pub_img['motor_pitch']:.3f}",
            "gimbalYawMove(°)": f"{self.gimbal.output_deg[0]:.2f}",
            "gimbalPitchMove(°)": f"{self.gimbal.output_deg[1]:.2f}",
            "gimbalCemter": f"{self.gimbal.center_status}",
            "LiDar(m)": f"{self.sub.lidar_range}",
            
            "centerDistance": self.gimbal.centerDistance,
            "Bbox_x1": ROS_Pub.pub_bbox["x0"], "Bbox_x2": ROS_Pub.pub_bbox["x1"],
            "Bbox_y1": ROS_Pub.pub_bbox["y0"], "Bbox_y2": ROS_Pub.pub_bbox["y1"],
            "distanceVisual": f"{self.gimbal.threeD_data['distance_visual']:.3f}" if self.gimbal.threeD_data['distance_visual'] is not None else "0.000",
            "distanceActual": f"{self.gimbal.threeD_data['distance_actual']:.3f}" if self.gimbal.threeD_data['distance_actual'] is not None else "0.000",
            "thetaDeg(°)": f"{self.gimbal.threeD_data['theta_deg']:.3f}" if self.gimbal.threeD_data['theta_deg'] is not None else "0.000",
            "phiDeg(°)": f"{self.gimbal.threeD_data['phi_deg']:.3f}" if self.gimbal.threeD_data['phi_deg'] is not None else "0.000",
            
            "posMode": f"{self.app_state.positioning.result[0]}" if self.app_state.positioning.result[0] is not None else "None",
            "pos_x": f"{self.app_state.positioning.result[1]}" if self.app_state.positioning.result[1] is not None else "None", 
            "pos_y": f"{self.app_state.positioning.result[2]}" if self.app_state.positioning.result[2] is not None else "None", 
            "pos_z": f"{self.app_state.positioning.result[3]}" if self.app_state.positioning.result[3] is not None else "None",
            "heightSource": "Lidar" if self.sub.lidar_range >= 10.0 else "Barometer",
            "Triangulation": f"{self._Triangulation_()}"
        }
        return self.data
        
    def write(self):
        self.log.write_row(self.get_data())
        
    def _Triangulation_(self):
        relative_altitude = self.sub.lidar_range if self.sub.lidar_range <= 10.0 else self.sub.relative_altitude
        print(f"Lidar: {self.sub.lidar_range}, Altitude Source: {'Lidar' if self.sub.lidar_range <= 10.0 else 'Barometer'}")
        return self.app_state.positioning.Triangulation(
                self.sub.latitude, self.sub.longitude, relative_altitude, 
                self.sub.motor_pitch, self.sub.motor_yaw, 
                self.sub.compass_heading)

# 清理資源與訊號處理
def cleanup_resources(app_state:AppState, sub:MinimalSubscriber, gimbal_task:GimbalTimerTask):
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
        if hasattr(sub, "destroy_node"):
            sub.destroy_node()
    except Exception as e:
        print(f"Error destroying nodes: {e}")

    # shutdown ROS
    if rclpy.ok():
        rclpy.shutdown()

    # 釋放其他非 ROS 資源
    if hasattr(app_state, "video_save") and app_state.video_save is not None:
        app_state.video_save.release()
    if hasattr(app_state, "cap") and app_state.cap.isOpened():
        app_state.cap.release()
    
    # 釋放 GStreamer stream
    if hasattr(app_state, 'out') and app_state.out is not None:
        app_state.out.release()
        print("GStreamer stream released.")

    # 關閉所有 OpenCV 視窗
    cv2.destroyAllWindows()

    print("[cleanup_resources] Done cleaning up.")

def signal_handler(sig, frame, app_state: AppState, sub: MinimalSubscriber, gimbal: GimbalTimerTask):
    print(f"\n[signal_handler] Received signal {sig}, cleaning up...")
    cleanup_resources(app_state, sub, gimbal)
    sys.exit(0)

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
        if rclpy.ok():
            rclpy.shutdown()

# 目標模型載入
pt_file = check_file(r'landpadv11.pt')
MODEL = YOLO(pt_file)

# 主要偵測函數：擔任主執行緒
def detect_loop(app_state: AppState, model: YOLO, obj_class:int, video_processor: VideoProcessor, 
                sub:MinimalSubscriber, gimbal:GimbalTimerTask):  
    global YOLO_FPS
    if SAVEIMG:
        video_processor._cale_record_fps_(app_state, model)
    not_read_count = 0
    while not app_state.stop_threads:
        detect_STAT = False
        ret, frame = app_state.cap.read()
        if not ret or frame is None:
            not_read_count += 1
            if not_read_count >= 3:
                break 
            continue
        frame = cv2.resize(frame, (1280, 720))
        # 執行 YOLO 推論
        t1 = time.time()
        results = model.predict(frame, imgsz=IMGSZ, conf=CONF_THRESHOLD, iou=0.45, 
                                nms=True, agnostic_nms=False, max_det=1, 
                                retina_masks=False
                               )
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
                ROS_Pub.pub_bbox["detect"] = ROS_Pub.pub_img["detect"] = True if gimbal.detect_countuers >= 5 else False
                # print("detect count:", gimbal.detect_countuers)
                ROS_Pub.pub_bbox["conf"] = max_conf
                ROS_Pub.pub_bbox["id"] = obj_class
                ROS_Pub.pub_bbox["name"] = model.names[obj_class]
                x0, y0, x1, y1 = map(int, max_xyxy.cpu())
                ROS_Pub.pub_bbox["x0"], ROS_Pub.pub_bbox["y0"] = x0, y0
                ROS_Pub.pub_bbox["x1"], ROS_Pub.pub_bbox["y1"] = x1, y1
                detect_STAT = True
            else:
                ROS_Pub.bbox_init()
        else:
            ROS_Pub.bbox_init()

        # 更新CSV
        if save_data:
            log.write()
        
        # 更新雲台
        gimbal.xyxy_update(detect_STAT, x0, y0, x1, y1)
        
        #計算座標
        position_results = app_state.positioning.position_calc(gimbal.center_status)
        if position_results[1] is not None:
            print("object position:", position_results)

        
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

            if SHOW or is_Stream:
                show_frame = video_processor.put_info(frame.copy())
            if is_Stream:
                # 推送到 GStreamer 管道
                frame = cv2.resize(show_frame,(VIDEO_WIDTH,VIDEO_HEIGHT))
                app_state.out.write(frame)
            if SHOW:
                cv2.imshow('YOLOv11', show_frame)
                if cv2.waitKey(1) == ord('q'):
                    app_state.stop_threads = True
                    break
                           
        # 檢測次數計數
        app_state.total_detect_count += 1
    # 離開迴圈後，清理資源
    cleanup_resources(app_state, sub, gimbal)

def gimbalTaskInit(sub:MinimalSubscriber):
    h_fov = json_config.get(["video_resolutions", "default", "Horizontal_FOV"])
    v_fov = json_config.get(["video_resolutions", "default", "Vertical_FOV"])
    return GimbalTimerTask(sub, trackMode, track_obj_size, h_fov, v_fov)

# Main
def main():
    # Global ROS Node
    global ROS_Pub
    ROS_Sub = MinimalSubscriber()
    gimbalTask = gimbalTaskInit(ROS_Sub)
    ROS_Pub = MinimalPublisher(ROS_Sub, gimbalTask)
    lidar = LidarPublisher()
    
    app_state = AppState(ROS_Sub, save_img=SAVEIMG, save_data=save_data)  # 是否要開啟錄影
    
    # 初始化 GStreamer Stream
    if is_Stream:
        pipeline = app_state._stream_GStreamer_(width=1280, height=720, fps=30, bitrate=10000000,
                                                ip=json_config.get(["img_send","Broadcast_Address"]),
                                                port=json_config.get(["img_send","port"]))
        app_state.out = cv2.VideoWriter(pipeline, cv2.CAP_GSTREAMER, 0, 15.0, (VIDEO_WIDTH, VIDEO_HEIGHT))
    
    # csv log
    global log
    if save_data:
        log = Log(app_state, ROS_Sub, gimbalTask)
    
    # 註冊訊號 (Ctrl+C / kill SIGTERM)
    signal.signal(signal.SIGINT,  lambda s,f: signal_handler(s, f, app_state, ROS_Sub, gimbalTask))
    signal.signal(signal.SIGTERM, lambda s,f: signal_handler(s, f, app_state, ROS_Sub, gimbalTask))

    # 建立 VideoProcessor
    global video_processor
    video_processor = VideoProcessor(app_state, ROS_Sub, gimbalTask, VIDEO_WIDTH, VIDEO_HEIGHT)

    # 開 ROS spin 執行緒
    app_state.ROS_spin = threading.Thread(target=_spinThread, args=(ROS_Pub, ROS_Sub, gimbalTask, lidar), daemon=True)
    app_state.ROS_spin.start()

    # 執行偵測 (主線程)
    try:
        detect_loop(app_state, MODEL, 0, video_processor, ROS_Sub, gimbalTask)
    finally:
        # 停止錄影
        if SAVEIMG:
            video_processor.stop_recording()
            print(f"media path: {app_state.name}")

if __name__ == "__main__":
    main()

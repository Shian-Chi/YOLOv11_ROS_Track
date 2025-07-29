import math, time, threading
from datetime import datetime

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from transforms3d import euler
from tutorial_interfaces.msg import Img, GimbalDegree
from std_msgs.msg import Float32, Float64, Bool, Int32, Int32MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy
from utils import CSVHandler
from parameter import Parameter

def system_time():
    # 獲取當前日期與時間
    current_time = datetime.now()
    # 格式化日期與時間（保留到小數點後三位）
    formatted_time = current_time.strftime('%Y-%m-%d %H:%M:%S') + f'.{current_time.microsecond // 1000:03d}'
    return formatted_time

def read_temp_txt():
    # 從 .temp.txt 讀取日期時間和 data 路徑
    with open("/home/ubuntu/track/track2/detect_task/.temp.txt", "r") as file:
        lines = file.readlines()
        time_str = lines[0].strip()  # 讀取時間
        data_path = lines[2].strip().split(": ")[1]  # 讀取 data 路徑
        return time_str, data_path

def parse_time(time_str):
    try:
        return datetime.strptime(time_str, "%Y-%m-%d %H:%M:%S.%f")
    except ValueError:
        return datetime.strptime(time_str, "%Y-%m-%d %H:%M:%S")

def check_time_difference(time_str):
    # 解析從 .temp.txt 讀取的時間（自動判斷有無微秒）
    saved_time = parse_time(time_str)
    current_time = datetime.now()
    # 計算時間差
    time_diff = (current_time - saved_time).total_seconds()
    # 如果時間差小於等於 3 秒，返回 True，否則返回 False
    return time_diff <= 10


class sub_Node(Node):
    def __init__(self):
        super().__init__('log_node')
        # ------ QoS ------
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.detectSub = self.create_subscription(Bool, '/target_detected', self.detect_cb, qos)
        self.centerSub = self.create_subscription(Bool, '/targetCenter', self.center_cb, qos)
        self.imuSub = self.create_subscription(Imu, 'mavros/imu/data', self.IMU_cb, qos)
        self.gimbalsub = self.create_subscription(GimbalDegree, '/gimbal_angle', self.gimbalAngle_cb, qos)
        self.yolo_fps = self.create_subscription(Int32, '/YOLO_FPS', self.yolo_fps_cb, qos)
        self.center_xyxySub = self.create_subscription(Int32MultiArray, '/target_xyxy', self.target_xyxy_cb, qos)
        self.targetPosSub = self.create_subscription(NavSatFix, '/target_position', self.target_pos_cb, qos)
        self.TriangulationPosSub = self.create_subscription(NavSatFix, '/target_position/Triangulation', self.trig_pos_cb, qos)
        self.threeDVisualSub = self.create_subscription(Float32, '/threeD_position/distanceVisual', self.visual_cb, qos)
        self.threeDActualSub = self.create_subscription(Float32, '/threeD_position/distanceActual', self.actual_cb, qos)
        self.threeDHorizontalSub = self.create_subscription(Float32, '/threeD_position/horizontalDeg', self.horizontalDeg_cb, qos)
        self.threeDVerticalSub = self.create_subscription(Float32, '/threeD_position/verticalDeg', self.verticalDeg_cb, qos)
        
        # MAVROS Topics
        self.AltitudeSub = self.create_subscription(Float64, 'mavros/global_position/rel_alt', self.Altcb, qos)
        self.GlobalPositionSub = self.create_subscription(NavSatFix, 'mavros/global_position/global', self.GPcb, qos)
        self.headingSub = self.create_subscription(Float64, 'mavros/global_position/compass_hdg', self.HDcb, qos)
        
        
        self.detected = False
        self.centered = False
        self.motor_yaw = 0.0
        self.motor_pitch = 0.0
        self.posMode = None
        
        # Triangulation position
        self.trig_latitude = 0.0
        self.trig_longitude = 0.0
        self.trig_altitude = 0.0
        
        # Two Point position
        self.target_latitude = 0.0
        self.target_longitude = 0.0
        self.target_altitude = 0.0

        # IMU data
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0

        # GPS data
        self.latitude = 0.0 
        self.longitude = 0.0
        self.gps_altitude = 0.0
        
        # Heading data
        self.heading = 0.0
        
        # Altitude data
        self.altitude = 0.0
        
        # YOLO FPS
        global YOLO_FPS
        YOLO_FPS = 0
        
        # Target bounding box coordinates
        self.target_xyxy = [None, None, None, None]
        
        # 3D Position
        self.threeD_visual = 0.0
        self.threeD_actual = 0.0
        self.threeD_horizontal = 0.0
        self.threeD_vertical =0.0
        
    def GPcb(self, msg:NavSatFix):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.gps_altitude = msg.altitude
    
    def HDcb(self, msg):
        self.heading = msg.data
    
    def Altcb(self, msg): 
        self.altitude = msg.data

    def yolo_fps_cb(self, msg: Int32):
        global YOLO_FPS
        YOLO_FPS = msg.data

    def target_xyxy_cb(self, msg: Int32MultiArray):
        self.target_xyxy = list(msg.data) # msg.data -> ('i', [0, 0, 0, 0])
    
    def gimbalAngle_cb(self, msg: GimbalDegree):
        self.motor_yaw = msg.yaw
        self.motor_pitch = msg.pitch

    def IMU_cb(self, msg: Imu):
        # ROS IMU -> NED → 歐拉角
        ned = euler.quat2euler([
            msg.orientation.w,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z])
        self.pitch = math.degrees(ned[0])
        self.roll  = math.degrees(ned[1])
        self.yaw   = math.degrees(ned[2])

    def detect_cb(self, msg: Bool):
        self.detected = msg.data

    def center_cb(self, msg: Bool):
        self.centered = msg.data

    def actual_cb(self, msg: Float32):
        self.threeD_actual = msg.data
        
    def visual_cb(self, msg: Float32):
        self.threeD_visual= msg.data
    
    def horizontalDeg_cb(self, msg: Float32):
        self.threeD_horizontal = msg.data
    
    def verticalDeg_cb(self, msg: Float32):
        self.threeD_vertical = msg.data
    
    def target_pos_cb(self, msg: NavSatFix):
        self.posMode = msg.header.frame_id
        self.target_altitude = msg.altitude
        self.target_latitude = msg.latitude
        self.target_longitude = msg.longitude
            
    def trig_pos_cb(self, msg:NavSatFix):
        self.trig_altitude = msg.altitude
        self.trig_latitude = msg.latitude
        self.trig_longitude = msg.longitude
    
class LogData:
    def __init__(self):
        self.csv_data = {
            "time": system_time(),                                                      # 系統時間
            "GPS/RTK": "None",                                                          # GPS/RTK 狀態                          
            "latitude": 0.0, "longitude": 0.0, "altitude(m)": 0.0,                      # GPS/RTK 座標
            "H": 0.0,                                                                   # 高度
            "DroneYawDeg(°)": 0.0, "DronePitchDeg(°)": 0.0, "DroneRollDeg(°)": 0.0,     # 無人機姿態
            "gimbalYawDeg(°)": 0.0, "gimbalPitchDeg(°)": 0.0,                           # 雲台姿態
            "gimbalYawMove(°)": 0.0, "gimbalPitchMove(°)": 0.0,                         # 雲台移動角度
            "global_yaw": 0.0, "global_pitch": 0.0,                                     # 全局姿態
            "gimbalCenter": False, "centerDistance": "None",                            # 雲台中心狀態與距離
            "LiDar(m)": 0.0,                                                            # LiDar 距離
            "FPS": 0.0,                                                                 # YOLO FPS                    
            "Bbox_x1": 0, "Bbox_x2": 0, "Bbox_y1": 0, "Bbox_y2": 0,                     # 目標邊界框
            "distanceVisual": 0.0, "distanceActual": 0.0,                               # 視覺距離與實際距離
            "thetaDeg(°)": 0.0, "phiDeg(°)": 0.0,                                       # 目標角度
            "posMode": "None",                                                          # 目標位置測量模式
            "2pt_pos_x": None, "2pt_pos_y": None, "2pt_pos_z": None,                    # 目標位置
            "trig_lon":0.0, "trig_lat":0.0, "trig_alt":0.0   # 三角測量
        }

class LogWrite(CSVHandler, LogData):
    def __init__(self, save_path):
        super().__init__(save_path)  
        LogData.__init__(self)
        self.logData = LogData()
        self.save_path = save_path

def wrap_angle(angle):
    return (angle + 180) % 360 - 180

para = Parameter()
def main(node: sub_Node):
    # 讀取 .temp.txt 的時間並檢查是否在 3 秒內
    time_str, data_path = read_temp_txt()
    if check_time_difference(time_str):
        log = LogWrite(data_path)

    while rclpy.ok():
        global_yaw = wrap_angle(node.yaw + node.motor_yaw)
        global_pitch = wrap_angle(node.pitch + node.motor_pitch)
        log.logData.csv_data = {
            "time": system_time(),
            "GPS/RTK": "None",
            "latitude": node.latitude, "longitude": node.longitude, "altitude(m)": node.gps_altitude,
            "H": node.altitude,
            "DroneYawDeg(°)": node.yaw, "DronePitchDeg(°)": node.pitch,
            "DroneRollDeg(°)": node.roll, "gimbalYawDeg(°)": node.motor_yaw,
            "gimbalPitchDeg(°)": node.motor_pitch,
            "global_yaw": global_yaw, "global_pitch": global_pitch,
            "gimbalCenter": node.centered, "centerDistance": 0.0,  
            "LiDar(m)": 0.0,  # Placeholder for LiDar distance
            "FPS": YOLO_FPS,  # Placeholder for FPS
            "Bbox_x1": node.target_xyxy[0], "Bbox_x2": node.target_xyxy[2], 
            "Bbox_y1": node.target_xyxy[1], "Bbox_y2": node.target_xyxy[3],
            "distanceVisual": node.threeD_visual, "distanceActual": node.threeD_actual,
            "3D_horizontalDeg(°)": node.threeD_horizontal, "3D_verticalDeg(°)": node.threeD_vertical,
            "posMode": node.posMode,
            "2pt_pos_x": node.target_longitude, "2pt_pos_y": node.target_latitude, "2pt_pos_z": node.target_altitude,
            "trig_lon": node.trig_longitude, "trig_lat": node.trig_latitude, "trig_alt": node.trig_altitude
        }
        
        log.write_row(log.logData.csv_data)
        time.sleep(0.02)

if __name__ == "__main__":
    try:
        rclpy.init()
        ros_node = sub_Node()
        ros_thread = threading.Thread(target=rclpy.spin, args=(ros_node, ), daemon=True)
        ros_thread.start()
        
        if para.data_save_enabled:
            main(ros_node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
import sys, os, math
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../')))

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import ReliabilityPolicy, QoSProfile
from sensor_msgs.msg import NavSatFix, Imu
from mavros_msgs.msg import Altitude
from transforms3d import euler
from std_msgs.msg import Float64
from tutorial_interfaces.msg import Img, Bbox

from ctrl.pid.motor import normalize_angle_180

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
        
        self.pub_img = {
            "detect": False,
            "camera_center": False,
            "motor_pitch": 0.0,
            "motor_yaw": 0.0,
            "target_latitude": 0.0,
            "target_longitude": 0.0,
            "hold_status": False,
            "send_info": False
        }

        self.pub_bbox = {
            "detect": False,
            "id": -1,
            "conf": -1.0,
            "name": "None",
            "x0": -1,
            "x1": -1,
            "y0": -1,
            "y1": -1
        }

        self.pub_motor = {
            "yaw_pluse": 0.0,
            "pitch_pluse": 0.0,
            "yaw_angle": 0.0,
            "pitch_angle": 0.0
        }
        
        
    def __initialHeight__(self):
        while self.altitude != 0.0:
            self.initialHeight = self.altitude
            break
    
    def holdcb(self, msg):
        self.hold = self.pub_img["hold_status"] = msg.hold_status

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
        
class MinimalPublisher(Node, PT_Ctrl):
    def __init__(self):
        super().__init__("minimal_publisher")
        # Img publish
        self.imgPublish = self.create_publisher(Img, "img", 1)
        img_timer_period = 1/10
        self.img_timer = self.create_timer(img_timer_period, self.img_callback)
                
        self.img = Img()
        
    def img_callback(self):
        self.pub_img['camera_center'] = PT_Ctrl.center_status
        
        yA, pA = PT_Ctrl.get_angle()
        self.pub_img['motor_pitch'] = pA + ROS_Sub.drone_pitch
        self.pub_img['motor_yaw'] = normalize_angle_180(yA) * -1
        
        (self.img.detect, 
         self.img.camera_center, 
         self.img.motor_pitch, 
         self.img.motor_yaw, 
         self.img.target_latitude, 
         self.img.target_longitude, 
         self.img.hold_status, 
         self.img.send_info) = self.pub_img.values()
        # print(f"pubData: detect:{pub_img['detect']}, center: {pub_img['camera_center']}")
        self.imgPublish.publish(self.img)
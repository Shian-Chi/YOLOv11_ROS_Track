import sys, os, math

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../")))

from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from sensor_msgs.msg import NavSatFix, Imu, Range
from mavros_msgs.msg import Altitude
from transforms3d import euler
from std_msgs.msg import Float64
from tutorial_interfaces.msg import Img, Bbox, MotorInfo, BodyInfo

from ctrl.pid.motor import normalize_angle_180


class topic_data(object):
    def __init__(self):
        self.pub_img = {
            "detect": False,
            "camera_center": False,
            "motor_pitch": 0.0,
            "motor_yaw": 0.0,
            "target_latitude": 0.0,
            "target_longitude": 0.0,
            "hold_status": False,
            "send_info": False,
        }

        self.pub_bbox = {
            "detect": False,
            "id": -1,
            "conf": -1.0,
            "name": "",
            "x0": -1,
            "x1": -1,
            "y0": -1,
            "y1": -1,
        }
        self.bbox_init()

        self.pub_motor = {
            "yaw_pluse": 0.0,
            "pitch_pluse": 0.0,
            "yaw_angle": 0.0,
            "pitch_angle": 0.0,
        }
        
        self.sub_lidar = {
            "range": 0.0
        }

    def bbox_init(self):
        """
        重置 bounding box 相關參數。
        """
        self.pub_img["detect"] = False
        self.pub_bbox["detect"] = False
        self.pub_bbox["id"] = -1
        self.pub_bbox["conf"] = 0.0
        self.pub_bbox["x0"] = self.pub_bbox["y0"] = 0
        self.pub_bbox["x1"] = self.pub_bbox["y1"] = 0
        self.pub_bbox["name"] = ""


class MinimalSubscriber(Node, topic_data):
    def __init__(self):
        Node.__init__(self, "minimal_subscriber")
        topic_data.__init__(self)
        self.GlobalPositionRTK_Sub = self.create_subscription(NavSatFix, "mavros/global_position/global", self.RTcb, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.GlobalPositionGPS_Sub = self.create_subscription(NavSatFix, "mavros/global_position/raw/fix", self.GPcb, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.AltitudeSub = self.create_subscription(Altitude, 'mavros/altitude', self.Altcb, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.imuSub = self.create_subscription(Imu, "mavros/imu/data", self.IMUcb, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.imgSub = self.create_subscription(Img, "img", self.imgcb, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.globalPosition = self.create_timer(1, self.postion)
        self.headingSub = self.create_subscription(Float64, "/mavros/global_position/compass_hdg", self.hdg_cb, 10)
        self.lidarSub = self.create_subscription(Range, "/mavros/distance_sensor/rangefinder_sub", self.lidarcb, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

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
        self.compass_heading = 0.0
        self.drone_pitch = 0.0
        self.drone_roll = 0.0
        self.drone_yaw = 0.0
        
        self.detect = False
        self.camera_center = False
        self.motor_pitch = 0.0
        self.motor_yaw = 0.0
        self.target_latitude = 0.0
        self.target_longitude = 0.0
        
        self.lidar_range = 0.0
        
    def imgcb(self, msg: Img) -> None:
        self.detect = msg.detect
        self.camera_center = msg.camera_center
        self.motor_pitch = msg.motor_pitch
        self.motor_yaw = msg.motor_yaw
        self.target_latitude = msg.target_latitude
        self.target_longitude = msg.target_longitude

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

    def hdg_cb(self, msg: Float64):
        self.compass_heading = msg.data

    def relAltcd(self, msg):
        self.relative_altitude = msg.data

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
    
    def lidarcb(self, msg: Range):
        self.lidar_range = msg.range
        print(f"LiDar: {msg.range:.2f}m")
    def get_gps_data(self):
        return self.latitude, self.longitude, self.gps_altitude


class MinimalPublisher(Node, topic_data):
    def __init__(self, sub: MinimalSubscriber, gimbalTask):
        # 手動繼承
        Node.__init__(self, "minimal_publisher")
        topic_data.__init__(self)

        self.ROS_Sub = sub
        self.PT_Ctrl = gimbalTask

        # Motor Info publish
        # self.motorInfoPublish = self.create_publisher(MotorInfo, "motor_info", 10)
        # self.motor_timer = self.create_timer(1/20, self.motor_callback)
        # self.motorInfo = MotorInfo()

        # Img publish
        self.imgPublish = self.create_publisher(Img, "img", 1)
        img_timer_period = 1 / 20
        self.img_timer = self.create_timer(img_timer_period, self.img_callback)
        self.img = Img()

    def img_callback(self):
        self.pub_img["camera_center"] = self.PT_Ctrl.center_status

        yA, pA = self.PT_Ctrl.get_angle()
        self.pub_img["motor_pitch"] = pA + self.ROS_Sub.drone_pitch
        self.pub_img["motor_yaw"] = normalize_angle_180(yA)

        (
            self.img.detect,
            self.img.camera_center,
            self.img.motor_pitch,
            self.img.motor_yaw,
            self.img.target_latitude,
            self.img.target_longitude,
            self.img.hold_status,
            self.img.send_info,
        ) = self.pub_img.values()
        # print(f"pubData: detect:{pub_img['detect']}, center: {pub_img['camera_center']}")
        self.imgPublish.publish(self.img)

    def motor_callback(self):
        # Yaw
        ye = int(self.PT_Ctrl.yaw.info.getEncoder())
        ya = float((self.PT_Ctrl.yaw.info.getAngle()))
        self.motorInfo.yaw_pluse = ye
        self.motorInfo.yaw_angle = ya
        # Pitch
        pe = int(self.PT_Ctrl.pitch.info.getEncoder())
        pa = float(self.PT_Ctrl.pitch.info.getAngle())
        self.motorInfo.pitch_pluse = pe
        self.motorInfo.pitch_angle = pa

        self.motorInfoPublish.publish(self.motorInfo)



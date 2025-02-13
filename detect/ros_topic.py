import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import NavSatFix, Imu
from rclpy.qos import ReliabilityPolicy, QoSProfile
from transforms3d import euler
from tutorial_interfaces.msg import Img, Bbox, Lidar, MotorInfo
import math, queue
from parameters_share import pub_img_data, pub_bbox_data, pub_motor_data

pub_img, pub_bbox, pub_motor  = pub_img_data, pub_bbox_data, pub_motor_data
class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__("minimal_subscriber")
        self.GlobalPositionSuub = self.create_subscription(
            NavSatFix, "mavros/global_position/global", self.GPcb, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.imuSub = self.create_subscription(Imu, "mavros/imu/data", self.IMUcb, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.holdSub = self.create_subscription(Img,"img", self.holdcb, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        # self.gimbalRemove = self.create_subscription(GimbalDegree, "gimDeg", self.gimAngDegcb, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.distance = self.create_subscription(Lidar, "lidar", self.lidarcb, 10)
        self.bboxPredcd = self.create_subscription(Bbox, 'bbox', self.bboxcb, 10)
        self.motorcb = self.create_subscription(MotorInfo, 'motor_info', self.motorInfocb, 10)

        self.hold = False
        self.latitude = 0.0
        self.longitude = 0.0
        self.gps_altitude = 0.0
        self.drone_pitch = 0.0
        self.drone_roll = 0.0
        self.drone_yaw = 0.0
        self.gimbalYaw = 0.0
        self.gimbalPitch = 0.0
        self.discm = 0.0
        self.detect = False
        self.ID = -1
        self.conf = -1
        self.x0 = 0
        self.y0 = 0
        self.x1 = 0
        self.y1 = 0

        self.gimbalYawDeg = 0.0
        self.gimbalPitchDeg = 0.0

    def gimAngDegcb(self, msg):
        self.gimbalYaw = msg.yaw
        self.gimbalPitch = msg.pitch

    def holdcb(self, msg):
        self.hold = pub_img["hold_status"] = msg.hold_status

    def GPcb(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.gps_altitude = msg.altitude

    def IMUcb(self, msg):
        ned_euler_data = euler.quat2euler([msg.orientation.w,
                                           msg.orientation.x,
                                           msg.orientation.y,
                                           msg.orientation.z])
        self.drone_pitch = math.degrees(ned_euler_data[0])
        self.drone_roll = math.degrees(ned_euler_data[1])
        self.drone_yaw = math.degrees(ned_euler_data[2])

    def lidarcb(self, msg):
        self.discm = msg.distance_cm

    def bboxcb(self, msg):
        self.detect = msg.detect
        self.ID = msg.class_id
        self.conf = msg.confidence
        self.x0 = msg.x0
        self.y0 = msg.y0
        self.x1 = msg.x1
        self.y1 = msg.y1

    def motorInfocb(self, msg):
        self.gimbalYawDeg = pub_img['motor_yaw'] = msg.yaw_angle
        self.gimbalPitchDeg = pub_img['motor_pitch'] = msg.pitch_angle

    def get_bbox(self):
        # print(f"get_bbox: {self.detect}")
        return self.x0, self.y0, self.x1, self.y1
ROS_Sub = MinimalSubscriber()

class MinimalPublish(Node):
    def __init__(self):
        super().__init__('track_topic')
        # Img publish
        self.imgPublish = self.create_publisher(Img, "img", 3)
        img_timer_period = 1/20
        self.img_timer = self.create_timer(img_timer_period, self.img_callback)

        # Bbox publish
        self.bboxPublish = self.create_publisher(Bbox, "bbox", 3)
        bbox_timer_period = 1/20
        self.img_timer = self.create_timer(bbox_timer_period, self.bbox_callback)

    def img_callback(self):
        msg = Img()
        pub_img['motor_pitch'] = pub_img['motor_pitch'] + ROS_Sub.drone_pitch
        msg.detect, msg.camera_center, msg.motor_pitch, msg.motor_yaw, \
            msg.target_latitude, msg.target_longitude, msg.hold_status, msg.send_info = pub_img.values()
        self.imgPublish.publish(msg)

    def bbox_callback(self):
        msg = Bbox()
        msg.detect = pub_bbox["detect"]
        msg.class_id = pub_bbox["class_id"]
        msg.confidence = pub_bbox["confidence"]
        msg.x0 = pub_bbox["x0"]
        msg.y0 = pub_bbox["y0"]
        msg.x1 = pub_bbox["x1"]
        msg.y1 = pub_bbox["y1"]
        self.bboxPublish.publish(msg)

        # print(pub_bbox.values())


def ROS_Topic_spin(node_queue:queue.Queue, para_queue:queue.Queue):
    node = node_queue.get()
    para_queue.get()
    
    if not rclpy.ok():
        rclpy.init(args=None)
    executor = MultiThreadedExecutor()
    for task in node:
        executor.add_node(task)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        for task in node:
            task.destroy_node()
        rclpy.shutdown()


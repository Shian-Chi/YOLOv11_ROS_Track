import math, time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from transforms3d import euler
from rclpy.qos import ReliabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Float32, Int32MultiArray

from tutorial_interfaces.msg import Img, GimbalDegree

class Pub_Node(Node):
    def __init__(self):
        super().__init__('img_node')
        self.img_pub = self.create_publisher(Img, 'img', 1)
        
         # ------ QoS ------
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.detectSub = self.create_subscription(Bool, '/target_detected', self.detect_cb, qos)
        self.centerSub = self.create_subscription(Bool, '/targetCenter', self.center_cb, qos)
        self.imuSub = self.create_subscription(Imu, 'mavros/imu/data', self.IMU_cb, qos)
        self.gimbalsub = self.create_subscription(GimbalDegree, '/gimbal_angle', self.gimbalAngle_cb, qos)
        self.estPositionSub = self.create_subscription(NavSatFix, 'mavros/global_position/global', self.estPos, qos)

        
        self.detected = False
        self.centered = False
        self.motor_yaw = 0.0
        self.motor_pitch = 0.0
        self.target_latitude = 0.0
        self.target_longitude = 0.0
        
        # IMU data
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0
        
        # Estimated Position
        self.estLat = 0.0
        self.estLon = 0.0
        self.estAlt = 0.0
        
    def gimbalAngle_cb(self, msg:GimbalDegree):
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

    def publish_img(self, detected, centered, motor_yaw, motor_pitch
                    , target_latitude, target_longitude):
        img_msg = Img()
        img_msg.detect = detected
        img_msg.camera_center = centered
        img_msg.motor_yaw = motor_yaw
        img_msg.motor_pitch = motor_pitch
        img_msg.target_latitude = target_latitude
        img_msg.target_longitude = target_longitude
        self.img_pub.publish(img_msg)
        # print(img_msg)


    def estPos(self, msg:NavSatFix):
        self.estLon = msg.longitude
        self.estLat = msg.latitude
        self.estAlt = msg.altitude
        
def wrap_angle(angle):
    return (angle + 180) % 360 - 180

def main(node:Pub_Node):        
    while rclpy.ok():
        global_yaw = wrap_angle(node.yaw + node.motor_yaw)
        global_pitch = wrap_angle(node.pitch + node.motor_pitch)
        node.publish_img(node.detected, node.centered, global_yaw, global_pitch,
                         node.target_latitude, node.target_longitude)
        rclpy.spin_once(node)
        time.sleep(0.02)

if __name__ == '__main__':
    try:
        rclpy.init()
        node = Pub_Node()
        main(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
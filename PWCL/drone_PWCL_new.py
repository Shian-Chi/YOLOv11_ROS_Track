import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float64
from geometry_msgs.msg import Point
import sys, signal
import time
import threading
from rclpy.qos import ReliabilityPolicy, QoSProfile
from mavros_msgs.msg import Altitude, State, PositionTarget, GlobalPositionTarget 
from sensor_msgs.msg import BatteryState, NavSatFix, Imu
from geometry_msgs.msg import PoseStamped, Vector3, TwistStamped, Quaternion
from geographic_msgs.msg import GeoPoseStamped
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from transforms3d import euler
import numpy as np
import cmath
# import math
import struct
from enum import Enum
import binascii

import drone_4g_v2 as drone_TCPIP
# from mavros_msgs.srv import DroneStatus, DroneMissionPath
# from mavros_msgs.msg import Img,Bbox,GimbalDegree

from tutorial_interfaces.srv import DroneStatus, DroneMissionPath
from tutorial_interfaces.msg import Img, Bbox, GimbalDegree

drone_point = [] #無人機不固定飛行點
# ipaddr = '10.147.20.230' # Grounds station vpn address
#ipaddr = '172.22.75.0' # Grounds station vpn address
ipaddr = '10.147.20.60' # Grounds station Wi-Fi address
port = 80
drone_id = bytes('\x01', 'utf-8')

speed_count = []
altitude_count = []
latitude_count = []
longitude_count = []
yaw_delta_count = []
point_count = 0

class DronePublishNode(Node):
    def __init__(self):
        super().__init__('drone_PWCL_publisher')
        self.gimbaldegreePub = self.create_publisher(GimbalDegree, 'gimbaldegree', 10)
        
    def pub_gimbal_degree(self, pitch, yaw):
        msg = GimbalDegree()
        msg.pitch = pitch
        msg.yaw = yaw
        self.gimbaldegreePub.publish(msg)
        self.get_logger().info(f'Publishing pitch :{msg.pitch}')
        self.get_logger().info(f'Publishing yaw : {msg.yaw}')

class DroneSubscribeNode(Node):
    def __init__(self):
        super().__init__('drone_PWCL_subscriber')
        # yaw,roll,pitch / angu_x,angu_y,angu_z / acc_x,acc_y,acc_z
        self.imuSub = self.create_subscription(Imu, 'mavros/imu/data', self.IMUcb, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)) 
        # heading 航向
        self.headingSub = self.create_subscription(Float64, 'mavros/global_position/compass_hdg', self.HDcb, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        # volocity 
        self.velocitySub = self.create_subscription(TwistStamped, 'mavros/local_position/velocity_body', self.VELcb, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        # gps_altitude / latitude / longitude
        self.GlobalPositionSub = self.create_subscription(NavSatFix, 'mavros/global_position/global', self.GPcb, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        
        self.InfoSub = self.create_subscription(Img, 'img', self.IMGcb, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)) 
        
        self.imgbox = self.create_subscription(Bbox, 'bbox', self.bboxcb, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)) 
        
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.angu_x = 0.0
        self.angu_y = 0.0
        self.angu_z = 0.0
        self.acc_x = 0.0
        self.acc_y = 0.0
        self.acc_z = 0.0
        self.velocity = 0.0
        self.gps_altitude = 0.0
        self.latitude = 0.0
        self.longitude = 0.0
        self.heading = 0.0 
        
        self.detect = False
        # self.first_detect = False
        # self.second_detect = False
        # self.third_detect = False
        self.camera_center = False
        self.motor_pitch = 0.0
        self.motor_yaw = 0.0
        self.target_latitude = 0.0
        self.target_longitude = 0.0
        self.hold_status = False
        self.send_info = False
        
        self.x0 = 0
        self.x1 = 0
        self.y0 = 0
        self.y1 = 0
        
    def bboxcb(self,Bbox):
        self.x0 = Bbox.x0
        self.x1 = Bbox.x1
        self.y0 = Bbox.y0
        self.y1 = Bbox.y1
        
    def IMGcb(self,Img):
        self.detect = Img.detect
        # self.first_detect = Img.first_detect
        # self.second_detect = Img.second_detect
        # self.third_detect = Img.third_detect
        self.camera_center = Img.camera_center
        self.motor_pitch = Img.motor_pitch
        self.motor_yaw = Img.motor_yaw
        self.target_latitude = Img.target_latitude
        self.target_longitude = Img.target_longitude
        self.hold_status = Img.hold_status
        self.send_info = Img.send_info
           
    def IMUcb(self, msg :Imu):
        ned_euler_data = euler.quat2euler([msg.orientation.w,
                                        msg.orientation.x,
                                        msg.orientation.y,
                                        msg.orientation.z]) 
        
        self.pitch = drone_TCPIP.radian_conv_degree(ned_euler_data[0])
        self.roll = drone_TCPIP.radian_conv_degree(ned_euler_data[1])
        self.yaw = drone_TCPIP.radian_conv_degree(ned_euler_data[2])

        #print(self.yaw)
    
        self.angu_x = msg.angular_velocity.x * 57.3
        self.angu_y = msg.angular_velocity.y * 57.3
        self.angu_z = msg.angular_velocity.z * 57.3
        
        self.acc_x = msg.linear_acceleration.x
        self.acc_y = msg.linear_acceleration.y
        self.acc_z = msg.linear_acceleration.z  
        
    
    def HDcb(self, msg):    # 航向
        self.heading = msg.data
        
    def VELcb(self, msg):   # 機體x軸的速度
        self.velocity = msg.twist.linear.x 

    def GPcb(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.gps_altitude = msg.altitude    # Positive is above the WGS 84 ellipsoid

class DroneTimerTaskNode(Node):
    def __init__(self):
        super().__init__('drone_PWCL_timer_task')
        self.sendDroneInfo_task = self.create_timer(1/20,self.sendDroneInfo_PWCL)
        self.recvission_task = self.create_timer(1/80,self.recv_task_code)
        self.seetarget_task = self.create_timer(1/30,self.see_target)
        self.sendimgbox_task = self.create_timer(1/10,self.send_img_box)
        self.sendgimbol_angle = self.create_timer(1/10,self.sendgimbol_angle)
        self.service_client = DroneServiceNode()
        self.topic_pub = DronePublishNode()
        
    def sendgimbol_angle(self):
        global droneSub
        head_code = b'\x47\x49\x4D\x42\x4F\x4C' #GIMBOL
        reserved = b'\x00\x00\x00\x00'
        end = bytes('\x45\x4E\x44', 'utf-8')
        packet = struct.pack('=6s1s1f1f4s3s',head_code,drone_id,droneSub.motor_pitch,droneSub.motor_yaw,reserved,end)
        drone_TCPIP.socket_send(packet)
        
    def send_img_box(self):
        global droneSub
        head_code = b'\x42\x4F\x58'
        end = bytes('\x00', 'utf-8')
        packet = struct.pack('=3s1i1i1i1i1s1s',head_code,droneSub.x0,droneSub.x1,droneSub.y0,droneSub.y1,drone_id,end)
        drone_TCPIP.socket_send(packet)
        
    def see_target(self):
        global droneSub
        head_code = b'\x48\x4F\x4C\x44'
        if droneSub.send_info == True:
            packet = struct.pack('=4s1f1f1d1d1d1d',head_code,droneSub.motor_pitch,droneSub.motor_yaw,droneSub.latitude,droneSub.longitude
                                            ,droneSub.target_latitude,droneSub.target_longitude)
            drone_TCPIP.socket_send(packet)
            print('-----------------Drone hover-----------------')
            
    def recvGS(self, result):   #接收地面站State
        global droneState
        status_check = b'\x53\x54\x41\x54\x55\x53\x4F\x4B'   #STATUSOK
        if result == drone_TCPIP.groundControlCommand.DRONE_TAKEOFF.value:
            droneState.droneState = drone_TCPIP.groundControlCommand.DRONE_TAKEOFF.value        
            status = drone_TCPIP.groundControlCommand.DRONE_TAKEOFF.value                #寫入takeoff指令給Service中的srv state
            self.get_logger().info('Dronestatus Request: "%s"' % status)
            print("drone takeoff...")
            
        if result == drone_TCPIP.groundControlCommand.DRONE_MISSION_START.value:
            droneState.droneState = drone_TCPIP.groundControlCommand.DRONE_MISSION_START.value  
            status = drone_TCPIP.groundControlCommand.DRONE_MISSION_START.value          #寫入mission_start指令給Service中的srv state
            self.get_logger().info('Dronestatus Request: "%s"' % status)
            print("mission start...")
            
        if result == drone_TCPIP.groundControlCommand.DRONE_RSEARCH_START.value:
            droneState.droneState = drone_TCPIP.groundControlCommand.DRONE_RSEARCH_START.value  
            status = drone_TCPIP.groundControlCommand.DRONE_RSEARCH_START.value          #寫入mission_start指令給Service中的srv state
            self.get_logger().info('Dronestatus Request: "%s"' % status)
            print("research start...")
            
        response = self.service_client.send_status_request(status)
        print('response :',response.check)
        while response.check is not True:
            response = self.service_client.send_status_request(status)
            print('response :',response.check)
            
        packet = struct.pack('=8s',status_check)
        drone_TCPIP.socket_send(packet)
        print(packet)
        
    def sendDroneInfo_PWCL(self):   #傳送無人機狀態
        # 頭碼及尾碼
        head = bytes('\x30', 'utf-8')
        end = bytes('\x40', 'utf-8')
        battery_temp = 0.0
        global droneSub
        packet = struct.pack('=1s9f1f1f1f1d1d1i1s1s',
                            head,
                            droneSub.roll,droneSub.pitch,droneSub.yaw,
                            droneSub.angu_x,droneSub.angu_y,droneSub.angu_z,
                            droneSub.acc_x,droneSub.acc_y,droneSub.acc_z,
                            droneSub.heading,
                            droneSub.velocity,
                            droneSub.gps_altitude,
                            droneSub.latitude,
                            droneSub.longitude,
                            int(battery_temp),
                            drone_id,
                            end)
        drone_TCPIP.socket_send(packet)
         
    def recv_task_code(self):
        path_ok_check = b'\x50\x41\x54\x48\x4F\x4B'#PATHOK
        data_head = None
        decode_data_head = None
        
        data_head = drone_TCPIP.socket_recv(4)
        if data_head is not None:
            decode_data_head = data_head.decode(encoding='utf-8',errors='replace')
            
        if decode_data_head == 'GMBO':
            pitch = 0.0         #1f
            yaw = 0.0      #1f
            data = drone_TCPIP.socket_recv(8)
            data_end = drone_TCPIP.socket_recv(4)
            decode_data = struct.unpack("2f", data)
            pitch = decode_data[0]
            yaw = decode_data[1]
            self.topic_pub.pub_gimbal_degree(pitch, yaw)
            if data_end == 'TUAV':
                print('recv end code: TUAV')
            print('Publish finish...')
            
            
        #控制封包頭碼皆為'CUAV' 資料皆為1byte
        if decode_data_head == 'CUAV':
            data = drone_TCPIP.socket_recv(1)
            data_end = drone_TCPIP.socket_recv(4)
            drone_status_data = data.decode(encoding='utf-8', errors='replace')
            if data_end == 'TUAV':
                print('recv end code: TUAV')
            self.recvGS(drone_status_data)
                
        if decode_data_head == 'GUAV':
            global drone_point,point_count
            data_length = drone_TCPIP.socket_recv(4) #飛行點數
            data_length_decode = struct.unpack("1i", data_length)
            point_length = data_length_decode[0]
            print('剩餘 %s 個路徑點... ' % point_length)
            speed = 0.0         #1f
            altitude = 0.0      #1f
            latitude = 0.0      #1d
            longitude = 0.0     #1d
            yaw_delta = 0.0     #1f
            
            if point_length != 0:
                data = drone_TCPIP.socket_recv(28) 
                decode_data = struct.unpack("2f2d1f", data) #decode_data = struct.unpack("2f2d1f", data)
                
                speed = decode_data[0]
                altitude = decode_data[1]
                latitude = decode_data[2]
                longitude = decode_data[3]
                yaw_delta = decode_data[4]
                
                speed_count.append(speed)
                altitude_count.append(altitude)
                latitude_count.append(latitude)
                longitude_count.append(longitude)
                yaw_delta_count.append(yaw_delta)
                
                point_length = point_length - 1
                point_count = point_count + 1 # 紀錄傳送點數
                
                if point_length == 0:
                    point_count_str = str(point_count)
                    self.get_logger().info('ServiceClient_point_count: "%s"' % point_count_str)
                    self.get_logger().info('ServiceClient_speed: "%s"' % speed_count)
                    self.get_logger().info('ServiceClient_altitude: "%s"' % altitude_count)
                    self.get_logger().info('ServiceClient_latitude: "%s"' % latitude_count)
                    self.get_logger().info('ServiceClient_longitude: "%s"' % longitude_count)
                    self.get_logger().info('ServiceClient_yaw_delta: "%s"' % yaw_delta_count)
                    response = self.service_client.send_missionpath_request(speed_count, altitude_count, latitude_count, 
                                                                            longitude_count, yaw_delta_count, point_count_str)
                    print('response',response.path_check)
                    while response.path_check is not True:
                        response = self.service_client.send_status_request(speed_count, altitude_count, latitude_count, 
                                                                            longitude_count, yaw_delta_count, point_count_str)
                        print('response :',response.path_check)
                    print('point_count = ',point_count)
                    packet = struct.pack('=6s1i',path_ok_check,point_count)
                    drone_TCPIP.socket_send(packet)
                    print(packet)
                    print('PATHOK')
                    
class DroneServiceNode(Node):
    def __init__(self):
        super().__init__('drone_TCPIP_client')
        self._status = self.create_client(DroneStatus, 'drone_status')
        self._missionpath = self.create_client(DroneMissionPath, 'drone_mission_path')
        while not self._status.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service _status not available, waiting again...')
        print('_status service OK')
        while not self._missionpath.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service _missionpath not available, waiting again...')
        print('_missionpath service OK')
        self.status_req = DroneStatus.Request()
        self.path_req = DroneMissionPath.Request()
        
    def send_status_request(self, status):
        self.status_req.status = status
        self.future_status = self._status.call_async(self.status_req)
        rclpy.spin_until_future_complete(self, self.future_status)
        return self.future_status.result()
    
    def send_missionpath_request(self, speed, altitude, latitude, longitude, yaw_delta, point_count):
        self.path_req.speed = speed
        self.path_req.altitude = altitude
        self.path_req.latitude = latitude
        self.path_req.longitude = longitude
        self.path_req.yaw_delta = yaw_delta
        self.path_req.point_count = point_count
        self.future_path = self._missionpath.call_async(self.path_req)
        rclpy.spin_until_future_complete(self, self.future_path)
        return self.future_path.result()
        
def signal_handler(signal, frame):
    print("\nprogram exiting gracefully")
    rclpy.shutdown()
    sys.exit(0)      
    
def pub_sub_init(args=None):
    global droneSub,droneTimer
    global droneState,ipaddr,port
    
    rclpy.init(args=args)
    signal.signal(signal.SIGINT, signal_handler)
    
    droneSub = DroneSubscribeNode()
    droneTimer = DroneTimerTaskNode()
    droneState = drone_TCPIP.drone_state()
    
    drone_TCPIP.socket_init(ipaddr,port)
    
    executor = MultiThreadedExecutor()
    executor.add_node(droneSub)
    executor.add_node(droneTimer)
    executor.spin()
    
    droneSub.destroy_node()
    droneTimer.destroy_node()
    rclpy.shutdown()

def main():
    pub_sub_init()

if __name__ == '__main__':
    main()

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
#from mavros_msgs.srv import DroneStatus, DroneMissionPath
from enum import Enum
import numpy as np
import cmath
import math
import struct
from tutorial_interfaces.srv import DroneStatus, DroneMissionPath
from tutorial_interfaces.msg import Img
# from mavros_msgs.srv import DroneStatus, DroneMissionPath 
# from mavros_msgs.msg import Img
drone_point = []
temp_detect_status = False

class groundControlCommand(Enum):
    DRONE_IDLE = '0'
    DRONE_TAKEOFF = '1'
    DRONE_MISSION_START = '2'
    DRONE_RSEARCH_START = '3'

class DroneSubscribeNode(Node):
    def __init__(self):
        super().__init__('drone_subscriber')
        self.AltitudeSub = self.create_subscription(Altitude, 'mavros/altitude', self.Altcb, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.LocalPositionSub = self.create_subscription(PoseStamped, 'mavros/local_position/pose', self.LPcb, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.velocitySub = self.create_subscription(TwistStamped, 'mavros/local_position/velocity_body', self.VELcb, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.GlobalPositionSuub = self.create_subscription(NavSatFix, 'mavros/global_position/global', self.GPcb, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.imuSub = self.create_subscription(Imu, 'mavros/imu/data', self.IMUcb, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.headingSub = self.create_subscription(Float64, 'mavros/global_position/compass_hdg', self.HDcb, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.StateSub = self.create_subscription(State, 'mavros/state', self.Statecb, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        #self.YoloImgSub = self.create_subscription(Img, 'img',  self.YOLOIMGcb, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.InfoSub = self.create_subscription(Img, 'img', self.IMGcb, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.altitude = 0.0
        self.local_x = 0.0
        self.local_y = 0.0
        self.local_z = 0.0
        self.latitude = 0.0
        self.longitude = 0.0
        self.gps_altitude = 0.0
        self.roll = 0.0
        self.pithch = 0.0
        self.yaw = 0.0
        self.angu_x = 0.0
        self.angu_y = 0.0
        self.angu_z = 0.0
        self.acc_x = 0.0
        self.acc_y = 0.0
        self.acc_z = 0.0
        self.velocity = 0.0
        self.heading = 0.0
        self.state = State()

        #prevent unused variable warning
        self.AltitudeSub

        self.detect = False
        self.camera_center = False
        self.motor_pitch = 0.0
        self.motor_yaw = 0.0
        self.target_latitude = 0.0
        self.target_longitude = 0.0
        self.hold_status = False
    
    def Statecb(self, msg :State):
        self.state = msg
    
    def Altcb(self, msg): 
        #self.get_logger().info('I heard altitude: "%s"' % msg.relative)
        self.altitude = msg.relative

    def LPcb(self, msg :NavSatFix ):
        #self.get_logger().info('I heard local_x: "{}", local_y: "{}"'.format(msg.pose.position.x, msg.pose.position.y))
        self.local_x = msg.pose.position.x
        self.local_y = msg.pose.position.y
        self.local_z = msg.pose.position.z

    def GPcb(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.gps_altitude = msg.altitude

    def IMUcb(self, msg :Imu):
        ned_euler_data = euler.quat2euler([msg.orientation.w,
                                        msg.orientation.x,
                                        msg.orientation.y,
                                        msg.orientation.z])
        self.pithch = radian_conv_degree(ned_euler_data[0])
        self.roll = radian_conv_degree(ned_euler_data[1])
        self.yaw = radian_conv_degree(ned_euler_data[2])

        #print(self.yaw)
    
        self.angu_x = msg.angular_velocity.x * 57.3
        self.angu_y = msg.angular_velocity.y * 57.3
        self.angu_z = msg.angular_velocity.z * 57.3
        
        self.acc_x = msg.linear_acceleration.x
        self.acc_y = msg.linear_acceleration.y
        self.acc_z = msg.linear_acceleration.z
    
    def VELcb(self, msg):
        self.velocity = msg.twist.linear.x 
    
    def HDcb(self, msg):
        self.heading = msg.data

    # def YOLOIMGcb(self, msg):
    #     self.yolo_detect_status = msg.detect_status
    #     self.camera_center = msg.target_center_status
        
    def get_altitude(self):
        return self.altitude
    def get_local_x(self):
        return self.local_x
    def get_local_y(self):
        return self.local_y
    def get_local_z(self):
        return self.local_z

    def IMGcb(self,Img):
        self.detect = Img.detect
        self.camera_center = Img.camera_center
        self.motor_pitch = Img.motor_pitch
        self.motor_yaw = Img.motor_yaw
        self.target_latitude = Img.target_latitude
        self.target_longitude = Img.target_longitude
        self.hold_status = Img.hold_status

class DronePublishNode(Node):
    def __init__(self, freqPositionLocal):
        super().__init__('drone_publisher')
        self.setPointPositionLocal_pub = self.create_publisher(PoseStamped, 'mavros/setpoint_position/local', 10)
        self.setPointPositionGlobal_pub = self.create_publisher(GlobalPositionTarget, 'mavros/setpoint_raw/global', 10)
        self.setCameraDetection_pub = self.create_publisher(Img, 'img', 1)
        self.setPointPositionLocal_timer = self.create_timer(1/freqPositionLocal, self.setPointPositionLocal_callback)
        self.setPointPositionGlobal_timer = self.create_timer(1/50, self.setPointPositionGlobal_callback)
        self.setCameraDetection_timer = self.create_timer(1/20, self.setCameraDetection_callback)

        # self.setRecvGC_timer = self.create_timer(1/10, self.recvGC)
        self.alwaysSend = False
        self.alwaysSendPosLocal = PoseStamped()
        self.alwaysSendGlobal = False
        self.alwaysSendPosGlobal = GlobalPositionTarget()
        self.vision_status = Img()
        self.hold_status = False

    def sendPositionLocal(self, data : PoseStamped): #用來做為單獨發送
        self.setPointPositionLocal_pub.publish(data)

    def setPointPositionLocal_callback(self): #用來作為連續發送
        if self.alwaysSend == True:
            self.setPointPositionLocal_pub.publish(self.alwaysSendPosLocal)

    def sendPositionGlobal(self, data : PoseStamped): #用來單獨發送
        self.setPointPositionGlobal_pub.publish(data)

    def setPointPositionGlobal_callback(self): #用來作為連續發送
        if self.alwaysSendGlobal == True:
            self.setPointPositionGlobal_pub.publish(self.alwaysSendPosGlobal)

    def setCameraDetection_callback(self):
        #print(self.vision_status)
        self.vision_status.hold_status = self.hold_status
        #self.setCameraDetection_pub.publish(self.vision_status)

    # def recvGC(self):
    #     global droneState
    #     result = drone_4g.drone_ground_control()
    #     if result == drone_4g.groundControlCommand.DRONE_TAKEOFF.value:
    #         droneState.droneState = drone_4g.groundControlCommand.DRONE_TAKEOFF.value
    #     if result == drone_4g.groundControlCommand.DRONE_MISSION_START.value:
    #         droneState.droneState = drone_4g.groundControlCommand.DRONE_MISSION_START.value
    
class DroneClientNode(Node):
    def __init__(self):
        super().__init__('drone_mavros_service')
        self._arming = self.create_client(CommandBool, 'mavros/cmd/arming')
        self._takeoff = self.create_client(CommandTOL, 'mavros/cmd/takeoff')
        self.land = self.create_client(CommandTOL, 'mavros/cmd/land')
        self._setMode = self.create_client(SetMode, 'mavros/set_mode')
        # self._hold_mode = self.create_client(ReturnAndDetectInformation, 'return_and_detect_information')

        while not self._arming.wait_for_service(timeout_sec=1.0):
            time.sleep(1)
        print("arming  client OK")
        while not self._takeoff.wait_for_service(timeout_sec=1.0):
            time.sleep(1)
        print("takeoff client OK")
        while not self.land.wait_for_service(timeout_sec=1.0):
            time.sleep(1)
        print("land client OK")
        while not self._setMode.wait_for_service(timeout_sec=1.0):
            time.sleep(1)
        print("setMode client OK")
        '''while not self._hold_mode.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service:hold_mode not available, waiting again...')
        print("_hold_mode client OK")'''

        '''all_clients = [self._arming, self._takeoff, self.land, self._setMode]
        
        while not all([c.wait_for_service(timeout_sec=1.0) for c in all_clients]):
            self.get_logger().info('service not available, waiting again...')
        time.sleep(1)'''
        
    def requestCmdArm(self): #無人機解鎖 
        CmdArm = CommandBool.Request()
        CmdArm.value = True
        self.future = self._arming.call_async(CmdArm)
        rclpy.spin_until_future_complete(self, self.future, timeout_sec = 3.0)
        print("Drone Arming Now")
        print(self.future.result())
        return self.future.result()
    
    def requestSetMode(self, value):
        setModeCmdReq = SetMode.Request()
        setModeCmdReq.custom_mode = value
        self.future = self._setMode.call_async(setModeCmdReq)
        rclpy.spin_until_future_complete(self, self.future, timeout_sec=1.0)
        print("OFFBOARD mode now")
        return self.future.result()
    
    def requestLand(self): #無人機降落
        setModeCmdReq = SetMode.Request()
        setModeCmdReq.custom_mode = "AUTO.LAND"
        self.future = self._setMode.call_async(setModeCmdReq)
        rclpy.spin_until_future_complete(self, self.future, timeout_sec=1.0)
        print("landing------")
        return self.future.result()

    def requestRTL(self): #無人機降落
        setModeCmdReq = SetMode.Request()
        setModeCmdReq.custom_mode = "AUTO.RTL"
        self.future = self._setMode.call_async(setModeCmdReq)
        rclpy.spin_until_future_complete(self, self.future, timeout_sec=1.0)
        print("RTL------")
        return self.future.result()

    def requestLoiter(self):
        setModeCmdReq = SetMode.Request()
        setModeCmdReq.custom_mode = "AUTO.LOITER"
        self.future = self._setMode.call_async(setModeCmdReq)
        rclpy.spin_until_future_complete(self, self.future, timeout_sec=1.0)
        print("LOITER------")
        return self.future.result()
    
    def requestDectectHold(self):
        Hold = ReturnAndDetectInformation.Request()
        Hold.hold_flag = True
        self.future = self._hold_mode.call_async(Hold)
        rclpy.spin_until_future_complete(self, self.future, timeout_sec=3.0)
        print("Requst Hold------")
        return self.future.result()
        
class DroneServiceNode(Node):
    def __init__(self):
        super().__init__('drone_service')
        self.srv_missionpath = self.create_service(DroneMissionPath, 'drone_mission_path', self.drone_path_recv)
        self.srv_status = self.create_service(DroneStatus, 'drone_status', self.drone_command)
        #self.drone_path = None
        #self.dronr_State = '0'
    def drone_path_recv(self, request, response):
        global drone_point
        drone_point.clear()

        print('len(request.point_count)',len(request.point_count))
        for i in range(int(request.point_count)):
            print('point:',i,'---------------------------')
            print('latitude:', request.latitude[i])
            print('longitude:', request.longitude[i])
            print('altitude:', request.altitude[i])
            print('speed:', request.speed[i])
            print('yaw_delta:', request.yaw_delta[i])
        
            drone_point.append([request.speed[i], request.altitude[i], request.latitude[i], request.longitude[i], request.yaw_delta[i]])
            print('len(drone_point)', len(drone_point))

        response.path_check = True
        print(request)
        print('response:',response)

        return response

    def drone_command(self, request, response):
        global droneState
        droneState.droneState = int(request.status)

        response.check = True
        print(request)
        print('response:',response)
        
        return response

class drone_state():
    def __init__(self):
        self.droneState = groundControlCommand.DRONE_IDLE.value

def takeoff(altitude, pub : DronePublishNode, sub : DroneSubscribeNode ,srv : DroneClientNode): #無人機起飛
    print("drone takeoff")
    data = PoseStamped()
    
    data.pose.position.x = 0.0
    data.pose.position.y = 0.0
    data.pose.position.z = altitude
    pub.sendPositionLocal(data)
    srv.requestSetMode("OFFBOARD")
    pub.alwaysSendPosLocal = data
    pub.alwaysSend = True

    while ((sub.get_altitude()) <= (altitude- 0.1)):
        time.sleep(0.5)
    print('takeoff complete')

def fly_to_global(pub : DronePublishNode, sub : DroneSubscribeNode, cli : DroneClientNode,latitude, longitude, altitude, delta_yaw, origin_lat, origin_lon):
    print('fly to latitude', latitude," longitude:", longitude, " delta_yaw:", delta_yaw)

    pub.alwaysSendPosGlobal.latitude = latitude
    pub.alwaysSendPosGlobal.longitude = longitude
    pub.alwaysSendPosGlobal.altitude = altitude
    #pub.alwaysSendPosGlobal.yaw_rate = 24.3

    m_convert_lng = 1/101775.45 #台灣經度1度約為101775.45m
    m_convert_lat = 1/110936.2 #緯度1度約為110936.2m
    
    temp_status = 0
    
    if latitude == 0.0:
        latitude = sub.latitude
    if longitude == 0.0:
        longitude = sub.longitude

    print('296--------',sub.detect)
    while (((abs(sub.latitude - latitude)*110936.32 > 3) or (abs(sub.longitude - longitude)*101775.45 > 3)) and (sub.detect == False)):
        time.sleep(0.1)
    
    if (latitude != 0.0) and (longitude != 0.0):
        while (((abs(sub.latitude - latitude)*110936.2 > 1.5) or (abs(sub.longitude - longitude)*101775.45 > 1.5)) and (sub.detect == False)):  #or (abs(sub.altitude - altitude) > 1.5)
            print("lat distance:", abs(sub.latitude - latitude)*110936.2, "lng distance:", abs(sub.longitude - longitude)*101775.45, "alt distance:", abs(sub.altitude - altitude))
            time.sleep(0.1)
    
    if ((delta_yaw != 0)) :
        print("delta_yaw", delta_yaw)

        if (delta_yaw <= 90.0 and delta_yaw > 0) or (delta_yaw >= -90.0 and delta_yaw <= 0):    #旋轉角度小於等於90度的作法
            target_yaw = sub.yaw - delta_yaw #z軸朝上 為了正角度為順時針轉取負號
            print(sub.yaw , delta_yaw, target_yaw)
            if target_yaw > 180 :
                target_yaw = target_yaw - 360
            if target_yaw < -180:
                target_yaw = target_yaw + 360

            print('target yaw:', target_yaw)
            pub.alwaysSendPosGlobal.yaw = degree_conv_radian(sub.yaw - delta_yaw)
            while ((abs(sub.yaw - target_yaw) > 1) and (sub.detect == False)):
                print("<90 target yaw rotation")
                time.sleep(0.1)
            print(sub.yaw)

        if delta_yaw > 90.0 or delta_yaw < -90.0: #旋轉角度大於90度的作法 
            origin_yaw = sub.yaw
            if delta_yaw > 0:
                delta90 = 90
            if delta_yaw < 0:
                delta90 = -90
            rotation90_num = int(delta_yaw/90)
            pos_neg_rotation90_num = rotation90_num
            if rotation90_num < 0:#例如轉-360度取整數後仍為負的會進不了迴圈
                rotation90_num = -rotation90_num
            print("rotation90_num", rotation90_num)

            for i in range(rotation90_num):
                if i == 0:
                    target_yaw = sub.yaw - delta90 #z軸朝上 為了正角度為順時針轉取負號
                if i != 0:
                    target_yaw = target_yaw - delta90 #z軸朝上 為了正角度為順時針轉取負號

                print("ENU_YAW:", sub.yaw , "DELTA_YAW:", delta_yaw, "TARGET:", target_yaw)
                if target_yaw > 180 :
                    target_yaw = target_yaw - 360
                if target_yaw < -180:
                    target_yaw = target_yaw + 360
                print('....')

                pub.alwaysSendPosGlobal.yaw = degree_conv_radian(sub.yaw - delta90)
                while ((abs(sub.yaw - target_yaw) > 0.1) and (sub.detect != True)):
                    time.sleep(0.1)
                print(sub.yaw)
            
            final_delta_yaw = delta_yaw - (90 * pos_neg_rotation90_num)
            if final_delta_yaw != 0:
                print("final_delta_yaw:", final_delta_yaw)
                pub.alwaysSendPosGlobal.yaw = degree_conv_radian(sub.yaw - final_delta_yaw)
                while ((sub.yaw > sub.yaw + 0.1 and sub.yaw < sub.yaw - 0.1) and (sub.detect != True)):
                    time.sleep(0.1)
                    
    if sub.detect == True: #辨識到目標
        print('---------------------------------------sub.yolo_detect_status == Tru')
        print('111111111111111111111111111111111111111111111111111111111111111111111')
        temp_lat = sub.latitude
        temp_lon = sub.longitude
        pub.alwaysSendPosGlobal.latitude = sub.latitude
        pub.alwaysSendPosGlobal.longitude = sub.longitude
        '''while ((abs(temp_lat - latitude)*110936.32 > 3) or (abs(temp_lon- longitude)*101775.45 > 3)):
            print('+++++++++++++++++++++++++++++')
            time.sleep(0.1)
        '''
        time.sleep(3)
        print('22222222222222222222222222222222')

        while sub.camera_center != True:
            print("waiting for target center")
            time.sleep(2)

        print('*'*10)
        print('*'*10)
        print("target is center now!")
        print('*'*10)
        print('*'*10)

        #cli.requestDectectHold()
        
        '''while sub.camera_center == True:
            time.sleep(0.1)
        time.sleep(5)'''
        '''temp_lat = sub.latitude
        temp_lon = sub.longitude
        pub.alwaysSendPosGlobal.latitude = sub.latitude
        pub.alwaysSendPosGlobal.longitude = sub.longitude'''
        '''time.sleep(5)
        pub.alwaysSendPosGlobal.latitude = origin_lat
        pub.alwaysSendPosGlobal.longitude = origin_lon'''
        '''while ((abs(temp_lat - latitude)*110936.32 > 3) or (abs(temp_lon- longitude)*101775.45 > 3)):
            time.sleep(0.1)'''
        '''global temp_detect_status
        temp_detect_status = True'''
        return True #this return can not correctly return the value of True
    
    time.sleep(1)
    print('reach the destination')
    return False

def fly_to_global_without_detect(pub : DronePublishNode, sub : DroneSubscribeNode, latitude, longitude, altitude, delta_yaw):
    print('fly to latitude', latitude," longitude:", longitude, " delta_yaw:", delta_yaw)

    pub.alwaysSendPosGlobal.latitude = latitude
    pub.alwaysSendPosGlobal.longitude = longitude
    pub.alwaysSendPosGlobal.altitude = altitude
    #pub.alwaysSendPosGlobal.yaw_rate = 24.3

    m_convert_lng = 1/101775.45 #台灣經度1度約為101775.45m
    m_convert_lat = 1/110936.2 #緯度1度約為110936.2m
    
    temp_status = 0
    
    if latitude == 0.0:
        latitude = sub.latitude
    if longitude == 0.0:
        longitude = sub.longitude

    while (((abs(sub.latitude - latitude)*110936.32 > 3) or (abs(sub.longitude - longitude)*101775.45 > 3))):
        time.sleep(0.1)
    
    if (latitude != 0.0) and (longitude != 0.0):
        while (((abs(sub.latitude - latitude)*110936.2 > 1.5) or (abs(sub.longitude - longitude)*101775.45 > 1.5))):  #or (abs(sub.altitude - altitude) > 1.5)
            print("+++++++lat distance:", abs(sub.latitude - latitude)*110936.2, "lng distance:", abs(sub.longitude - longitude)*101775.45, "alt distance:", abs(sub.altitude - altitude))
            time.sleep(0.1)

    if ((delta_yaw != 0)) :
        print("delta_yaw", delta_yaw)

        if (delta_yaw <= 90.0 and delta_yaw > 0) or (delta_yaw >= -90.0 and delta_yaw <= 0):    #旋轉角度小於等於90度的作法
            target_yaw = sub.yaw - delta_yaw #z軸朝上 為了正角度為順時針轉取負號
            print(sub.yaw , delta_yaw, target_yaw)
            if target_yaw > 180 :
                target_yaw = target_yaw - 360
            if target_yaw < -180:
                target_yaw = target_yaw + 360

            print('target yaw:', target_yaw)
            pub.alwaysSendPosGlobal.yaw = degree_conv_radian(sub.yaw - delta_yaw)
            while ((abs(sub.yaw - target_yaw) > 1) and (sub.detect == False)):
                print("<90 target yaw rotation")
                time.sleep(0.1)
            print(sub.yaw)

        if delta_yaw > 90.0 or delta_yaw < -90.0: #旋轉角度大於90度的作法 
            origin_yaw = sub.yaw
            if delta_yaw > 0:
                delta90 = 90
            if delta_yaw < 0:
                delta90 = -90
            rotation90_num = int(delta_yaw/90)
            pos_neg_rotation90_num = rotation90_num
            if rotation90_num < 0:#例如轉-360度取整數後仍為負的會進不了迴圈
                rotation90_num = -rotation90_num
            print("rotation90_num", rotation90_num)

            for i in range(rotation90_num):
                if i == 0:
                    target_yaw = sub.yaw - delta90 #z軸朝上 為了正角度為順時針轉取負號
                if i != 0:
                    target_yaw = target_yaw - delta90 #z軸朝上 為了正角度為順時針轉取負號

                print("ENU_YAW:", sub.yaw , "DELTA_YAW:", delta_yaw, "TARGET:", target_yaw)
                if target_yaw > 180 :
                    target_yaw = target_yaw - 360
                if target_yaw < -180:
                    target_yaw = target_yaw + 360
                print('....')

                pub.alwaysSendPosGlobal.yaw = degree_conv_radian(sub.yaw - delta90)
                while ((abs(sub.yaw - target_yaw) > 0.1) and (sub.detect != True)):
                    time.sleep(0.1)
                print(sub.yaw)
            
            final_delta_yaw = delta_yaw - (90 * pos_neg_rotation90_num)
            if final_delta_yaw != 0:
                print("final_delta_yaw:", final_delta_yaw)
                pub.alwaysSendPosGlobal.yaw = degree_conv_radian(sub.yaw - final_delta_yaw)
                while ((sub.yaw > sub.yaw + 0.1 and sub.yaw < sub.yaw - 0.1) and (sub.detect != True)):
                    time.sleep(0.1)
    
    time.sleep(1)
    print('reach the destination')
    
def radian_conv_degree(Radian) : 
    return ((Radian / math.pi) * 180)

def degree_conv_radian(Degree) : 
    return ((Degree / 180) * math.pi)

def _droneSpinThread(pub, sub, cli, srv):
    #print("_droneSpinThread start")

    executor = MultiThreadedExecutor()
    executor.add_node(sub)
    executor.add_node(pub)
    executor.add_node(srv)
    executor.spin()

def rotation_angle(current_heading, target_heading):
    """
    判斷當前航向和目標航向之間的旋轉角度

    參數：
    current_heading：當前航向，以度為單位
    target_heading：目標航向，以度為單位

    返回值：
    旋轉角度，以度為單位
    """
    diff = (target_heading - current_heading) % 360

    if diff <= 180:
        return float(diff)
    else:
        return float(diff - 360)

def check_arm(sub : DroneSubscribeNode, srv : DroneClientNode):
    if sub.state.armed == False:
        i = 0
        while not srv.requestCmdArm():
            i+=1
            print('wating arm')
            time.sleep(0.5)
            if i == 10:
                return False
        print("check arm done")
        return True

def takeoff_global(pub : DronePublishNode, sub : DroneSubscribeNode, srv : DroneClientNode, rel_alt : float):
    data = GlobalPositionTarget()
    data.coordinate_frame = 6 #FRAME_GLOBAL_REL_ALT 
    data.type_mask = 0
    data.velocity.x = 0.25
    data.velocity.y = 0.25
    current_latitude = sub.latitude
    current_longitude = sub.longitude
    data.latitude= current_latitude
    print(current_latitude)
    data.longitude = current_longitude
    print(current_longitude)
    data.altitude = rel_alt
    data.yaw = degree_conv_radian(sub.yaw)
    pub.alwaysSendPosGlobal = data
    pub.alwaysSendGlobal = True

    result = check_arm(sub, srv)
    if result == False:
        return False
    
    srv.requestSetMode("OFFBOARD")

    '''while ((sub.get_altitude()) <= (rel_alt- 1)):
    print(sub.get_altitude())
        time.sleep(1/50)
    print('takeoff complete')'''

def calculate_bearing(lat1, lon1, lat2, lon2):
    '''R = 6371  # 地球半徑，單位為公里
    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)
    delta_lat = math.radians(lat2 - lat1)
    delta_lon = math.radians(lon2 - lon1)

    y = math.sin(delta_lon) * math.cos(lat2_rad)
    x = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(delta_lon)

    bearing_rad = math.atan2(y, x)
    bearing_deg = math.degrees(bearing_rad)'''
    #台灣經度1度約為101775.45m
    #緯度1度約為110936.2m
    lat_error_m = (lat2 - lat1)*110936.2 #y
    lng_error_m = (lon2 - lon1)*101775.45 #x

    bearing_deg = math.atan2(lat_error_m, lng_error_m)
    bearing_deg = math.degrees(bearing_deg)

    # 將方位角轉換為正負180度
    if bearing_deg > 180:
        bearing_deg -= 360
    elif bearing_deg < -180:
        bearing_deg += 360

    return bearing_deg*(-1)

def drone_moving_along_the_x(pub : DronePublishNode, sub : DroneSubscribeNode, origin_heading):

    current_lat = sub.latitude #y
    current_lon = sub.longitude #x

    safty_distance = 10.0 #設置安全距離 以免因為PITCH一直沒辦法收斂導致無人機一直向前飛

    theta = sub.heading-origin_heading

    delta_y = -0.1 * math.sin(math.radians(theta))
    delta_x = 0.1 * math.cos(math.radians(theta))

    delta_lat = delta_y*(1/101775.45)
    delta_lon = delta_x*(1/110936.32)

    while (90.0 - sub.motor_pitch >= 5.0):
        print("moving x: %fm, y: %fm", delta_x, delta_y)
        target_lat= sub.latitude + delta_lat
        target_lon = sub.longitude + delta_lon

        safty_distance = safty_distance - abs((delta_y**2)+(delta_x**2))
        '''if safty_distance <= 0.0:
            print("Over safety distance")
            return '''

        pub.alwaysSendPosGlobal.latitude = target_lat
        pub.alwaysSendPosGlobal.longitude = target_lon

        while ((abs(sub.latitude - target_lat)*110936.32 > 3) or (abs(sub.longitude - target_lon)*101775.45 > 3)):
            time.sleep(0.1)
        print("pitch error: %f degrees",  90.0 - sub.motor_pitch)
    
    print("drone forward finished")


def signal_handler(signal, frame):
    print("\nprogram exiting gracefully")
    rclpy.shutdown()
    sys.exit(0)

if __name__ == '__main__':

    #system interrrupt (ctrl + c)
    signal.signal(signal.SIGINT, signal_handler)

    global droneSub
    global dronePub
    global droneCli
    global droneSrv
    global droneState
    #global posLocal

    freq = 50 #publish發佈頻率
    takeoffAltitude = 20.0 #無人機起飛高度

    rclpy.init()

    dronePub = DronePublishNode(50)
    droneSub = DroneSubscribeNode()
    droneCli = DroneClientNode()
    droneSrv = DroneServiceNode()
    droneState = drone_state()

    droneSpinThread = threading.Thread(target=_droneSpinThread, args=(dronePub, droneSub, droneCli, droneSrv))
    droneSpinThread.start()
    #rclpy.spin(droneSrv)
    time.sleep(3)
    
    while droneSub.latitude == 0 and droneSub.latitude == 0:
        time.sleep(0.1)

    print("_droneSpinThread start")
    droneCli.requestLoiter()

    while True:
        if droneState.droneState == 1: #groundControlCommand.DRONE_TAKEOFF.value:
            droneState.droneState = groundControlCommand.DRONE_IDLE.value
            
            origin_latitude = droneSub.latitude
            origin_longitude = droneSub.longitude
            takeoff_global(dronePub, droneSub, droneCli, takeoffAltitude)
            temp_status = False
            
            origin_latitude = droneSub.latitude
            origin_longitude = droneSub.longitude
            origin_heading = droneSub.heading
            
            while True:
                if droneState.droneState == 2:
                    print("mission start---------")


                    #開始旋轉直到辨識到目標(旋轉一圈沒辨識到目標辨識失敗)
                    result = fly_to_global(dronePub, droneSub, droneCli,origin_latitude, origin_longitude, takeoffAltitude, 360.0, origin_latitude, origin_longitude)

                    #如果成功辨識到目標，機頭開使與雲台鏡頭方向對齊
                    if result == True:
                        temp_yaw = droneSub.motor_yaw
                        fly_to_global_without_detect(dronePub, droneSub, origin_latitude, origin_longitude, 20.0, temp_yaw)
                    
                        #向前飛，使得雲台的Pitch垂直於目標
                        drone_moving_along_the_x(dronePub, droneSub, origin_heading)

                        #降落
                        droneCli.requestLand()


                    
                    '''print('len',len(drone_point))

                    point_len = len(drone_point)
                    lon1 = math.radians(drone_point[0][3]) #起始經度
                    lat1 = math.radians(drone_point[0][2]) #起始緯度

                    lon2 = math.radians(drone_point[1][3]) #第二個點經度
                    lat2 = math.radians(drone_point[1][2]) #第二個點緯度

                    print("轉向當前前進方位角")
                    bearing = calculate_bearing(lat1, lon1, lat2, lon2)
                    print(bearing)
                    fly_to_global(dronePub, droneSub, droneCli,origin_latitude, origin_longitude, takeoffAltitude, bearing, origin_latitude, origin_longitude)
                    print("轉向前進方位角完成")'''

                    #搜尋開始
                    '''for row in range(len(drone_point)):
                        if row == 0:
                            print("first point")
                            fly_to_global(dronePub, droneSub, droneCli, drone_point[row][2], drone_point[row][3], 10.0, 0.0, origin_latitude, origin_longitude)
                            if droneSub.first_detect == True:
                                print("land test1")
                                fly_to_global(dronePub, droneSub, droneCli, origin_latitude, origin_longitude, takeoffAltitude, bearing, origin_latitude, origin_longitude) #懸停
                                dronePub.hold_status = True #懸停完成發布
                                while(droneSub.camera_center == False):
                                    print("waitng camera center!")
                                    time.sleep(1)
                                while(droneSub.camera_center == True):
                                    print("wait camera_center = False")
                                    time.sleep(1)
                                dronePub.hold_status = False
                                dronePub.altitude = takeoffAltitude-5.0
                                print("down 5m")
                                dronePub.hold_status = True
                                while(droneSub.camera_center == False):
                                    print("waitng camera center!")
                                    time.sleep(1)
                                while(droneSub.camera_center == True):
                                    print("wait camera_center = False")
                                    time.sleep(1)
                                fly_to_global_without_detect(dronePub, droneSub, origin_latitude, origin_longitude, 10.0, 0.0)
                                droneCli.requestLand()
                                print("land test1")
                                droneState.droneState = 0
                                temp_status = True
                                break
                            
                        elif (row > 0) and (row+1 < point_len) :
                            print("new point")
                            print(row)
                            fly_to_global(dronePub, droneSub, droneCli, drone_point[row][2], drone_point[row][3], 10.0, drone_point[row][4], origin_latitude, origin_longitude)
                            if droneSub.first_detect == True:
                                print("land test111")
                                fly_to_global(dronePub, droneSub, droneCli, origin_latitude, origin_longitude, takeoffAltitude, bearing, origin_latitude, origin_longitude) #懸停
                                dronePub.hold_status = True #懸停完成發布
                                while(droneSub.camera_center == False):
                                    print("waitng camera center!")
                                    time.sleep(1)
                                while(droneSub.camera_center == True):
                                    print("wait camera_center = False")
                                    time.sleep(1)
                                dronePub.hold_status = False
                                dronePub.altitude = takeoffAltitude-5.0
                                print("down 5m")
                                dronePub.hold_status = True
                                while(droneSub.camera_center == False):
                                    print("waitng camera center!")
                                    time.sleep(1)
                                while(droneSub.camera_center == True):
                                    print("wait camera_center = False")
                                    time.sleep(1)
                                fly_to_global_without_detect(dronePub, droneSub, origin_latitude, origin_longitude, 10.0, 0.0)
                                droneCli.requestLand()
                                print("land test1")
                                droneState.droneState = 0
                                temp_status = True
                                break
                        
                        else:
                            print("end point")
                            
                            fly_to_global(dronePub, droneSub, droneCli, drone_point[row][2], drone_point[row][3], 10.0, drone_point[row][4], origin_latitude, origin_longitude)
                            if droneSub.first_detect == True:
                                print("land test1")
                                fly_to_global(dronePub, droneSub, droneCli, origin_latitude, origin_longitude, takeoffAltitude, bearing, origin_latitude, origin_longitude) #懸停
                                dronePub.hold_status = True #懸停完成發布
                                while(droneSub.camera_center == False):
                                    print("waitng camera center!")
                                    time.sleep(1)
                                while(droneSub.camera_center == True):
                                    print("wait camera_center = False")
                                    time.sleep(1)
                                dronePub.hold_status = False
                                dronePub.altitude = takeoffAltitude-5.0
                                print("down 5m")
                                dronePub.hold_status = True
                                while(droneSub.camera_center == False):
                                    print("waitng camera center!")
                                    time.sleep(1)
                                while(droneSub.camera_center == True):
                                    print("wait camera_center = False")
                                    time.sleep(1)
                                fly_to_global_without_detect(dronePub, droneSub, origin_latitude, origin_longitude, 10.0, 0.0)
                                droneCli.requestLand()
                                print("land test1")
                                droneState.droneState = 0
                                temp_status = True
                                break
                    if temp_status == False: #如果任務執行到最後都沒有辨識到目標
                        print("fly to origin point without detection")
                        fly_to_global_without_detect(dronePub, droneSub, origin_latitude, origin_longitude, 10.0, 0.0)
                        droneCli.requestLand()
                        droneState.droneState = 0
                    temp_status = False'''
                    
                time.sleep(0.1)

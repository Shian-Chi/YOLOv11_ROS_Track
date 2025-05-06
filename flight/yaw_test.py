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
from mavros_msgs.msg import Img
from transforms3d import euler
import numpy as np
import cmath
import math
import struct

class DroneSubscribeNode(Node):
    def __init__(self):
        super().__init__('drone_subscriber')
        self.AltitudeSub = self.create_subscription(Altitude, 'mavros/altitude', self.Altcb, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.GlobalPositionSuub = self.create_subscription(NavSatFix, 'mavros/global_position/global', self.GPcb, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.imuSub = self.create_subscription(Imu, 'mavros/imu/data', self.IMUcb, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.headingSub = self.create_subscription(Float64, 'mavros/global_position/compass_hdg', self.HDcb, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.StateSub = self.create_subscription(State, 'mavros/state', self.Statecb, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.altitude = 0.0
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
    
    def Statecb(self, msg :State):
        self.state = msg
        print(self.state.mode)
    
    def Altcb(self, msg): 
        #self.get_logger().info('I heard altitude: "%s"' % msg.relative)
        self.altitude = msg.relative

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
        
    def get_altitude(self):
        return self.altitude

class DronePublishNode(Node):
    def __init__(self):
        super().__init__('drone_publisher')
        self.setPointPositionLocal_pub = self.create_publisher(PoseStamped, 'mavros/setpoint_position/local', 10)
        self.setPointPositionGlobal_pub = self.create_publisher(GlobalPositionTarget, 'mavros/setpoint_raw/global', 10)
        self.setPointPositionLocal_timer = self.create_timer(1/50, self.setPointPositionLocal_callback)
        self.setPointPositionGlobal_timer = self.create_timer(1/50, self.setPointPositionGlobal_callback)
        self.alwaysSend = False
        self.alwaysSendPosLocal = PoseStamped()
        self.alwaysSendGlobal = False
        self.alwaysSendPosGlobal = GlobalPositionTarget()

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

class DroneServiceNode(Node):
    def __init__(self):
        super().__init__('drone_service')
        self._arming = self.create_client(CommandBool, 'mavros/cmd/arming')
        self._takeoff = self.create_client(CommandTOL, 'mavros/cmd/takeoff')
        self.land = self.create_client(CommandTOL, 'mavros/cmd/land')
        self._setMode = self.create_client(SetMode, 'mavros/set_mode')

        while not self._arming.wait_for_service(timeout_sec=1.0):
            time.sleep(1)
        print("arming service OK")
        while not self._takeoff.wait_for_service(timeout_sec=1.0):
            time.sleep(1)
        print("takeoff service OK")
        while not self.land.wait_for_service(timeout_sec=1.0):
            time.sleep(1)
        print("land service OK")
        while not self._setMode.wait_for_service(timeout_sec=1.0):
            time.sleep(1)
        print("setMode service OK")

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
        rclpy.spin_until_future_complete(self, self.future, timeout_sec=3.0)
        print("OFFBOARD mode now")
        return self.future.result()
    
    def requestLand(self): #無人機降落
        setModeCmdReq = SetMode.Request()
        setModeCmdReq.custom_mode = "AUTO.LAND"
        self.future = self._setMode.call_async(setModeCmdReq)
        rclpy.spin_until_future_complete(self, self.future, timeout_sec=3.0)
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
    
    def requestDisarm(self):
        CmdArm = CommandBool.Request()
        CmdArm.value = False
        self.future = self._arming.call_async(CmdArm)
        rclpy.spin_until_future_complete(self, self.future, timeout_sec = 3.0)
        print("Drone Disarming Now")
        print(self.future.result())
        return self.future.result()


def fly_to_global(pub : DronePublishNode, sub : DroneSubscribeNode, latitude, longitude, altitude, delta_yaw):
    print('fly to latitude', latitude," longitude:", longitude, " delta_yaw:", delta_yaw)

    pub.alwaysSendPosGlobal.latitude = latitude
    pub.alwaysSendPosGlobal.longitude = longitude
    pub.alwaysSendPosGlobal.altitude = altitude
    pub.alwaysSendPosGlobal.yaw = degree_conv_radian(sub.yaw)
    #pub.alwaysSendPosGlobal.yaw_rate = 24.3

    m_convert_lng = 1/101775.45 #台灣經度1度約為101775.45m
    m_convert_lat = 1/110936.2 #緯度1度約為110936.2m
    
    if latitude == 0.0:
        latitude = sub.latitude
    if longitude == 0.0:
        longitude = sub.longitude

    while ((abs(sub.latitude - latitude)*110936.32 > 3) or (abs(sub.longitude - longitude)*101775.45 > 3)):
        print("approaching target...")
        time.sleep(0.1)
    
    if delta_yaw != 0:
        '''print('first')
        pub.alwaysSendPosGlobal.yaw = degree_conv_radian(sub.yaw - 90.0)#degree_conv_radian(-sub.yaw + 90)
        time.sleep(10)
    
        while True:
            print(sub.yaw)
            time.sleep(0.1)'''
       
        if (delta_yaw <= 90.0 and delta_yaw > 0) or (delta_yaw >= -90.0 and delta_yaw <= 0):    #旋轉角度小於等於90度的作法
            target_yaw = sub.yaw - delta_yaw #z軸朝上 為了正角度為順時針轉取負號
            print(sub.yaw , delta_yaw, target_yaw)
            if target_yaw > 180 :
                target_yaw = target_yaw - 360
            if target_yaw < -180:
                target_yaw = target_yaw + 360

            print('target yaw:', target_yaw)
            pub.alwaysSendPosGlobal.yaw = degree_conv_radian(sub.yaw - delta_yaw)
            while abs(sub.yaw - target_yaw) > 1:
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
                    target_yaw = takeofftarget_yaw - delta90 #z軸朝上 為了正角度為順時針轉取負號

                print("ENU_YAW:", sub.yaw , "DELTA_YAW:", delta_yaw, "TARGET:", target_yaw)
                if target_yaw > 180 :
                    target_yaw = target_yaw - 360
                if target_yaw < -180:
                    target_yaw = target_yaw + 360

                pub.alwaysSendPosGlobal.yaw = degree_conv_radian(sub.yaw - delta90)
                while abs(sub.yaw - target_yaw) > 0.1:
                    time.sleep(0.1)
                print(sub.yaw)
            
            final_delta_yaw = delta_yaw - (90 * pos_neg_rotation90_num)
            if final_delta_yaw != 0:
                print("fina;_delta_yaw:", final_delta_yaw)
                pub.alwaysSendPosGlobal.yaw = degree_conv_radian(sub.yaw - final_delta_yaw)
                while sub.yaw > sub.yaw + 0.1 and sub.yaw < sub.yaw - 0.1:
                    time.sleep(0.1)


    if (latitude != 0.0) and (longitude != 0.0):
        while ((abs(sub.latitude - latitude)*110936.32 > 1.5) or (abs(sub.longitude - longitude)*101775.45 > 1.5) ): 
            print("lat distance:", abs(sub.latitude - latitude)*110936.2, "lng distance:", abs(sub.longitude - longitude)*101775.45, "alt distance:", abs(sub.altitude - altitude))
            time.sleep(0.1)
    '''if (latitude == 0.0) and (longitude != 0.0):
        while ((abs(sub.longitude - longitude) > m_convert_lng*0.5) and (abs(sub.altitude - altitude) < 0.1)):
            print("lng distance:", abs(sub.longitude - longitude)*101775.45, "alt distance:", abs(sub.altitude - altitude))
            time.sleep(0.1)
    if (latitude != 0.0) and (longitude == 0.0):
         while ((abs(sub.latitude - latitude) > m_convert_lat*0.5) and (abs(sub.altitude - altitude) < 0.1)):
            print("lat distance:", abs(sub.latitude - latitude)*110936.2, "alt distance:", abs(sub.altitude - altitude))
            time.sleep(0.1)'''

    time.sleep(1)
    print('reach the destination')

def radian_conv_degree(Radian) : 
    return ((Radian / math.pi) * 180)

def degree_conv_radian(Degree) : 
    return ((Degree / 180) * math.pi)

def _droneSpinThread(pub, sub, srv):
    #print("_droneSpinThread start")

    executor = MultiThreadedExecutor()
    executor.add_node(sub)
    executor.add_node(pub)
    executor.spin()

def check_arm(sub : DroneSubscribeNode, srv : DroneServiceNode):
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

def takeoff_global(pub : DronePublishNode, sub : DroneSubscribeNode, srv : DroneServiceNode, rel_alt : float):
    data = GlobalPositionTarget()
    data.coordinate_frame = 6 #FRAME_GLOBAL_REL_ALT 
    data.type_mask = 0
    data.velocity.x = 0.25
    data.velocity.y = 0.25
    current_latitude = sub.latitude
    current_longitude = sub.longitude
    data.latitude= current_latitude
    data.longitude = current_longitude
    print("Current latitude and longitude:",current_longitude,',',current_longitude)
    data.altitude = rel_alt
    data.yaw = degree_conv_radian(0.0) #起飛機頭永遠朝正東
    pub.alwaysSendPosGlobal = data
    pub.alwaysSendGlobal = True
    #dronePub.sendPositionGlobal(data)

    #droneSrv.requestSetMode("OFFBOARD")
    #srv.requestCmdArm() #無人機解鎖

    result = check_arm(sub, srv)
    if result == False:
        return False
    
    srv.requestSetMode("OFFBOARD")

    while ((sub.get_altitude()) <= (rel_alt- 0.4)):
        time.sleep(1/50)
    print('takeoff complete')

def calculate_bearing(lat1, lon1, lat2, lon2):

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


def drone_moving_along_the_x(meter:float, pub : DronePublishNode, sub : DroneSubscribeNode, origin_heading):

    current_lat = sub.latitude #y
    current_lon = sub.longitude #x

    theta = sub.heading-origin_heading
    print('---sub.heading:', sub.heading)

    delta_y = -meter * math.sin(math.radians(theta))
    delta_x = meter * math.cos(math.radians(theta))

    delta_lat = delta_y*(1/101775.45)
    delta_lon = delta_x*(1/110936.32)

    print("moving x: %fm, y: %fm", delta_x, delta_y)

    target_lat= sub.latitude + delta_lat
    target_lon = sub.longitude +delta_lon

    pub.alwaysSendPosGlobal.latitude = target_lat
    pub.alwaysSendPosGlobal.longitude = target_lon

    while ((abs(sub.latitude - target_lat)*110936.32 > 3) or (abs(sub.longitude - target_lon)*101775.45 > 3)):
        print("approaching target...")
        time.sleep(0.1)


def signal_handler(signal, frame):
    print("\nprogram exiting gracefully")
    rclpy.shutdown()
    sys.exit(0)


if __name__ == '__main__':

    #system interrrupt (ctrl + c)
    signal.signal(signal.SIGINT, signal_handler)

    rclpy.init()

    global droneSub
    global dronePub
    global droneSrv
    
    droneSub = DroneSubscribeNode()
    dronePub = DronePublishNode()
    droneSrv = DroneServiceNode()

    droneSpinThread = threading.Thread(target=_droneSpinThread, args=(dronePub, droneSub, droneSrv))
    droneSpinThread.start()


    while droneSub.latitude == 0.0 and droneSub.longitude == 0.0:
        time.sleep(1)

    print("system will start in 3s")
    time.sleep(3)
    droneSrv.requestLoiter()
    time.sleep(3)
        
    origin_latitude = droneSub.latitude
    origin_longitude = droneSub.longitude
    
    takeoff_height = 5.0
    takeoff_global(dronePub, droneSub, droneSrv, takeoff_height)
    
    origin_heading = droneSub.heading
    print('origin_heading:', origin_heading)

    time.sleep(10)

    fly_to_global(dronePub, droneSub, origin_latitude, origin_longitude, takeoff_height, 0.0)

    print("rotation 60 degrees")

    fly_to_global(dronePub, droneSub, origin_latitude, origin_longitude, takeoff_height, 60.0)

    time.sleep(5)

    drone_moving_along_the_x(5.0, dronePub, droneSub, origin_heading)
    time.sleep(10)

    droneSrv.requestLand()

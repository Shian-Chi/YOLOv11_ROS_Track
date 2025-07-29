import time
import numpy as np
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32MultiArray, Float32, Float64, String
from sensor_msgs.msg import NavSatFix, Imu
from rclpy.qos import ReliabilityPolicy, QoSProfile
from transforms3d import euler
from tutorial_interfaces.msg import GimbalDegree
from parameter import Parameter

para = Parameter()

class vision_ros(Node):
    def __init__(self):
        super().__init__('gimbal_node')

        # ------ 狀態 ------
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.detectSTAT = False
        self.gimbalCenter = False
        self.rel_alt = 0.0

        self.latitude = 0.0
        self.longitude = 0.0
        self.altitude = 0.0
        self.heading = 0.0

        self.motor_yaw = 0.0
        self.motor_pitch = 0.0
        # ------ Publisher ------
        self.estimated_pos = self.create_publisher(NavSatFix, '/target_position', 5)
        self.Triangulation_pos = self.create_publisher(NavSatFix, '/target_position/Triangulation', 5)
        
        # ------ QoS ------
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        # ------ Subscriber ------
        self.detectSub = self.create_subscription(Bool, '/target_detected', self.detect_cb, qos)
        self.centerSub = self.create_subscription(Bool, '/targetCenter', self.Center_cb, qos)
        self.AltitudeSub = self.create_subscription(Float64, 'mavros/global_position/rel_alt', self.rel_altcb, qos)
        self.imuSub = self.create_subscription(Imu, 'mavros/imu/data', self.IMU_cb, qos)
        self.GlobalPositionSuub = self.create_subscription(NavSatFix, 'mavros/global_position/global', self.GPcb, qos)
        self.gimbalsub = self.create_subscription(GimbalDegree, '/gimbal_angle', self.gimbalAngle_cb, qos)
        self.headingSub = self.create_subscription(Float64, 'mavros/global_position/compass_hdg', self.HDcb, qos)

    # ---------- Publishers ----------
    def publish_estimated_position(self, STAT:str, lat, lon, alt):
        pos = NavSatFix()
        pos.header.frame_id = STAT
        pos.latitude = lat
        pos.longitude = lon
        pos.altitude = alt
        self.estimated_pos.publish(pos)
        
    def publish_triangulation_position(self, STAT:str, lat, lon, alt):
        pos = NavSatFix()
        pos.header.frame_id = STAT
        pos.latitude = lat
        pos.longitude = lon
        pos.altitude = alt
        self.Triangulation_pos.publish(pos)
        
    # ---------- Callbacks ----------
    def detect_cb(self, msg: Bool):
        self.detectSTAT = msg.data

    def Center_cb(self, msg: Bool):
        self.gimbalCenter = self.centerSub = msg.data

    def rel_altcb(self, msg: Float64):
        self.rel_alt = msg.data

    def gimbalAngle_cb(self, msg: GimbalDegree):
        self.motor_yaw = msg.yaw
        self.motor_pitch = msg.pitch

    def GPcb(self, msg: NavSatFix):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.altitude = msg.altitude

    def HDcb(self, msg: Float64):
        self.heading = msg.data

    def IMU_cb(self, msg: Imu):
        # ROS IMU -> NED → 歐拉角
        ned = euler.quat2euler([
            msg.orientation.w,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z])
        self.pitch = math.degrees(ned[0])
        self.roll = math.degrees(ned[1])
        self.yaw = math.degrees(ned[2])

    # ---------- Function ----------
    def get_gps_data(self):
        return self.longitude, self.latitude, self.altitude


class vector():
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z

    def as_np(self):
        return np.array([self.x, self.y, self.z])

    def copy_from(self, other: "vector"):
        self.x, self.y, self.z = other.x, other.y, other.z

    def __str__(self):
        return f"vector(x={self.x:.6f}, y={self.y:.6f}, z={self.z:.6f})"


class verticalTargetPositioning():
    def __init__(self):
        self.zeroPos = vector()
        self.firPos = vector()
        self.groundTargetPos = vector()
        self.zeroTargemathAangles = vector()
        self.firTargemathAangles = vector()
        self.error_message = ""
        self.target_Path = []

    def D_xy_Val(self):
        try:
            return (self.firPos.z - self.zeroPos.z) / \
                (math.tan(math.radians(self.zeroTargemathAangles.y)) -
                 math.tan(math.radians(self.firTargemathAangles.y)))
        except ZeroDivisionError:
            print("[垂直定位] 除以零錯誤")
            return None

    def positionTarget(self):
        D = self.D_xy_Val()
        if D is None:
            return None, None, None
        x = self.firPos.x + D * math.cos(math.radians(self.zeroTargemathAangles.z))
        y = self.firPos.y + D * math.sin(math.radians(self.zeroTargemathAangles.y))
        z = self.firPos.z - D * math.tan(math.radians(self.firTargemathAangles.y))
        self.target_Path.append([x, y, z])
        return x, y, z


class horizontalTargetPositioning():
    def __init__(self):
        self.zeroPos = vector()
        self.firPos = vector()
        self.groundTargetPos = vector()
        self.zeroTargemathAangles = vector()
        self.firTargemathAangles = vector()
        self.error_message = ""
        self.target_Path = []

    def positionTarget(self):
        angle1 = math.radians(self.firTargemathAangles.y)
        angle2 = math.radians(self.zeroTargemathAangles.y)
        tan1 = math.tan(angle1)
        tan2 = math.tan(angle2)

        if math.isclose(tan1, tan2, abs_tol=1e-6):
            return None, None, None

        x = (tan1 * self.firPos.x - tan2 * self.zeroPos.x) / (tan1 - tan2)
        y = (tan1 * self.firPos.y - tan2 * self.zeroPos.y) / (tan1 - tan2)
        z = self.firPos.z - math.tan(angle1) / math.cos(
            math.radians(self.firTargemathAangles.z)) * (x - self.zeroPos.x)

        self.target_Path.append([x, y, z])
        return x, y, z


class targetPositioning(verticalTargetPositioning):
    def __init__(self, sub: vision_ros):
        super().__init__()
        self.sub = sub
        self.Hresult = [None, None, None, None] # horizontal
        self.Vresult = [None, None, None, None] # vertical
        self.vertical = verticalTargetPositioning()
        self.horizontal = horizontalTargetPositioning()
        # 可選：啟用高度選擇調試模式
        self.debug_altitude = False

    def get_uav_data(self):
        """獲取無人機數據，智能選擇高度感測器"""
        lat, lon, alt = self.sub.get_gps_data()

        return {
            'time': time.time(),
            'latitude': lat,
            'longitude': lon,
            'altitude': alt,
            'heading': self.sub.heading,
            'gimbalPitchDeg': self.sub.motor_pitch,
            'gimbalYawDeg': self.sub.motor_yaw,
        }

    def _select_altitude_sensor(self, lidar_range, barometer_altitude):
        """選擇高度感測器

        Args:
            lidar_range: LiDAR測量高度
            barometer_altitude: 氣壓計高度

        Returns:
            選擇的高度值
        """
        # 檢查LiDAR數據是否有效
        lidar_valid = (lidar_range is not None and
                       0.05 < lidar_range < 9.5 and  # 有效範圍
                       lidar_range != 1.0)           # 排除無效測量標誌

        # 基於氣壓計高度決定使用哪個感測器
        if barometer_altitude < 10.0 and lidar_valid:
            # 低高度且LiDAR有效：使用LiDAR（更精確）
            selected_altitude = lidar_range
            altitude_source = "LiDAR"
        else:
            # 高高度或LiDAR無效：使用氣壓計
            selected_altitude = barometer_altitude
            altitude_source = "Barometer"

        # 可選的調試輸出
        if self.debug_altitude:
            print(f"[高度選擇] LiDAR={lidar_range:.2f}m, "
                  f"氣壓計={barometer_altitude:.2f}m, "
                  f"選用={selected_altitude:.2f}m ({altitude_source})")

        return selected_altitude

    def _select_altitude_sensor_advanced(self, lidar_range, barometer_altitude):
        """進階版高度選擇器（可選用）

        提供更平滑的感測器切換
        """
        LIDAR_MAX = 9.0      # LiDAR使用上限
        TRANSITION = 1.5     # 過渡區間長度

        # 檢查LiDAR是否有效
        lidar_valid = (lidar_range is not None and
                       0.05 < lidar_range < 9.5 and
                       lidar_range != 1.0)

        if not lidar_valid:
            return barometer_altitude

        if barometer_altitude <= LIDAR_MAX:
            # 完全使用LiDAR
            return lidar_range
        elif barometer_altitude <= LIDAR_MAX + TRANSITION:
            # 線性混合過渡區間
            blend_factor = (barometer_altitude - LIDAR_MAX) / TRANSITION
            blended_altitude = (lidar_range * (1 - blend_factor) +
                                barometer_altitude * blend_factor)

            if self.debug_altitude:
                print(f"[混合模式] 混合比例={blend_factor:.2f}, "
                      f"結果={blended_altitude:.2f}m")

            return blended_altitude
        else:
            # 完全使用氣壓計
            return barometer_altitude

    def position_calc(self, isCenter):
        self.Hresult = [None, None, None, None]
        self.Vresult = [None, None, None, None]
        if not isCenter:
            print("CameraisCenter =", isCenter)
            return [None, None, None, None]

        row = self.get_uav_data()
        self.firPos.x = row['longitude']
        self.firPos.y = row['latitude']
        self.firPos.z = row['altitude']
        self.firTargemathAangles.y = row['gimbalPitchDeg']
        self.firTargemathAangles.z = row['gimbalYawDeg']

        # if self.zeroPos.x == self.zeroPos.y == self.zeroPos.z == 0.0:
        #     self.zeroPos.copy_from(self.firPos)
        #     self.zeroTargemathAangles.copy_from(self.firTargemathAangles)
        #     tri = self.Triangulation(row['latitude'], row['longitude'], row['altitude'],
        #                              row['gimbalPitchDeg'], row['gimbalYawDeg'], row['heading'])
        #     if tri is not None:
        #         self.result = ["triangulation"] + list(tri)
        #         print(f"[初始定位] lat={tri[0]:.7f}, lon={tri[1]:.7f}, alt={tri[2]:.2f}")
        #     return self.result

        angle_threshold_deg = 1.0

        if abs(self.firTargemathAangles.y - self.zeroTargemathAangles.y) > angle_threshold_deg:
            self.horizontal.zeroPos.copy_from(self.zeroPos)
            self.horizontal.firPos.copy_from(self.firPos)
            self.horizontal.zeroTargemathAangles.copy_from(self.zeroTargemathAangles)
            self.horizontal.firTargemathAangles.copy_from(self.firTargemathAangles)
            horizontal_val = self.horizontal.positionTarget()
            if horizontal_val and all(v is not None for v in horizontal_val):
                x, y, z = horizontal_val
                self.Hresult = ["horizontal", x, y, z]
                self.target_Path.append([x, y, z])
                print(f"[水平定位] x={x:.7f}, y={y:.7f}, z={z:.2f}")
                self.zeroPos.copy_from(self.firPos)
                self.zeroTargemathAangles.copy_from(self.firTargemathAangles)

        if abs(self.firPos.z - self.zeroPos.z) > 1.0 and \
           abs(self.firTargemathAangles.y - self.zeroTargemathAangles.y) > 1.0:
            self.vertical.zeroPos.copy_from(self.zeroPos)
            self.vertical.firPos.copy_from(self.firPos)
            self.vertical.zeroTargemathAangles.copy_from(self.zeroTargemathAangles)
            self.vertical.firTargemathAangles.copy_from(self.firTargemathAangles)
            vertical_val = self.vertical.positionTarget()
            if vertical_val and all(v is not None for v in vertical_val):
                x, y, z = vertical_val
                self.Vresult = ["vertical", x, y, z]
                self.target_Path.append([x, y, z])
                print(f"[垂直定位] x={x:.7f}, y={y:.7f}, z={z:.2f}")
                self.zeroPos.copy_from(self.firPos)
                self.zeroTargemathAangles.copy_from(self.firTargemathAangles)

        v_STAT = self.Vresult[0] is not None 
        h_STAT = self.Hresult[0] is not None 
        
        if para.position_Mode == 'vertical':
            if self.Vresult[0] is not None:
                return self.Vresult
        elif para.position_Mode == 'horizontal':
            if self.Hresult[0] is not None:
                return self.Hresult
        
        if v_STAT:
            return self.Vresult
        elif h_STAT:
            return self.Hresult
        else:
            return [None, None, None, None]

    def Triangulation(self, lat0, lon0, h0, gimbal_pitch, gimbal_yaw, heading=10.0):
        R_earth = 6378137.0
        total_yaw = math.radians((heading + gimbal_yaw) % 360)
        total_pitch = node.pitch + gimbal_pitch
        theta = math.radians(abs(total_pitch))
        if math.isclose(theta, 0.0, abs_tol=1e-6):
            print("[警告] pitch 太小，無法進行 Triangulation")
            return [None, None, None, None]
        try:
            R = h0 / math.tan(theta)
        except ZeroDivisionError:
            print("[錯誤] Tan(θ)=0 導致除以 0 錯誤")
            return [None, None, None, None]

        dx = R * math.sin(total_yaw)
        dy = R * math.cos(total_yaw)

        dLat = dy / R_earth
        dLon = dx / (R_earth * math.cos(math.radians(lat0)))

        lat_target = lat0 + math.degrees(dLat)
        lon_target = lon0 + math.degrees(dLon)

        return ["Triangulation", lat_target, lon_target, h0]

if __name__ == "__main__":
    rclpy.init()
    node = vision_ros()
    pos_estimator = targetPositioning(node)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            # 目標置中時執行定位
            result = pos_estimator.position_calc(node.gimbalCenter)
            if result[0] is not None:
                print(result)
                node.publish_estimated_position(result[0], result[2], result[1], result[3])
                
            result = pos_estimator.Triangulation(node.latitude, node.longitude, node.rel_alt,
                                                 node.motor_pitch, node.motor_yaw, node.heading)
            if result[0] is not None:
                node.publish_triangulation_position(result[0], result[2], result[1], result[3])
            time.sleep(0.03)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
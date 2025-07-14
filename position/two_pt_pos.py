import time
import numpy as np
import math 
from ros.ros_topic import MinimalSubscriber

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
                (math.tan(math.radians(self.zeroTargemathAangles.y)) - \
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
        z = self.firPos.z - math.tan(angle1) / math.cos(math.radians(self.firTargemathAangles.z)) * (x - self.zeroPos.x)

        self.target_Path.append([x, y, z])
        return x, y, z

class targetPositioning(verticalTargetPositioning):
    def __init__(self, sub: MinimalSubscriber):
        super().__init__()
        self.sub = sub
        self.result = [None, None, None, None]
        self.vertical = verticalTargetPositioning()
        self.horizontal = horizontalTargetPositioning()
        # 可選：啟用高度選擇調試模式
        self.debug_altitude = False

    def get_uav_data(self):
        """獲取無人機數據，智能選擇高度感測器"""
        lat, lon, _ = self.sub.get_gps_data()
        lidar_range = self.sub.lidar_range
        barometer_altitude = self.sub.relative_altitude
        
        # 修正後的高度選擇邏輯
        altitude = self._select_altitude_sensor_advanced(lidar_range, barometer_altitude)
        
        return {
            'time': time.time(),
            'latitude': lat,
            'longitude': lon,
            'altitude': altitude,
            'heading': self.sub.compass_heading,
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
        self.result = [None, None, None, None]
        if not isCenter:
            print("CameraisCenter =", isCenter)
            return self.result

        row = self.get_uav_data()
        self.firPos.x = row['longitude']
        self.firPos.y = row['latitude']
        self.firPos.z = row['altitude']
        self.firTargemathAangles.y = row['gimbalPitchDeg']
        self.firTargemathAangles.z = row['gimbalYawDeg']

        if self.zeroPos.x == self.zeroPos.y == self.zeroPos.z == 0.0:
            self.zeroPos.copy_from(self.firPos)
            self.zeroTargemathAangles.copy_from(self.firTargemathAangles)
            tri = self.Triangulation(row['latitude'], row['longitude'], row['altitude'],
                                     row['gimbalPitchDeg'], row['gimbalYawDeg'], row['heading'])
            if tri is not None:
                self.result = ["triangulation"] + list(tri)
                print(f"[初始定位] lat={tri[0]:.7f}, lon={tri[1]:.7f}, alt={tri[2]:.2f}")
            return self.result

        angle_threshold_deg = 1.0

        if abs(self.firTargemathAangles.y - self.zeroTargemathAangles.y) > angle_threshold_deg:
            self.horizontal.zeroPos.copy_from(self.zeroPos)
            self.horizontal.firPos.copy_from(self.firPos)
            self.horizontal.zeroTargemathAangles.copy_from(self.zeroTargemathAangles)
            self.horizontal.firTargemathAangles.copy_from(self.firTargemathAangles)
            horizontal_val = self.horizontal.positionTarget()
            if horizontal_val and all(v is not None for v in horizontal_val):
                x, y, z = horizontal_val
                self.result = ["horizontal", x, y, z]
                self.target_Path.append([x, y, z])
                print(f"[水平定位] x={x:.7f}, y={y:.7f}, z={z:.2f}")
                self.zeroPos.copy_from(self.firPos)
                self.zeroTargemathAangles.copy_from(self.firTargemathAangles)
                return self.result

        if abs(self.firPos.z - self.zeroPos.z) > 1.0 and \
           abs(self.firTargemathAangles.y - self.zeroTargemathAangles.y) > 1.0:
            self.vertical.zeroPos.copy_from(self.zeroPos)
            self.vertical.firPos.copy_from(self.firPos)
            self.vertical.zeroTargemathAangles.copy_from(self.zeroTargemathAangles)
            self.vertical.firTargemathAangles.copy_from(self.firTargemathAangles)
            vertical_val = self.vertical.positionTarget()
            if vertical_val and all(v is not None for v in vertical_val):
                x, y, z = vertical_val
                self.result = ["vertical", x, y, z]
                self.target_Path.append([x, y, z])
                print(f"[垂直定位] x={x:.7f}, y={y:.7f}, z={z:.2f}")
                self.zeroPos.copy_from(self.firPos)
                self.zeroTargemathAangles.copy_from(self.firTargemathAangles)
                return self.result

        return self.result

    def Triangulation(self, lat0, lon0, h0, pitch, yaw, heading=10.0):
        R_earth = 6378137.0
        theta = math.radians(abs(pitch))
        if math.isclose(theta, 0.0, abs_tol=1e-6):
            print("[警告] pitch 太小，無法進行 Triangulation")
            return None
        try:
            R = h0 / math.tan(theta)
        except ZeroDivisionError:
            print("[錯誤] Tan(θ)=0 導致除以 0 錯誤")
            return None

        total_angle = math.radians((heading + yaw) % 360)
        dx = R * math.sin(total_angle)
        dy = R * math.cos(total_angle)

        dLat = dy / R_earth
        dLon = dx / (R_earth * math.cos(math.radians(lat0)))

        lat_target = lat0 + math.degrees(dLat)
        lon_target = lon0 + math.degrees(dLon)

        return lat_target, lon_target, h0
    
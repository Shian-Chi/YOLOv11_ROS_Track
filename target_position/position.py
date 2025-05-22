import sys
import math
import numpy as np

# 如果垂直目標定位程式碼在另一個文件中，請導入它
# from your_module import vector, verticalTargetPositioning

# 否則，直接將改進後的代碼粘貼在這裡
from math import sin, cos, tan, isclose
import numpy as np


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
        return f"vector(x={self.x}, y={self.y}, z={self.z})"
        
class verticalTargetPositioning():
    def __init__(self):
        self.zeroPos = vector()          # 前一個位置
        self.firPos = vector()           # 當前位置
        self.groundTargetPos = vector()  # 計算得到的地面目標位置
        self.zeroTargetAngles = vector() # 前一個角度
        self.firTargetAngles = vector()  # 當前角度
        self.D_xy = None                 # 水平距離
        self.error_message = ""          # 錯誤信息
    
    def D_xy_Value(self):
        """計算水平距離 D_xy"""
        # 檢查兩個俯仰角的正切值是否幾乎相等
        if isclose(tan(self.firTargetAngles.y), tan(self.zeroTargetAngles.y), abs_tol=1e-10):
            self.error_message = "兩個觀測角度太接近，無法進行三角測量"
            raise ValueError(self.error_message)
        
        # 檢查高度差是否為零
        if isclose(self.firPos.z, self.zeroPos.z, abs_tol=1e-10):
            self.error_message = "兩個觀測點高度相同，無法進行垂直三角測量"
            raise ValueError(self.error_message)
            
        # 計算水平距離
        self.D_xy = (self.firPos.z - self.zeroPos.z) / (tan(self.firTargetAngles.y) - tan(self.zeroTargetAngles.y))
        return self.D_xy
    
    def groundTargetPostion(self):
        """計算地面目標位置"""
        self.error_message = ""  # 重置錯誤信息
        
        try:
            # 計算水平距離
            self.D_xy = self.D_xy_Value()
            
            # 使用方位角和水平距離計算地面坐標
            self.groundTargetPos.x = self.firPos.x + self.D_xy * cos(self.firTargetAngles.z)
            self.groundTargetPos.y = self.firPos.y + self.D_xy * sin(self.firTargetAngles.z)
            
            # 計算目標高度（理論上應該接近零或地面高度）
            self.groundTargetPos.z = self.firPos.z - self.D_xy * tan(self.firTargetAngles.y)
            
            return self.groundTargetPos.x, self.groundTargetPos.y, self.groundTargetPos.z
            
        except ValueError as e:
            # 捕獲並存儲錯誤信息
            if not self.error_message:
                self.error_message = str(e)
            print(f"計算錯誤: {self.error_message}")
            return None, None, None
    
    def update(self, longitude, latitude, altitude,
               imuRoll=0.0, imuPitch=0.0, imuYaw=0.0,
               motorRoll=0.0, motorPitch=0.0, motorYaw=0.0):
        """更新位置和角度信息"""
        # 保存前一個位置和角度
        self.zeroPos.copy_from(self.firPos)
        self.zeroTargetAngles.copy_from(self.firTargetAngles)
        
        # 更新當前位置
        self.firPos.x = longitude
        self.firPos.y = latitude
        self.firPos.z = altitude
        
        # 更新當前角度
        self.updateAngles(imuRoll, imuPitch, imuYaw, motorRoll, motorPitch, motorYaw)
        
    def updateAngles(self, imuRoll=0.0, imuPitch=0.0, imuYaw=0.0, 
                     motorRoll=0.0, motorPitch=0.0, motorYaw=0.0):
        """更新當前角度"""
        self.firTargetAngles.x = imuRoll + motorRoll    # 橫滾角
        self.firTargetAngles.y = imuPitch + motorPitch  # 俯仰角
        self.firTargetAngles.z = imuYaw + motorYaw      # 偏航角
    
    def get_error(self):
        """獲取最近的錯誤信息"""
        return self.error_message

    def get_status(self):
        """獲取當前狀態信息"""
        return {
            "current_position": (self.firPos.x, self.firPos.y, self.firPos.z),
            "previous_position": (self.zeroPos.x, self.zeroPos.y, self.zeroPos.z),
            "current_angles": (self.firTargetAngles.x, self.firTargetAngles.y, self.firTargetAngles.z),
            "previous_angles": (self.zeroTargetAngles.x, self.zeroTargetAngles.y, self.zeroTargetAngles.z),
            "calculated_distance": self.D_xy,
            "target_position": (self.groundTargetPos.x, self.groundTargetPos.y, self.groundTargetPos.z),
            "error": self.error_message
        }

# 測試函數
def test_known_target():
    """測試已知目標位置的情況"""
    print("===== 測試已知目標位置 =====")
    
    # 創建已知的目標位置
    target_x = 100.0
    target_y = 100.0
    target_z = 0.0
    
    # 創建測試軌跡
    positions = [
        {"long": 0.0, "lat": 0.0, "alt": 100.0},  # 初始位置
        {"long": 20.0, "lat": 20.0, "alt": 90.0},  # 第二個位置
        {"long": 40.0, "lat": 40.0, "alt": 80.0},  # 第三個位置
    ]
    
    # 存儲計算的目標位置結果
    calculated_positions = []
    
    # 創建垂直目標定位系統
    vtp = verticalTargetPositioning()
    
    # 設置初始位置，但不進行計算
    initial_pos = positions[0]
    
    # 計算指向已知目標的俯仰角和偏航角
    pitch = -math.atan2(initial_pos["alt"] - target_z, 
                      math.sqrt((target_x - initial_pos["long"])**2 + (target_y - initial_pos["lat"])**2))
    yaw = math.atan2(target_y - initial_pos["lat"], target_x - initial_pos["long"])
    
    # 更新初始位置和角度
    vtp.update(
        longitude=initial_pos["long"], 
        latitude=initial_pos["lat"], 
        altitude=initial_pos["alt"],
        imuPitch=pitch, 
        imuYaw=yaw
    )
    
    print(f"初始位置: 經度={initial_pos['long']}, 緯度={initial_pos['lat']}, 高度={initial_pos['alt']}")
    print(f"初始角度: 俯仰角={math.degrees(pitch):.2f}°, 偏航角={math.degrees(yaw):.2f}°")
    print(f"目標真實位置: 經度={target_x}, 緯度={target_y}, 高度={target_z}")
    
    # 對於後續的每個位置
    for i, pos in enumerate(positions[1:], 1):
        # 計算指向目標的角度
        pitch = -math.atan2(pos["alt"] - target_z, 
                          math.sqrt((target_x - pos["long"])**2 + (target_y - pos["lat"])**2))
        yaw = math.atan2(target_y - pos["lat"], target_x - pos["long"])
        
        # 更新位置和角度
        vtp.update(
            longitude=pos["long"], 
            latitude=pos["lat"], 
            altitude=pos["alt"],
            imuPitch=pitch, 
            imuYaw=yaw
        )
        
        print(f"\n位置 {i}: 經度={pos['long']}, 緯度={pos['lat']}, 高度={pos['alt']}")
        print(f"角度 {i}: 俯仰角={math.degrees(pitch):.2f}°, 偏航角={math.degrees(yaw):.2f}°")
        
        # 計算目標位置
        calc_x, calc_y, calc_z = vtp.groundTargetPostion()
        
        if calc_x is not None:
            # 計算誤差
            error_x = calc_x - target_x
            error_y = calc_y - target_y
            error_z = calc_z - target_z
            total_error = math.sqrt(error_x**2 + error_y**2 + error_z**2)
            
            print(f"計算得到的目標位置: 經度={calc_x:.2f}, 緯度={calc_y:.2f}, 高度={calc_z:.2f}")
            print(f"誤差: dx={error_x:.2f}, dy={error_y:.2f}, dz={error_z:.2f}, 總誤差={total_error:.2f}")
            
            calculated_positions.append((calc_x, calc_y, calc_z))
        else:
            print(f"計算錯誤: {vtp.get_error()}")

def test_edge_cases():
    """測試邊緣情況"""
    print("\n===== 測試邊緣情況 =====")
    
    # 創建垂直目標定位系統
    vtp = verticalTargetPositioning()
    
    # 測試情況1：相同高度的觀測點
    print("\n測試情況1：相同高度的觀測點")
    vtp.update(longitude=0.0, latitude=0.0, altitude=100.0, imuPitch=-0.3, imuYaw=0.0)
    vtp.update(longitude=10.0, latitude=0.0, altitude=100.0, imuPitch=-0.4, imuYaw=0.0)
    x, y, z = vtp.groundTargetPostion()
    print(f"結果: {vtp.get_error() if x is None else f'x={x}, y={y}, z={z}'}")
    
    # 測試情況2：相同觀測角度
    print("\n測試情況2：相同觀測角度")
    vtp.update(longitude=0.0, latitude=0.0, altitude=100.0, imuPitch=-0.3, imuYaw=0.0)
    vtp.update(longitude=10.0, latitude=0.0, altitude=90.0, imuPitch=-0.3, imuYaw=0.0)
    x, y, z = vtp.groundTargetPostion()
    print(f"結果: {vtp.get_error() if x is None else f'x={x}, y={y}, z={z}'}")
    
    # 測試情況3：零俯仰角（水平方向）
    print("\n測試情況3：零俯仰角（水平方向）")
    vtp.update(longitude=0.0, latitude=0.0, altitude=100.0, imuPitch=0.0, imuYaw=0.0)
    vtp.update(longitude=10.0, latitude=0.0, altitude=90.0, imuPitch=-0.1, imuYaw=0.0)
    x, y, z = vtp.groundTargetPostion()
    print(f"結果: {vtp.get_error() if x is None else f'x={x}, y={y}, z={z}'}")

def test_random_trajectory():
    """測試隨機軌跡"""
    print("\n===== 測試隨機軌跡 =====")
    
    # 設置隨機種子以獲得可重複的結果
    np.random.seed(42)
    
    # 創建已知的目標位置
    target_x = 100.0
    target_y = 100.0
    target_z = 0.0
    
    # 創建隨機軌跡
    num_points = 5
    positions = []
    
    # 在目標周圍生成隨機點
    for i in range(num_points):
        # 產生一個在目標周圍的隨機點
        long = np.random.uniform(0, target_x*1.5)
        lat = np.random.uniform(0, target_y*1.5)
        alt = np.random.uniform(50, 150)
        positions.append({"long": long, "lat": lat, "alt": alt})
    
    # 存儲計算的目標位置結果
    calculated_positions = []
    
    # 創建垂直目標定位系統
    vtp = verticalTargetPositioning()
    
    # 設置初始位置
    initial_pos = positions[0]
    
    # 計算指向已知目標的俯仰角和偏航角
    pitch = -math.atan2(initial_pos["alt"] - target_z, 
                      math.sqrt((target_x - initial_pos["long"])**2 + (target_y - initial_pos["lat"])**2))
    yaw = math.atan2(target_y - initial_pos["lat"], target_x - initial_pos["long"])
    
    # 更新初始位置和角度
    vtp.update(
        longitude=initial_pos["long"], 
        latitude=initial_pos["lat"], 
        altitude=initial_pos["alt"],
        imuPitch=pitch, 
        imuYaw=yaw
    )
    
    print(f"初始位置: 經度={initial_pos['long']:.2f}, 緯度={initial_pos['lat']:.2f}, 高度={initial_pos['alt']:.2f}")
    
    # 對於後續的每個位置
    for i, pos in enumerate(positions[1:], 1):
        # 添加一些角度噪聲以模擬真實情況
        angle_noise = np.random.normal(0, 0.01)  # 標準差為0.01弧度（約0.57度）
        
        # 計算指向目標的角度
        pitch = -math.atan2(pos["alt"] - target_z, 
                          math.sqrt((target_x - pos["long"])**2 + (target_y - pos["lat"])**2))
        pitch += angle_noise  # 添加噪聲
        
        yaw = math.atan2(target_y - pos["lat"], target_x - pos["long"])
        yaw += angle_noise  # 添加噪聲
        
        # 更新位置和角度
        vtp.update(
            longitude=pos["long"], 
            latitude=pos["lat"], 
            altitude=pos["alt"],
            imuPitch=pitch, 
            imuYaw=yaw
        )
        
        print(f"\n位置 {i}: 經度={pos['long']:.2f}, 緯度={pos['lat']:.2f}, 高度={pos['alt']:.2f}")
        
        # 計算目標位置
        calc_x, calc_y, calc_z = vtp.groundTargetPostion()
        
        if calc_x is not None:
            # 計算誤差
            error_x = calc_x - target_x
            error_y = calc_y - target_y
            error_z = calc_z - target_z
            total_error = math.sqrt(error_x**2 + error_y**2 + error_z**2)
            
            print(f"計算得到的目標位置: 經度={calc_x:.2f}, 緯度={calc_y:.2f}, 高度={calc_z:.2f}")
            print(f"誤差: 總誤差={total_error:.2f}")
            
            calculated_positions.append((calc_x, calc_y, calc_z))
        else:
            print(f"計算錯誤: {vtp.get_error()}")

if __name__ == "__main__":
    print("垂直目標定位系統測試程序")
    print("========================\n")
    
    # 測試已知目標位置
    test_known_target()
    
    # 測試邊緣情況
    test_edge_cases()
    
    # 測試隨機軌跡
    test_random_trajectory()
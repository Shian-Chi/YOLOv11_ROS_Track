import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import smbus2
import time

class LidarPublisher(Node):
    def __init__(self):
        super().__init__('lidar_publisher')

        self.publisher_ = self.create_publisher(Range, '/mavros/distance_sensor/rangefinder_sub', 10)
        self.timer = self.create_timer(0.1, self.publish_distance)
        self.bus = smbus2.SMBus(8)  # Jetson I2C bus 8
        self.lidar_address = 0x62
        self.count = 0

        # 校正參數
        self.lidar_offset_cm = 7.0     # Lidar 到機身底部的安裝距離
        self.bias_cm = 8.8             # 實測誤差補償（調整後）
        self.temp_compensation = True   # 啟用溫度補償
        self.reference_temp = 25.0     # 參考溫度（攝氏度）
        self.temp_coefficient = 0.05   # 溫度補償係數（cm/°C）

        # 移動平均濾波器
        self.filter_window = 5
        self.distance_buffer = []

        self.configure_lidar()

    def configure_lidar(self):
        """設定 Lidar v4 高精度模式"""
        try:
            self.write_register(0xE4, 0x01)  # Reset to default
            time.sleep(0.1)
            
            # 設定電源模式為常開（高精度模式需要）
            self.write_register(0xE2, 0xFF)  # Always on power mode
            time.sleep(0.05)
            
            # 啟用高精度模式 - 平均32次測量
            self.write_register(0xEB, 0x20)  # High accuracy mode (avg 32 samples)
            time.sleep(0.05)
            
            # 設定最大採集次數
            self.write_register(0x05, 0xFF)  # Max acquisition count
            time.sleep(0.05)
            
            # 禁用快速終止以獲得更準確測量
            self.write_register(0xE5, 0x00)  # Disable quick termination
            time.sleep(0.05)
            
            # 設定檢測敏感度
            self.write_register(0x1C, 0x40)  # Set detection sensitivity
            time.sleep(0.05)
            
            self.get_logger().info("[LIDAR] High accuracy mode configured successfully")
            
        except Exception as e:
            self.get_logger().error(f"[LIDAR] Configuration failed: {e}")

    def write_register(self, reg, value):
        """寫入暫存器"""
        try:
            self.bus.write_byte_data(self.lidar_address, reg, value)
            self.get_logger().debug(f"[LIDAR] Set register 0x{reg:02X} = 0x{value:02X}")
        except Exception as e:
            self.get_logger().error(f"[LIDAR] Register 0x{reg:02X} write failed: {e}")

    def read_temperature(self):
        """讀取板子溫度"""
        try:
            temp = self.bus.read_byte_data(self.lidar_address, 0xE0)
            # 轉換為有符號數值（8位元補數）
            if temp > 127:
                temp = temp - 256
            return float(temp)
        except Exception as e:
            self.get_logger().debug(f"[LIDAR] Temperature read failed: {e}")
            return None

    def apply_temperature_compensation(self, distance, temp):
        """根據溫度調整距離"""
        if temp is not None and self.temp_compensation:
            compensation = (temp - self.reference_temp) * self.temp_coefficient
            compensated_distance = distance + compensation
            self.get_logger().debug(f"[LIDAR] Temp: {temp}°C, Compensation: {compensation:.2f}cm")
            return compensated_distance
        return distance

    def check_measurement_status(self):
        """檢查測量狀態"""
        try:
            status = self.bus.read_byte_data(self.lidar_address, 0x01)
            busy = status & 0x01
            signal_overflow = (status & 0x02) >> 1
            reference_overflow = (status & 0x04) >> 2
            low_power = (status & 0x08) >> 3
            dc_bias_done = (status & 0x10) >> 4
            dc_error = (status & 0x20) >> 5
            
            if dc_error:
                self.get_logger().warn("[LIDAR] DC error detected - measurements may be inaccurate")
            if signal_overflow:
                self.get_logger().debug("[LIDAR] Signal overflow detected")
            if reference_overflow:
                self.get_logger().debug("[LIDAR] Reference overflow detected")
            if low_power:
                self.get_logger().debug("[LIDAR] Device in low power mode")
                
            # 返回測量是否有效（沒有嚴重錯誤）
            return not dc_error
            
        except Exception as e:
            self.get_logger().debug(f"[LIDAR] Status check failed: {e}")
            return True  # 如果無法讀取狀態，假設正常

    def apply_moving_average_filter(self, new_distance):
        """應用移動平均濾波器"""
        self.distance_buffer.append(new_distance)
        
        # 保持緩衝區大小
        if len(self.distance_buffer) > self.filter_window:
            self.distance_buffer.pop(0)
        
        # 計算移動平均
        if len(self.distance_buffer) >= 3:  # 至少需要3個數據點
            return sum(self.distance_buffer) / len(self.distance_buffer)
        else:
            return new_distance

    def wait_for_measurement_complete(self):
        """等待測量完成"""
        timeout = 0.1  # 100ms timeout
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            try:
                status = self.bus.read_byte_data(self.lidar_address, 0x01)
                busy = status & 0x01
                if not busy:
                    return True
                time.sleep(0.005)  # 5ms
            except:
                time.sleep(0.005)
                
        self.get_logger().warn("[LIDAR] Measurement timeout")
        return False

    def publish_distance(self):
        """發布距離測量結果"""
        raw = self.read_lidar_distance()
        
        if raw is not None and raw > 5:  # 忽略低於 5cm 的錯誤讀值
            # 檢查測量狀態
            if not self.check_measurement_status():
                self.get_logger().debug("[LIDAR] Skipping measurement due to status error")
                return
                
            # 溫度補償
            temp = self.read_temperature()
            temp_compensated = self.apply_temperature_compensation(raw, temp)
            
            # 原有校正邏輯
            corrected = max(0.0, temp_compensated - self.lidar_offset_cm + self.bias_cm)
            
            # 移動平均濾波
            filtered = self.apply_moving_average_filter(corrected)
            
            # 計算平均值和實際對地高度
            avg = (temp_compensated + filtered) / 2.0
            est_altitude_cm = filtered + self.lidar_offset_cm  # 實際對地高度

            # 終端輸出資訊
            temp_str = f"temp={temp:.1f}°C" if temp is not None else "temp=N/A"
            print(f"raw={raw:.1f} cm | temp_comp={temp_compensated:.1f} cm | "
                  f"corrected={corrected:.1f} cm | filtered={filtered:.1f} cm | "
                  f"avg={avg:.1f} cm | est_altitude={est_altitude_cm:.1f} cm | {temp_str}")
            
            self.count += 1

            # 發佈 ROS Range 訊息（轉換為公尺）
            msg = Range()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'lidar_down'
            msg.radiation_type = Range.INFRARED
            msg.field_of_view = 0.0477  # 4.77 degrees from spec
            msg.min_range = 0.05
            msg.max_range = 10.0
            msg.range = filtered / 100.0  # 使用濾波後的結果
            self.publisher_.publish(msg)

    def read_lidar_distance(self):
        """讀取 LiDAR 距離"""
        try:
            # 使用帶偏差校正的測量命令
            self.bus.write_byte_data(self.lidar_address, 0x00, 0x04)
            
            # 等待測量完成
            if not self.wait_for_measurement_complete():
                return None
            
            # 讀取距離數據
            low = self.bus.read_byte_data(self.lidar_address, 0x10)
            high = self.bus.read_byte_data(self.lidar_address, 0x11)
            distance = float((high << 8) + low)

            # 範圍檢查
            if distance < 5.0 or distance > 1000.0:
                self.get_logger().debug(f"[LIDAR] Out-of-range reading: {distance:.1f} cm")
                return None

            # 檢查是否為無效測量（1cm 表示無效）
            if distance <= 1.0:
                self.get_logger().debug("[LIDAR] Invalid measurement (signal too weak)")
                return None

            return distance
            
        except Exception as e:
            self.get_logger().error(f"[LIDAR] Read failed: {e}")
            return None

    def get_device_info(self):
        """獲取設備資訊"""
        try:
            # 讀取硬體版本
            hw_version = self.bus.read_byte_data(self.lidar_address, 0xE1)
            
            # 讀取韌體版本
            fw_low = self.bus.read_byte_data(self.lidar_address, 0x72)
            fw_high = self.bus.read_byte_data(self.lidar_address, 0x73)
            fw_version = (fw_high << 8) + fw_low
            
            self.get_logger().info(f"[LIDAR] Hardware version: 0x{hw_version:02X}")
            self.get_logger().info(f"[LIDAR] Firmware version: 0x{fw_version:04X}")
            
        except Exception as e:
            self.get_logger().warn(f"[LIDAR] Could not read device info: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = LidarPublisher()
    
    # 顯示設備資訊
    node.get_device_info()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[INFO] Lidar node interrupted. Exiting safely.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from mavros_msgs.msg import DistanceSensor
import smbus2
import time, os
from datetime import datetime, timezone, timedelta
import ntplib


# Create an NTP client and try to get the time from the NTP server
try:
    client = ntplib.NTPClient()
    taipei_timezone = timezone(timedelta(hours=8))
    response = client.request('pool.ntp.org')
    dt_utc = datetime.fromtimestamp(response.tx_time, timezone.utc)
    dt_taipei = dt_utc.astimezone(taipei_timezone)
except (ntplib.NTPException, OSError) as e:
    print("Unable to obtain network time, use system time instead")
    taipei_timezone = timezone(timedelta(hours=8))
    dt_taipei = datetime.now(taipei_timezone)

def writeData(path:str, data):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'a') as file:
        file.write(data + '\n')


# Format time (format: year, month, day, hour, minute, second)
formatted_time = dt_taipei.strftime('%Y_%m%d_%H%M%S')
txt_path = f"/home/ubuntu/track/track2/lidar/distances_{formatted_time}.txt"

class LidarPublisher(Node):
    def __init__(self):
        super().__init__('lidar_publisher')
        # This publisher is for MAVLink's distance_sensor message
        self.publisher_ = self.create_publisher(DistanceSensor, '/mavros/distance_sensor/rangefinder_sub', 10)
        self.timer = self.create_timer(0.1, self.publish_distance)  # 10Hz
        self.bus = smbus2.SMBus(8)  # I2C Bus number may vary
        self.lidar_address = 0x62
        self.path = txt_path
        self.count = 0
        
    def publish_distance(self):
        distance = self.read_lidar_distance()
        if distance is not None:
            # Create MAVLink DistanceSensor message
            msg = DistanceSensor()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'lidar_frame'
            msg.radiation_type = DistanceSensor.TYPE_LASER
            msg.field_of_view = 0.0477  # Beam divergence in radians
            msg.min_distance = 0.05  # 5 cm minimum range
            msg.max_distance = 10.0  # 10 meters maximum range
            msg.distance = distance / 100.0  # Convert cm to meters
            msg.sensor_id = 1  # Set a unique sensor ID
            self.publisher_.publish(msg)

    def read_lidar_distance(self):
        try:
            # Write to register 0x00 to trigger distance measurement
            self.bus.write_byte_data(self.lidar_address, 0x00, 0x04)
            time.sleep(0.02)
            
            # Read the distance from registers 0x10 (low) and 0x11 (high)
            low_byte = self.bus.read_byte_data(self.lidar_address, 0x10)
            high_byte = self.bus.read_byte_data(self.lidar_address, 0x11)
            distance = (high_byte << 8) + low_byte
            wData = f"{self.count}: {distance}cm"
            print(f"alt: {wData}")
            writeData(self.path, wData)
            self.count += 1
            return distance
        except Exception as e:
            self.get_logger().error(f"Error reading from LIDAR: {e}")
            return None

def main(args=None):
    rclpy.init(args=args)
    lidar_publisher = LidarPublisher()
    rclpy.spin(lidar_publisher)
    lidar_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

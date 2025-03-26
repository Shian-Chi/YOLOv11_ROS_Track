import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import ReliabilityPolicy, QoSProfile
from sensor_msgs.msg import Imu
from transforms3d import euler

import sys, time, math
import threading

try:
    from ctrl.pid.PID_Calc import PID_Ctrl
    from ctrl.pid.motor import motorCtrl, motorInitPositions
except:
    from pid.PID_Calc import PID_Ctrl
    from pid.motor import motorCtrl, motorInitPositions
    
import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))
from utils import JsonHandler, check_file

from tutorial_interfaces.msg import Bbox, MotorInfo

json_config_file = check_file("config.json")
json_config = JsonHandler(json_config_file)
VIDEO_WIDTH = json_config.get(["video_resolutions", "default", "width"])
VIDEO_HEIGHT = json_config.get(["video_resolutions", "default", "height"])
yawInitPos = json_config.get(["motorInitPosition", "yaw"])
pitchInitPos = json_config.get(["motorInitPosition", "pitch"])
trackErrorRange = json_config.get(["trackErrorRange", "default"])
uintDegreeEn = json_config.get(["encoder", "uintDegreeEncoder"])
fov = json_config.get(["camera", "FOV"])
trackMode = json_config.get(["trackMode", "default"])
object_size = json_config.get(["trackMode", "size"])

json_pub_file = check_file("publish.json")
json_publish = JsonHandler(json_pub_file)
pub_bbox = json_publish.get(["pub_bbox"])
pid = PID_Ctrl()
yaw = motorCtrl(1, "yaw", 90.0)
pitch = motorCtrl(2, "pitch", 360.0)


class CenterBasedCalculator:
    """
    使用目標中心 (x_center, y_center) 來精確計算方向角和距離，完全符合推導公式。

    Attributes:
        HFOV (float): 水平視角 (θFOV)，單位度 (degree)。
        VFOV (float): 垂直視角 (φFOV)，單位度 (degree)。
        image_width (int): 影像寬度 (Wx)。
        image_height (int): 影像高度 (Hy)。
    """

    def __init__(self, HFOV, VFOV, image_width, image_height):
        """
        初始化 CenterBasedCalculator。

        Args:
            HFOV (float): 水平視角 (度)。
            VFOV (float): 垂直視角 (度)。
            image_width (int): 影像寬度。
            image_height (int): 影像高度。
        """
        self.HFOV = HFOV       # θFOV: 水平視角
        self.VFOV = VFOV       # φFOV: 垂直視角
        self.image_width = image_width    # Wx: 影像寬度
        self.image_height = image_height  # Hy: 影像高度

    def calc_horizontal_angle(self, pixel_x):
        """
        使用精確的 arctan-tan 公式計算水平方向角度 (度)。
        
        公式: θ = tan^(-1)[(2x/Wx - 1) * tan(θFOV/2)]

        Args:
            pixel_x (float): 目標在影像中的 x 座標 (像素)。

        Returns:
            float: 水平角度 (度)。
        """
        norm_x = (2 * pixel_x / self.image_width) - 1
        theta = math.atan(
            norm_x * math.tan(math.radians(self.HFOV / 2))
        )
        return math.degrees(theta)

    def calc_vertical_angle(self, pixel_y):
        """
        使用精確的 arctan-tan 公式計算垂直方向角度 (度)。
        
        公式: φ = tan^(-1)[(2y/Hy - 1) * tan(φFOV/2)]

        Args:
            pixel_y (float): 目標在影像中的 y 座標 (像素)。

        Returns:
            float: 垂直角度 (度)。
        """
        norm_y = (2 * pixel_y / self.image_height) - 1
        phi = math.atan(
            norm_y * math.tan(math.radians(self.VFOV / 2))
        )
        return math.degrees(phi)

    def calc_visual_line_distance(self, actual_width, W_image_px):
        """
        計算視線距離 Dc，使用精確公式:
        Dc = (wobj * Wx) / (2W * tan(θFOV/2))

        其中:
        - wobj: 物體實際寬度
        - Wx: 影像寬度 (像素)
        - W: bounding box 寬度 (像素)
        - θFOV: 水平視角

        Args:
            actual_width (float): 物體實際寬度。
            W_image_px (float): 物體在影像中的寬度 (像素)。

        Returns:
            float or None: 視線距離 Dc。如果 W_image_px <= 0，回傳 None。
        """
        if W_image_px <= 0:
            return None

        Dc = (actual_width * self.image_width) / (
            2 * W_image_px * math.tan(math.radians(self.HFOV / 2))
        )
        return Dc

    def calc_3d_position_by_center(self, actual_width, x1, y1, x2, y2):
        """
        使用精確數學模型計算 3D 位置:
        1. 使用中心點 (x, y) 計算方向角 (θ, φ)
        2. 使用 bounding box 寬度計算視線距離 Dc
        3. 使用餘弦定理計算實際距離 Dobj
        4. 轉換至相機座標系 (x_c, y_c, z_c)

        Args:
            actual_width (float): 物體實際寬度。
            x1 (float): bounding box 左上角的 x 座標 (像素)。
            y1 (float): bounding box 左上角的 y 座標 (像素)。
            x2 (float): bounding box 右下角的 x 座標 (像素)。
            y2 (float): bounding box 右下角的 y 座標 (像素)。

        Returns:
            dict or None: 包含以下鍵值的字典，如計算失敗回傳 None。
                - distance_visual (float): 視線距離 Dc (鏡頭到目標的直線距離)。
                - distance_actual (float): 實際距離 Dobj (鏡頭到目標的距離)。
                - theta_deg (float): 水平角 (度)。
                - phi_deg (float): 垂直角 (度)。
                - x_c (float): 相機座標系中的 X 。
                - y_c (float): 相機座標系中的 Y 。
                - z_c (float): 相機座標系中的 Z 。
        """
        # 1) 計算物體中心點
        center_x = (x1 + x2) / 2
        center_y = (y1 + y2) / 2
        
        # 2) 計算水平和垂直方向角(度)
        theta_deg = self.calc_horizontal_angle(center_x)
        phi_deg = self.calc_vertical_angle(center_y)
        
        # 3) 使用 bounding box 寬度計算視線距離
        W_image_px = x2 - x1
        Dc = self.calc_visual_line_distance(actual_width, W_image_px)
        
        if Dc is None:
            return None
        
        # 4) 轉換角度至弧度
        theta_rad = math.radians(theta_deg)
        phi_rad = math.radians(phi_deg)
        
        # 5) 計算實際距離 Dobj = Dc / (cos(φ) * cos(θ))
        Dobj = Dc / (math.cos(phi_rad) * math.cos(theta_rad))
        
        # 6) 轉換至相機坐標系
        x_c = Dobj * math.cos(phi_rad) * math.cos(theta_rad)
        y_c = Dobj * math.cos(phi_rad) * math.sin(theta_rad)
        z_c = Dobj * math.sin(phi_rad)
        
        return {
            'distance_visual': Dc,    # 視線距離
            'distance_actual': Dobj,  # 實際距離
            'theta_deg': theta_deg,   # 水平角(度)
            'phi_deg': phi_deg,       # 垂直角(度)
            'x_c': x_c,               # 相機坐標系 X
            'y_c': y_c,               # 相機坐標系 Y
            'z_c': z_c                # 相機坐標系 Z
        }

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.imuSub = self.create_subscription(Imu, 'mavros/imu/data', self.IMUcb, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.imuSub  # prevent unused variable warning
        
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
    def IMUcb(self, msg :Imu):
        ned_euler_data = euler.quat2euler([msg.orientation.w,
                                        msg.orientation.x,
                                        msg.orientation.y,
                                        msg.orientation.z])
        self.pitch = math.degrees(ned_euler_data[0])
        self.roll = math.degrees(ned_euler_data[1])
        self.yaw = math.degrees(ned_euler_data[2])

class GimbalPublish(Node):
    def __init__(self):
        super().__init__('gimbal_publisher')
        # Read motor Info
        self.motorInfoPublish = self.create_publisher(MotorInfo, "motor_info", 10)
        self.motor_timer = self.create_timer(1/20, self.motor_callback)

        self.motorInfo = MotorInfo()
        
    def motor_callback(self):
        # Yaw
        ye = int(yaw.info.getEncoder()) 
        ya = float((yaw.info.getAngle()))
        self.motorInfo.yaw_pluse =  ye
        self.motorInfo.yaw_angle = ya
        # Pitch
        pe = int(pitch.info.getEncoder())
        pa = float(pitch.info.getAngle())
        self.motorInfo.pitch_pluse = pe
        self.motorInfo.pitch_angle = pa
        
        self.motorInfoPublish.publish(self.motorInfo)
        
def getGimbalEncoders():
    Y_ret, Y_Encoder= yaw.getEncoder()
    P_ret, P_Encoder= pitch.getEncoder()
    return Y_Encoder, P_Encoder

class GimbalTimerTask(Node):
    def __init__(self, mode='pid', obj_size=1.6, HFOV=None, VFOV=None):
        super().__init__('gimbal_timer_task')
        self.__gimbal_init__()

        self.mode = mode
        
        self.error_range = trackErrorRange # 允許誤差範圍(百分比)
        # 設定畫面「理想中心點」的 (x, y) 座標, 取影像的寬高各一半，即可得到影像的中心位置
        self.img_center_x, self.img_center_y = VIDEO_WIDTH / 2, VIDEO_HEIGHT / 2
        
        # 允許的「中心誤差」距離閾值
        # 這裡取「畫面短邊」(寬或高中較小者)的 7% ；若超過此距離則視為未居中
        # 例: 若畫面為 1280 x 720，則 min(1280,720)=720；720*error_range=144 像素
        self.allowable_distance = min(VIDEO_WIDTH, VIDEO_HEIGHT) * self.error_range
        # 寬度方向允許誤差： (影片寬度的一半) * 誤差比例
        self.width_error_range = VIDEO_WIDTH / 2 * self.error_range
        # 高度方向允許誤差： (影片高度的一半) * 誤差比例
        self.height_error_range = VIDEO_HEIGHT / 2 * self.error_range
        
        # ROS Tasks
        self.gimbal_task = self.create_timer(1 / 15, self.gimdal_ctrl)

        self.yaw = yaw
        self.pitch = pitch
        
        # 讀取雲台角度及發佈
        self.publish = GimbalPublish()
        self.subscribe = MinimalSubscriber()
        self._spin_thrd = threading.Thread(target=self._ros_spin, args=(self.publish, self.subscribe))
        self._spin_thrd.start()
        
        if self.mode == "deg":
            if obj_size is None or obj_size <= 0 \
                or HFOV is None or VFOV is None:
                raise ValueError("Object size or FOV must be a positive number.")
        self.object_size = obj_size
        print(f"Tracking mode: {self.mode}")
        
        self.visual_ranging = CenterBasedCalculator(HFOV, VFOV, 1280, 720)
        self.D_c, self.D_obj = None, None
        self.detect = False
        self.detect_countuers = 0
        self.center_status = False
        self.xyxy = [0, 0, 0, 0]
        self.center_x, self.center_y = 0, 0
        self.output_deg = [0, 0]
        self.threeD_data = {
            'distance_visual': None,    # 視線距離
            'distance_actual': None,  # 實際距離
            'theta_deg': None,   # 水平角(度)
            'phi_deg': None,       # 垂直角(度)
            'x_c': None,               # 相機坐標系 X
            'y_c': None,               # 相機坐標系 Y
            'z_c': None                # 相機坐標系 Z
        }
        self.pitchEncoder, self.yawEncoder = 0, 0
        self.pitchAngle, self.yawAngle = 0.0, 0.0
        self.centerDistance = None # 預設距離
        self.motor_running = False
    
    def detect_count(self, status):
        if status:
            self.detect_countuers +=1
        else:
            self.detect_countuers = 0
    
    def get_angle(self):
        return yaw.info.getAngle(), pitch.info.getAngle()
    
    def _ros_spin(self, pub, sub):
        executor = MultiThreadedExecutor()
        executor.add_node(pub)
        executor.add_node(sub)
        executor.spin()
    
    def move_deg_calc(self):
        if self.mode == 'deg':
            if self.xyxy is None:
                return 0, 0 # No move
            data = self.visual_ranging.calc_3d_position_by_center(self.object_size, *self.xyxy)
            
            print(f"mode: deg, output: {data['theta_deg']}, {data['phi_deg']}")
            self.output = [data['theta_deg'], data['phi_deg']]
        else:
            if self.xyxy is None:
                return 0, 0 # No move
            # 計算 x,y 的中心位置
            self.output = self.PID_calc_angle()
            self.threeD_data = self.visual_ranging.calc_3d_position_by_center(self.object_size, *self.xyxy)
            
        self.xyxy = None
        return self.output
    
    def gimbal_stabilization(self, yaw_val:int, pitch_val:int):
        """_summary_

        Args:
            yaw_val (int): Yaw motor degrees data
            pitch_val (int): Pitch motor degrees data

        Returns:
            Bool: Recv motor echo data
        """
        mini_uav_pitch = max(min(self.subscribe.pitch, 1.0), -1.0)
        uav_pitch_val = int(mini_uav_pitch * 100)

        # Ensure yaw and pitch references exist
        y_ret = self.yaw.incrementTurnVal(yaw_val)
        p_ret = self.pitch.incrementTurnVal(pitch_val + uav_pitch_val)
        return y_ret, p_ret
    
    def gimdal_ctrl(self):
        self.distance = 9999 # 預設距離
        y_ret, p_ret = None, None
        
        # 連續辨識到4次才會開始追蹤
        if self.detect_countuers > 3:
            # 計算移動角度
            self.output_deg = self.move_deg_calc()
            # Motor rotation
            y_val = int(self.output_deg[0] * 100)
            p_val = int(self.output_deg[1] * 100)
            y_ret, p_ret = self.gimbal_stabilization(y_val, p_val)
            self.centerDistance = ((self.center_x - self.img_center_x) ** 2 + (self.center_y - self.img_center_y) ** 2) ** 0.5
        else: # 不滿4次
            self.output_deg = [0.0, 0.0]
            # 使用歐幾里得距離(Euclidean distance)來判斷 目標中心 與 畫面中心 相距多少
            # 其中 self.center_x, self.center_y 為「偵測到的目標中心座標」
            self.detect = False
            self.center_status = False
            self.centerDistance = None
            return 
        
        # 比較算出的距離與允許距離(self.allowable_distance), 若距離小於等於允許誤差，則視為「目標已經居中」
        self.center_status = self.centerDistance <= self.allowable_distance
        self.motor_running = y_ret and p_ret
        
    def PID_calc_angle(self):
        # 計算 x,y 的中心位置
        self.center_x, self.center_y = (self.xyxy[0] + self.xyxy[2]) / 2, (self.xyxy[1] + self.xyxy[3]) / 2
        pid_output, err = pid.pid_run(self.center_x, self.center_y)
        return pid_output
    
    def xyxy_update(self, detect: bool, x0=0, y0=0, x1=0, y1=0):
        self.detect = detect
        if self.detect:
            self.xyxy = [x0, y0, x1, y1]
        else:
            self.xyxy = None
        self.detect_count(self.detect)
        
    def close(self):
        """
        關閉雲台計時器 & 自己的 Executor 執行緒。
        *不要*在這裡直接呼叫 destroy_node() 或 rclpy.shutdown()，以免造成衝突。
        """
        yaw.stop()
        pitch.stop()
        
        # 1) 先銷毀自己在這裡建立的 timer
        if self.gimbal_task:
            self.destroy_timer(self.gimbal_task)

        # 2) 從 executor 移除這些 node，讓 spin() 自動結束
        if self.publish in self.executor.get_nodes():
            self.executor.remove_node(self.publish)
        if self in self.executor.get_nodes():
            self.executor.remove_node(self)

        # 3) 關閉 executor
        self.executor.shutdown()

        # 4) 等 spin thread 結束
        if self._spin_thrd.is_alive():
            self._spin_thrd.join()

        print("[GimbalTimerTask] close() done.")
        
    def __gimbal_init__(self):
        print(f"Motor init postion: yaw: {yawInitPos}, pitch: {pitchInitPos}")
        motorInitPositions(yaw, yawInitPos)
        time.sleep(1)
        motorInitPositions(pitch, pitchInitPos)

def main_center_demo():
    HFOV = 53.6  # θFOV: 水平視角
    VFOV = 33.1  # φFOV: 垂直視角
    image_width = 1280
    image_height = 720
    actual_width = 11.0  # 目標真實寬度 (cm)

    # Capture
    # import cv2
    # from detect.camera_function import gstreamer_pipeline

    # cap = cv2.VideoCapture(gstreamer_pipeline(0, image_width, image_height))

    # 初始化 "中心點法" 計算器
    center_calc = CenterBasedCalculator(HFOV, VFOV, image_width, image_height)

    # 假設 bounding box
    # bbox = [[340, 520, 260, 460]]  # (x1, x2, y1, y2)
    bbox = [[948.27, 351.78, 1191, 567.69]] # (x1, x2, y1, y2)
        
    # 測試
    # from ultralytics import YOLO
    # pt_file = check_file(r'/home/ubuntu/track/track2/detect/landpadv11.pt')
    # model = YOLO(pt_file)
    # print("YOLO detected")
    try:
        for i in range(20):
            # ret, frame = cap.read()
            # if not ret:
            #     print("Failed to capture image")
            #     return

            # results = model.predict(frame, conf=0.4)
            # if len(results[0]) == 0:
            #     continue
            # bbox = results[0].boxes.xyxy.cpu().numpy()
            # x0, y0, x1, y1 = bbox[0]
            # # Calculate 3D position
            # if len(bbox) == 0:
            #     print("No object detected.")
            #     continue
            
            # print(f"Bbox{bbox[0]}")
            result = center_calc.calc_3d_position_by_center(actual_width, *bbox[0])
            # if result is None:
            #     print("Bounding box invalid (width <= 0).")
            #     # return
            # output = [result['theta_deg'], result['phi_deg']]
                
            # print(f"output angle: {output}")
            # # Motor rotation
            # yaw.incrementTurnVal(int(output[0] * 100))
            # pitch.incrementTurnVal(int(output[1] * 100))
            # time.sleep(0.1)
            
            # # Save image
            # cv2.rectangle(frame, (int(x0), int(y0)), (int(x1), int(y1)), (0, 255, 0), 2)

            # cv2.imwrite(f"/home/ubuntu/track/track2/detect/output{count}.jpg", frame)  # Save image as output.jpg
            # print(f"Image saved as 'output{count}.jpg'")
            # count += 1
            
            # Print information
            print("[Center-based] 3D Position in Camera Frame:")
            print(f" X_c= {result['x_c']:.2f},  Y_c= {result['y_c']:.2f},  Z_c= {result['z_c']:.2f} cm")
            print(f" distance_visual= {result['distance_visual']:.2f} cm,  theta= {result['theta_deg']:.2f}°,  phi= {result['phi_deg']:.2f}°")
            break
    finally:
        yaw.stop()
        pitch.stop()
        
if __name__ == "__main__":
    main_center_demo()
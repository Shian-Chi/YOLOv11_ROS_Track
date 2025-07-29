import sys, time
import threading
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Int32MultiArray, Float32
from ctrl.pid.PID_Calc import PID_Ctrl
from ctrl.pid.motor import motorCtrl, motorInitPositions
from parameter import Parameter
from tutorial_interfaces.msg import GimbalDegree

# ---------- 參數設定 ----------
para = Parameter()
VIDEO_WIDTH  = para.VIDEO_WIDTH
VIDEO_HEIGHT = para.VIDEO_HEIGHT
yawInitPos   = para.motor_init_position_yaw
pitchInitPos = para.motor_init_position_pitch
trackErrorRange = para.track_error_range
uintDegreeEn = para.uint_degree_encoder
trackMode    = para.track_mode
object_size  = para.object_size
authSuccessCount = para.auth_success_count
HFOV = para.horizontal_fov
VFOV = para.vertical_fov

# ---------- 初始化馬達 ----------
y_pid   = PID_Ctrl(kp=0.0064, ki=0.00000025, kd=0.0000003, set_point=VIDEO_WIDTH/2)  # 畫面水平
p_pid   = PID_Ctrl(kp=0.0051, ki=0.00000025, kd=0.0000003, set_point=VIDEO_HEIGHT/2) # 畫面垂直
yaw   = motorCtrl(1, "yaw",   90.0)
pitch = motorCtrl(2, "pitch", 360.0)


# ================== CenterBasedCalculator  ==================
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

# ============================================================


class Gimbal_Node(Node):
    def __init__(self):
        super().__init__('gimbal_node')

        # ------ 狀態 ------
        self.yaw   = 0.0
        self.pitch = 0.0
        self.roll  = 0.0
        self.detectSTAT = False
        self.target_xyxy = [0, 0, 1280, 720]

        # ------ Publisher ------
        self.gimbal_pub = self.create_publisher(GimbalDegree, '/gimbal_angle', 1)
        self.iscenter_pub = self.create_publisher(Bool, '/targetCenter', 1)
        self.threeD_pos_Visual_pub = self.create_publisher(Float32, '/threeD_position/distanceVisual', 1)
        self.threeD_deg_Actual_pub = self.create_publisher(Float32, '/threeD_position/distanceActual', 1)
        self.threeD_deg_H_pub = self.create_publisher(Float32, '/threeD_position/horizontalDeg', 1)
        self.threeD_deg_V_pub = self.create_publisher(Float32, '/threeD_position/verticalDeg', 1)

        # ------ QoS ------
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        # ------ Subscriber ------
        self.center_xyxySub = self.create_subscription(Int32MultiArray, '/target_xyxy', self.target_xyxy_cb, qos)
        self.detectSub = self.create_subscription(Bool, '/target_detected', self.detect_cb, qos)

    # ---------- Publishers ----------
    def publish_gimbal_angle(self, yaw: float, pitch: float):
        msg = GimbalDegree()
        msg.yaw   = yaw
        msg.pitch = pitch
        # self.get_logger().debug(f"PUB angle  yaw={yaw:.2f}, pitch={pitch:.2f}")
        self.gimbal_pub.publish(msg)

    def publish_center_status(self, is_centered: bool):
        self.iscenter_pub.publish(Bool(data=is_centered))

    def publish_threeD_position(self, distance_visual: float, distance_actual: float):
        msg = Float32()
        msg.data = distance_visual
        self.threeD_pos_Visual_pub.publish(msg)

        msg.data = distance_actual
        self.threeD_deg_Actual_pub.publish(msg)
        
    def publish_threeD_deg(self, horizontal_deg: float, vertical_deg: float):
        msg = Float32()
        msg.data = horizontal_deg
        self.threeD_deg_H_pub.publish(msg)

        msg.data = vertical_deg
        self.threeD_deg_V_pub.publish(msg)
    
    # ---------- Callbacks ----------
    def detect_cb(self, msg: Bool):
        self.detectSTAT = msg.data

    def target_xyxy_cb(self, msg: Int32MultiArray):
        self.target_xyxy = list(msg.data) # msg.data -> ('i', [0, 0, 0, 0])

def getGimbalEncoders():
    _, yaw_encoder   = yaw.getEncoder()
    _, pitch_encoder = pitch.getEncoder()
    return yaw_encoder, pitch_encoder

class GimbalTask:
    def __init__(self, ros_node: Gimbal_Node, mode='pid',
                 obj_size=1.6, HFOV=None, VFOV=None):

        self.ros_node = ros_node
        self.mode = mode.lower()           # 'pid' or 'deg'
        self.yaw = yaw
        self.pitch = pitch

        # ------- 誤差區域 -------
        self.error_range = trackErrorRange
        self.img_center_x, self.img_center_y = VIDEO_WIDTH/2, VIDEO_HEIGHT/2
        self.allowable_distance = min(VIDEO_WIDTH, VIDEO_HEIGHT) * self.error_range
        self.width_error_range  = VIDEO_WIDTH/2  * self.error_range
        self.height_error_range = VIDEO_HEIGHT/2 * self.error_range

        # ------- 3D 推算器 -------
        if self.mode == "deg":
            if obj_size <= 0 or HFOV is None or VFOV is None:
                raise ValueError("Object size & FOV 必須為正且不可 None")
        self.visual_ranging = CenterBasedCalculator(HFOV, VFOV, VIDEO_WIDTH, VIDEO_HEIGHT)
        self.object_size = obj_size

        # ------- 內部狀態 -------
        self.detect_counters = 0
        self.center_status   = False
        self.xyxy = None
        self.output_deg = [0.0, 0.0]
        self.threeD_data = dict.fromkeys(
            ['distance_visual', 'distance_actual', 'theta_deg',
             'phi_deg', 'x_c', 'y_c', 'z_c'], None)

    # ------------ 外部接口 ------------
    def xyxy_update(self, detect: bool, xyxy:list=[0,0,1280,720]):
        self.detect = detect
        self.xyxy   = xyxy if detect else None
        self.detect_counters = (self.detect_counters + 1) if detect else 0

    # ------------ 控制程式 ------------
    def gimbal_ctrl(self):
        self.xyxy_update(self.ros_node.detectSTAT, self.ros_node.target_xyxy)
        # print(f"target xyxy {self.ros_node.target_xyxy}")
        if self.detect_counters <= authSuccessCount or self.xyxy is None:
            return  # 尚未累積到可信次數，不動作
        # print(f"Detected target at: {self.xyxy}")
        self.output_deg = self._calc_output_deg()
        # print("PID output:",self.output_deg)
        # ------------------- 發送給馬達 -------------------
        y_val = int(self.output_deg[1][0] * 100)
        p_val = int(self.output_deg[1][1] * 100)
        self._send_angle_to_gimbal(y_val, p_val)
        # ------------------- 是否居中 -------------------
        cx, cy = (self.xyxy[0]+self.xyxy[2])/2, (self.xyxy[1]+self.xyxy[3])/2
        center_dist = math.hypot(cx-self.img_center_x, cy-self.img_center_y)
        self.center_status = center_dist <= self.allowable_distance
        # ---------- 發佈 ROS Topic ----------
        self._rosPub_()

    def _calc_output_deg(self):
        threeD_data_calc = self.visual_ranging.calc_3d_position_by_center(
                            self.object_size, *self.xyxy)
        self.threeD_data.update(threeD_data_calc)
        
        cx, cy = (self.xyxy[0]+self.xyxy[2])/2, (self.xyxy[1]+self.xyxy[3])/2
        pid_y_val = y_pid.pid_run(cx)
        pid_p_val = p_pid.pid_run(cy)
        
        return [
                [threeD_data_calc['theta_deg'], threeD_data_calc['phi_deg']],
                [pid_y_val, pid_p_val]
                ]

    def _send_angle_to_gimbal(self, yaw_val: int, pitch_val: int):
        p_ret = self.pitch.incrementTurnVal(pitch_val)
        if self.pitch.info.getAngle() > 90.0:
            return False, p_ret
        y_ret = self.yaw.incrementTurnVal(yaw_val)
        return y_ret, p_ret

    def _rosPub_(self):
        self.ros_node.publish_gimbal_angle(self.yaw.info.getAngle(),
                                            self.pitch.info.getAngle())
        self.ros_node.publish_center_status(self.center_status)
        self.ros_node.publish_threeD_position(
            self.threeD_data['distance_visual'],
            self.threeD_data['distance_actual']
        )
        self.ros_node.publish_threeD_deg(
            self.threeD_data['theta_deg'],
            self.threeD_data['phi_deg']
        )
        
    
    # ------------ 關閉 ------------
    def close(self):
        yaw.stop()
        pitch.stop()
        

if __name__ == '__main__':
    rclpy.init(args=None)
    ros_node = Gimbal_Node()
    ros_thread = threading.Thread(target=rclpy.spin, args=(ros_node, ), daemon=True)
    ros_thread.start()
    
    gimbal_task = GimbalTask(ros_node, mode=trackMode, obj_size=object_size, HFOV=HFOV, VFOV=VFOV)
    ros_node.get_logger().info("Gimbal tracking node started.")

    motorInitPositions(yaw, yawInitPos)
    motorInitPositions(pitch, pitchInitPos)
    
    try:
        while rclpy.ok():
            gimbal_task.gimbal_ctrl()
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print("Error:", e)
    finally:
        gimbal_task.close()
        rclpy.shutdown()
        sys.exit(0)
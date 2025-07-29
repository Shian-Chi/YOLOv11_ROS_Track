from typing import Union, Optional, Tuple
from dataclasses import dataclass

class Parameter:
    """
    系統參數配置類
    統一管理所有系統配置參數，不使用字典結構
    """
    
    def __init__(self):
        # ==================== 視頻參數 ====================
        self.VIDEO_WIDTH: int = 1280
        self.VIDEO_HEIGHT: int = 720
        self.VIDEO_FPS: int = 30
        self.VIDEO_CODEC: str = 'H264'
        self.VIDEO_BITRATE: int = 5000000
        
        # ==================== 攝像頭視野參數 ====================
        self.horizontal_fov: float = 54.04  # 水平視野角度
        self.vertical_fov: float = 33.1     # 垂直視野角度
        self.camera_fov = 77.0
        # ==================== YOLO 檢測參數 ====================
        self.yolo_image_size: int = 640     # YOLO 輸入圖像大小
        self.yolo_confidence: float = 0.5
        self.image_show_enabled: bool = False 
        # ==================== 圖像保存參數 ====================        
        self.image_save_enabled = True      # 是否保存圖像
        
        # ==================== 數據保存參數 ====================
        self.data_save_enabled: bool = True    # 是否保存數據
        
        # ==================== 認證參數 ====================
        self.auth_success_count: int = 2       # 認證成功次數
        
        # ==================== 圖像傳輸參數 ====================
        self.img_send_enabled: bool = False             # 是否啟用圖像傳輸
        self.img_send_localhost: str = "127.0.0.1"      # 本地主機地址
        self.img_send_broadcast: str = "10.147.20.60"   # 廣播地址
        self.img_send_port: int = 5001                  # 傳輸端口
        
        # ==================== 馬達參數 ====================
        self.motor_init_position_yaw: float = 0.0  # 馬達初始位置（偏航）
        self.motor_init_position_pitch: float = 45.0
        
        # ==================== 追蹤參數 ====================
        self.track_mode: str = "pid"  # 追蹤模式（pid 或 deg）
        self.track_error_range: float = 0.15   # 追蹤誤差範圍
        
        # ==================== 編碼器參數 ====================
        self.uint_degree_encoder: float = 91.02222222222223  # 度數編碼器單位
        self.rotate_encoder: int = 32768                      # 旋轉編碼器值
        
        # ==================== 物體參數 ====================
        self.object_size: float = 1.6          # 目標物體大小（公尺）

        # ==================== 定位模式 ====================
        self.position_Mode = 'vertical'        # 'vertical' or 'horizontal'
        
    def get_video_resolution(self) -> Tuple[int, int]:
        """獲取視頻解析度"""
        return self.VIDEO_WIDTH, self.VIDEO_HEIGHT
    
    def get_video_config(self) -> dict:
        """獲取完整視頻配置（僅用於外部接口）"""
        return {
            'width': self.VIDEO_WIDTH,
            'height': self.VIDEO_HEIGHT,
            'fps': self.VIDEO_FPS,
            'codec': self.VIDEO_CODEC,
            'bitrate': self.VIDEO_BITRATE
        }
    
    def get_fov_config(self) -> Tuple[float, float]:
        """獲取視野角度配置"""
        return self.horizontal_fov, self.vertical_fov
    
    def get_yolo_config(self) -> Tuple[int, float]:
        """獲取YOLO配置"""
        return self.yolo_image_size, self.yolo_confidence
    
    def get_network_config(self) -> Tuple[str, str, int]:
        """獲取網路配置"""
        return self.img_send_localhost, self.img_send_broadcast, self.img_send_port
    
    def is_image_display_enabled(self) -> bool:
        """檢查是否啟用圖像顯示"""
        return self.image_show_enabled
    
    def is_image_save_enabled(self) -> bool:
        """檢查是否啟用圖像保存"""
        return self.image_save_enabled
    
    def is_data_save_enabled(self) -> bool:
        """檢查是否啟用數據保存"""
        return self.data_save_enabled
    
    def is_image_transmission_enabled(self) -> bool:
        """檢查是否啟用圖像傳輸"""
        return self.img_send_enabled
    
    def set_video_resolution(self, width: int, height: int) -> None:
        """設置視頻解析度"""
        if width > 0 and height > 0:
            self.VIDEO_WIDTH = width
            self.VIDEO_HEIGHT = height
        else:
            raise ValueError("視頻解析度必須大於0")
    
    def set_yolo_confidence(self, confidence: float) -> None:
        """設置YOLO信心閾值"""
        if 0.0 <= confidence <= 1.0:
            self.yolo_confidence = confidence
        else:
            raise ValueError("YOLO信心閾值必須在0.0到1.0之間")
    
    def set_track_error_range(self, error_range: float) -> None:
        """設置追蹤誤差範圍"""
        if error_range > 0:
            self.track_error_range = error_range
        else:
            raise ValueError("追蹤誤差範圍必須大於0")
    
    def enable_image_display(self, enabled: bool = True) -> None:
        """啟用/禁用圖像顯示"""
        self.image_show_enabled = enabled
    
    def enable_image_save(self, enabled: bool = True) -> None:
        """啟用/禁用圖像保存"""
        self.image_save_enabled = enabled
    
    def enable_data_save(self, enabled: bool = True) -> None:
        """啟用/禁用數據保存"""
        self.data_save_enabled = enabled
    
    def enable_image_transmission(self, enabled: bool = True) -> None:
        """啟用/禁用圖像傳輸"""
        self.img_send_enabled = enabled
    
    def set_network_address(self, localhost: str, broadcast: str, port: int) -> None:
        """設置網路地址配置"""
        self.img_send_localhost = localhost
        self.img_send_broadcast = broadcast
        self.img_send_port = port
    
    def reset_to_defaults(self) -> None:
        """重置所有參數到默認值"""
        self.__init__()
    
    def get_summary(self) -> str:
        """獲取參數配置摘要"""
        summary = []
        summary.append("=== 系統參數配置摘要 ===")
        summary.append(f"視頻: {self.VIDEO_WIDTH}x{self.VIDEO_HEIGHT}@{self.VIDEO_FPS}fps")
        summary.append(f"YOLO: {self.yolo_image_size}px, 信心度{self.yolo_confidence}")
        summary.append(f"視野: H{self.horizontal_fov}°, V{self.vertical_fov}°")
        summary.append(f"功能: 顯示={self.image_show_enabled}, 保存={self.image_save_enabled}, 傳輸={self.img_send_enabled}")
        summary.append(f"網路: {self.img_send_broadcast}:{self.img_send_port}")
        summary.append(f"追蹤誤差範圍: {self.track_error_range}")
        return "\n".join(summary)

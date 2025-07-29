import cv2, queue
import threading as thrd

def gstreamer_pipeline_2(
    sensor_id=0,
    capture_width=1280, capture_height=720,
    display_width=1280,  display_height=720,
    framerate=25,       # 保持 25 fps
    flip_method=0):
    return (
        "nvarguscamerasrc sensor-id=%d "
        # ★ 曝光：允許 30–50 ms，介於『最暗』與『正常』之間
        "exposuretimerange=\"30000 50000\" "
        # ★ 類比增益：1–4（最高 ≈ +12 dB），先給一點餘地
        "gainrange=\"1 4\" "
        # ★ 數位增益：1–2，避免 ISP 再把噪訊推太高
        "ispdigitalgainrange=\"1 2\" "
        # ★ 自動曝光保持 ON（aelock=false）才會在範圍內自動
        "aelock=false "
        # ★ 白平衡維持自動（wbmode=0），先不動
        "! video/x-raw(memory:NVMM),width=%d,height=%d,framerate=%d/1 "
        "! nvvidconv flip-method=%d "
        # 不加對比、飽和度，先保持乾淨訊號
        "! video/x-raw,width=%d,height=%d,format=BGRx "
        "! videoconvert "
        "! video/x-raw,format=BGR ! appsink"
        % (sensor_id,
           capture_width, capture_height, framerate,
           flip_method, display_width, display_height)
    )

def gstreamer_pipeline_1(
    sensor_id=0,
    capture_width=1280,
    capture_height=720,
    display_width=1280,
    display_height=720,
    framerate=30,
    flip_method=0,
):
    """
    Generate GStreamer pipeline for camera input with low latency optimization.
    """
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "queue max-size-buffers=1 leaky=downstream ! "  # 限制緩衝區，減少延遲
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! "
        "queue max-size-buffers=1 leaky=downstream ! "  # 再次減少緩衝
        "appsink drop=true sync=false"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )


class Camera:
    def __init__(self, video_src, width=640, height=480, frame_rate=30):
        """
        Initialize the Camera class.
        """
        self.video_src = video_src
        self.width = width
        self.height = height
        self.frame_rate = frame_rate

        self.stop_thread = False
        self.read_thread = None
        self.display_thread = None
        self.consume_thread = None

        # Initialize video capture
        self.cap = cv2.VideoCapture(self.video_src, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            raise ValueError(f"Failed to open video source: {self.video_src}")

        FRAME_QUEUE_SIZE = 1 # 限制隊列大小為1，減少延遲
        self.frame_queue = queue.Queue(maxsize=FRAME_QUEUE_SIZE)

        
    def start(self):
        if self.read_thread is None or not self.read_thread.is_alive():
            self.read_thread = thrd.Thread(target=self._read_frames, daemon=True)
            print("Starting camera read thread...")
            self.read_thread.start()

    def _read_frames(self):
        while not self.stop_thread:
            ret, frame = self.cap.read()
            if not ret:
                print("Failed to read frame.")
                break
            if self.frame_queue.full():
                self.frame_queue.get_nowait()  # 丢弃队列中最旧的一帧
            self.frame_queue.put(frame)
    
    
    def isOpened(self):
        return self.cap.isOpened()
    
    def stop(self):
        self.stop_thread = True
        if self.read_thread and self.read_thread.is_alive():
            self.read_thread.join()
        self.cap.release()
        cv2.destroyAllWindows()
        print("Camera stopped and resources released.")
        
    def __del__(self):
        self.stop()
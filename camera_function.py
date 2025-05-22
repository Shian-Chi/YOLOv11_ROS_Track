import cv2, queue, time
from pathlib import Path
import threading as thrd
from utils import check_imshow, increment_path, current_path

def gstreamer_pipeline_2(
    sensor_id=0,
    capture_width=1920, capture_height=1080,
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

def gstreamer_pipeline(
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
    def __init__(self, video_src, width=640, height=480, frame_rate=30, show=False):
        """
        Initialize the Camera class.
        """
        self.video_src = video_src
        self.width = width
        self.height = height
        self.frame_rate = frame_rate
        self.show = show if check_imshow else False

        self.stop_thread = False
        self.read_thread = None
        self.display_thread = None
        self.consume_thread = None
        self.frame = None

        # Initialize video capture
        self.cap = cv2.VideoCapture(self.video_src, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            raise ValueError(f"Failed to open video source: {self.video_src}")

    def start(self):
        if self.read_thread is None or not self.read_thread.is_alive():
            self.read_thread = thrd.Thread(target=self._read_frames, daemon=True)
            self.read_thread.start()
        if self.show and (self.display_thread is None or not self.display_thread.is_alive()):
            self.display_thread = thrd.Thread(target=self._display_frames, daemon=True)
            self.display_thread.start()
        elif not self.show and (self.consume_thread is None or not self.consume_thread.is_alive()):
            self.consume_thread = thrd.Thread(target=self._consume_frames, daemon=True)
            self.consume_thread.start()

    def _read_frames(self):
        while not self.stop_thread:
            ret, self.frame = self.cap.read()
            if not ret:
                print("Failed to read frame.")
                break
            thrd.Event().wait(0.01)
    
    def _display_frames(self):
        if not check_imshow():
            print("Image display is not supported.")
            return
        while not self.stop_thread:
            if self.frame is not None:
                cv2.imshow("Camera Feed", self.frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.stop_thread = True
            thrd.Event().wait(0.01)

    def _consume_frames(self):
        print("Consuming frames without display...")
        while not self.stop_thread:
            if self.frame is not None:
                pass
            thrd.Event().wait(0.01)

    def get_frame(self):
        return self.frame
    
    def isOpened(self):
        return self.cap.isOpened()
    
    def stop(self):
        self.stop_thread = True
        if self.read_thread and self.read_thread.is_alive():
            self.read_thread.join()
        if self.display_thread and self.display_thread.is_alive():
            self.display_thread.join()
        if self.consume_thread and self.consume_thread.is_alive():
            self.consume_thread.join()
        self.cap.release()
        cv2.destroyAllWindows()
        print("Camera stopped and resources released.")
        
    def __del__(self):
        self.stop()

class CameraSave:
    def __init__(self, name, width=640, height=480, frame_rate=30):
        self.name = name
        self.width = width
        self.height = height
        self.frame_rate = frame_rate
        
        self.save_path = str(Path(self.name).with_suffix('.avi'))
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        self.out_media = cv2.VideoWriter(self.save_path, fourcc, self.frame_rate, (self.width, self.height))

        self.stop_thread = False
        self.frame_queue = queue.Queue()
        self.save_Thread = thrd.Thread(target=self._save_frame, daemon=True)
    
    def _save_frame(self):
        counter = 0
        while not self.stop_thread:
            if not self.frame_queue.empty():
                frame = self.frame_queue.get()
                frame = cv2.resize(frame, (self.width, self.height))
                self.out_media.write(frame)
                counter = 0
            else:
                if counter == 10:
                    print("No frame received in 10 seconds. Stopping recording...")
                    self.stop_thread = True
                else:
                    counter += 1
                    time.sleep(1)
            thrd.Event().wait(0.01)
            
    def save(self, frame):
        self.frame_queue.put(frame)
        
    def start(self):
        if not self.save_Thread.is_alive():
            self.save_Thread.start()
    
    def stop(self):
        self.stop_thread = True
        if self.save_Thread.is_alive():
            self.save_Thread.join()
        self.out_media.release()
        print(f"Camera save stopped. Video saved to {self.save_path}.")
    
def start_camera():
    """
    Start the camera with the specified parameters.
    """
    try:
        camera.start()
        while not camera.stop_thread:
            thrd.Event().wait(0.1)  # Main thread wait
    except KeyboardInterrupt:
        print("Stopping camera...")
    finally:
        camera.stop()

def start_recording():
    try:
        while not camera.stop_thread:
            thrd.Event().wait(0.1)  # Main thread wait
    except KeyboardInterrupt:
        print("Save Video")
    finally:
        camera_save.stop()

if __name__ == "__main__":
    global camera, camera_save
    pipeline = gstreamer_pipeline(
        sensor_id=0,
        capture_width=1280,
        capture_height=720,
        display_width=1280,
        display_height=720,
        framerate=30,
        flip_method=0,
    )
    print(f"Generated GStreamer pipeline: {pipeline}")
    camera = Camera(pipeline, show=False)
    camera_save = CameraSave(f"{current_path()}processed_video", width=640, height=480, frame_rate=30)
    
    camera.start()
    camera_save.start()  # Start the save thread
    
    print("Camera started. Press Ctrl+C to stop...")
    try:
        while not camera.stop_thread:
            frame = camera.get_frame()
            if frame is not None:
                camera_save.save(frame)
    except KeyboardInterrupt:
        print("Stopping camera...")
    finally:
        camera.stop()
        camera_save.stop()

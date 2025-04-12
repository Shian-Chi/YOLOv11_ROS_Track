import cv2
from datetime import datetime
import os, time
import queue

class H264_RTSP():
    def __init__(self, frame_queue:queue.Queue, width, height, fps):
        self.frame_queue = frame_queue
        self.width, self.height, self.fps = width, height, fps
        # 输出 GStreamer 管道
        self.pipeline = (
            f"appsrc ! videoconvert ! nvvidconv ! video/x-raw(memory:NVMM), format=NV12, width={self.width}, height={self.height}, framerate={self.fps}/1 ! "
            f"nvv4l2h264enc bitrate=800000 ! rtph264pay config-interval=1 pt=96 ! "
            f"udpsink host=127.0.0.1 port=8554"
        )
        os.environ["GST_DEBUG"] = "3"  # 启用 GStreamer 的调试信息

        # 初始化 GStreamer 写入器
        self.out = cv2.VideoWriter(self.pipeline, cv2.CAP_GSTREAMER, 0, 30.0, (1280, 720))
        self.__write_isOpened__()

        self.run_status = False
        
    def __write_isOpened__(self):
        if not self.out.isOpened():
            print("Failed to open GStreamer pipeline")
            exit()
    
    def stream(self):
        # 推送到 GStreamer 管道
        while self.run_status:
            if not self.frame_queue.empty():
                self.out.write(self.frame_queue.get_nowait())
            time.sleep(0.0333333)

    def run(self):
        try:
            self.run_status = True
            self.stream()
        finally:
            self.out.release()
            
    def stop(self):
        self.run_status = False
        self.out.release()

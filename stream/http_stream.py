import time
import io
import cv2
from http.server import BaseHTTPRequestHandler, HTTPServer

# GStreamer 攝像頭管道
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

# 創建一個處理 HTTP 請求的類
class VideoStreamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/video':
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()

            # 初始化攝像頭
            cap = cv2.VideoCapture(gstreamer_pipeline(capture_width=1280, capture_height=720, flip_method=0), cv2.CAP_GSTREAMER)

            while True:
                # 讀取一幀
                ret, frame = cap.read()
                if not ret:
                    break

                # 將幀轉換為 JPEG 格式
                _, jpeg = cv2.imencode('.jpg', frame)
                frame_data = jpeg.tobytes()

                try:
                    # 發送幀
                    self.wfile.write(b'--frame\r\n')
                    self.send_header('Content-type', 'image/jpeg')
                    self.send_header('Content-length', str(len(frame_data)))
                    self.end_headers()
                    self.wfile.write(frame_data)
                    self.wfile.write(b'\r\n')

                    time.sleep(0.1)
                except BrokenPipeError:
                    print("Client disconnected, stopping video stream.")
                    break  # 如果連接被中斷，則停止視頻流

            cap.release()
        else:
            self.send_response(404)
            self.end_headers()

# 啟動 HTTP 服務器
def run(server_class=HTTPServer, handler_class=VideoStreamHandler, port=8080):
    server_address = ('', port)
    httpd = server_class(server_address, handler_class)
    print(f'Starting server on port {port}')
    httpd.serve_forever()

if __name__ == '__main__':
    run()
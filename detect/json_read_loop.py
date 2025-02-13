from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
import json
import time

def init_data():
    """
    回傳一組初始化的 JSON 結構
    """
    data = {
        "pub_img": {
            "detect": True,
            "camera_center": False,
            "motor_pitch": 0.0,
            "motor_yaw": 0.0,
            "target_latitude": 0.0,
            "target_longitude": 0.0,
            "hold_status": False,
            "send_info": False,
        },
        "pub_bbox": {
            "detect": True,
            "class_id": None,
            "confidence": None,
            "x0": None,
            "x1": None,
            "y0": None,
            "y1": None
        },
        "pub_motor": {
            "yaw_pluse": 0.0,
            "pitch_pluse": 0.0,
            "yaw_angle": 0.0,
            "pitch_angle": 0.0
        }
    }
    return data


class JSONFileHandler(FileSystemEventHandler):
    """
    當監控目標檔案有任何修改時，觸發 on_modified，並讀取更新後的 JSON
    """
    def __init__(self, file_path):
        self.file_path = file_path

    def on_modified(self, event):
        # 確保事件檔案就是我們指定監控的 file_path
        if event.src_path.endswith(self.file_path):
            try:
                with open(self.file_path, "r", encoding="utf-8") as file:
                    data = json.load(file)
                    print("JSON 文件已更新:")
                    print(json.dumps(data, indent=4, ensure_ascii=False))
            except json.JSONDecodeError:
                print(f"文件 {self.file_path} 格式錯誤，無法解析！")
            except Exception as e:
                print(f"讀取文件時出現錯誤: {e}")


def monitor_json(file_path):
    """
    使用 watchdog 實現高即時性的 JSON 文件監控。
    """
    event_handler = JSONFileHandler(file_path)
    observer = Observer()
    # 監控當前資料夾 (path=".")，如果想監控其他路徑，可自行修改
    observer.schedule(event_handler, path=".", recursive=False)
    observer.start()

    print(f"正在監控 {file_path} 的變化")
    try:
        while True:
            time.sleep(1)  # 讓主執行緒持續運行
    except KeyboardInterrupt:
        observer.stop()
    observer.join()


def main():
    # 1. 先初始化 JSON 資料
    data = init_data()
    
    # 2. 將初始化後的 JSON 寫入檔案
    file_path = "publish.json"
    with open(file_path, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=4)
    print(f"已將初始化後的 JSON 寫入檔案: {file_path}")

    # 3. 監控 publish.json 檔案，任何改動都會即時讀取並印出
    monitor_json(file_path)


if __name__ == "__main__":
    main()

import torch
import os, sys, time
import re, glob
import json, csv
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime, timezone, timedelta
from pathlib import Path
import threading, queue
import cv2

def current_network_time(date_format='%Y-%m-%d', time_format='%H:%M:%S'):
    """
    獲取台北時區的當前時間，優先從 NTP 服務器獲取，若無法獲取則使用系統時間。
    
    :param date_format: 日期的格式，默認為 'YYYY-MM-DD'。
    :param time_format: 時間的格式，默認為 'HH:MM:SS'。
    :return: (狀態, 日期, 時間)
        狀態: True 表示使用 NTP 時間，False 表示使用系統時間。
        日期: 格式化的日期字符串。
        時間: 格式化的時間字符串。
    """
    import ntplib
    ntp_servers = ['pool.ntp.org', 'time.google.com', 'time.windows.com']
    taipei_timezone = timezone(timedelta(hours=8))  # 台北時區
    status = False  # 預設為失敗狀態
    
    for server in ntp_servers:
        try:
            client = ntplib.NTPClient()
            response = client.request(server)  # 請求 NTP 服務器
            dt_utc = datetime.fromtimestamp(response.tx_time, timezone.utc)  # UTC 時間
            dt_taipei = dt_utc.astimezone(taipei_timezone)  # 轉換為台北時區
            status = True  # 成功獲取 NTP 時間
            break
        except (ntplib.NTPException, OSError) as e:
            print(f"Unable to connect to {server}. Error: {e}")
            continue

    # 若所有 NTP 服務器不可用，使用系統時間
    if not status:
        print("Unable to obtain network time, using system time instead.")
        dt_taipei = datetime.now(taipei_timezone)

    # 格式化日期和時間
    formatted_date = dt_taipei.strftime(date_format)
    formatted_time = dt_taipei.strftime(time_format)
    return status, formatted_date, formatted_time

def system_time():
    now = datetime.now()
    date = now.strftime("%Y-%m-%d")  # 格式化日期
    time = now.strftime("%H:%M:%S")  # 格式化時間
    return date, time

def current_path(append_slash: bool = False) -> str:
    """
    返回當前工作目錄，根據用戶選擇決定是否添加末尾的斜線。
    
    :param append_slash: 是否在路徑末尾添加斜線，默認為 False。
    :return: 當前工作目錄的路徑（根據選項添加斜線）
    """
    path = os.getcwd()
    if append_slash:
        return path if path.endswith(os.sep) else path + os.sep
    return path

def create_folder(folder_name, base_path="."):
    """
    創建新檔案夾並檢測是否已存在。
    """
    folder_path = os.path.join(base_path, folder_name)
    if os.path.exists(folder_path):
        return f"Folder '{folder_name}' already exists at {folder_path}."
    else:
        os.makedirs(folder_path)
        return f"Folder '{folder_name}' created successfully at {folder_path}."

def check_file_exists(file_path):
    """
    Check if a file exists at the specified path.
    :param file_path: The path to the file to check.
    :return: True if the file exists, False otherwise.
    """
    if os.path.isfile(file_path):
        print(f"The file '{file_path}' exists.")
        return True
    else:
        print(f"The file '{file_path}' does not exist.")
        return False

def check_file(file):
    """
    搜索檔案，如果未找到，回傳 False。
    若找到多個匹配的檔案，回傳檔案路徑的列表。
    若找到唯一匹配的檔案，回傳該檔案的路徑。
    """
    if Path(file).is_file() or file == '':
        return file
    else:
        files = glob.glob('./**/' + file, recursive=True)
        
        if not files:  # 如果沒找到檔案
            return False
        
        return files if len(files) > 1 else files[0]

def increment_path(path, exist_ok=True, sep=''):
    """
    Increment path, e.g., runs/exp -> runs/exp1.
    If exist_ok=True and path exists, the original path is returned.
    If exist_ok=False or path does not exist, it increments the suffix.
    
    Args:
        path (str or Path): Base path to increment.
        exist_ok (bool): Whether to return the original path if it exists.
        sep (str): Separator between the base name and the increment number.
        
    Returns:
        Path: Incremented path.
    """
    path = Path(path)
    
    # If the path exists and `exist_ok` is True, return the original path
    if path.exists() and exist_ok:
        return path

    # Get all matching directories/files
    parent_dir = path.parent
    base_name = path.stem
    dirs = glob.glob(f"{parent_dir}/{base_name}{sep}*")
    
    # Extract numbers from matching paths
    matches = [re.search(rf"{re.escape(base_name)}{sep}(\d+)", Path(d).name) for d in dirs]
    indices = [int(m.group(1)) for m in matches if m]
    
    # Determine the next available index
    next_index = max(indices) + 1 if indices else 1
    return parent_dir / f"{base_name}{sep}{next_index}"

def plot_multiple_line_charts(data, title="Multiple Line Chart", xlabel="X-axis", ylabel="Y-axis", save_as=None):
    """
    繪製多條折線圖在一個圖表中。
    """
    plt.figure(figsize=(10, 6))
    for label, (x, y) in data.items():
        plt.plot(x, y, marker='o', linestyle='-', label=label)

    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    if save_as:
        plt.savefig(save_as)
        print(f"Chart saved as {save_as}")
    else:
        if check_imshow():  # 確認環境是否支持顯示圖像
            plt.show()
        else:
            print("Cannot show line charts")

def hexStr(response) ->str:
    if response is not None:
        hex_string = ' '.join(f'{byte:02x}' for byte in response)
        return hex_string
    return None

def isdocker() ->bool:
    """
    檢測是否在 Docker 容器中運行。
    """
    return Path('/workspace').exists() or Path('/.dockerenv').exists()

def check_imshow() ->bool:
    """
    檢查是否支持顯示圖像功能。
    """
    try:
        assert not isdocker(), 'cv2.imshow() is disabled in Docker environments'
        cv2.imshow('test', np.zeros((1, 1, 3)))
        cv2.waitKey(1)
        cv2.destroyAllWindows()
        cv2.waitKey(1)
        return True
    except Exception as e:
        print(f'WARNING: Environment does not support cv2.imshow() or PIL Image.show() image displays\n{e}')
        return False

def display_frames(name: str, frame, size=(1920, 1080)):
    """
    Display the processed images from a queue or a single frame.
    :param name: Window name for display.
    :param frame: A queue.Queue instance or a single frame.
    :return: Tuple (stop_event, display_thread) for controlling the display.
    """
    stop_event = threading.Event()
    
    def show():
        is_q = isinstance(frame, queue.Queue)
        while not stop_event.is_set():
            if is_q:
                if not frame.empty():
                    # current_frame = frame.get()
                    current_frame = cv2.resize(frame.get(), size)
                    cv2.imshow(name, current_frame)
            else:
                cv2.imshow(name, frame)
            
            # Press 'q' to exit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                stop_event.set()
                break

        cv2.destroyAllWindows()

    # 啟動顯示執行緒
    display_thread = threading.Thread(target=show, daemon=True)
    display_thread.start()
    return stop_event, display_thread

class JsonHandler:
    def __init__(self, file_path):
        """
        初始化 JsonHandler 並加載指定的 JSON 文件。
        :param file_path: JSON 文件路徑
        """
        self.file_path = file_path
        self.data = self._load_json()
    def _load_json(self):
        """
        從文件讀取 JSON 資料並加載到內存。
        :return: Python 字典格式的 JSON 資料
        """
        try:
            with open(self.file_path, "r") as file:
                return json.load(file)
        except FileNotFoundError:
            print(f"文件 {self.file_path} 不存在，初始化為空字典。")
            return {}
        except json.JSONDecodeError:
            print(f"文件 {self.file_path} 格式錯誤，無法解析。初始化為空字典。")
            return {}
    def save_json(self):
        """
        將當前的 JSON 資料保存回文件。
        """
        try:
            with open(self.file_path, "w") as file:
                json.dump(self.data, file, indent=4)
        except Exception as e:
            print(f"保存 JSON 文件時出錯: {e}")
    def get(self, keys, default=None):
        """
        獲取 JSON 中指定鍵的值，支持多層鍵路徑。
        :param keys: 鍵路徑列表，如 ["video_resolutions", "FHD", "width"]
        :param default: 如果鍵不存在，返回的默認值
        :return: 鍵對應的值
        """
        value = self.data
        for key in keys:
            if isinstance(value, dict) and key in value:
                value = value[key]
            else:
                return default
        return value
    def update(self, updates):
        for keys, value in updates.items():
            if not isinstance(keys, tuple):
                raise ValueError(
                    f"鍵路徑應為元組格式，例如: ('key1', 'key2', ...)，但獲得了: {keys}. "
                    "如果您想更新嵌套鍵值，請使用 ('key1', 'key2') 的形式，而不是 'key1, key2'。"
                )
            ref = self.data
            for key in keys[:-1]:
                if key not in ref:
                    ref[key] = {}
                ref = ref[key]
            ref[keys[-1]] = value
    def update_nowait(self, updates):
        """
        批量更新 JSON 中的鍵值對，並立即保存到文件。
        :param updates: 包含多個更新操作的字典
        """
        self.update(updates)
        self.save_json()

class CSVHandler:
    def __init__(self, filename="data.csv", fieldnames=None):
        """
        初始化 CSVHandler
        :param filename: CSV 檔案名稱
        :param fieldnames: CSV 欄位名稱（字典的 key）
        """
        self.filename = filename
        self.fieldnames = fieldnames if fieldnames else []

        # 如果檔案不存在，寫入標題（欄位名稱）
        if not os.path.isfile(self.filename):
            self._write_header()

    def _write_header(self):
        """寫入標題欄位（僅當檔案不存在時執行）"""
        if self.fieldnames:
            with open(self.filename, mode='w', newline='', encoding='utf-8') as file:
                writer = csv.DictWriter(file, fieldnames=self.fieldnames)
                writer.writeheader()

    def write_row(self, row_data):
        """
        寫入單筆資料（字典格式）
        :param row_data: 欲寫入的資料字典
        """
        with open(self.filename, mode='a', newline='', encoding='utf-8') as file:
            writer = csv.DictWriter(file, fieldnames=self.fieldnames)
            writer.writerow(row_data)

    def read_all(self):
        """
        讀取整個 CSV 檔案並回傳為列表
        :return: 包含所有資料的列表
        """
        data = []
        with open(self.filename, mode='r', encoding='utf-8') as file:
            reader = csv.DictReader(file)
            for row in reader:
                data.append(row)
        return data

    def read_last_row(self):
        """
        讀取 CSV 檔案的最後一行
        :return: 最後一行的字典，若檔案為空則回傳 None
        """
        last_row = None
        with open(self.filename, mode='r', encoding='utf-8') as file:
            reader = csv.DictReader(file)
            for row in reader:
                last_row = row  # 迴圈結束時，last_row 會是最後一行
        return last_row

    def test(self):
        """
        測試函式，寫入當前時間資訊
        """
        now = datetime.now()
        data = {
            "data": str(now.date()),  # 確保日期轉換為字串
            "time": str(now.time())   # 確保時間轉換為字串
        }
        self.write_row(data)  # 直接使用類別的 write_row 方法

def csv_test():
    # 測試使用  
    fieldnames = ["data", "time"]
    csv_handler = CSVHandler(filename="data.csv", fieldnames=fieldnames)

    # 測試寫入當前時間
    csv_handler.test()

    # 讀取 CSV 內容
    print(csv_handler.read_all())  # 顯示所有內容
    print(csv_handler.read_last_row())  # 顯示最後一行
        
def transfer_to_cpu(data):
    """
    Transfers data from GPU to CPU.

    Args:
        data (torch.Tensor or list or dict): The data to be transferred.
            - If a tensor, it will be transferred to the CPU.
            - If a list, each tensor in the list will be transferred to the CPU.
            - If a dict, each value in the dict will be transferred to the CPU.

    Returns:
        Transferred data in the same structure as input.
    """
    if isinstance(data, torch.Tensor):
        return data.cpu()
    elif isinstance(data, list):
        return [transfer_to_cpu(item) for item in data]
    elif isinstance(data, dict):
        return {key: transfer_to_cpu(value) for key, value in data.items()}
    else:
        raise TypeError(f"Unsupported data type: {type(data)}")
from multiprocessing import shared_memory
import pickle
import atexit  # 用於註冊清理函數
from utils import JsonHandler


# 初始化數據
pub_img_data = {
    "detect": False,
    "camera_center": False,
    "motor_pitch": 0.0,
    "motor_yaw": 0.0,
    "target_latitude": 0.0,
    "target_longitude": 0.0,
    "hold_status": False,
    "send_info": False
}

pub_bbox_data = {
    "detect": False,
    "class_id": -1,
    "confidence": 0.0,
    "name": "None",
    "x0": -1,
    "x1": -1,
    "y0": -1,
    "y1": -1
}

pub_motor_data = {
    "yaw_pluse": 0.0,
    "pitch_pluse": 0.0,
    "yaw_angle": 0.0,
    "pitch_angle": 0.0
}

def create_shared_memory(data_dict):
    """
    Create shared memory and initialize with pickled data.
    """
    serialized_data = pickle.dumps(data_dict)
    buffer_size = max(len(serialized_data), 1024)  # 確保至少分配 1KB
    shm = shared_memory.SharedMemory(create=True, size=buffer_size)
    shm.buf[:len(serialized_data)] = serialized_data
    return shm

# Shared Memory / Data
def create_shared_memory(data_dict):
    """
    Create shared memory and initialize with pickled data.
    """
    serialized_data = pickle.dumps(data_dict)
    buffer_size = max(len(serialized_data), 1024)  # 確保至少分配 1KB
    shm = shared_memory.SharedMemory(create=True, size=buffer_size)
    shm.buf[:len(serialized_data)] = serialized_data
    return shm


def write_to_shared_memory(shm_obj, data):
    """
    Serialize and write `data` dict into shared memory block `shm_obj`.
    """
    try:
        serialized_data = pickle.dumps(data)
        if len(serialized_data) > shm_obj.size:
            raise ValueError("Data size exceeds shared memory size.")
        shm_obj.buf[:len(serialized_data)] = serialized_data
    except Exception as e:
        print(f"Error writing to shared memory: {e}")

if __name__ == "__main__":
    # 創建共享內存
    pub_img_shm = create_shared_memory(pub_img_data)
    pub_bbox_shm = create_shared_memory(pub_bbox_data)
    pub_motor_shm = create_shared_memory(pub_motor_data)

    # 保存共享內存名稱到文件，供其他程序使用
    with open("shared_memory_names.txt", "w") as f:
        f.write(f"{pub_img_shm.name},{pub_bbox_shm.name},{pub_motor_shm.name}")

    print("共享內存名稱已保存。")

    # 註冊清理函數
    def cleanup_shared_memory():
        """在程序結束時清理共享內存"""
        pub_img_shm.close()
        pub_img_shm.unlink()
        pub_bbox_shm.close()
        pub_bbox_shm.unlink()
        pub_motor_shm.close()
        pub_motor_shm.unlink()

    atexit.register(cleanup_shared_memory)

    # 程式執行中的額外邏輯（可選）
    print("共享內存已創建，可以在其他腳本中使用。按 Ctrl+C 或結束程序以清理內存。")
    try:
        while True:
            pass  # 模擬保持程序運行
    except KeyboardInterrupt:
        print("\n程序中斷，清理內存...")

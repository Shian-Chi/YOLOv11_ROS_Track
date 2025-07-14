import socket
import base64
import serial
import time
from datetime import datetime

# NTRIP caster 設定
caster = "ntrip.locosystech.com"
port = 2101
mountpoint = "LO-333326"
username = "LHU"
password = "1234"

# Serial port 設定
serial_port = "/dev/ttyRTK"
baudrate = 115200

# 串口初始化
ser = serial.Serial(serial_port, baudrate, timeout=1)

# 構造 NTRIP 請求
auth = base64.b64encode(f"{username}:{password}".encode()).decode()
request = (
    f"GET /{mountpoint} HTTP/1.1\r\n"
    f"Host: {caster}\r\n"
    f"Ntrip-Version: Ntrip/2.0\r\n"
    f"User-Agent: NTRIP PythonClient/1.0\r\n"
    f"Authorization: Basic {auth}\r\n\r\n"
)

def log(msg, level="INFO"):
    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    print(f"[{now}] [{level}] {msg}")

def connect():
    """建立並返回 socket 連線"""
    while True:
        try:
            log(f"連接到 NTRIP Caster {caster}:{port}...")
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect((caster, port))
            sock.send(request.encode())

            response = sock.recv(1024)
            if b"200 OK" not in response:
                raise Exception("連線失敗：NTRIP 回應不是 200 OK")
            
            log("✅ 成功連線，開始接收 RTCM 資料")
            return sock
        except Exception as e:
            log(f"連線錯誤：{e}", level="ERROR")
            time.sleep(5)

# 初次連線
sock = connect()

# 用來計算每秒資料量
last_time = time.time()
bytes_this_second = 0

try:
    while True:
        try:
            data = sock.recv(1024)
            if not data:
                raise Exception("資料接收為空，可能已斷線")
            
            ser.write(data)
            bytes_this_second += len(data)

            now = time.time()
            if now - last_time >= 1:
                log(f"📦 每秒接收 {bytes_this_second} bytes")
                bytes_this_second = 0
                last_time = now

        except Exception as e:
            log(f"⚠️ 錯誤：{e}", level="WARNING")
            sock.close()
            log("嘗試重新連線...")
            sock = connect()

except KeyboardInterrupt:
    log("🛑 使用者中止程式", level="INFO")
finally:
    sock.close()
    ser.close()
    log("串口與連線已關閉", level="INFO")

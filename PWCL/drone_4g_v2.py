import socket
import time
import struct
import math
from enum import Enum

uav_socket = None   # 全域變數初始化為 None
GUAV_flag= False    #socket初始化為false,從地面站接收的封包如果不限長度的任意點封包收到頭碼時會為true直到收到尾碼
drone_point = []    #無人機不固定飛行點

class groundControlCommand(Enum):
    DRONE_IDLE = '0'
    DRONE_TAKEOFF = '1'
    DRONE_MISSION_START = '2'
    DRONE_RSEARCH_START = '3'

class drone_state():
    def __init__(self):
        self.droneState = groundControlCommand.DRONE_IDLE.value  

#socket初始化
def socket_init(addr: str, port: int):

    global uav_socket, GUAV_flag # 使用 global 關鍵字聲明全域變數
    print('socket init')

    uav_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    uav_socket.connect((addr, port))
    uav_socket.setblocking(False)
    print('connect to station on :', addr)
    time.sleep(3)

#socket發送
def socket_send(data):
    global uav_socket # 使用 global 關鍵字聲明全域變數
    uav_socket.send(data)

#socket接收
def socket_recv(recv_len):
    global uav_socket # 使用 global 關鍵字聲明全域變數
    try:
        data = uav_socket.recv(recv_len, socket.MSG_DONTWAIT)
    except BlockingIOError as e: #非阻塞
        data = None
        #print('BlockingIOError')
    return data

def radian_conv_degree(Radian) : 
    return ((Radian / math.pi) * 180)

def degree_conv_radian(Degree) : 
    return ((Degree / 180) * math.pi)  

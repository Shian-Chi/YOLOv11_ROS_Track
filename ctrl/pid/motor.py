import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../')))
from utils import JsonHandler, check_file, hexStr

try:
    from ctrl.pid.motorInit import MotorSet
    json_file = check_file("config.json")
except:
    from motorInit import MotorSet
    json_file = check_file(r"../../config.json")
    
   
import numpy as np
import struct
import time
from typing import List


json_config = JsonHandler(json_file)
VIDEO_WIDTH = json_config.get(["video_resolutions", "default", "width"])
VIDEO_HEIGHT = json_config.get(["video_resolutions", "default", "height"])
uintDegreeEn = json_config.get(["encoder", "uintDegreeEncoder"])
motor = MotorSet()

HC = np.uint8(62)  # header Code


class motorInformation:
    def __init__(self, ID, mode, maxAngles: float):
        self.ID = ID
        self.encoder = np.uint16(0)
        self.encoderOffset = np.uint16(0)
        self.encoderRaw = np.uint16(0)
        self.angle = float(0.0)
        self.speed = float(0.0)
        self.powers = float(0.0)
        self.current = float(0.0)
        self.temperature = 0
        self.maxAngles = maxAngles
        self.mode = mode
        self.bootEncoderVal = 0

    def update_encoder(self, encoder_value):
        if 0 <= encoder_value <= 32767:
            self.encoder = encoder_value
            self.angle = (encoder_value / 32767.0)
        else:
            print(self.mode, ": Invalid encoder value")

    def update_speed(self, speed_value):
        self.speed = speed_value

    def update_power(self, power):
        self.powers = power

    def update_encoderOffset(self, value):
        self.encoderOffset = value

    def update_encoderRaw(self, value):
        self.encoderRaw = value

    def update_voltage_current(self, current):
        self.current = current

    def getEncoder(self):
        return self.encoder

    def getAngle(self):
        self.angle = self.encoder / uintDegreeEn
        return self.angle

    def getSpeed(self):
        return self.speed

    def bootEncoder(self, val):
        self.bootEncoderVal = val


class motorCtrl:
    def __init__(self, motorID, mode, maxAngles: float):
        self.ID = np.uint8(motorID)
        if mode is None:
            if self.ID == 1:
                self.mode = "yaw"
            if self.ID == 2:
                self.mode = "pitch"
            if self.ID == 3:
                self.mode = "row"
        else:
            self.mode = mode

        self.info = motorInformation(motorID, self.mode, maxAngles)
        self.bootZero()

    def stop(self):
        cmd = 129  # 0x81
        data = struct.pack("5B", HC, cmd, self.ID, 0, HC + cmd + self.ID + 0)
        info = motor.echo(data, 5)
        return info == data

    def singleTurnVal(self, dir, value: int):
        cmd = np.uint8(165)  # 0xA5
        check_sum = Checksum(value + dir)
        value = np.uint16(value)
        buffer = struct.pack("6BH2B", HC, cmd, self.ID, 4,
                             HC + cmd + self.ID + 4, dir, value, 0, check_sum)
        info = motor.echo(buffer, 10, 13)
        if info is None:
            return False
        if len(info) > 0:
            self.parse_motor_packet(info)
            return True
        return False

    def incrementTurnVal(self, value: int):
        cmd = np.uint8(167)  # 0xA7
        check_sum = Checksum(value)
        buffer = struct.pack("<5BiB", HC, cmd, self.ID, 4,
                             HC + cmd + self.ID + 4, value, check_sum)
        info = motor.echo(buffer, 10, 13)
        if info is None:
            return False
        if len(info) > 0:
            self.parse_motor_packet(info)
            return True
        return False

    def readEncoder(self):
        cmd = 0x90
        buffer = struct.pack("<5B", HC, cmd, self.ID,
                             0, HC + cmd + self.ID + 0)
        info = motor.echo(buffer, 5, 12)
        if info is not None:
            info = struct.unpack("12B", info)
            # header, command, id, size, cmdSum, encoderLow, encoderHigh, encoderRawLow, encoderRawHigh, encoderOffsetLow, encoderOffsetHigh, dataSum = info
            # print(info,"\n")
            
            cs_s = Checksum(sum(info[:4])) == info[4]           
            ds_s = Checksum(info[5:-1]) == info[-1]
            
            if info[0] == 62 and cs_s and ds_s:
                encoder = info[6] << 8 | info[5]
                encoderRaw = info[8] << 8 | info[7]
                encoderOffset = info[10] << 8 | info[9]
                self.info.update_encoder(encoder)
                self.info.update_encoderRaw(encoderRaw)
                self.info.update_encoderOffset(encoderOffset)
                return True
        return False
    
    def parse_motor_packet(self, frame):
        """
        通用的封包解析函式：
        - 不限定 frame[1] (command) 的值
        - 只要結構不變，仍可解析出 encoder。
        回傳 (command, encoder_val)；若校驗失敗或資料不全則回傳 None。
        """
        # 1) 基本長度檢查
        if len(frame) < 13:
            return False, None

        header = frame[0]   # 0x3E
        command = frame[1]  # 控制碼
        motor_id = frame[2] # ID
        data_len = frame[3] # 資料長度
        cmd_sum = frame[4]  # 效驗碼

        # 驗證 Header
        if header != 0x3E:
            print("motor_packet: header not 0x3E")
            return False, None
        
        # 計算命令段校驗和
        # calc_cmd_sum = (header + command + motor_id + data_len) & 0xFF
        calc_cmd_sum = Checksum(header + command + motor_id + data_len)
        if calc_cmd_sum != cmd_sum:
            print("motor_packet: command checksum error")
            return False, None
        
        # 2) 資料段（8 bytes），在 frame[5..12]
        data = frame[5:5+8]
        if len(data) < 8:
            print("motor_packet: data length error")
            return False, None
        
        # 驗證資料段校驗和：假設 data[7] 為 data_sum，前 7 bytes 累加 & 0xFF
        calc_data_sum = sum(data[0:7]) & 0xFF
        if calc_data_sum != data[7]:
            print("motor_packet: data checksum error")
            return False, None

        # 3) 取出 encoder（假設 data[5]、data[6] 分別是低位、高位）
        encoder_low  = data[5]
        encoder_high = data[6]
        encoder_val = (encoder_high << 8) | encoder_low
        # print("motor_packet: updated encoder")
        self.info.update_encoder(encoder_val)
        return command, encoder_val
            
    def getEncoder(self):
        if self.readEncoder():
            return True, int(self.info.getEncoder())
        return False, int(self.info.getEncoder())

    def getAngle(self):
        ret, angle = self.getEncoder()
        return ret, angle/uintDegreeEn
    
    def bootZero(self):
        """Attempt to set motor position to zero by adjusting rotations."""
        max_attempts = 3  # 設置最大嘗試次數以防止無窮迴圈
        for _ in range(max_attempts):
            ret, angle = self.getAngle()
            print(f"{self.mode}: Boot Degrees: {angle}")
            if not ret:
                print("Failed to read motor angle.")
                continue

            if self.ID == 1:
                if angle >= 270 and angle <= 360.0:
                    self.singleTurnVal(0, 0)
                elif angle <= 90:
                    self.singleTurnVal(1, 0)
            elif self.ID == 2:
                self.singleTurnVal(1, 0)
            
            # 判斷是否已接近0度
            if 359.0 <= angle or angle <= 1.0:
                print(f"Motor boot {self.mode} Init finished at angle: {angle}")
                return True

            time.sleep(0.5)  # 等待0.5秒再次檢查角度
        print("Motor boot failed to reach zero position within attempts.")
        return False


def motorInitPositions(targer:motorCtrl, degs:float):
    targer.incrementTurnVal(int(degs*100))
    print(f"{targer.mode} run motor Init Positions")
    
 
# Calculate Checksum of received data
def calc_value_Checksum(value):
    value = value & 0xFFFFFFFF
    return value & 0xFF


def Checksum(value):
    if isinstance(value, (tuple, list)):
        val = np.array(value, dtype=np.uint8)
    else:
        val = np.array([value >> 24 & 0xFF, value >> 16 & 0xFF, value >> 8 & 0xFF, value & 0xFF], dtype=np.uint8)
    
    total = np.sum(val, dtype=np.uint32)
    check_sum = np.uint8(total & 0xFF)
    return check_sum


def motorSend(data, size):
    return motor.send(data, size)


def search_command(response, command, length):
    if response is None:
        return None

    # Find all the locations where Head Code appears
    p = [i for i, value in enumerate(response) if value == HC]
    for i in p:
        if i + 1 < len(response) and response[i + 1] == command:
            if i + 13 <= len(response):
                rep = response[i:i + length]  # Starting from i, take 13 bytes
                return rep
    return None


def normalize_angle_360(angle):
    """
    Normalize an angle to the range [0, 360).
    
    :param angle: Input angle in degrees.
    :return: Normalized angle within [0, 360).
    """
    return angle % 360

def normalize_angle_180(angle):
    """
    Normalize an angle to the range [-180, 180).
    
    :param angle: Input angle in degrees.
    :return: Normalized angle within [-180, 180).
    """
    return (angle + 180) % 360 - 180


def main():
    motor1 = motorCtrl(1, "yaw", 360)
    motor2 = motorCtrl(2, "pitch", 180)

    
    try:
        while True:
            time.sleep(0.5)
            # info1, info2 = [motor1.incrementTurnVal(0), motor2.incrementTurnVal(100)]
            print(f"Yaw: {motor1.getAngle()}, Pitch: {motor2.getAngle()}")
    except KeyboardInterrupt:
        motor1.stop()
        motor2.stop()
    
if __name__ == "__main__":
    main()
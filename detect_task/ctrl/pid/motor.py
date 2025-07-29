import numpy as np
import struct
import time
from ctrl.pid.serialInit import MotorSet

uintDegreeEn = 91.02222222222223
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
        if 0 <= encoder_value <= 32768:
            self.encoder = encoder_value
            self.angle = (encoder_value / 32768.0)
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
    def __init__(self, motorID, mode=None, maxAngles: float = 360.0):
        self.ID  = np.uint8(motorID)
        self.mode = mode or {1: "yaw", 2: "pitch", 3: "row"}.get(self.ID, "unk")
        self.info = motorInformation(motorID, self.mode, maxAngles)

        # 嘗試歸零
        for _ in range(1):
            self.bootZero()
            time.sleep(0.5)

    # ------------------------------------------------ 停止指令
    def stop(self):
        cmd = 0x81
        pkt = struct.pack(
            "5B",
            0x3E,                # 停止封包 header 固定 0x3E
            cmd,
            self.ID,
            0x00,
            checksum_u8([0x3E, cmd, self.ID, 0x00])
        )
        info = motor.echo(pkt, 5, 5)
        return info[:5] == pkt if info else False

    # ------------------------------------------------ 單圈定位
    def singleTurnVal(self, direction: int, value: int):
        cmd = 0xA5
        value = np.uint16(value)
        pkt = struct.pack(
            "6BH2B",
            HC, cmd, self.ID, 4,
            checksum_u8([HC, cmd, self.ID, 4]),
            direction,
            value,
            0x00,
            checksum_u8([direction, value & 0xFF, value >> 8, 0x00])
        )
        info = motor.echo(pkt, 10, 13)
        if info:
            self.parse_motor_packet(info)
            return True
        return False

    # ------------------------------------------------ 增量移動
    def incrementTurnVal(self, value: int):
        cmd = 0xA7
        pkt = struct.pack(
            "<5BiB",
            HC, cmd, self.ID, 4,
            checksum_u8([HC, cmd, self.ID, 4]),
            value,
            checksum_u8(value)
        )
        info = motor.echo(pkt, 10, 13)
        if info:
            self.parse_motor_packet(info)
            return True
        return False

    # ------------------------------------------------ 讀編碼器
    def readEncoder(self):
        cmd = 0x90
        pkt = struct.pack(
            "<5B",
            HC, cmd, self.ID, 0x00,
            checksum_u8([HC, cmd, self.ID, 0x00])
        )
        info = motor.echo(pkt, 5, 12)
        if not info:        # 無回傳
            return False

        info = struct.unpack("12B", info)
        cmd_sum_ok  = checksum_u8(info[:4]) == info[4]
        data_sum_ok = checksum_u8(info[5:-1]) == info[-1]

        if info[0] == 0x3E and cmd_sum_ok and data_sum_ok:
            enc       = (info[6]  << 8) | info[5]
            enc_raw   = (info[8]  << 8) | info[7]
            enc_off   = (info[10] << 8) | info[9]
            self.info.update_encoder(enc)
            self.info.update_encoderRaw(enc_raw)
            self.info.update_encoderOffset(enc_off)
            return True
        return False

    # ------------------------------------------------ 解析回應
    def parse_motor_packet(self, frame: bytes):
        if len(frame) < 13 or frame[0] != 0x3E:
            return False, None

        if checksum_u8(frame[:4]) != frame[4]:
            return False, None

        data = frame[5:13]
        if checksum_u8(data[:-1]) != data[-1]:
            return False, None

        encoder_val = (data[6] << 8) | data[5]
        self.info.update_encoder(encoder_val)
        return frame[1], encoder_val
 
    def getEncoder(self):
        if self.readEncoder():
            return True, int(self.info.getEncoder())
        return False, int(self.info.getEncoder())

    def getAngle(self):
        ret, angle = self.getEncoder()
        return ret, angle/uintDegreeEn
    
    def bootZero(self):
        """Attempt to set motor position to zero by adjusting rotations."""
        max_attempts = 1  # 設置最大嘗試次數以防止無窮迴圈
        for _ in range(max_attempts):
            ret, angle = self.getAngle()
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
                return True

            time.sleep(0.05)  
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

def checksum_u8(data) -> int:                           # ★ 取代原 Checksum
    """
    8-bit 加總校驗和：把輸入逐 byte 相加 (&0xFF)。
    · data 可為 int / list / bytes / numpy array ……
    """
    if isinstance(data, int):
        data = [(data >> 24) & 0xFF, (data >> 16) & 0xFF,
                (data >> 8) & 0xFF,  data & 0xFF]
    return sum(data) & 0xFF        # 回傳 python int (0-255)

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
            motor1.singleTurnVal(0, 0), motor2.singleTurnVal(1,0)
            print(f"Yaw: {motor1.getAngle()}, Pitch: {motor2.getAngle()}")
    except KeyboardInterrupt:
        motor1.stop()
        motor2.stop()
    
if __name__ == "__main__":
    main()
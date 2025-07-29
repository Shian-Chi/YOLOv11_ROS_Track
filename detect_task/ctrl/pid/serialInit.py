#!/usr/bin/env python3
import errno
import time
import subprocess
import serial

class MotorSet:
    def __init__(self):
        self.ser = None
        self.error_count = 0
        self.baudrate = 460800
        self.port = '/dev/ttyTHS0'
        self._sudo_pass = "123456789"
        self.init_serial()
        
    # ---------------------------------------------------------------------
    def _fix_port_permission(self) -> bool:
        """
        Run port.bash with sudo to set correct permissions.
        Returns True if script exits with code 0.
        """
        cmd = f'echo "{self._sudo_pass}" | sudo -S bash /home/ubuntu/track/track2/scripts/port.bash'
        print("Permission denied → running port.bash …")
        result = subprocess.run(cmd, shell=True)
        return result.returncode == 0

    # ---------------------------------------------------------------------
    def init_serial(self, retry_on_perm_err: bool = True):
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=0.1
            )
            print("Successfully opened serial port")
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()

        except serial.SerialException as e:
            # Detect "Permission denied"
            if (isinstance(e, OSError) and e.errno == errno.EACCES) or "Permission" in str(e):
                if retry_on_perm_err and self._fix_port_permission():
                    # Wait a moment and retry once
                    time.sleep(1)
                    self.init_serial(retry_on_perm_err=False)
                    return
            # Other errors or second failure
            print(f"Failed to initialize serial port: {e}")
            self.handle_io_error()

    # ---------------------------------------------------------------------
    def handle_io_error(self):
        print("Handling I/O error…")
        if self.ser and self.ser.is_open:
            self.ser.close()
        time.sleep(1)
        self.init_serial(retry_on_perm_err=False)

    # ---------------------------------------------------------------------
    def send(self, buf=b'\x01', size=0):
        if size == 0:
            size = len(buf)
        try:
            send_time = (size * 8) / self.baudrate
            t1 = time.time()
            w_len = self.ser.write(buf[:size])
            actual_send_time = time.time() - t1
            if actual_send_time < send_time:
                time.sleep(send_time - actual_send_time)
            return w_len
        except (serial.SerialException, OSError) as e:
            print(f"Error in send: {e}")
            self.handle_io_error()
            return 0

    # ---------------------------------------------------------------------
    def recv(self, size):
        try:
            if self.ser.in_waiting >= size:
                return self.ser.read(size)
            return None
        except (serial.SerialException, OSError) as e:
            print(f"Error in recv: {e}")
            self.handle_io_error()
            return None

    # ---------------------------------------------------------------------
    def reset_buffer(self):
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    # ---------------------------------------------------------------------
    def echo(self, s_buff, s_size: int, r_size=0, redo=1):
        for _ in range(redo):
            self.send(s_buff, s_size)
            start = time.time()
            while time.time() - start < 0.1:
                data = self.recv(r_size)
                if data and len(data) >= r_size:
                    self.reset_buffer()
                    return data
            self.reset_buffer()
        return None
    
if __name__ == "__main__":
    import numpy as np
    import struct
    HC = 0xA5  # Head Code
    motor = MotorSet()
    
    def Checksum(value):
        if isinstance(value, (tuple, list)):
            val = np.array(value, dtype=np.uint8)
        else:
            val = np.array([value >> 24 & 0xFF, value >> 16 & 0xFF, value >> 8 & 0xFF, value & 0xFF], dtype=np.uint8)
        
        total = np.sum(val, dtype=np.uint32)
        check_sum = np.uint8(total & 0xFF)
        return check_sum
    
    def parse_motor_packet(frame):
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
        return command, encoder_val
    
    def singleTurnVal(dir, value: int):
        cmd = np.uint8(165)  # 0xA5
        check_sum = Checksum(value + dir)
        value = np.uint16(value)
        buffer = struct.pack("6BH2B", HC, cmd, 0, 4,
                             Checksum(HC + cmd + 0 + 4), dir, value, 0, check_sum)
        info = motor.echo(buffer, 10, 13)
        if info is None:
            return False
        if len(info) > 0:
            parse_motor_packet(info)
            return True
        return False
    
    def checksum_u8(seq) -> int:
        """計算 8-bit 加總 (低 8 位)"""
        if isinstance(seq, int):
            seq = [(seq >> 24) & 0xFF, (seq >> 16) & 0xFF,
                (seq >> 8) & 0xFF, seq & 0xFF]
        return sum(seq) & 0xFF

    def stop_motor(motor_id: int = 0x01) -> bool:
        HEADER = 0x3E       # 固定
        CMD    = 0x81       # 停止指令
        LEN    = 0x00       # 無資料段

        chk = checksum_u8([HEADER, CMD, motor_id, LEN])

        packet = struct.pack("5B", HEADER, CMD, motor_id, LEN, chk)

        # 馬達大多不會「原樣回送」同一包；這裡假設會回 5 Byte ACK
        resp = motor.echo(packet, 5, 5)

        if resp and resp[:5] == packet:
            print("✓ motor stop ACK received")
            return True
        print("✗ no / bad ACK")
        return False
    
    # Example usage
    try:
        ret = singleTurnVal(0, 0)

    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        stop_motor()

        if motor.ser and motor.ser.is_open:
            motor.ser.close()
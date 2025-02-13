import serial
import time

class MotorSet:
    def __init__(self):
        self.ser = None
        self.error_count = 0
        self.baudrate = 460800
        
        try:
            # run in Jetson
            self.port = '/dev/ttyTHS0'
        except:
            # run in Mac 
            self.port = '/dev/tty.usbserial-AR0K3S0Y'
            
        self.init_serial()
        
    def init_serial(self):
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,  
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=0.1
            )
            
            if self.ser.is_open:
                print(f'Successfully opened serial port')
            else:
                self.ser.open()
                print(f"{self.ser.in_waiting}, Successfully opened serial port")
                
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
        except serial.SerialException as e:
            print(f"Failed to initialize serial port: {e}")
            self.error_handler()

    def handle_io_error(self):
        print("Handling I/O error...")
        if self.ser.is_open:
            self.ser.close()
        time.sleep(1)
        self.init_serial()
        self.reset_buffer()
    
    def send(self, buf=b'\x01', size=0):
        if size == 0:
            size = len(buf)
        
        try:
            send_time = (size * 8) / self.baudrate
            t1 = time.time()
            wLen = self.ser.write(buf[:size])            
            actual_send_time = time.time() - t1
            
            if actual_send_time < send_time:
                time.sleep(send_time - actual_send_time)
            
            return wLen
        except serial.SerialException as e:
            self.handle_io_error()
            print(f"Error in send method: {e}")
            return 0
        except OSError as e:
            print(f"OS error during send: {e}")
            self.handle_io_error()
            return 0
        
    def recv(self, size):
        try:
            l = self.ser.in_waiting
            if l > 0:
                return self.ser.read(size)
            else:
                return None
        except OSError:
            pass    
        except serial.SerialException as e:
            print(f"Error during recv: {e}")
            self.handle_io_error()
            return None

    def reset_buffer(self):
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
    
    def echo(self, sBuff, sSize:int, rSize=0, redo=1):
        for _ in range(redo):
            self.send(sBuff, sSize)
            start_time = time.time()
            
            while True:
                if self.ser.in_waiting >= rSize:
                    data = self.recv(rSize)
                    if data is not None and len(data) >= rSize:
                        self.reset_buffer()
                        return data
                    
                if time.time() - start_time >= 0.1:
                    break
            
            self.reset_buffer()
        
        return None

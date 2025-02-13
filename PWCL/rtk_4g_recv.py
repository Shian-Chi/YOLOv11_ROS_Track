import socket
import serial

HOST = '114.32.137.51'
PORT = 8080

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))

#serial
Com = serial.Serial('/dev/ttyRTK', 115200)

while True:

    try:
        indata = s.recv(2048)
        Com.write(indata)
        #if len(indata) == 0: # connection closed
            #s.close()
            #print('server closed connection.')
            #break
        print(len(indata))
        
    except socket.error:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((HOST, PORT))

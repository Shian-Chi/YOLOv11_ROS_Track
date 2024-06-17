import socket
import serial

HOST = '10.147.20.196'
PORT = 2090

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

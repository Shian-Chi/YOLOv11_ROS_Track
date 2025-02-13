# 模擬無人機(Rover端)從xbee接收RTK基站的RTCM再解封包傳給Rover端的RTK模組
# 有包"RTCM"頭碼
import serial
import struct
import time
import binascii

recv_data_list=[None]*84                        #存取ZigBee協議+RTCM資料的陣列
zig_bee_data=[None]*64                          #存取RTCM資料的陣列    

#serial_in (xbee端)
ser_in = serial.Serial()
ser_in.port = "/dev/ttyUSB0"
ser_in.baudrate = 115200
ser_in.bytesize = serial.EIGHTBITS                  #number of bits per bytes
ser_in.parity = serial.PARITY_NONE                  #set parity check
ser_in.stopbits = serial.STOPBITS_ONE               #number of stop bits
ser_in.timeout = 0.2
ser_in.open()


# serial_out (RTK端)
ser_out = serial.Serial()
ser_out.port = "/dev/ttyRTK"                               #com10
ser_out.baudrate = 115200
ser_out.bytesize = serial.EIGHTBITS                 #number of bits per bytes
ser_out.parity = serial.PARITY_NONE                 #set parity check
ser_out.stopbits = serial.STOPBITS_ONE              #number of stop bits
ser_out.timeout = 0.2
ser_out.open()



while True:  
    check_sum = 0x00                                #先把CheckSum清空
    
    uart_fifo = ser_in.inWaiting()
    if uart_fifo >= 84:
    
        recv_data_list = ser_in.read(84)                #把COMport收到的資料給'recv_data_list'
        data_Head = recv_data_list[15:19]
        if data_Head is not None:
            decode_data_Head = data_Head.decode(encoding='utf-8',errors='replace')
        
        if ((len(recv_data_list) == 84) and (decode_data_Head == 'RTCM')):                   #判斷'recv_data_list'是否剛好為84個Byte & 是否收到RTCM頭碼
            print('recv Head code "RTCM"')
            # print('local recv:',recv_data_list) 
            # print('recv len:',len(recv_data_list))        
            for x in range(3,83):                       #在recv_data_list[3]~recv_data_list[78]的資料相加，並且算出CheckSum
                #print('x = ',x)  
                check_sum += recv_data_list[x]
                #print('check_sum = ',hex(check_sum))  
            #x = 0    
            check_sum = check_sum & 0xFF
            check_sum_byte = 0xFF - check_sum
            #check_sum_byte = check_sum_byte.to_bytes(1, byteorder='big')
            
            if check_sum_byte == recv_data_list[83]:    #驗證CheckSum是否與程式算出的CheckSum正確
                zig_bee_data = recv_data_list[19:83]
                if len(zig_bee_data) != 0:
                    ser_out.write(zig_bee_data)         #把收到的RTCM傳給RTK Rover
                    # print('send:',zig_bee_data)
            else:                                       #如果CheckSum是錯誤的
                print('check_sum_byte = ',check_sum_byte)
                print('recv_data_list[83] = ',recv_data_list[83])
                print('CheckSum Error: lost Packet !!!')
                #time.sleep(5)
            
    #以下都是判斷'recv_data_list'的陣列是否剛好為84Byte        
    elif ((len(recv_data_list) < 84) and (len(recv_data_list) > 0)):
        print('len =',len(recv_data_list))
        print('local recv:',recv_data_list)
        print('Packet Error: Packet Miss !!!')
        #time.sleep(5)   
    elif (len(recv_data_list) > 84):
        print('len =',len(recv_data_list))
        print('Packet Error: Packet Overflow !!!') 
        #time.sleep(5)  
    #判斷是否收到RTCM頭碼

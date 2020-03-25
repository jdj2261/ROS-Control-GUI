import time
import serial

# configure the serial connections (the parameters differs on the device you are connecting to)
ser = serial.Serial(
    port='/dev/ttyMP0',
    baudrate=115200
    # parity=serial.PARITY_ODD
    # stopbits=serial.STOPBITS_TWO,
    # bytesize=serial.SEVENBITS
)

ser.isOpen()

# 03 F3 F3 01 01 64 00 00 00 00 00 65
print 'beofore write'

input_str='03 F3 F3 01 00 64 00 00 00 00 00 64'

input_str = input_str.replace(' ', '')
print input_str

if( len(input_str) % 2 == 1 ):
    print 'input length must be even size'
else:
    input_count = len(input_str) / 2 
    default_elements = []
    for i in range(0, input_count):
        default_elements.append(0)
    packet = bytearray(default_elements)

    for i in range(0, input_count):
        hex_str = str(input_str[i*2:i*2+2])
        print 'hex_str : [' + hex_str + ']'
        packet[i] = hex_str.decode('hex')

    while True:
        print str(packet).encode('hex')
        ser.write(packet)
        time.sleep(0.1)
    print 'after write'

# while 1 :
#     # get keyboard input
#     input = raw_input(">> ")
#         # Python 3 users
#         # input = input(">> ")
        
#     if input == 'exit':
#         ser.close()
#         exit()
#     else:
#         # send the character to the device
#         # (note that I happend a \r\n carriage return and line feed to the characters - this is requested by my device)
#         ser.write(input + '\r\n')
#         # out = ''
#         # # let's wait one second before reading output (let's give device time to answer)
#         time.sleep(0.1)
#         # while ser.inWaiting() > 0:
#         #     out += (ser.read(1)).encode('hex')+' '

#         # if out != '':
#         #     print ">>" + out
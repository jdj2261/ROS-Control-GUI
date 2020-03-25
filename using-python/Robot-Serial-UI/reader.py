import time
import serial

class RobocareSerialReader():
    def __init__(self, serial, widget):
        self.__serial = serial
        print 'RobocareSerialReader : init'
        self.__widget = widget
        self.__is_end = False

    def run(self, tmp):
        print 'RobocareSerialReader read start'
        while not self.__is_end:
            out = ''
            while self.__serial.inWaiting() > 0:
                read_packet = bytearray(self.__serial.read(1))
                # out += str(hex(read_packet[0]))+' '
                out += format(read_packet[0], '02x')+' '

            if out != '':
                if self.__widget == None:
                    pass
                else:
                    self.__widget.append(out)
                    # print out
            time.sleep(0.1)
        print 'RobocareSerialReader read fin'

    def stop(self):
        self.__is_end = True
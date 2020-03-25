#!/usr/bin/python
#-*-coding:utf-8-*-
import time
import serial
import thread
from writer import RobocareSerialWriter
from reader import RobocareSerialReader
from view import *
import PySide
from PySide.QtGui import *
from PySide import QtGui, QtCore
from PySide.QtCore import Qt
import sys

class RobocareSerial():

    def __init__(self):
        self.__reader = None
        self.__writer = None
        self.__serial = None

    def open(self, port):
        try:
            self.__serial = serial.Serial(
                port='/dev/'+port,
                baudrate=9600
                # parity=serial.PARITY_ODD
                # stopbits=serial.STOPBITS_TWO,
                # bytesize=serial.SEVENBITS
            )
            
        except:
            self.showMsgBox(u'/dev/'+port+u' 포트를 여는 데 실패했습니다.')
            return False

        return True

    def close(self):
        self.stop()
        time.sleep(0.2)
        self.__serial.close()

    def stop(self):
        if self.__reader != None:
            self.__reader.stop()
        if self.__writer != None:
            self.__writer.stop()

    def stop_write(self):
        if self.__writer != None:
            self.__writer.stop()

    def stop_read(self):
        if self.__reader != None:
            self.__reader.stop()

    def write(self, input, duration=0.1, count=1):
        if self.__serial != None:
            if self.__serial.is_open:
                if self.__writer != None:
                    if not self.__writer.is_end():
                        self.__writer.stop()
                        time.sleep(0.1)
                self.__writer = RobocareSerialWriter(serial=self.__serial, hex_str=input, duration=duration, send_count=count)
                thread.start_new_thread(self.__writer.run, ("", ))
            else:
                self.showMsgBox(u'먼저 포트를 열어 주세요')
        else:
            self.showMsgBox(u'먼저 포트를 열어 주세요')

    def read(self, widget):
        self.__reader = RobocareSerialReader(serial=self.__serial, widget=widget)
        thread.start_new_thread(self.__reader.run, ("", ))

    def showMsgBox(self, str):
        msgBox = QMessageBox()
        msgBox.setText(str)
        msgBox.exec_()
        print str

    def test(self):
        input_str='03 F3 F3 01 00 64 00 00 00 00 00 64'
        writer = RobocareSerialWriter(serial=self.__serial, hex_str=input_str, duration=0.1, send_count=100)
        thread.start_new_thread(writer.run, ("", ))
        reader = RobocareSerialReader(serial=self.__serial, widget=None)
        thread.start_new_thread(reader.run, ("", ))
        # writer.run()
        time.sleep(2)
        writer.stop()
        reader.stop()
        time.sleep(0.5)

class RobocareSerialUI(QtGui.QMainWindow):
    def __init__(self) :
        super(RobocareSerialUI, self).__init__()
        self.__rs = RobocareSerial()
        self.init_window()
        self.add_widget()
        self.init_main_view()

    def init_window(self) :
        self.setGeometry(50, 50, 600, 600)
        self.setWindowTitle('Robocare Serial')
        self.show()

    def add_widget(self):
        self.__stacked_widget = QStackedWidget()
        self.__wheel_view = RobocareSerialWheelView(self.__rs)
        self.__stacked_widget.addWidget(self.__wheel_view)
        self.__body_view = RobocareSerialBodyView(self.__rs)
        self.__stacked_widget.addWidget(self.__body_view)
        self.__sonar_view = RobocareSerialSonarView(self.__rs)
        self.__stacked_widget.addWidget(self.__sonar_view)
        self.__etc_view = RobocareSerialEtcView(self.__rs)
        self.__stacked_widget.addWidget(self.__etc_view)

        self.__stacked_widget.setCurrentWidget(self.__wheel_view)
        # self.__stacked_widget.setCurrentWidget(self.__body_view)
        # self.__stacked_widget.setCurrentWidget(self.__sonar_view)
        # self.__stacked_widget.setCurrentWidget(self.__etc_view)

    def init_main_view(self):

        widget = QWidget(self)
        layout = QVBoxLayout()
        widget.setLayout(layout)

        top_layout = QGridLayout()
        self.__read_display = QTextEdit()
        self.__read_display.setFont(QFont(u"나눔고딕", 11, weight=QFont.Bold))
        top_layout.addWidget(self.__read_display, 0, 0, 1, 4)

        open_layout = QVBoxLayout()

        self.__port_combo = QComboBox()
        self.__port_combo.setFont(QFont(u"나눔고딕", 20, weight=QFont.Bold))
        self.__port_combo.addItems(["ttyMP0", "ttyMP1", "ttyMP2", "ttyMP3", "ttyUSB0", "ttyUSB1", "ttyUSB2", "ttyUSB3", "ttyUSB4"])
        open_layout.addWidget(self.__port_combo)

        self.__conn_button = QPushButton(u'연결')
        self.__conn_button.setFont(QFont(u"나눔고딕", 20, weight=QFont.Bold))
        # self.__conn_button.setStyleSheet("color: black; border:2px solid rgb(0,0,0);")
        self.__conn_button.clicked.connect(self.clicked_conn)
        open_layout.addWidget(self.__conn_button)

        self.__disconn_button = QPushButton(u'해제')
        self.__disconn_button.setFont(QFont(u"나눔고딕", 20, weight=QFont.Bold))
        # self.__disconn_button.setStyleSheet("color: black; border:2px solid rgb(0,0,0);")
        self.__disconn_button.clicked.connect(self.clicked_disconn)
        self.__disconn_button.setEnabled(False)
        open_layout.addWidget(self.__disconn_button)

        top_layout.addLayout(open_layout, 0, 4, 1, 1)
        
        layout.addLayout(top_layout)

        mid_layout = QHBoxLayout()
        wheel_button = QPushButton(u'휠')
        # wheel_button.setStyleSheet("color: black; border:2px solid rgb(0,0,0);")
        wheel_button.clicked.connect(self.clicked_wheel)
        mid_layout.addWidget(wheel_button)

        body_button = QPushButton(u'바디')
        # body_button.setStyleSheet("color: black; border:2px solid rgb(0,0,0);")
        body_button.clicked.connect(self.clicked_body)
        mid_layout.addWidget(body_button)

        sonar_button = QPushButton(u'소나')
        # sonar_button.setStyleSheet("color: black; border:2px solid rgb(0,0,0);")
        sonar_button.clicked.connect(self.clicked_sonar)
        mid_layout.addWidget(sonar_button)

        led_button = QPushButton(u'터치, LED')
        # led_button.setStyleSheet("color: black; border:2px solid rgb(0,0,0);")
        led_button.clicked.connect(self.clicked_etc)
        mid_layout.addWidget(led_button)
        
        layout.addLayout(mid_layout)

        layout.addWidget(self.__stacked_widget)

        self.setCentralWidget(widget)

    def clicked_conn(self):
        print 'clicked_conn'
        port_name = self.__port_combo.currentText()
        result = self.__rs.open(port_name)
        if result:
            self.__conn_button.setEnabled(False)
            self.__disconn_button.setEnabled(True)
            self.__rs.read(self.__read_display)
        else:
            self.__conn_button.setEnabled(True)
            self.__disconn_button.setEnabled(False)

    def clicked_disconn(self):
        print 'clicked_disconn'
        self.__disconn_button.setEnabled(False)
        self.__rs.close()
        self.__conn_button.setEnabled(True)

    def clicked_wheel(self):
        print "clicked_wheel"
        self.__stacked_widget.setCurrentWidget(self.__wheel_view)

    def clicked_body(self):
        print "clicked_body"
        self.__stacked_widget.setCurrentWidget(self.__body_view)

    def clicked_sonar(self):
        print "clicked_sonar"
        self.__stacked_widget.setCurrentWidget(self.__sonar_view)
    
    def clicked_etc(self):
        print "clicked_etc"
        self.__stacked_widget.setCurrentWidget(self.__etc_view)

if __name__ == '__main__':
    # rs = RobocareSerial()
    # rs.open('ttyMP0')
    # rs.test()
    # rs.close()
    qt_app = QApplication(sys.argv)
    ui = RobocareSerialUI()
    qt_app.exec_()

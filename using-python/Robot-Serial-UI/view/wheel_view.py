#!/usr/bin/python
#-*-coding:utf-8-*-
import PySide
from PySide.QtGui import QApplication, QWidget, QLabel, QComboBox, QLineEdit, QPushButton, QVBoxLayout, QFormLayout, QHBoxLayout, QFont, QGridLayout
from PySide import QtGui, QtCore
from PySide.QtCore import QSize
import time

class RobocareSerialWheelView(QWidget):

    def __init__(self, robocare_serial):
        self.__robocare_serial = robocare_serial
        QWidget.__init__(self)

        layout = QGridLayout()
        self.setLayout(layout)

        self.__line1 = QLineEdit()
        self.__line1.setPlaceholderText("03 F3 F3 ...") 
        self.__line1.setFont(QFont(u"나눔고딕",20,weight=QFont.Bold))
        layout.addWidget(self.__line1, 0, 0, 1, 3)

        button = QPushButton(u'한번 보내기',self)
        button.setFont(QFont(u"나눔고딕",15,weight=QFont.Bold))
        button.clicked.connect(self.clicked_custom_once)
        layout.addWidget(button, 1, 0, 1, 1)

        button = QPushButton(u'100번 보내기',self)
        button.setFont(QFont(u"나눔고딕",15,weight=QFont.Bold))
        button.clicked.connect(self.clicked_custom_repeat)
        layout.addWidget(button, 1, 1, 1, 1)

        # button = QPushButton(u'인코더 읽기',self)
        # button.setFont(QFont(u"나눔고딕",15,weight=QFont.Bold))
        # button.clicked.connect(self.clicked_encoder)
        # layout.addWidget(button, 1, 2, 1, 1)

        self.__line2 = QLineEdit()
        self.__line2.setPlaceholderText("03 F3 F3 ...") 
        self.__line2.setFont(QFont(u"나눔고딕",20,weight=QFont.Bold))
        layout.addWidget(self.__line2, 2, 0, 1, 3)

        button = QPushButton(u'버전 체크',self)
        button.setFont(QFont(u"나눔고딕",15,weight=QFont.Bold))
        button.clicked.connect(self.clicked_version)
        layout.addWidget(button, 3, 0, 1, 1)

        button = QPushButton(u'스톱',self)
        button.setFont(QFont(u"나눔고딕",15,weight=QFont.Bold))
        button.clicked.connect(self.clicked_stop)
        layout.addWidget(button, 3, 1, 1, 1)

        button = QPushButton(u'인코더 읽기',self)
        button.setFont(QFont(u"나눔고딕",15,weight=QFont.Bold))
        button.clicked.connect(self.clicked_encoder)
        layout.addWidget(button, 3, 2, 1, 1)

        button = QPushButton(u'좌회전',self)
        button.setFont(QFont(u"나눔고딕",15,weight=QFont.Bold))
        button.clicked.connect(self.clicked_lt)
        layout.addWidget(button, 4, 0, 1, 1)

        button = QPushButton(u'전진',self)
        button.setFont(QFont(u"나눔고딕",15,weight=QFont.Bold))
        button.clicked.connect(self.clicked_foward)
        layout.addWidget(button, 4, 1, 1, 1)

        button = QPushButton(u'우회전',self)
        button.setFont(QFont(u"나눔고딕",15,weight=QFont.Bold))
        button.clicked.connect(self.clicked_rt)
        layout.addWidget(button, 4, 2, 1, 1)

        button = QPushButton(u'왼쪽 이동',self)
        button.setFont(QFont(u"나눔고딕",15,weight=QFont.Bold))
        button.clicked.connect(self.clicked_left)
        layout.addWidget(button, 5, 0, 1, 1)

        button = QPushButton(u'후진',self)
        button.setFont(QFont(u"나눔고딕",15,weight=QFont.Bold))
        button.clicked.connect(self.clicked_back)
        layout.addWidget(button, 5, 1, 1, 1)

        button = QPushButton(u'오른쪽 이동',self)
        button.setFont(QFont(u"나눔고딕",15,weight=QFont.Bold))
        button.clicked.connect(self.clicked_right)
        layout.addWidget(button, 5, 2, 1, 1)                                        

    def clicked_custom_once(self):
        print "clicked_version"
        input_str=self.__line1.text()
        self.__robocare_serial.write(input_str, 0.1, 1)

    def clicked_custom_repeat(self):
        print "clicked_version"
        input_str=self.__line1.text()
        self.__robocare_serial.write(input_str, 0.1, 100)
# 버전 체크, 인코더 읽기, 이동, 스톱
    def clicked_version(self):
        print "clicked_version"
        input_str='03 F3 F3 0F 0F'
        self.__line2.setText(input_str)
        self.__robocare_serial.write(input_str, 0.1, 1)

    def clicked_encoder(self):
        print "clicked_version"
        input_str='03 F3 F3 0E 0E'
        self.__line2.setText(input_str)
        self.__robocare_serial.write(input_str, 0.1, 1)

    def clicked_stop(self):
        print "clicked_version"
        input_str='03 F3 F3 09 09'
        self.__line2.setText(input_str)
        self.__robocare_serial.write(input_str, 0.1, 1)

    def clicked_lt(self):
        print "clicked_lt"
        input_str='03 F3 F3 01 00 00 00 00 00 16 00 16'
        self.__line2.setText(input_str)
        self.__robocare_serial.write(input_str, 0.1, 100)              

    def clicked_rt(self):
        print "clicked_rt"
        input_str='03 F3 F3 01 00 00 00 00 80 16 00 96'
        self.__line2.setText(input_str)
        self.__robocare_serial.write(input_str, 0.1, 100)

    def clicked_foward(self):
        print "clicked_foward"
        input_str='03 F3 F3 01 00 00 00 64 00 00 00 64'
        self.__line2.setText(input_str)
        self.__robocare_serial.write(input_str, 0.1, 100)

    def clicked_back(self):
        print "clicked_back"
        input_str='03 F3 F3 01 00 00 80 64 00 00 00 E4'
        self.__line2.setText(input_str)
        self.__robocare_serial.write(input_str, 0.1, 100)

    def clicked_left(self):
        print "clicked_left"
        input_str='03 F3 F3 01 80 64 00 00 00 00 00 E4'
        self.__line2.setText(input_str)
        self.__robocare_serial.write(input_str, 0.1, 100)

    def clicked_right(self):
        print "clicked_right"
        input_str='03 F3 F3 01 00 64 00 00 00 00 00 64'
        self.__line2.setText(input_str)
        self.__robocare_serial.write(input_str, 0.1, 100)

# 버전 체크, 인코더 읽기, 이동, 스톱
    def clicked(self):
        print "clicked"
        input_str='03 F3 F3 01 00 64 00 00 00 00 00 64'
        # self.__robocare_serial.write(input_str, 0.1, 100)

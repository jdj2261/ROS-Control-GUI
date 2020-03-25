#!/usr/bin/python
#-*-coding:utf-8-*-
import PySide
from PySide.QtGui import QApplication, QWidget, QLabel, QComboBox, QLineEdit, QPushButton, QVBoxLayout, QFormLayout, QHBoxLayout, QFont, QGridLayout
from PySide import QtGui, QtCore
from PySide.QtCore import QSize
import time

class RobocareSerialBodyView(QWidget):

    def __init__(self, robocare_serial):
        self.__is_xtion_type = True
        self.__robocare_serial = robocare_serial
        QWidget.__init__(self)

        layout = QGridLayout()
        self.setLayout(layout)

        self.__line1 = QLineEdit()
        if self.__is_xtion_type:
            self.__line1.setPlaceholderText("08 F8 F8 05 00 64 00 00 00 00 00 64 00 00 00 00 00 00 00 00 00 00 10 10 00 00 00 00 00 00 00 E8")
        else:
            self.__line1.setPlaceholderText("08 F8 F8 05 00 64 00 00 00 00 00 64 00 00 00 00 00 00 00 00 10 10 00 00 00 00 00 00 E8")
        self.__line1.setFont(QFont(u"나눔고딕",8,weight=QFont.Bold))
        layout.addWidget(self.__line1, 0, 0, 1, 4)

        button = QPushButton(u'보내기',self)
        button.setFont(QFont(u"나눔고딕",15,weight=QFont.Bold))
        button.clicked.connect(self.clicked_custom_once)
        layout.addWidget(button, 1, 0, 1, 1)

        button = QPushButton(u'홈위치',self)
        button.setFont(QFont(u"나눔고딕",15,weight=QFont.Bold))
        button.clicked.connect(self.clicked_home)
        layout.addWidget(button, 1, 1, 1, 1)

        button = QPushButton(u'홈위치 변경',self)
        button.setFont(QFont(u"나눔고딕",15,weight=QFont.Bold))
        button.clicked.connect(self.clicked_set_home)
        layout.addWidget(button, 1, 2, 1, 1)

        button = QPushButton(u'버전 체크',self)
        button.setFont(QFont(u"나눔고딕",15,weight=QFont.Bold))
        button.clicked.connect(self.clicked_version)
        layout.addWidget(button, 1, 3, 1, 1)

        # button = QPushButton(u'인코더 읽기',self)
        # button.setFont(QFont(u"나눔고딕",15,weight=QFont.Bold))
        # button.clicked.connect(self.clicked_encoder)
        # layout.addWidget(button, 1, 2, 1, 1)

        self.__line2 = QLineEdit()
        if self.__is_xtion_type:
            self.__line1.setPlaceholderText("08 F8 F8 05 00 64 00 00 00 00 00 64 00 00 00 00 00 00 00 00 00 00 10 10 00 00 00 00 00 00 00 E8")
        else:
            self.__line1.setPlaceholderText("08 F8 F8 05 00 64 00 00 00 00 00 64 00 00 00 00 00 00 00 00 10 10 00 00 00 00 00 00 E8")
        self.__line2.setFont(QFont(u"나눔고딕",8,weight=QFont.Bold))
        layout.addWidget(self.__line2, 2, 0, 1, 4)

        button = QPushButton(u'팔 위치 1',self)
        button.setFont(QFont(u"나눔고딕",15,weight=QFont.Bold))
        button.clicked.connect(self.clicked_a1)
        layout.addWidget(button, 3, 0, 1, 1)

        button = QPushButton(u'팔 위치 2',self)
        button.setFont(QFont(u"나눔고딕",15,weight=QFont.Bold))
        button.clicked.connect(self.clicked_a2)
        layout.addWidget(button, 3, 1, 1, 1)

        button = QPushButton(u'팔 위치 3',self)
        button.setFont(QFont(u"나눔고딕",15,weight=QFont.Bold))
        button.clicked.connect(self.clicked_a3)
        layout.addWidget(button, 3, 2, 1, 1)

        button = QPushButton(u'팔 위치 4',self)
        button.setFont(QFont(u"나눔고딕",15,weight=QFont.Bold))
        button.clicked.connect(self.clicked_a4)
        layout.addWidget(button, 3, 3, 1, 1)

        button = QPushButton(u'팔 위치 5',self)
        button.setFont(QFont(u"나눔고딕",15,weight=QFont.Bold))
        button.clicked.connect(self.clicked_a5)
        layout.addWidget(button, 4, 0, 1, 1)

        button = QPushButton(u'팔 위치 6',self)
        button.setFont(QFont(u"나눔고딕",15,weight=QFont.Bold))
        button.clicked.connect(self.clicked_a6)
        layout.addWidget(button, 4, 1, 1, 1)

        button = QPushButton(u'머리 위치 1',self)
        button.setFont(QFont(u"나눔고딕",15,weight=QFont.Bold))
        button.clicked.connect(self.clicked_h1)
        layout.addWidget(button, 4, 2, 1, 1)

        button = QPushButton(u'머리 위치 2',self)
        button.setFont(QFont(u"나눔고딕",15,weight=QFont.Bold))
        button.clicked.connect(self.clicked_h2)
        layout.addWidget(button, 4, 3, 1, 1)                                        

    def clicked_custom_once(self):
        print "clicked_custom_once"
        input_str=self.__line1.text()
        self.__robocare_serial.write(input_str, 0.1, 1)

    def clicked_home(self):
        print "clicked_home"
        if self.__is_xtion_type:
            input_str='08 F8 F8 05 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 10 10 10 10 10 10 10 10 10 90'
        else:
            input_str='08 F8 F8 05 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 10 10 10 10 10 10 10 10 90'
        self.__robocare_serial.write(input_str, 0.1, 1)

    def clicked_set_home(self):
        print "clicked_set_home"
        input_str='08 F8 F8 07 07'
        self.__robocare_serial.write(input_str, 0.1, 1)
        time.sleep(0.2)
        input_str='07 F7 F7 07 07'
        self.__robocare_serial.write(input_str, 0.1, 1)

    def clicked_version(self):
        print "clicked_version"
        input_str='08 F8 F8 0F 0F'
        self.__line2.setText(input_str)
        self.__robocare_serial.write(input_str, 0.1, 1)

    def clicked_a1(self):
        print "clicked_a1"
        if self.__is_xtion_type:
            input_str='08 F8 F8 05 80 5A 00 00 00 00 80 5A 00 00 00 00 00 00 00 00 00 00 20 20 20 20 20 20 20 20 20 C4'
        else:
            input_str='08 F8 F8 05 80 5A 00 00 00 00 80 5A 00 00 00 00 00 00 00 00 20 20 20 20 20 20 20 20 A4'
        self.__line2.setText(input_str)
        self.__robocare_serial.write(input_str, 0.1, 1)

    def clicked_a2(self):
        print "clicked_a2"
        if self.__is_xtion_type:
            input_str='08 F8 F8 05 00 AA 00 00 00 00 00 AA 00 00 00 00 00 00 00 00 00 00 20 20 20 20 20 20 20 20 20 64'
        else:
            input_str='08 F8 F8 05 00 AA 00 00 00 00 00 AA 00 00 00 00 00 00 00 00 20 20 20 20 20 20 20 20 44'
        self.__line2.setText(input_str)
        self.__robocare_serial.write(input_str, 0.1, 1)

    def clicked_a3(self):
        print "clicked_a3"
        if self.__is_xtion_type:
            input_str='08 F8 F8 05 00 AA 80 0A 00 00 00 AA 80 0A 00 00 00 00 00 00 00 00 20 20 20 20 20 20 20 20 20 68'
        else:
            input_str='08 F8 F8 05 00 AA 80 0A 00 00 00 AA 80 0A 00 00 00 00 00 00 20 20 20 20 20 20 20 20 48'
        self.__line2.setText(input_str)
        self.__robocare_serial.write(input_str, 0.1, 1)

    def clicked_a4(self):
        print "clicked_a4"
        if self.__is_xtion_type:
            input_str='08 F8 F8 05 00 AA 00 78 00 00 00 AA 00 78 00 00 00 00 00 00 00 00 20 20 20 20 20 20 20 20 20 44'
        else:
            input_str='08 F8 F8 05 00 AA 00 78 00 00 00 AA 00 78 00 00 00 00 00 00 20 20 20 20 20 20 20 20 24'
        self.__line2.setText(input_str)
        self.__robocare_serial.write(input_str, 0.1, 1)

    def clicked_a5(self):
        print "clicked_a5"
        if self.__is_xtion_type:
            input_str='08 F8 F8 05 00 AA 00 78 00 14 00 AA 00 78 00 14 00 00 00 00 00 00 20 20 20 20 20 20 20 20 20 6C'
        else:
            input_str='08 F8 F8 05 00 AA 00 78 00 14 00 AA 00 78 00 14 00 00 00 00 20 20 20 20 20 20 20 20 4C'
        self.__line2.setText(input_str)
        self.__robocare_serial.write(input_str, 0.1, 1)

    def clicked_a6(self):
        print "clicked_a6"
        if self.__is_xtion_type:
            input_str='08 F8 F8 05 00 AA 00 78 80 32 00 AA 00 78 80 32 00 00 00 00 00 00 20 20 20 20 20 20 20 20 20 A8'
        else:
            input_str='08 F8 F8 05 00 AA 00 78 80 32 00 AA 00 78 80 32 00 00 00 00 20 20 20 20 20 20 20 20 88'
        self.__line2.setText(input_str)
        self.__robocare_serial.write(input_str, 0.1, 1)

    def clicked_h1(self):
        print "clicked_h1"
        if self.__is_xtion_type:
            input_str='08 F8 F8 05 00 00 00 00 00 00 00 00 00 00 00 00 00 41 00 28 00 2D 20 20 20 20 20 20 20 20 20 A6'
        else:
            input_str='08 F8 F8 05 00 00 00 00 00 00 00 00 00 00 00 00 00 41 00 28 20 20 20 20 20 20 20 20 69'
        self.__line2.setText(input_str)
        self.__robocare_serial.write(input_str, 0.1, 1)

    def clicked_h2(self):
        print "clicked_h2"
        if self.__is_xtion_type:
            input_str='08 F8 F8 05 00 00 00 00 00 00 00 00 00 00 00 00 80 41 80 28 80 0A 20 20 20 20 20 20 20 20 20 03'
        else:
            input_str='08 F8 F8 05 00 00 00 00 00 00 00 00 00 00 00 00 80 41 80 0A 20 20 20 20 20 20 20 20 4B'
        self.__line2.setText(input_str)
        self.__robocare_serial.write(input_str, 0.1, 1)

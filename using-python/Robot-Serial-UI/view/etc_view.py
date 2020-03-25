#!/usr/bin/python
#-*-coding:utf-8-*-
import PySide
from PySide.QtGui import QApplication, QWidget, QLabel, QComboBox, QLineEdit, QPushButton, QVBoxLayout, QFormLayout, QHBoxLayout, QFont, QGridLayout
from PySide import QtGui, QtCore
from PySide.QtCore import QSize
import time

class RobocareSerialEtcView(QWidget):

    def __init__(self, robocare_serial):
        self.__robocare_serial = robocare_serial
        QWidget.__init__(self)

        layout = QGridLayout()
        self.setLayout(layout)

        self.__line1 = QLineEdit()
        self.__line1.setPlaceholderText("06 F6 F6 F1 0E")
        self.__line1.setFont(QFont(u"나눔고딕",15,weight=QFont.Bold))
        layout.addWidget(self.__line1, 0, 0, 1, 3)

        button = QPushButton(u'보내기',self)
        button.setFont(QFont(u"나눔고딕",15,weight=QFont.Bold))
        button.clicked.connect(self.clicked_custom_once)
        layout.addWidget(button, 1, 0, 1, 1)

        self.__line2 = QLineEdit()
        self.__line2.setPlaceholderText("06 F6 F6 F1 0E")
        self.__line2.setFont(QFont(u"나눔고딕",15,weight=QFont.Bold))
        layout.addWidget(self.__line2, 2, 0, 1, 3)

        button = QPushButton(u'버전 체크',self)
        button.setFont(QFont(u"나눔고딕",15,weight=QFont.Bold))
        button.clicked.connect(self.clicked_version)
        layout.addWidget(button, 3, 0, 1, 1)

        button = QPushButton(u'터치 읽기',self)
        button.setFont(QFont(u"나눔고딕",15,weight=QFont.Bold))
        button.clicked.connect(self.clicked_read)
        layout.addWidget(button, 3, 1, 1, 1)

        button = QPushButton(u'100번 읽기',self)
        button.setFont(QFont(u"나눔고딕",15,weight=QFont.Bold))
        button.clicked.connect(self.clicked_read_100)
        layout.addWidget(button, 3, 2, 1, 1)

        button = QPushButton(u'빨강',self)
        button.setFont(QFont(u"나눔고딕",15,weight=QFont.Bold))
        button.clicked.connect(self.clicked_red)
        layout.addWidget(button, 4, 0, 1, 1)

        button = QPushButton(u'녹색',self)
        button.setFont(QFont(u"나눔고딕",15,weight=QFont.Bold))
        button.clicked.connect(self.clicked_green)
        layout.addWidget(button, 4, 1, 1, 1)

        button = QPushButton(u'파랑',self)
        button.setFont(QFont(u"나눔고딕",15,weight=QFont.Bold))
        button.clicked.connect(self.clicked_blue)
        layout.addWidget(button, 4, 2, 1, 1)

    def clicked_custom_once(self):
        print "clicked_custom_once"
        input_str=self.__line1.text()
        self.__robocare_serial.write(input_str, 0.1, 1)

    def clicked_read(self):
        print "clicked_read"
        input_str='06 F6 F6 F1 0E'
        self.__line2.setText(input_str)
        self.__robocare_serial.write(input_str, 0.1, 1)

    def clicked_read_100(self):
        print "clicked_set_home"
        input_str='06 F6 F6 F1 0E'
        self.__line2.setText(input_str)
        self.__robocare_serial.write(input_str, 0.1, 100)

    def clicked_version(self):
        print "clicked_version"
        input_str='06 F6 F6 0F 0F'
        self.__line2.setText(input_str)
        self.__robocare_serial.write(input_str, 0.1, 1)

    def clicked_red(self):
        print "clicked_red"
        input_str='06 F6 F6 F0 01 00 00 01 00 00 01 00 00 00 01'
        self.__line2.setText(input_str)
        self.__robocare_serial.write(input_str, 0.1, 1)

    def clicked_green(self):
        print "clicked_green"
        input_str='06 F6 F6 F0 00 01 00 00 01 00 00 01 00 00 01'
        self.__line2.setText(input_str)
        self.__robocare_serial.write(input_str, 0.1, 1)

    def clicked_blue(self):
        print "clicked_blue"
        input_str='06 F6 F6 F0 00 00 01 00 00 01 00 00 01 00 01'
        self.__line2.setText(input_str)
        self.__robocare_serial.write(input_str, 0.1, 1)

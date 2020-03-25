#!/usr/bin/python
#-*-coding:utf-8-*-
import PySide
from PySide.QtGui import QApplication, QWidget, QLabel, QComboBox, QLineEdit, QPushButton, QVBoxLayout, QFormLayout, QHBoxLayout, QFont, QGridLayout
from PySide import QtGui, QtCore
from PySide.QtCore import QSize
import time

class RobocareSerialSonarView(QWidget):

    def __init__(self, robocare_serial):
        self.__robocare_serial = robocare_serial
        QWidget.__init__(self)

        layout = QGridLayout()
        self.setLayout(layout)

        self.__line1 = QLineEdit()
        self.__line1.setPlaceholderText("05 F5 F5 F5 0A")
        self.__line1.setFont(QFont(u"나눔고딕",15,weight=QFont.Bold))
        layout.addWidget(self.__line1, 0, 0, 1, 3)

        button = QPushButton(u'보내기',self)
        button.setFont(QFont(u"나눔고딕",15,weight=QFont.Bold))
        button.clicked.connect(self.clicked_custom_once)
        layout.addWidget(button, 1, 0, 1, 1)

        self.__line2 = QLineEdit()
        self.__line2.setPlaceholderText("05 F5 F5 F5 0A")
        self.__line2.setFont(QFont(u"나눔고딕",15,weight=QFont.Bold))
        layout.addWidget(self.__line2, 2, 0, 1, 3)

        button = QPushButton(u'버전 체크',self)
        button.setFont(QFont(u"나눔고딕",15,weight=QFont.Bold))
        button.clicked.connect(self.clicked_version)
        layout.addWidget(button, 3, 0, 1, 1)

        button = QPushButton(u'소나 읽기',self)
        button.setFont(QFont(u"나눔고딕",15,weight=QFont.Bold))
        button.clicked.connect(self.clicked_read)
        layout.addWidget(button, 3, 1, 1, 1)

        button = QPushButton(u'100번 읽기',self)
        button.setFont(QFont(u"나눔고딕",15,weight=QFont.Bold))
        button.clicked.connect(self.clicked_read_100)
        layout.addWidget(button, 3, 2, 1, 1)

    def clicked_custom_once(self):
        print "clicked_custom_once"
        input_str=self.__line1.text()
        self.__robocare_serial.write(input_str, 0.1, 1)

    def clicked_read(self):
        print "clicked_read"
        input_str='05 F5 F5 0F 0A'
        self.__line2.setText(input_str)
        self.__robocare_serial.write(input_str, 0.1, 1)

    def clicked_read_100(self):
        print "clicked_set_home"
        input_str='05 F5 F5 0F 0A'
        self.__line2.setText(input_str)
        self.__robocare_serial.write(input_str, 0.1, 100)

    def clicked_version(self):
        print "clicked_version"
        input_str='05 F5 F5 0F 0F'
        self.__line2.setText(input_str)
        self.__robocare_serial.write(input_str, 0.1, 1)

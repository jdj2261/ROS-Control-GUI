#!/usr/bin/python
#-*-coding:utf-8-*-
import time
import sys, os
import serial
import thread
import PySide
import rospy

from PySide.QtGui import *
from PySide.QtCore import *
from PySide import QtGui, QtCore

from geometry_msgs.msg import *
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseActionGoal

from events.button import ButtonEvent


class Communicate(QtCore.QObject):
    signal = QtCore.Signal(str)

class NavControlUI(QtGui.QMainWindow):
    def __init__(self) :
        super(NavControlUI, self).__init__()

        self.__comm = Communicate()
        self.__comm.signal.connect(self.get_signal)

        self.init_subscribe()
        self.init_window()
        self.add_widget()
        self.init_main_view()

    def init_subscribe(self):
        self.__subscribers = []

        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.init_pose_callback)
        rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, self.goal_pose_callback)

    def init_window(self) :
        self.setGeometry(200, 200, 1000, 300)
        self.setWindowTitle('Nav Control')
        self.show()

    def add_widget(self):
        self.__stacked_widget = QStackedWidget()
        pass

    def init_main_view(self):

        self.__btn = ButtonEvent()

        widget = QWidget(self)
        layout = QVBoxLayout()
        widget.setLayout(layout)

        # Title Layout 
        title_layout = QGridLayout()
        title_label = QLabel(u'Navigation Control UI', self)
        title_label.setFont(QFont(u"나눔고딕", 20, weight=QFont.Bold))
        title_label.setAlignment(QtCore.Qt.AlignCenter)
        title_layout.addWidget(title_label, 0, 0)

        layout.addLayout(title_layout)

        # odom Layout

        # pose
        odom_pos_layout = QGridLayout()

        odom_pos_label = QLabel(u"Robot's Position", self)
        odom_pos_label.setFont(QFont(u"나눔고딕", 15))

        pos_X_label = QLabel(u"X", self)
        pos_X_label.setFont(QFont(u"나눔고딕", 15))
        pos_X_label.setAlignment(QtCore.Qt.AlignCenter)
        pos_Y_label = QLabel(u"Y", self)
        pos_Y_label.setFont(QFont(u"나눔고딕", 15))
        pos_Y_label.setAlignment(QtCore.Qt.AlignCenter)
        pos_Z_label = QLabel(u"Theta", self)
        pos_Z_label.setFont(QFont(u"나눔고딕", 15))
        pos_Z_label.setAlignment(QtCore.Qt.AlignCenter)

        odom_pos_layout.addWidget(odom_pos_label, 0, 0)
        odom_pos_layout.addWidget(pos_X_label, 0, 1)
        odom_pos_layout.addWidget(pos_Y_label, 0, 2)
        odom_pos_layout.addWidget(pos_Z_label, 0, 3)

        self.__pos_X_input = QLineEdit(self)
        self.__pos_X_input.setFont(QFont(u"나눔고딕", 15))
        self.__pos_X_input.setEnabled(False)
        self.__pos_Y_input = QLineEdit(self)
        self.__pos_Y_input.setFont(QFont(u"나눔고딕", 15))
        self.__pos_Y_input.setEnabled(False)
        self.__pos_Z_input = QLineEdit(self)
        self.__pos_Z_input.setFont(QFont(u"나눔고딕", 15))
        self.__pos_Z_input.setEnabled(False)

        odom_pos_layout.addWidget(self.__pos_X_input, 1, 1)
        odom_pos_layout.addWidget(self.__pos_Y_input, 1, 2)
        odom_pos_layout.addWidget(self.__pos_Z_input, 1, 3)
        layout.addLayout(odom_pos_layout)

        # vel
        odom_vel_layout = QGridLayout()

        odom_vel_label = QLabel(u"Robot's Velocity", self)
        odom_vel_label.setFont(QFont(u"나눔고딕", 15))

        vel_X_label = QLabel(u"X", self)
        vel_X_label.setFont(QFont(u"나눔고딕", 15))
        vel_X_label.setAlignment(QtCore.Qt.AlignCenter)
        vel_Y_label = QLabel(u"Y", self)
        vel_Y_label.setFont(QFont(u"나눔고딕", 15))
        vel_Y_label.setAlignment(QtCore.Qt.AlignCenter)
        vel_Z_label = QLabel(u"Theta", self)
        vel_Z_label.setFont(QFont(u"나눔고딕", 15))
        vel_Z_label.setAlignment(QtCore.Qt.AlignCenter)

        odom_vel_layout.addWidget(odom_vel_label, 0, 0)
        odom_vel_layout.addWidget(vel_X_label, 0, 1)
        odom_vel_layout.addWidget(vel_Y_label, 0, 2)
        odom_vel_layout.addWidget(vel_Z_label, 0, 3)

        self.__vel_X_input = QLineEdit(self)
        self.__vel_X_input.setFont(QFont(u"나눔고딕", 15))
        self.__vel_X_input.setEnabled(False)
        self.__vel_Y_input = QLineEdit(self)
        self.__vel_Y_input.setFont(QFont(u"나눔고딕", 15))
        self.__vel_Y_input.setEnabled(False)
        self.__vel_Z_input = QLineEdit(self)
        self.__vel_Z_input.setFont(QFont(u"나눔고딕", 15))
        self.__vel_Z_input.setEnabled(False)

        odom_vel_layout.addWidget(self.__vel_X_input, 1, 1)
        odom_vel_layout.addWidget(self.__vel_Y_input, 1, 2)
        odom_vel_layout.addWidget(self.__vel_Z_input, 1, 3)

        layout.addLayout(odom_vel_layout)

        # Initial Pose
        init_pos_layout = QGridLayout()

        init_pos_label = QLabel(u"Robot's Initpose", self)
        init_pos_label.setFont(QFont(u"나눔고딕", 15))

        init_X_label = QLabel(u"X", self)
        init_X_label.setFont(QFont(u"나눔고딕", 15))
        init_X_label.setAlignment(QtCore.Qt.AlignCenter)
        init_Y_label = QLabel(u"Y", self)
        init_Y_label.setFont(QFont(u"나눔고딕", 15))
        init_Y_label.setAlignment(QtCore.Qt.AlignCenter)
        init_Z_label = QLabel(u"Theta", self)
        init_Z_label.setFont(QFont(u"나눔고딕", 15))
        init_Z_label.setAlignment(QtCore.Qt.AlignCenter)
        init_W_label = QLabel(u"W", self)
        init_W_label.setFont(QFont(u"나눔고딕", 15))
        init_W_label.setAlignment(QtCore.Qt.AlignCenter)

        init_pos_layout.addWidget(init_pos_label, 0, 0)
        init_pos_layout.addWidget(init_X_label, 0, 1)
        init_pos_layout.addWidget(init_Y_label, 0, 2)
        init_pos_layout.addWidget(init_Z_label, 0, 3)
        init_pos_layout.addWidget(init_W_label, 0, 4)

        self.__init_X_input = QLineEdit(self)
        self.__init_X_input.setFont(QFont(u"나눔고딕", 15))
        self.__init_X_input.setEnabled(False)
        self.__init_Y_input = QLineEdit(self)
        self.__init_Y_input.setFont(QFont(u"나눔고딕", 15))
        self.__init_Y_input.setEnabled(False)
        self.__init_Z_input = QLineEdit(self)
        self.__init_Z_input.setFont(QFont(u"나눔고딕", 15))
        self.__init_Z_input.setEnabled(False)
        self.__init_W_input = QLineEdit(self)
        self.__init_W_input.setFont(QFont(u"나눔고딕", 15))
        self.__init_W_input.setEnabled(False)

        init_pos_layout.addWidget(self.__init_X_input, 1, 1)
        init_pos_layout.addWidget(self.__init_Y_input, 1, 2)
        init_pos_layout.addWidget(self.__init_Z_input, 1, 3)
        init_pos_layout.addWidget(self.__init_W_input, 1, 4)
        layout.addLayout(init_pos_layout)

        # Goal Layout
        go_to_goal_layout = QGridLayout()

        go_to_goal_label = QLabel(u"Go To Goal", self)
        go_to_goal_label.setFont(QFont(u"나눔고딕", 15))

        goal_X_label = QLabel(u"X", self)
        goal_X_label.setFont(QFont(u"나눔고딕", 15))
        goal_X_label.setAlignment(QtCore.Qt.AlignCenter)
        goal_Y_label = QLabel(u"Y", self)
        goal_Y_label.setFont(QFont(u"나눔고딕", 15))
        goal_Y_label.setAlignment(QtCore.Qt.AlignCenter)
        goal_Z_label = QLabel(u"Theta", self)
        goal_Z_label.setFont(QFont(u"나눔고딕", 15))
        goal_Z_label.setAlignment(QtCore.Qt.AlignCenter)
        goal_W_label = QLabel(u"W", self)
        goal_W_label.setFont(QFont(u"나눔고딕", 15))
        goal_W_label.setAlignment(QtCore.Qt.AlignCenter)

        self.__goal_X_output = QLineEdit(self)
        self.__goal_X_output.setFont(QFont(u"나눔고딕", 15))
        self.__goal_X_output.setText("0.0")
        self.__goal_Y_output = QLineEdit(self)
        self.__goal_Y_output.setFont(QFont(u"나눔고딕", 15))
        self.__goal_Y_output.setText("0.0")
        self.__goal_Z_output = QLineEdit(self)
        self.__goal_Z_output.setFont(QFont(u"나눔고딕", 15))
        self.__goal_Z_output.setText("0.0")
        self.__goal_W_output = QLineEdit(self)
        self.__goal_W_output.setFont(QFont(u"나눔고딕", 15))
        self.__goal_W_output.setText("1.0")
        # self.__goal_Z_output.text("1.0")

        # goal button
        go_to_goal_button = QPushButton(u'GO',self)
        go_to_goal_button.setFont(QFont(u"나눔고딕",20,weight=QFont.Bold))
        go_to_goal_button.clicked.connect(lambda: self.__btn.go(
            self.__goal_X_output.text(), 
            self.__goal_Y_output.text(),
            self.__goal_Z_output.text(),
            self.__goal_W_output.text()))


        # stop button
        stop_button = QPushButton(u'STOP',self)
        stop_button.toggle()
        stop_button.setCheckable(True)
        stop_button.setFont(QFont(u"나눔고딕",20,weight=QFont.Bold))
        
        stop_button.clicked.connect(lambda: self.__btn.on_stop(
            stop_button,
            self.__goal_x_txt, 
            self.__goal_y_txt, 
            self.__goal_z_txt, 
            self.__goal_w_txt))
         

        self.__goal_X_input = QLineEdit(self)
        self.__goal_X_input.setFont(QFont(u"나눔고딕", 15))
        self.__goal_X_input.setEnabled(False)
        self.__goal_Y_input = QLineEdit(self)
        self.__goal_Y_input.setFont(QFont(u"나눔고딕", 15))
        self.__goal_Y_input.setEnabled(False)
        self.__goal_Z_input = QLineEdit(self)
        self.__goal_Z_input.setFont(QFont(u"나눔고딕", 15))
        self.__goal_Z_input.setEnabled(False)
        self.__goal_W_input = QLineEdit(self)
        self.__goal_W_input.setFont(QFont(u"나눔고딕", 15))
        self.__goal_W_input.setEnabled(False)

        go_to_goal_layout.addWidget(go_to_goal_label, 0, 0)
        go_to_goal_layout.addWidget(goal_X_label, 0, 1)
        go_to_goal_layout.addWidget(goal_Y_label, 0, 2)
        go_to_goal_layout.addWidget(goal_Z_label, 0, 3)
        go_to_goal_layout.addWidget(goal_W_label, 0, 4)
        go_to_goal_layout.addWidget(go_to_goal_button, 1, 5)
        go_to_goal_layout.addWidget(stop_button, 2, 5)

        go_to_goal_layout.addWidget(self.__goal_X_output, 1, 1)
        go_to_goal_layout.addWidget(self.__goal_Y_output, 1, 2)
        go_to_goal_layout.addWidget(self.__goal_Z_output, 1, 3)
        go_to_goal_layout.addWidget(self.__goal_W_output, 1, 4)

        go_to_goal_layout.addWidget(self.__goal_X_input, 2, 1)
        go_to_goal_layout.addWidget(self.__goal_Y_input, 2, 2)
        go_to_goal_layout.addWidget(self.__goal_Z_input, 2, 3)
        go_to_goal_layout.addWidget(self.__goal_W_input, 2, 4)


        layout.addLayout(go_to_goal_layout)

        # layout.addWidget(self.__stacked_widget)

        self.setCentralWidget(widget)

    def get_signal(self, s):

        if str(s).startswith('pos_x|'):
            pos_x_txt = str(s).replace('pos_x|', '').strip()
            self.__pos_X_input.setText(pos_x_txt)
        elif str(s).startswith('pos_y|'):
            pos_y_txt = str(s).replace('pos_y|', '').strip()
            self.__pos_Y_input.setText(pos_y_txt)
        elif str(s).startswith('pos_z|'):
            pos_z_txt = str(s).replace('pos_z|', '').strip()
            self.__pos_Z_input.setText(pos_z_txt)
        elif str(s).startswith('vel_x|'):
            vel_x_txt = str(s).replace('vel_x|', '').strip()
            self.__vel_X_input.setText(vel_x_txt)
        elif str(s).startswith('vel_y|'):
            vel_y_txt = str(s).replace('vel_y|', '').strip()
            self.__vel_Y_input.setText(vel_y_txt)
        elif str(s).startswith('vel_z|'):
            vel_z_txt = str(s).replace('vel_z|', '').strip()
            self.__vel_Z_input.setText(vel_z_txt)
        elif str(s).startswith('init_x|'):
            init_x_txt = str(s).replace('init_x|', '').strip()
            self.__init_X_input.setText(init_x_txt)
        elif str(s).startswith('init_y|'):
            init_y_txt = str(s).replace('init_y|', '').strip()
            self.__init_Y_input.setText(init_y_txt)
        elif str(s).startswith('init_z|'):
            init_z_txt = str(s).replace('init_z|', '').strip()
            self.__init_Z_input.setText(init_z_txt)
        elif str(s).startswith('init_w|'):
            init_w_txt = str(s).replace('init_w|', '').strip()
            self.__init_W_input.setText(init_w_txt)
        elif str(s).startswith('goal_x|'):
            self.__goal_x_txt = str(s).replace('goal_x|', '').strip()
            self.__goal_X_input.setText(self.__goal_x_txt)
        elif str(s).startswith('goal_y|'):
            self.__goal_y_txt = str(s).replace('goal_y|', '').strip()
            self.__goal_Y_input.setText(self.__goal_y_txt)
        elif str(s).startswith('goal_z|'):
            self.__goal_z_txt = str(s).replace('goal_z|', '').strip()
            self.__goal_Z_input.setText(self.__goal_z_txt)
        elif str(s).startswith('goal_w|'):
            self.__goal_w_txt = str(s).replace('goal_w|', '').strip()
            self.__goal_W_input.setText(self.__goal_w_txt)

    def odom_callback(self,msg):

        pos_x_signal = 'pos_x|'
        pos_y_signal = 'pos_y|'
        pos_z_signal = 'pos_z|'

        vel_x_signal = 'vel_x|'
        vel_y_signal = 'vel_y|'
        vel_z_signal = 'vel_z|'

        pos_x_signal = pos_x_signal + str(round(msg.pose.pose.position.x,3)) 
        pos_y_signal = pos_y_signal + str(round(msg.pose.pose.position.y,3))
        pos_z_signal = pos_z_signal + str(round(msg.pose.pose.orientation.z,3))

        vel_x_signal = vel_x_signal + str(round(msg.twist.twist.linear.x,3))
        vel_y_signal = vel_y_signal + str(round(msg.twist.twist.linear.y,3))
        vel_z_signal = vel_z_signal + str(round(msg.twist.twist.angular.z,3))

        self.__comm.signal.emit(pos_x_signal)
        self.__comm.signal.emit(pos_y_signal)
        self.__comm.signal.emit(pos_z_signal)
        self.__comm.signal.emit(vel_x_signal)
        self.__comm.signal.emit(vel_y_signal)
        self.__comm.signal.emit(vel_z_signal)

    def init_pose_callback(self,msg):

        init_x_signal = 'init_x|'
        init_y_signal = 'init_y|'
        init_z_signal = 'init_z|'
        init_w_signal = 'init_w|'

        init_x_signal = init_x_signal + str(round(msg.pose.pose.position.x,3)) 
        init_y_signal = init_y_signal + str(round(msg.pose.pose.position.y,3))
        init_z_signal = init_z_signal + str(round(msg.pose.pose.orientation.z,3))
        init_w_signal = init_w_signal + str(round(msg.pose.pose.orientation.w,3))

        self.__comm.signal.emit(init_x_signal)
        self.__comm.signal.emit(init_y_signal)
        self.__comm.signal.emit(init_z_signal)
        self.__comm.signal.emit(init_w_signal)

    def goal_pose_callback(self,msg):

        self.__goal_x_signal = 'goal_x|'
        self.__goal_y_signal = 'goal_y|'
        self.__goal_z_signal = 'goal_z|'
        self.__goal_w_signal = 'goal_w|'

        self.__goal_x_signal = self.__goal_x_signal + str(round(msg.goal.target_pose.pose.position.x,3)) 
        self.__goal_y_signal = self.__goal_y_signal + str(round(msg.goal.target_pose.pose.position.y,3))
        self.__goal_z_signal = self.__goal_z_signal + str(round(msg.goal.target_pose.pose.orientation.z,3))
        self.__goal_w_signal = self.__goal_w_signal + str(round(msg.goal.target_pose.pose.orientation.w,3))

        self.__comm.signal.emit(self.__goal_x_signal)
        self.__comm.signal.emit(self.__goal_y_signal)
        self.__comm.signal.emit(self.__goal_z_signal)
        self.__comm.signal.emit(self.__goal_w_signal)

if __name__ == '__main__':
    # rs = RobocareSerial()
    # rs.open('ttyMP0')
    # rs.test()
    # rs.close()
    qt_app = QApplication(sys.argv)
    ui = NavControlUI()
    qt_app.exec_()

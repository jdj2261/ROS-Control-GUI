#-*- coding:utf-8 -*-
# Code formatting: yapf

import rospy
from geometry_msgs.msg import *
from nav_msgs.msg import Odometry
import rospkg

from events.ros_manage import RosManager


class ButtonEvent():

    def __init__(self):
        self.__rm = RosManager()
 

    def go(self, x, y, z, w):
        global g_x, g_y, g_z, g_w
        g_x = x
        g_y = y
        g_z = z
        g_w = w
        print('GO PRESSE!!' + str(x) + " " + str(y) + " " + str(z) + " " + str(w))
        self.__rm.pub_go_to_goal(x,y,z,w)

    def on_stop(self, b, x, y, z, w):
        if b.isChecked():
            print("button pressed")
            rospy.loginfo("Goal cancelled")
            b.setStyleSheet('QPushButton {color: red}')
            b.setDefault(True)
            self.__rm.pub_cancel_goal()
        else :
            print("button released")
            rospy.loginfo("You can move the robot!")
            b.setStyleSheet('QPushButton {color: black}')
            b.setDefault(False)
            self.__rm.pub_go_to_goal(x,y,z,w)
            rospy.loginfo("Moving~~")





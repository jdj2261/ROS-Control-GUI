#-*- coding:utf-8 -*-
import rospy

# from PySide.QtCore import *
# from PySide import QtGui, QtCore

from std_msgs.msg import Bool
from geometry_msgs.msg import *
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseActionGoal
from actionlib_msgs.msg import GoalID

# class Communicate(QtCore.QObject):
#     signal = QtCore.Signal(str)

class RosManager():
    
    def __init__(self):

        # self.__comm = Communicate()
        # self.__comm.signal.connect(self.get_signal)

        self.__pub_go_to_goal = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=1)
        self.__pub_cancel_goal = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)

        # rospy.Subscriber('/odom', Odometry, self.odom_callback)

    def pub_go_to_goal(self, x, y, z, w = '1.0'):

        goal_target = MoveBaseActionGoal()

        goal_target.goal.target_pose.header.frame_id = "map"      
        goal_target.goal.target_pose.header.stamp = rospy.get_rostime()                     
        goal_target.goal.target_pose.pose.position.x = float(x)                    
        goal_target.goal.target_pose.pose.position.y = float(y)
        goal_target.goal.target_pose.pose.orientation.z = float(z)
        goal_target.goal.target_pose.pose.orientation.w = float(w)

        self.__pub_go_to_goal.publish(goal_target)

    def pub_cancel_goal(self):
        goal_cancel = GoalID()
        self.__pub_cancel_goal.publish(goal_cancel)
        
    # def odom_callback(self, msg):
    #     pos_x_signal = 'pos_x|'
    #     pos_y_signal = 'pos_y|'
    #     pos_z_signal = 'pos_z|'

    #     vel_x_signal = 'vel_x|'
    #     vel_y_signal = 'vel_y|'
    #     vel_z_signal = 'vel_z|'

    #     pos_x_signal = pos_x_signal + str(round(msg.pose.pose.position.x,3)) 
    #     pos_y_signal = pos_y_signal + str(round(msg.pose.pose.position.y,3))
    #     pos_z_signal = pos_z_signal + str(round(msg.pose.pose.orientation.z,3))

    #     vel_x_signal = vel_x_signal + str(round(msg.twist.twist.linear.x,3))
    #     vel_y_signal = vel_y_signal + str(round(msg.twist.twist.linear.y,3))
    #     vel_z_signal = vel_z_signal + str(round(msg.twist.twist.angular.z,3))

    #     self.__comm.signal.emit(pos_x_signal)
    #     self.__comm.signal.emit(pos_y_signal)
    #     self.__comm.signal.emit(pos_z_signal)
    #     self.__comm.signal.emit(vel_x_signal)
    #     self.__comm.signal.emit(vel_y_signal)
    #     self.__comm.signal.emit(vel_z_signal)


        

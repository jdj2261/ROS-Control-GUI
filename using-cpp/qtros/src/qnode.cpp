/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "qtros/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace qtros {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
  init_argc(argc),
  init_argv(argv)
{}

QNode::~QNode() {
  if(ros::isStarted()) {
    ros::shutdown(); // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  wait();
}

bool QNode::init() {
  ros::init(init_argc,init_argv,"qtros");
//  if ( ! ros::master::check() ) {
//    return false;
//  }
  ros::start(); // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle n;
  // Add your ros communications here.


  cmd_vel_pub_      = n.advertise<geometry_msgs::Twist>("cmd_vel",1);
  goal_pose_pub_    = n.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal",1);
  simple_goal_pub_  = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1);
  cancel_goal_pub_  = n.advertise<actionlib_msgs::GoalID>("/move_base/cancel",1);

  odom_sub_         = n.subscribe("/odom",1,&QNode::OdomCallback, this);
  init_pose_sub_    = n.subscribe("/initialpose",1,&QNode::InitialPoseCallback, this);
  goal_pose_sub_    = n.subscribe("/move_base/goal",1,&QNode::GoalPoseCallback, this);

  start();
  return true;
}

void QNode::OdomCallback(const nav_msgs::Odometry &msg)
{
  m_pose_x = msg.pose.pose.position.x;
  m_pose_y = msg.pose.pose.position.y;
  m_pose_z = msg.pose.pose.orientation.z;
  m_pose_w = msg.pose.pose.orientation.w;

  m_vel_x = msg.twist.twist.linear.x;
  m_vel_y = msg.twist.twist.linear.y;
  m_vel_z = msg.twist.twist.angular.z;

  Q_EMIT OdomUpdated(m_pose_x);
  Q_EMIT OdomUpdated(m_pose_y);
  Q_EMIT OdomUpdated(m_pose_z);
  Q_EMIT OdomUpdated(m_pose_w);

  Q_EMIT OdomUpdated(m_vel_x);
  Q_EMIT OdomUpdated(m_vel_y);
  Q_EMIT OdomUpdated(m_vel_z);

}

void QNode::InitialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped &msg)
{
  m_init_x = msg.pose.pose.position.x;
  m_init_y = msg.pose.pose.position.y;
  m_init_z = msg.pose.pose.orientation.z;
  m_init_w = msg.pose.pose.orientation.w;

  Q_EMIT InitUpdated(m_init_x);
  Q_EMIT InitUpdated(m_init_y);
  Q_EMIT InitUpdated(m_init_z);
  Q_EMIT InitUpdated(m_init_w);
}

void QNode::GoalPoseCallback(const move_base_msgs::MoveBaseActionGoal &msg)
{
  m_goal_x = msg.goal.target_pose.pose.position.x;
  m_goal_y = msg.goal.target_pose.pose.position.y;
  m_goal_z = msg.goal.target_pose.pose.orientation.z;
  m_goal_w = msg.goal.target_pose.pose.orientation.w;

  Q_EMIT GoalUpdated(m_goal_x);
  Q_EMIT GoalUpdated(m_goal_y);
  Q_EMIT GoalUpdated(m_goal_z);
  Q_EMIT GoalUpdated(m_goal_w);

}

void QNode::run() {
  ros::Rate loop_rate(50);
  int count = 0;
  while ( ros::ok() ) {

//    std_msgs::String msg;
//    std::stringstream ss;
//    ss << "hello world " << count;
//    msg.data = ss.str();
//    chatter_publisher_.publish(msg);
//    log(Info,std::string("I sent: ")+msg.data);
    ros::spinOnce();
    loop_rate.sleep();
//    ++count;
  }
  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
  Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::log( const LogLevel &level, const std::string &msg) {
  logging_model.insertRows(logging_model.rowCount(),1);
  std::stringstream logging_model_msg;
  switch ( level ) {
  case(Debug) : {
    ROS_DEBUG_STREAM(msg);
    logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
    break;
  }
  case(Info) : {
    ROS_INFO_STREAM(msg);
    logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
    break;
  }
  case(Warn) : {
    ROS_WARN_STREAM(msg);
    logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
    break;
  }
  case(Error) : {
    ROS_ERROR_STREAM(msg);
    logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
    break;
  }
  case(Fatal) : {
    ROS_FATAL_STREAM(msg);
    logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
    break;
  }
  }
  QVariant new_row(QString(logging_model_msg.str().c_str()));
  logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
  Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::sendCmdVelMsg(geometry_msgs::Twist msg)
{
  cmd_vel_pub_.publish( msg );
  log( Info , "Send CmdMsg" );
}

void QNode::sendGoalMsg(move_base_msgs::MoveBaseActionGoal msg)
{
  goal_pose_pub_.publish(msg);
  log( Info , "Send GoalMsg");
}

void QNode::sendSimpleGoalMsg(geometry_msgs::PoseStamped msg)
{
  simple_goal_pub_.publish(msg);
  log( Info , "Send SimpleGoalMsg");
}

void QNode::sendCancelGoalMsg(actionlib_msgs::GoalID msg)
{
  cancel_goal_pub_.publish(msg);
  log( Info , "Send SimpleGoalMsg");
}

//void QNode::simple_goal_pub_(move_base_msgs::MoveBaseActionGoal msg)
//{
//  simple_goal_pub_.publish(msg);
//  log( Info , "Send simple_goal_pub");
//}

}  // namespace qtros

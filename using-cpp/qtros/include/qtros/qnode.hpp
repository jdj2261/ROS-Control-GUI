/**
 * @file /include/qtros/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef qtros_QNODE_HPP_
#define qtros_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <move_base_msgs/MoveBaseActionGoal.h>
#include <actionlib_msgs/GoalID.h>
#include <nav_msgs/Odometry.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace qtros {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
  Q_OBJECT
public:
  QNode(int argc, char** argv );
  virtual ~QNode();
  bool init();
  bool init(const std::string &master_url, const std::string &host_url);
  void run();

  /*********************
  ** Logging
  **********************/
  enum LogLevel
  {
    Debug, Info, Warn, Error,Fatal
  };

  QStringListModel* loggingModel()
  {
    return &logging_model;
  }

  void sendCmdVelMsg(geometry_msgs::Twist msg);
  void sendGoalMsg(move_base_msgs::MoveBaseActionGoal msg);
  void sendSimpleGoalMsg(geometry_msgs::PoseStamped msg);
  void log( const LogLevel &level, const std::string &msg);
  void OdomCallback(const nav_msgs::Odometry &msg);
  void InitialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped &msg);
  void GoalPoseCallback(const move_base_msgs::MoveBaseActionGoal &msg);


Q_SIGNALS:
  void loggingUpdated();
  void rosShutdown();

private:
  int init_argc;
  char** init_argv;

  ros::Publisher chatter_publisher_;
  ros::Publisher cmd_vel_pub_;
  ros::Publisher goal_pose_pub_;
  ros::Publisher simple_goal_pub_;
  ros::Publisher cancel_goal_pub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber init_pose_sub_;
  ros::Subscriber goal_pose_sub_;


  QStringListModel logging_model;
};

}  // namespace qtros

#endif /* qtros_QNODE_HPP_ */

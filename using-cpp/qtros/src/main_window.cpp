/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "qtros/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace qtros {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
  : QMainWindow(parent)
  , qnode(argc,argv)
{
  ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
  QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

  setWindowIcon(QIcon(":/images/icon.png"));
  ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

  /*********************
  ** Logging
  **********************/
  ui.view_logging->setModel(qnode.loggingModel());
  QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));


  /*******************************
  ** Button test - explicit way
  ********************************/
//  qRegisterMetaType<geometry_msgs::Twist>("geometry_msgs::Twist");
  QObject::connect(ui.go_Button, SIGNAL(clicked()), this, SLOT(go_to_goal()));
  QObject::connect(ui.stop_button, SIGNAL(clicked()), this, SLOT(go_to_cancel()));
  QObject::connect(ui.clear_costmap_button, SIGNAL(clicked()), this, SLOT(clear_costmap()));

  QObject::connect(&qnode, SIGNAL(OdomUpdated(float)), this, SLOT(updateOdom(float)));
  QObject::connect(&qnode, SIGNAL(InitUpdated(float)), this, SLOT(updateInit(float)));
  QObject::connect(&qnode, SIGNAL(GoalUpdated(float)), this, SLOT(updateGoal(float)));


//  ui.lineEdit_4;

  qnode.init();
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
  QMessageBox msgBox;
  msgBox.setText("Couldn't find the ros master.");
  msgBox.exec();
  close();
}

void MainWindow::showButtonTestMessage() {
    QMessageBox msgBox;
    msgBox.setText("Button test ...");
    msgBox.exec();
    //close();
}


/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

//void MainWindow::on_button_connect_clicked(bool check ) {
//  if ( ui.checkbox_use_environment->isChecked() ) {
//    if ( !qnode.init() ) {
//      showNoMasterMessage();
//    } else {
//      ui.button_connect->setEnabled(false);
//    }
//  } else {
//    if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
//                      ui.line_edit_host->text().toStdString()) ) {
//      showNoMasterMessage();
//    } else {
//      ui.button_connect->setEnabled(false);
//      ui.line_edit_master->setReadOnly(true);
//      ui.line_edit_host->setReadOnly(true);
//      ui.line_edit_topic->setReadOnly(true);
//    }
//  }
//}


void MainWindow::on_go_Button_clicked(bool check )
{

    m_goal_msg.goal.target_pose.header.frame_id = "map";
    m_goal_msg.goal.target_pose.header.stamp = ros::Time();
    m_goal_msg.goal.target_pose.pose.position.x = ui.go_to_goal_x->text().toFloat();
    m_goal_msg.goal.target_pose.pose.position.y = ui.go_to_goal_y->text().toFloat();
    m_goal_msg.goal.target_pose.pose.orientation.z = ui.go_to_goal_theta->text().toFloat();
    m_goal_msg.goal.target_pose.pose.orientation.w = ui.go_to_goal_w->text().toFloat();

    m_simple_goal_msg.header.frame_id = "map";
    m_simple_goal_msg.header.stamp = ros::Time();
    m_simple_goal_msg.pose.position.x = ui.go_to_goal_x->text().toFloat();
    m_simple_goal_msg.pose.position.y = ui.go_to_goal_y->text().toFloat();;
    m_simple_goal_msg.pose.orientation.z = ui.go_to_goal_theta->text().toFloat();;
    m_simple_goal_msg.pose.orientation.w = ui.go_to_goal_w->text().toFloat();;

    qnode.sendGoalMsg(m_goal_msg);
    qnode.sendSimpleGoalMsg(m_simple_goal_msg);

}

void MainWindow::on_stop_button_clicked(bool check)
{
    actionlib_msgs::GoalID cancel_goal_msg;
    move_base_msgs::MoveBaseActionGoal goal_msg;
    geometry_msgs::PoseStamped simple_goal_msg;

    goal_msg.goal.target_pose.header.frame_id = "map";
    goal_msg.goal.target_pose.header.stamp = ros::Time();
    goal_msg.goal.target_pose.pose.position.x = qnode.m_goal_x;
    goal_msg.goal.target_pose.pose.position.y = qnode.m_goal_y;
    goal_msg.goal.target_pose.pose.orientation.z = qnode.m_goal_z;
    goal_msg.goal.target_pose.pose.orientation.w = qnode.m_goal_w;

    simple_goal_msg.header.frame_id = "map";
    simple_goal_msg.header.stamp = ros::Time();
    simple_goal_msg.pose.position.x = qnode.m_goal_x;
    simple_goal_msg.pose.position.y = qnode.m_goal_y;
    simple_goal_msg.pose.orientation.z = qnode.m_goal_z;
    simple_goal_msg.pose.orientation.w = qnode.m_goal_w;

    if (ui.stop_button->isChecked()) qnode.sendCancelGoalMsg(cancel_goal_msg);
    else
    {
      qnode.sendGoalMsg(goal_msg);
//      qnode.sendSimpleGoalMsg(simple_goal_msg);
    }
}

void MainWindow::on_clear_costmap_button_clicked(bool check)
{
    std_srvs::Empty em;
    qnode.sendClearCostmapSrv(em);
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView()
{
  ui.view_logging->scrollToBottom();
}

void MainWindow::go_to_goal()
{
    logging_model = qnode.loggingModel();
    logging_model->insertRows(logging_model->rowCount(), 1);

    QString qstr_pos_x, qstr_pos_y, qstr_pos_theta, qstr_pos_w;

    qstr_pos_x      = ui.go_to_goal_x->text();
    qstr_pos_y      = ui.go_to_goal_y->text();
    qstr_pos_theta  = ui.go_to_goal_theta->text();
    qstr_pos_w      = ui.go_to_goal_w->text();

    std::stringstream logging_model_msg;
    logging_model_msg << "go to goal -->" << " " << qstr_pos_x.toStdString()
                      << " " << qstr_pos_y.toStdString()
                      << " " << qstr_pos_theta.toStdString()
                      << " " << qstr_pos_w.toStdString();

    QVariant new_row(QString(logging_model_msg.str().c_str()));
    logging_model->setData(logging_model->index(logging_model->rowCount()-1), new_row);

    std::cout << logging_model_msg.str().c_str() << std::endl;
}

void MainWindow::go_to_cancel()
{
    logging_model = qnode.loggingModel();
    logging_model->insertRows(logging_model->rowCount(), 1);
    std::stringstream logging_model_msg;

    if (ui.stop_button->isChecked())
    {
      logging_model_msg << "STOP!!";
    }
    else
    {
      logging_model_msg << "RELEASE!!";
    }

    QVariant new_row(QString(logging_model_msg.str().c_str()));
    logging_model->setData(logging_model->index(logging_model->rowCount()-1), new_row);

    std::cout << logging_model_msg.str().c_str() << std::endl;
}

void MainWindow::clear_costmap()
{
  logging_model = qnode.loggingModel();
  logging_model->insertRows(logging_model->rowCount(), 1);
  std::stringstream logging_model_msg;
  logging_model_msg << "Clear Costmap!!";

  QVariant new_row(QString(logging_model_msg.str().c_str()));
  logging_model->setData(logging_model->index(logging_model->rowCount()-1), new_row);

  std::cout << logging_model_msg.str().c_str() << std::endl;
}

void MainWindow::updateOdom(float)
{
    float pose_value[4] = {0.0,};
    pose_value[0] = qnode.m_pose_x;
    pose_value[1] = qnode.m_pose_y;
    pose_value[2] = qnode.m_pose_z;
    pose_value[3] = qnode.m_pose_w;

    float vel_value[4] = {0.0,};
    vel_value[0] = qnode.m_vel_x;
    vel_value[1] = qnode.m_vel_y;
    vel_value[2] = qnode.m_vel_z;
    vel_value[3] = qnode.m_vel_w;

    ui.Pose_x->setAlignment(Qt::AlignRight);
    ui.Pose_y->setAlignment(Qt::AlignRight);
    ui.Pose_theta->setAlignment(Qt::AlignRight);
    ui.Pose_w->setAlignment(Qt::AlignRight);

    ui.Vel_x->setAlignment(Qt::AlignRight);
    ui.Vel_y->setAlignment(Qt::AlignRight);
    ui.Vel_theta->setAlignment(Qt::AlignRight);
    ui.Vel_w->setAlignment(Qt::AlignRight);

    ui.Pose_x->setText(QString::number(pose_value[0]));
    ui.Pose_y->setText(QString::number(pose_value[1]));
    ui.Pose_theta->setText(QString::number(pose_value[2]));
    ui.Pose_w->setText(QString::number(pose_value[3]));

    ui.Vel_x->setText(QString::number(vel_value[0]));
    ui.Vel_y->setText(QString::number(vel_value[1]));
    ui.Vel_theta->setText(QString::number(vel_value[2]));
    ui.Vel_w->setText("-");
}

void MainWindow::updateInit(float)
{
    float init_value[4] = {0.0,};
    init_value[0] = qnode.m_init_x;
    init_value[1] = qnode.m_init_y;
    init_value[2] = qnode.m_init_z;
    init_value[3] = qnode.m_init_w;

    ui.Init_x->setAlignment(Qt::AlignRight);
    ui.Init_y->setAlignment(Qt::AlignRight);
    ui.Init_theta->setAlignment(Qt::AlignRight);
    ui.Init_w->setAlignment(Qt::AlignRight);

    ui.Init_x->setText(QString::number(init_value[0]));
    ui.Init_y->setText(QString::number(init_value[1]));
    ui.Init_theta->setText(QString::number(init_value[2]));
    ui.Init_w->setText(QString::number(init_value[3]));
}

void MainWindow::updateGoal(float)
{
    float goal_value[4] = {0.0,};
    goal_value[0] = qnode.m_goal_x;
    goal_value[1] = qnode.m_goal_y;
    goal_value[2] = qnode.m_goal_z;
    goal_value[3] = qnode.m_goal_w;

    ui.Goal_x->setAlignment(Qt::AlignRight);
    ui.Goal_y->setAlignment(Qt::AlignRight);
    ui.Goal_theta->setAlignment(Qt::AlignRight);
    ui.Goal_w->setAlignment(Qt::AlignRight);

    ui.Goal_x->setText(QString::number(goal_value[0]));
    ui.Goal_y->setText(QString::number(goal_value[1]));
    ui.Goal_theta->setText(QString::number(goal_value[2]));
    ui.Goal_w->setText(QString::number(goal_value[3]));
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
  QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/


void MainWindow::closeEvent(QCloseEvent *event)
{

  QMainWindow::closeEvent(event);
}

}  // namespace qtros


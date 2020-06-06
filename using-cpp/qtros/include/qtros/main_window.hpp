/**
 * @file /include/qtros/main_window.hpp
 *
 * @brief Qt based gui for qtros.
 *
 * @date November 2010
 **/
#ifndef qtros_MAIN_WINDOW_H
#define qtros_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace qtros {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  MainWindow(int argc, char** argv, QWidget *parent = 0);
  ~MainWindow();

  void ReadSettings(); // Load up qt program settings at startup
  void WriteSettings(); // Save qt program settings when closing

  void closeEvent(QCloseEvent *event); // Overloaded function
  void showNoMasterMessage();
  void showButtonTestMessage();

  move_base_msgs::MoveBaseActionGoal m_goal_msg;
  geometry_msgs::PoseStamped m_simple_goal_msg;

public Q_SLOTS:
  /******************************************
  ** Auto-connections (connectSlotsByName())
  *******************************************/
  void on_actionAbout_triggered();
  void on_go_Button_clicked(bool check);
  void on_stop_button_clicked(bool check);
  void on_clear_costmap_button_clicked(bool check);


  /******************************************
    ** Manual connections
    *******************************************/
  void updateLoggingView(); // no idea why this can't connect automatically
  void go_to_goal();
  void go_to_cancel();
  void clear_costmap();
  void updateOdom(float);
  void updateInit(float);
  void updateGoal(float);



private:
  Ui::MainWindowDesign ui;
  QNode qnode;
  QStringListModel* logging_model;
};

}  // namespace qtros

#endif // qtros_MAIN_WINDOW_H

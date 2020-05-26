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
  QObject::connect(ui.left_button, SIGNAL(clicked()), this, SLOT(moveLeft()));
  QObject::connect(ui.test_button, SIGNAL(clicked()), this, SLOT(logTest()));

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

void MainWindow::on_test_button_clicked(bool check ) {
    showButtonTestMessage();
    geometry_msgs::Twist msg;
    msg.linear.x = 1.0;
    msg.angular.z = 0.5;
    qnode.sendCmdVelMsg(msg);
}


/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
  ui.view_logging->scrollToBottom();
}

void MainWindow::moveLeft() {
    logging_model = qnode.loggingModel();
    logging_model->insertRows(logging_model->rowCount(), 1);
    std::stringstream logging_model_msg;
    logging_model_msg << "move to left ...";
    QVariant new_row(QString(logging_model_msg.str().c_str()));
    logging_model->setData(logging_model->index(logging_model->rowCount()-1), new_row);

    std::cout << logging_model->rowCount() << std::endl;
    std::cout << logging_model_msg.str().c_str() << std::endl;
}

void MainWindow::logTest() {
    logging_model = qnode.loggingModel();
    logging_model->insertRows(logging_model->rowCount(), 1);
    std::stringstream logging_model_msg;
    QString qstr_pos_x, qstr_pos_y, qstr_pos_theta;

    qstr_pos_x      = ui.Pose_x->text();
    qstr_pos_y      = ui.Pose_y->text();
    qstr_pos_theta  = ui.Pose_theta->text();


    std::cout << "Test : " << qstr_pos_x.toFloat() << std::endl;
    std::cout << "Test : " << qstr_pos_y.toFloat() << std::endl;
    std::cout << "Test : " << qstr_pos_theta.toFloat() << std::endl;

    logging_model_msg << "logging test";
    QVariant new_row(QString(logging_model_msg.str().c_str()));
    logging_model->setData(logging_model->index(logging_model->rowCount()-1), new_row);

//    std::cout << logging_model->rowCount() << std::endl;
    std::cout << logging_model_msg.str().c_str() << std::endl;
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


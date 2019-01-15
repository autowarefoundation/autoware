// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/


#ifndef JSK_RVIZ_PLUGINS_TABLET_CONTROLLER_PANEL_H_
#define JSK_RVIZ_PLUGINS_TABLET_CONTROLLER_PANEL_H_

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/panel.h>
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
#  include <QtWidgets>
#else
#  include <QtGui>
#endif
#include <QPainter>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QDialog>
#include <QListWidget>
#include <QListWidgetItem>
#include <QLabel>
#include <QTimer>
#include <QRadioButton>
#include <QPaintEvent>
#include <QMouseEvent>
#include <geometry_msgs/Twist.h>
#include <jsk_rviz_plugins/StringStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <boost/thread.hpp>
#endif

namespace jsk_rviz_plugins
{
  class TabletCmdVelArea: public QWidget
  {
    Q_OBJECT
  public:
    TabletCmdVelArea(QWidget* parent, ros::Publisher& pub_cmd_vel);
    virtual QSize minimumSizeHint() const;
    virtual QSize sizeHint() const;
  protected:
    virtual void paintEvent(QPaintEvent* event);
    virtual void mouseMoveEvent(QMouseEvent* event);
    virtual void mousePressEvent(QMouseEvent* event);
    virtual void mouseReleaseEvent(QMouseEvent* event);
    virtual void publishVelocity(int mouse_x, int mouse_y, int cx, int cy);
    virtual void publishCmdVel(double x, double y, double theta);
    int mouse_x_;
    int mouse_y_;
    ros::Publisher pub_cmd_vel_;
  };
  
  class TabletControllerPanel: public rviz::Panel
  {
    Q_OBJECT
  public:
    TabletControllerPanel(QWidget* parent = 0);
    virtual ~TabletControllerPanel();
    virtual void load(const rviz::Config& config);
    virtual void save(rviz::Config config) const;

  protected:
    ////////////////////////////////////////////////////////
    // methods
    ////////////////////////////////////////////////////////
    virtual void spotCallback(
      const visualization_msgs::MarkerArray::ConstPtr& marker);
    virtual QString defaultButtonStyleSheet();
    virtual QString executeButtonStyleSheet();
    virtual QString radioButtonStyleSheet();
    virtual QString listStyleSheet();
    ////////////////////////////////////////////////////////
    // GUI variables
    ////////////////////////////////////////////////////////
    
    QVBoxLayout* layout_;
    QPushButton* task_button_;
    QPushButton* spot_button_;
    TabletCmdVelArea* cmd_vel_area_;
    
    QDialog* task_dialog_;
    QVBoxLayout* task_dialog_layout_;
    QHBoxLayout* task_dialog_button_layout_;
    QPushButton* task_execute_button_;
    QPushButton* task_cancel_button_;
    std::vector<QRadioButton*> task_radio_buttons_;

    std::vector<std::string> spots_;
    QDialog* spot_dialog_;
    QVBoxLayout* spot_dialog_layout_;
    QHBoxLayout* spot_dialog_button_layout_;
    QPushButton* spot_go_button_;
    QPushButton* spot_cancel_button_;
    QListWidget* spot_list_;
    ////////////////////////////////////////////////////////
    // ROS variables
    ////////////////////////////////////////////////////////
    ros::Publisher pub_cmd_vel_;
    ros::Publisher pub_spot_;
    ros::Publisher pub_start_demo_;
    ros::Subscriber sub_spots_;
    boost::mutex mutex_;
    
    
  protected Q_SLOTS:
    ////////////////////////////////////////////////////////
    // callbacks
    ////////////////////////////////////////////////////////
    void taskButtonClicked();
    void taskCancelClicked();
    void taskExecuteClicked();
    void spotButtonClicked();
    void spotGoClicked();
    void spotCancelClicked();
  private:
    
  };
}

#endif

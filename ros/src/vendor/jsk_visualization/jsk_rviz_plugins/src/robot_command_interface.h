#ifndef ROBOT_COMMAND_INTERFACE_H
#define ROBOT_COMMAND_INTERFACE_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/panel.h>
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
#  include <QtWidgets>
#else
#  include <QtGui>
#endif
#include <jsk_rviz_plugins/EusCommand.h>
#include <resource_retriever/retriever.h>
#endif

namespace jsk_rviz_plugins
{
  class RobotCommandInterfaceAction: public rviz::Panel
  {
    Q_OBJECT
    public:
    RobotCommandInterfaceAction( QWidget* parent = 0 );

  protected Q_SLOTS:
    bool callRequestEusCommand(const std::string& command);
    void buttonCallback(int i);
  protected:
    void popupDialog(const std::string& text);
    // The ROS node handle.
    ros::NodeHandle nh_;
    QSignalMapper* signal_mapper_;
    std::map<int, std::string> euscommand_mapping_;
    std::map<int, std::string> emptyservice_mapping_;
    //std::vector<QToolButton*> buttons_;
  };

}

#endif // TELEOP_PANEL_H

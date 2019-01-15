#include <stdio.h>

#include "rviz/config.h"
#include "robot_command_interface.h"
#include "ros/time.h"
#include <ros/package.h>
#include <boost/format.hpp>
#include <exception>
#include <std_srvs/Empty.h>

namespace jsk_rviz_plugins
{

  // Exception class
  class RobotCommandParseException: public std::runtime_error
  {
  public:
    RobotCommandParseException(const std::string& text): std::runtime_error(text) {}
  };

  RobotCommandInterfaceAction::RobotCommandInterfaceAction( QWidget* parent )
    : rviz::Panel( parent )
  {
    resource_retriever::Retriever r;
    signal_mapper_ = new QSignalMapper(this);
    ros::NodeHandle nh("~");
    QHBoxLayout* layout = new QHBoxLayout();
    // Parse yaml file from parameter
    if (nh.hasParam("robot_command_buttons")) {
      try {
        XmlRpc::XmlRpcValue robot_command_buttons_xmlrpc;
        nh.param("robot_command_buttons", robot_command_buttons_xmlrpc, robot_command_buttons_xmlrpc);
        if (robot_command_buttons_xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeArray) {
          throw RobotCommandParseException("~robot_comamnd_buttons should be an array");
        }
        else {
          for (size_t i = 0; i < robot_command_buttons_xmlrpc.size(); i++) {
            XmlRpc::XmlRpcValue button_xmlrpc = robot_command_buttons_xmlrpc[i];
            if (button_xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
              throw RobotCommandParseException("element of ~robot_comamnd_buttons should be an struct");
            }
            else {
              std::string name;
              QToolButton* button = new QToolButton();
              //button->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
              button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

              if (button_xmlrpc.hasMember("name")) {
                name = (std::string)button_xmlrpc["name"];
              }
              else {
                throw RobotCommandParseException("element of ~robot_comamnd_buttons should have name field");
              }
              button->setText(QString(name.c_str()));
              if (button_xmlrpc.hasMember("icon")) {
                // TODO: resolve path
                std::string icon;
                icon = (std::string)button_xmlrpc["icon"];
                if (icon.find("package://") == 0) {
                  icon.erase(0, strlen("package://"));
                  size_t package_end = icon.find("/");
                  std::string package = icon.substr(0, package_end);
                  icon.erase(0, package_end);
                  std::string package_path;
                  package_path = ros::package::getPath(package);
                  icon = package_path + icon;
                }
                button->setIcon(QIcon(QPixmap(QString(icon.c_str()))));
                button->setIconSize(QSize(80, 80));
                button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
              }
              std::string type;
              if (button_xmlrpc.hasMember("type")) {
                type = (std::string)button_xmlrpc["type"];
              }
              if (type == "euscommand") {
                if (button_xmlrpc.hasMember("command")) {
                  euscommand_mapping_[i] = (std::string)button_xmlrpc["command"];
                  button->setToolTip(euscommand_mapping_[i].c_str());
                }
                else {
                  throw RobotCommandParseException("type: euscommand requires command field");
                }
              }
              else if (type == "emptysrv") {
                if (button_xmlrpc.hasMember("srv")) {
                  emptyservice_mapping_[i] = (std::string)button_xmlrpc["srv"];
                  button->setToolTip(emptyservice_mapping_[i].c_str());
                }
                else {
                  throw RobotCommandParseException("type: emptysrv requires srv field");
                }
              }
              else {
                throw RobotCommandParseException("type field is required");
              }
              // connect
              connect(button, SIGNAL(clicked()), signal_mapper_, SLOT(map()));
              signal_mapper_->setMapping(button, i);
              layout->addWidget(button);
            }
          }
        }
      }
      catch (RobotCommandParseException& e) {
        popupDialog((boost::format("Malformed ~robot_command_buttons parameter.\n"
                                  "%s\n"
                                  "See package://jsk_rviz_plugins/config/default_robot_command.yaml")
                     % e.what()).str().c_str());
      }
    }
    else {
      popupDialog("You need to specify ~robot_command_buttons parameter.\n"
                  "See package://jsk_rviz_plugins/launch/robot_command_interface_sample.launch");
    }
    layout->addStretch();
    connect(signal_mapper_, SIGNAL(mapped(int)), this, SLOT(buttonCallback(int)));
    // QToolButton* button = new QToolButton();
    
    // // button->setPopupMode(QToolButton::MenuButtonPopup);
    // button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    // button->setIcon(QIcon(QPixmap(QString("/home/garaemon/ros/hydro/src/jsk-ros-pkg/jsk_visualization/jsk_rviz_plugins/icons/stop-imp.png"))));
    
    // button->setText("Hello worpld");
    // layout->addWidget(button);
    this->setLayout(layout);
  }

  bool RobotCommandInterfaceAction::callRequestEusCommand(const std::string& command){
    ros::ServiceClient client = nh_.serviceClient<jsk_rviz_plugins::EusCommand>("/eus_command", true);
    jsk_rviz_plugins::EusCommand srv;
    srv.request.command = command;
    return client.call(srv);
  }

  void RobotCommandInterfaceAction::buttonCallback(int i)
  {
    ROS_INFO("buttonCallback(%d)", i);
    if (euscommand_mapping_.find(i) != euscommand_mapping_.end()) {
      if(!callRequestEusCommand(euscommand_mapping_[i])) {
        popupDialog((boost::format("Failed to call %s") % euscommand_mapping_[i]).str().c_str());
      }
    }
    else if (emptyservice_mapping_.find(i) != emptyservice_mapping_.end()) {
      std_srvs::Empty emp;
      if (!ros::service::call(emptyservice_mapping_[i], emp)) {
        popupDialog((boost::format("Failed to call %s") % emptyservice_mapping_[i]).str().c_str());
      }
    }
    else {
      popupDialog((boost::format("Failed to find corresponding command for %d") % i).str().c_str());
    }
  }

  void RobotCommandInterfaceAction::popupDialog(const std::string& text)
  {
    QMessageBox msg_box;
    msg_box.setText("Unexpected error");
    msg_box.setText(QString(text.c_str()));
    msg_box.exec();
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jsk_rviz_plugins::RobotCommandInterfaceAction, rviz::Panel )

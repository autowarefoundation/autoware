#include <ros/ros.h>
#include "autoware_rosbag_plugin.h"
#include <QApplication>




int main(int argc, char **argv)
{
  ros::init(argc, argv, "offline_test");
  ros::NodeHandle nh;

  QApplication a(argc, argv);
  Autoware_Rosbag_Plugin w;
  w.show();

  return a.exec();

//  ROS_INFO("Hello world!");
}

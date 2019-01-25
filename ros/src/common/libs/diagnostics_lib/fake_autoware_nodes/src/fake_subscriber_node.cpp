#include <ros/ros.h>

//headers in diag_lib
#include <diag_lib/diag_manager.h>

//headers in fake_autoware_nodes
#include <fake_autoware_nodes/fake_subscriber.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "fake_subscriber_node");
  fake_subscriber fake_sub;
  ros::spin();
  return 0;
}
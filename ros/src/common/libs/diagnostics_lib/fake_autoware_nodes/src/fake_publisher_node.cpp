//headers in ROS
#include <ros/ros.h>

//headers in fake_autoware_nodes
#include <fake_autoware_nodes/fake_publisher.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "fake_publisher_node");
  fake_publisher fake_pub;
  fake_pub.run();
  return 0;
}
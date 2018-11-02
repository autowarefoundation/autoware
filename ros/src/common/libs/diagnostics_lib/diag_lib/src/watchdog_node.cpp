#include <ros/ros.h>

#include <diag_lib/watchdog.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "watchdog_node");
  watchdog wd;
  wd.run();
  return 0;
}
#include <ros/ros.h>

#include <lgsvl_simulator_bridge/lgsvl_simulator_launcher.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "lgsvl_sim_launcher");
  lgsvl_simulator_launcher launcher;
  launcher.launch();
  return 0;
}
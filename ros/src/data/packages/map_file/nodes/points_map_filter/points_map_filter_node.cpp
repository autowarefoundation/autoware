// headers for ros
#include <ros/ros.h>

#include <map_file/points_map_filter.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "points_map_filter");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  points_map_filter filter(nh,pnh);
  filter.init();
  filter.run();
  ros::spin();
  return 0;
}
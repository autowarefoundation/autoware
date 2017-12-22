#ifndef WAYAREA_TO_GRID_H
#define WAYAREA_TO_GRID_H

// C++ includes
#include <iostream>
#include <vector>
#include <string>
#include <chrono>

// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_listener.h>

// Autoware includes
#include <vector_map/vector_map.h>

// Grid Map library
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_cv/grid_map_cv.hpp>

// Open CV
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

namespace object_map
{

class WayareaToGrid
{
public:
  WayareaToGrid();
  ~WayareaToGrid();

  void run() const;

private:
  // handle
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // publisher
  ros::Publisher grid_map_pub_;

  // ros param
  std::string sensor_frame_;
  std::string map_frame_;
  double map_resolution_;
  double map_length_x_;
  double map_length_y_;
  double map_position_x_;
  double map_position_y_;

  // variables
  std::vector<std::vector<geometry_msgs::Point>> area_points_;
  tf::TransformListener tf_listener_;

  // initializer
  void initForROS();
  void initForVMap();

  // function
  std::vector<geometry_msgs::Point> searchAreaPoints(const vector_map::Area& area, const vector_map::VectorMap& vmap) const;
  void fillPolygonCV(grid_map::GridMap& map, const std::vector<std::vector<geometry_msgs::Point>>& area_points_vec) const;
};

}  // namespace object_map

#endif  // WAYAREA_TO_GRID

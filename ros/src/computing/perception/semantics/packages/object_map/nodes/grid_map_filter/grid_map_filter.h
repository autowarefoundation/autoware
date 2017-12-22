#ifndef GRID_MAP_FILTER_H
#define GRID_MAP_FILTER_H

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

class GridMapFilter
{
public:
  GridMapFilter();
  ~GridMapFilter();

  void run() const;

private:
  // handle
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // publisher
  ros::Publisher grid_map_pub_;

  // subscriber
  ros::Subscriber occupancy_grid_sub_;

  // ros param
  std::string map_frame_;
  std::string map_topic_;
  double dist_transform_distance_;
  bool use_dist_transform_;
  bool use_wayarea_;
  bool use_fill_circle_;
  int fill_circle_cost_thresh_;
  double circle_radius_;

  // variables
  std::vector<std::vector<geometry_msgs::Point>> area_points_;
  tf::TransformListener tf_listener_;

  // callbacks
  void callbackFromOccupancyGrid(const nav_msgs::OccupancyGridConstPtr& msg) const;

  // initializer
  void initForROS();
  void initForVMap();

  // function
  std::vector<geometry_msgs::Point> searchAreaPoints(const vector_map::Area& area, const vector_map::VectorMap& vmap) const;
  void distanceTransform(grid_map::GridMap& map, const std::string& layer) const;
  void fillPolygonCV(grid_map::GridMap& map, const std::vector<std::vector<geometry_msgs::Point>>& area_points_vec) const;
  void fillCircleCV(grid_map::GridMap& map, const std::string& layer, const double cost_threshold, const double radius) const;
};

}  // namespace object_map
#endif  // GRID_MAP_FILTER_H

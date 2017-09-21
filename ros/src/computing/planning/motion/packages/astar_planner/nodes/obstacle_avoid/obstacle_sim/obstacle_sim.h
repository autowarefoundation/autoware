/*

*/

#ifndef OBSTACLE_SIM_H
#define OBSTACLE_SIM_H

// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Polygon.h>

// C++ includes
#include <iostream>
#include <vector>

namespace astar_planner
{

class ObstacleSim
{
public:
  ObstacleSim();
  ~ObstacleSim();

  void run();
  
private:
  // handle
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // ros publisher
  ros::Publisher lane_array_pub_;
  ros::Publisher obstacle_sim_points_pub_;
  ros::Publisher obstacle_sim_pointcloud_pub_;
  ros::Publisher obstacle_marker_pub_;

  // ros subscriver
  ros::Subscriber nav_goal_sub_;

  // callbacks
  void callbackFromNavGoal(const geometry_msgs::PoseStampedConstPtr& msg);

  // initializer
  void initForROS();

  void publishPoints();

  // debug
  void displayObstaclePoints();
  void displayObstacleMarker(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2, const geometry_msgs::Point& p3, const geometry_msgs::Point& p4, const std::string& frame);

  // ros param
  double obstacle_height_;
  double obstacle_width_;
  double points_interval_;
  std::string obstacle_frame_;

  std::vector<geometry_msgs::Point> obstacle_points_;
  tf::TransformListener tf_listener_;
  std::string world_frame_;

  // flag
  bool sub_navgoal;

}; // class ObstacleSim

} // namespace astar_planner

#endif  /* ifndef COMMAND_SENDER_H */

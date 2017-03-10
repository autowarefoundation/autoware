#ifndef ASTAR_NAVI_NODE_H
#define ASTAR_NAVI_NODE_H

#define DEBUG 0

#include "astar_util.h"
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <iostream>
#include <vector>
#include <queue>
#include <string>
#include <chrono>

class AstarSearch
{
 public:
  AstarSearch();
  ~AstarSearch();

  //-- FOR DEBUG -------------------------
  void publishPoseArray(const ros::Publisher &pub, const std::string &frame);
  geometry_msgs::PoseArray debug_pose_array_;
  //--------------------------------------

  bool makePlan(const geometry_msgs::Pose &start_pose, const geometry_msgs::Pose &goal_pose, const nav_msgs::OccupancyGrid &map);
  void reset();
  void broadcastPathTF();
  nav_msgs::Path getPath() {return path_;}

 private:
  bool search();
  void resizeNode(int width, int height, int angle_size);
  void createStateUpdateTable(int angle_size);
  void createStateUpdateTableLocal(int angle_size); //
  void poseToIndex(const geometry_msgs::Pose &pose, int *index_x, int *index_y, int *index_theta);
  bool isOutOfRange(int index_x, int index_y);
  void setPath(const SimpleNode &goal);
  void setMap(const nav_msgs::OccupancyGrid &map);
  bool setStartNode();
  bool setGoalNode();
  bool isGoal(double x, double y, double theta);
  bool detectCollision(const SimpleNode &sn);
  bool calcWaveFrontHeuristic(const SimpleNode &sn);
  bool detectCollisionWaveFront(const WaveFrontNode &sn);

  // ROS param
  std::string path_frame_;        // publishing path frame
  int angle_size_;                // descritized angle size
  double minimum_turning_radius_; // varying by vehicles
  int obstacle_threshold_;        // more than this value is regarded as obstacles
  double goal_radius_;            // meter
  double goal_angle_;             // degree
  bool use_back_;                 // use backward driving
  double robot_length_;
  double robot_width_;
  double base2back_;
  double curve_weight_;
  double reverse_weight_;
  bool use_wavefront_heuristic_;
  bool use_2dnav_goal_;

  bool node_initialized_;
  std::vector<std::vector<NodeUpdate>> state_update_table_;
  nav_msgs::MapMetaData map_info_;
  std::vector<std::vector<std::vector<AstarNode>>> nodes_;
  std::priority_queue<SimpleNode, std::vector<SimpleNode>, std::greater<SimpleNode>> openlist_;
  std::vector<SimpleNode> goallist_;

  // Pose in global(/map) frame
  geometry_msgs::PoseStamped start_pose_;
  geometry_msgs::PoseStamped goal_pose_;

  // Pose in OccupancyGrid frame
  geometry_msgs::PoseStamped start_pose_local_;
  geometry_msgs::PoseStamped goal_pose_local_;

  // Transform which converts OccupancyGrid frame to global frame
  tf::Transform map2ogm_;

  tf::TransformListener tf_listener_;
  tf::TransformBroadcaster tf_broadcaster_;

  // Searched path
  nav_msgs::Path path_;
};

#endif // ASTAR_NAVI_NODE_H

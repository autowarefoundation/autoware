#include "search_info_ros.h"

SearchInfo::SearchInfo()
  : map_set_(false)
  , start_set_(false)
  , goal_set_(false)
{
}

SearchInfo::~SearchInfo()
{
}

void SearchInfo::mapCallback(const nav_msgs::OccupancyGridConstPtr &msg)
{
  map_ = *msg;

  // TODO: what frame do we use?
  std::string map_frame = "map";
  std::string ogm_frame = msg->header.frame_id;
  // Set transform
  tf::StampedTransform map2ogm_frame;
  try
  {
    tf_listener_.lookupTransform(map_frame, ogm_frame, ros::Time(0), map2ogm_frame);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    return;
  }

  tf::Transform map2ogm;
  geometry_msgs::Pose ogm_in_map = astar::transformPose(map_.info.origin, map2ogm_frame);
  tf::poseMsgToTF(ogm_in_map, map2ogm);
  ogm2map_ = map2ogm.inverse();

  map_set_ = true;
}


void SearchInfo::startCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
{
  if (!map_set_)
    return;

  start_pose_global_.header = msg->header;
  start_pose_global_.pose   = msg->pose.pose;
  start_pose_local_.pose    = astar::transformPose(start_pose_global_.pose, ogm2map_);
  start_pose_local_.header  = start_pose_global_.header;

  start_set_ = true;
}


void SearchInfo::currentPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
  if (!map_set_)
    return;

  start_pose_global_ = *msg;
  start_pose_local_.pose = astar::transformPose(start_pose_global_.pose, ogm2map_);

  start_set_ = true;
}


void SearchInfo::goalCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
  if (!map_set_)
    return;

  ROS_INFO("Subcscribed goal pose!");

  // TODO: what frame do we use?
  std::string global_frame  = "map";
  std::string goal_frame  = msg->header.frame_id;

  // Get transform (map to world in Autoware)
  tf::StampedTransform map2world;
  try
  {
    tf_listener_.lookupTransform(global_frame, goal_frame, ros::Time(0), map2world);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    return;
  }

  // Set pose in Global frame
  geometry_msgs::Pose msg_pose = msg->pose;
  goal_pose_global_.pose   = astar::transformPose(msg_pose, map2world);
  goal_pose_global_.header = msg->header;
  goal_pose_local_.pose    = astar::transformPose(goal_pose_global_.pose, ogm2map_);
  goal_pose_local_.header = goal_pose_global_.header;

  goal_set_ = true;
}

void SearchInfo::reset()
{
  map_set_   = false;
  start_set_ = false;
  goal_set_  = false;
}

/*
*/

#include "astar_navi.h"

AstarNavi::AstarNavi() : nh_(), private_nh_("~")
{
  private_nh_.param<double>("waypoint_velocity", waypoint_velocity_, 5.0);
  private_nh_.param<double>("update_rate", update_rate_, 1.0);

  lane_pub_ = nh_.advertise<autoware_msgs::LaneArray>("lane_waypoints_array", 1, true);
  debug_pose_pub_ = nh_.advertise<geometry_msgs::PoseArray>("astar_debug_poses", 1, true);
  costmap_sub_ = nh_.subscribe("/grid_map_filter_visualization/distance_transform", 1, &AstarNavi::costmapCallback, this);
  current_pose_sub_ = nh_.subscribe("/current_pose", 1, &AstarNavi::currentPoseCallback, this);
  goal_pose_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &AstarNavi::goalPoseCallback, this);

  costmap_initialized_ = false;
  current_pose_initialized_ = false;
  goal_pose_initialized_ = false;
}

AstarNavi::~AstarNavi()
{
}

void AstarNavi::costmapCallback(const nav_msgs::OccupancyGrid &msg)
{
  costmap_ = msg;
  tf::poseMsgToTF(costmap_.info.origin, local2costmap_);

  costmap_initialized_ = true;
}

void AstarNavi::currentPoseCallback(const geometry_msgs::PoseStamped &msg)
{
  if (!costmap_initialized_)
  {
    return;
  }

  current_pose_.pose = transformPose(msg.pose, getTransform(costmap_.header.frame_id, msg.header.frame_id));
  current_pose_.header.frame_id = costmap_.header.frame_id;
  current_pose_.header.stamp = msg.header.stamp;

  current_pose_initialized_ = true;

  // ROS_INFO_STREAM("Subscribed current pose and transform from " << msg.header.frame_id << " to " <<  current_pose_.header.frame_id << "\n" << current_pose_.pose);
}

void AstarNavi::goalPoseCallback(const geometry_msgs::PoseStamped &msg)
{
  if (!costmap_initialized_)
  {
    return;
  }

  goal_pose_.pose = transformPose(msg.pose, getTransform(costmap_.header.frame_id, msg.header.frame_id));
  goal_pose_.header.frame_id = costmap_.header.frame_id;
  goal_pose_.header.stamp = msg.header.stamp;

  goal_pose_initialized_ = true;

  ROS_INFO_STREAM("Subscribed goal pose and transform from " << msg.header.frame_id << " to " <<  goal_pose_.header.frame_id << "\n" << goal_pose_.pose);
}

tf::Transform AstarNavi::getTransform(const std::string &from, const std::string &to)
{
  tf::StampedTransform stf;
  try
  {
    tf_listener_.lookupTransform(from, to, ros::Time(0), stf);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
  }
  return stf;
}

void AstarNavi::run()
{
  ros::Rate rate(update_rate_);

  while (ros::ok())
  {
    ros::spinOnce();

    if (!costmap_initialized_ || !current_pose_initialized_ || !goal_pose_initialized_)
    {
      rate.sleep();
      continue;
    }

    // Initialize vector for A* search, this runs only once
    astar.initialize(costmap_);

    ros::WallTime start = ros::WallTime::now();

    // Execute astar search
    bool result = astar.findPath(current_pose_.pose, goal_pose_.pose, -1);

    ros::WallTime end = ros::WallTime::now();

    ROS_INFO("Astar planning: %f [s]", (end - start).toSec());

    astar.publishPoseArray(debug_pose_pub_, "velodyne");

    if (result)
    {
      ROS_INFO("Found GOAL!");
      publishPathAsWaypoints(lane_pub_, astar.getPath(), waypoint_velocity_);
    }
    else
    {
      ROS_INFO("can't find goal...");
    }

    astar.reset();
    rate.sleep();
  }
}

void AstarNavi::publishPathAsWaypoints(const ros::Publisher& pub, const nav_msgs::Path& path, const double waypoint_velocity)
{
  autoware_msgs::lane lane;

  lane.header = path.header;
  lane.header.frame_id = "velodyne";
  lane.increment = 0;
  for (const auto& pose : path.poses)
  {
    autoware_msgs::waypoint wp;
    wp.pose = pose;
    wp.twist.twist.linear.x = waypoint_velocity / 3.6;

    lane.waypoints.push_back(wp);
  }

  autoware_msgs::LaneArray lane_array;
  lane_array.lanes.push_back(lane);
  pub.publish(lane_array);

  return;
}

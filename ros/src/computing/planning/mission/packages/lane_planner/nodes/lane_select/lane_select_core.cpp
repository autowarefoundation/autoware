/*
 *  Copyright (c) 2015, Nagoya University

 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "lane_select_core.h"

namespace lane_planner
{
// Constructor
LaneSelectNode::LaneSelectNode()
  : private_nh_("~")
  , current_lane_idx_(-1)
  , right_lane_idx_(-1)
  , left_lane_idx_(-1)
  , is_lane_array_subscribed_(false)
  , is_current_pose_subscribed_(false)
  , is_current_velocity_subscribed_(false)
  , last_change_time_(ros::Time::now())
{
  initForROS();
  initForViz();
}

// Destructor
LaneSelectNode::~LaneSelectNode()
{
}

void LaneSelectNode::initForROS()
{
  // setup subscriber
  sub1_ = nh_.subscribe("traffic_waypoints_array", 100, &LaneSelectNode::callbackFromLaneArray, this);
  sub2_ = nh_.subscribe("current_pose", 100, &LaneSelectNode::callbackFromPoseStamped, this);
  sub3_ = nh_.subscribe("current_velocity", 100, &LaneSelectNode::callbackFromTwistStamped, this);

  // setup publisher
  pub1_ = nh_.advertise<waypoint_follower::lane>("base_waypoints", 10);
  pub2_ = nh_.advertise<std_msgs::Int32>("closest_waypoint", 10);
  vis_pub1_ = nh_.advertise<visualization_msgs::MarkerArray>("lane_select_marker", 10);

  // get from rosparam
  private_nh_.param<int32_t>("lane_change_interval", lane_change_interval_, int32_t(2));
  private_nh_.param<double>("distance_threshold", distance_threshold_, double(3.0));
}

void LaneSelectNode::initCommonParamForLaneMarker(visualization_msgs::Marker *marker)
{
  marker->header.frame_id = "map";
  marker->header.stamp = ros::Time();
  marker->type = visualization_msgs::Marker::LINE_STRIP;
  marker->action = visualization_msgs::Marker::ADD;
  marker->scale.x = 1.0;
}

void LaneSelectNode::initForViz()
{
  std_msgs::ColorRGBA color_current;
  color_current.b = 1.0;
  color_current.g = 0.7;
  color_current.a = 0.2;

  std_msgs::ColorRGBA color_neighbor;
  color_neighbor.r = 0.5;
  color_neighbor.b = 0.5;
  color_neighbor.g = 0.5;
  color_neighbor.a = 0.2;

  std_msgs::ColorRGBA color_closest_wp;
  color_closest_wp.r = 1.0;
  color_closest_wp.b = 1.0;
  color_closest_wp.g = 1.0;
  color_closest_wp.a = 1.0;

  // current_lane_marker_
  initCommonParamForLaneMarker(&current_lane_marker_);
  current_lane_marker_.ns = "current_lane_marker";
  current_lane_marker_.color = color_current;

  // left_lane_marker_
  initCommonParamForLaneMarker(&left_lane_marker_);
  left_lane_marker_.ns = "left_lane_marker";
  left_lane_marker_.color = color_neighbor;

  // right_lane_marker_
  initCommonParamForLaneMarker(&right_lane_marker_);
  right_lane_marker_.ns = "right_lane_marker";
  right_lane_marker_.color = color_neighbor;

  // closest_waypoints_marker_
  closest_waypoints_marker_.header.frame_id = "map";
  closest_waypoints_marker_.header.stamp = ros::Time();
  closest_waypoints_marker_.ns = "closest_waypoints_marker";
  closest_waypoints_marker_.type = visualization_msgs::Marker::POINTS;
  closest_waypoints_marker_.action = visualization_msgs::Marker::ADD;
  closest_waypoints_marker_.scale.x = 0.5;
  closest_waypoints_marker_.color = color_closest_wp;
}

void LaneSelectNode::processing()
{
  if (!is_current_pose_subscribed_ || !is_lane_array_subscribed_ || !is_current_velocity_subscribed_)
  {
    ROS_ERROR("Necessary topics are not subscribed yet.");
    return;
  }

  // search closest waypoint number for each lanes
  if (!getClosestWaypointNumberForEachLanes())
  {
    current_lane_idx_ = -1;
    right_lane_idx_ = -1;
    left_lane_idx_ = -1;
    return;
  }

  // if current_lane_idx_ = -1, find the lane index which has the most closest waypoint
  if (current_lane_idx_ == -1)
  {
    findCurrentLane();
    findNeighborLanes();
    publish();
    return;
  }

  // if closest waypoint on current lane is not -1,
  if (std::get<1>(tuple_vec_.at(static_cast<uint32_t>(current_lane_idx_))) != -1)
  {
    const int32_t &change_flag =
        std::get<0>(tuple_vec_.at(static_cast<uint32_t>(current_lane_idx_)))
            .waypoints.at(static_cast<uint32_t>(std::get<1>(tuple_vec_.at(static_cast<uint32_t>(current_lane_idx_)))))
            .change_flag;

    // if change flag of current_lane is left or right, lane change
    if (change_flag == enumToInteger(ChangeFlag::right) || change_flag == enumToInteger(ChangeFlag::left))
      changeLane(change_flag);
  }

  publish();
  return;
}

void LaneSelectNode::changeLane(const int32_t &change_flag)
{
  ros::Time current_time = ros::Time::now();
  double dt = (current_time - last_change_time_).toSec();
  if (dt < lane_change_interval_)
    return;

  if (change_flag == enumToInteger(ChangeFlag::right) && right_lane_idx_ != -1)
    current_lane_idx_ = right_lane_idx_;
  else if (change_flag == enumToInteger(ChangeFlag::left) && left_lane_idx_ != -1)
    current_lane_idx_ = left_lane_idx_;

  findNeighborLanes();

  // for visualize
  createCurrentLaneMarker();

  last_change_time_ = ros::Time::now();
}

bool LaneSelectNode::getClosestWaypointNumberForEachLanes()
{
  for (auto &el : tuple_vec_)
  {
    int32_t number =
        getClosestWaypointNumber(std::get<0>(el), current_pose_.pose, current_velocity_.twist, std::get<1>(el));
    std::get<1>(el) = number;
    ROS_INFO("closest: %d", std::get<1>(el));
  }

  // confirm if all closest waypoint numbers are -1. If so, output error
  int32_t accum = 0;
  for (const auto &el : tuple_vec_)
  {
    accum += std::get<1>(el);
  }
  if (accum == (-1) * static_cast<int32_t>(tuple_vec_.size()))
  {
    ROS_ERROR("cannot get closest waypoints");
    return false;
  }

  // for visualize
  createClosestWaypointsMarker();
  return true;
}

void LaneSelectNode::findCurrentLane()
{
  std::vector<uint32_t> idx_vec;
  idx_vec.reserve(tuple_vec_.size());
  for (uint32_t i = 0; i < tuple_vec_.size(); i++)
  {
    if (std::get<1>(tuple_vec_.at(i)) == -1)
      continue;
    idx_vec.push_back(i);
  }
  current_lane_idx_ = findMostClosestLane(idx_vec, current_pose_.pose.position);

  // for visualize
  createCurrentLaneMarker();
}

int32_t LaneSelectNode::findMostClosestLane(const std::vector<uint32_t> idx_vec, const geometry_msgs::Point p)
{
  std::vector<double> dist_vec;
  dist_vec.reserve(idx_vec.size());
  for (const auto &el : idx_vec)
  {
    uint32_t closest_number = static_cast<uint32_t>(std::get<1>(tuple_vec_.at(el)));
    dist_vec.push_back(
        getTwoDimensionalDistance(p, std::get<0>(tuple_vec_.at(el)).waypoints.at(closest_number).pose.pose.position));
  }
  std::vector<double>::iterator itr = std::min_element(dist_vec.begin(), dist_vec.end());
  return idx_vec.at(static_cast<uint32_t>(std::distance(dist_vec.begin(), itr)));
}

void LaneSelectNode::findNeighborLanes()
{
  uint32_t current_closest_num =
      static_cast<uint32_t>(std::get<1>(tuple_vec_.at(static_cast<uint32_t>(current_lane_idx_))));
  const geometry_msgs::Pose &current_closest_pose =
      std::get<0>(tuple_vec_.at(static_cast<uint32_t>(current_lane_idx_))).waypoints.at(current_closest_num).pose.pose;

  std::vector<uint32_t> left_lane_idx_vec;
  left_lane_idx_vec.reserve(tuple_vec_.size());
  std::vector<uint32_t> right_lane_idx_vec;
  right_lane_idx_vec.reserve(tuple_vec_.size());
  for (uint32_t i = 0; i < tuple_vec_.size(); i++)
  {
    if (i == static_cast<uint32_t>(current_lane_idx_) || std::get<1>(tuple_vec_.at(i)) == -1)
      continue;

    uint32_t target_num = static_cast<uint32_t>(std::get<1>(tuple_vec_.at(i)));
    const geometry_msgs::Point &target_p = std::get<0>(tuple_vec_.at(i)).waypoints.at(target_num).pose.pose.position;

    geometry_msgs::Point converted_p;
    convertPointIntoRelativeCoordinate(target_p, current_closest_pose, &converted_p);

    ROS_INFO("distance: %lf", converted_p.y);
    if (fabs(converted_p.y) > distance_threshold_)
    {
      ROS_INFO("%d lane is far from current lane...", i);
      continue;
    }

    if (converted_p.y > 0)
      left_lane_idx_vec.push_back(i);
    else
      right_lane_idx_vec.push_back(i);
  }

  if (!left_lane_idx_vec.empty())
  {
    left_lane_idx_ = findMostClosestLane(left_lane_idx_vec, current_closest_pose.position);

    // for visualize
    createLeftLaneMarker();
  }
  else
  {
    left_lane_idx_ = -1;
  }
  if (!right_lane_idx_vec.empty())
  {
    right_lane_idx_ = findMostClosestLane(right_lane_idx_vec, current_closest_pose.position);

    // for visualize
    createRightLaneMarker();
  }
  else
  {
    right_lane_idx_ = -1;
  }
}

void LaneSelectNode::createCurrentLaneMarker()
{
  const std::vector<waypoint_follower::waypoint> &wps =
      std::get<0>(tuple_vec_.at(static_cast<uint32_t>(current_lane_idx_))).waypoints;
  current_lane_marker_.points.clear();
  current_lane_marker_.points.shrink_to_fit();
  current_lane_marker_.points.reserve(wps.size());
  for (const auto &el : wps)
  {
    current_lane_marker_.points.push_back(el.pose.pose.position);
  }
  publishForVisualize();
}

void LaneSelectNode::createRightLaneMarker()
{
  const std::vector<waypoint_follower::waypoint> &wps =
      std::get<0>(tuple_vec_.at(static_cast<uint32_t>(right_lane_idx_))).waypoints;

  right_lane_marker_.points.clear();
  right_lane_marker_.points.shrink_to_fit();
  right_lane_marker_.points.reserve(wps.size());
  for (const auto &el : wps)
  {
    right_lane_marker_.points.push_back(el.pose.pose.position);
  }
  publishForVisualize();
}

void LaneSelectNode::createLeftLaneMarker()
{
  const std::vector<waypoint_follower::waypoint> &wps =
      std::get<0>(tuple_vec_.at(static_cast<uint32_t>(left_lane_idx_))).waypoints;

  left_lane_marker_.points.clear();
  left_lane_marker_.points.shrink_to_fit();
  left_lane_marker_.points.reserve(wps.size());
  for (const auto &el : wps)
  {
    left_lane_marker_.points.push_back(el.pose.pose.position);
  }
  publishForVisualize();
}

void LaneSelectNode::createClosestWaypointsMarker()
{
  closest_waypoints_marker_.points.clear();
  closest_waypoints_marker_.points.shrink_to_fit();
  closest_waypoints_marker_.points.reserve(tuple_vec_.size());
  for (uint32_t i = 0; i < tuple_vec_.size(); i++)
  {
    if (std::get<1>(tuple_vec_.at(i)) == -1)
      continue;

    closest_waypoints_marker_.points.push_back(std::get<0>(tuple_vec_.at(i))
                                                   .waypoints.at(static_cast<uint32_t>(std::get<1>(tuple_vec_.at(i))))
                                                   .pose.pose.position);
  }
  publishForVisualize();
}

void LaneSelectNode::publishForVisualize()
{
  visualization_msgs::MarkerArray marker_array;
  marker_array.markers.push_back(closest_waypoints_marker_);
  marker_array.markers.push_back(current_lane_marker_);
  marker_array.markers.push_back(left_lane_marker_);
  marker_array.markers.push_back(right_lane_marker_);

  vis_pub1_.publish(marker_array);
}

void LaneSelectNode::publish()
{
  ROS_INFO("current_lane_idx: %d", current_lane_idx_);
  ROS_INFO("right_lane_idx: %d", right_lane_idx_);
  ROS_INFO("left_lane_idx: %d", left_lane_idx_);

  // publish current global lane
  waypoint_follower::lane global_lane;
  global_lane = std::get<0>(tuple_vec_.at(static_cast<uint32_t>(current_lane_idx_)));
  pub1_.publish(global_lane);

  // publish closest waypoint
  std_msgs::Int32 closest_waypoint;
  closest_waypoint.data = std::get<1>(tuple_vec_.at(static_cast<uint32_t>(current_lane_idx_)));
  pub2_.publish(closest_waypoint);

  is_current_pose_subscribed_ = false;
  is_current_velocity_subscribed_ = false;
}

void LaneSelectNode::callbackFromLaneArray(const waypoint_follower::LaneArrayConstPtr &msg)
{
  tuple_vec_.reserve(msg->lanes.size());
  for (const auto &el : msg->lanes)
  {
    auto t = std::make_tuple(el, -1);
    tuple_vec_.push_back(t);
  }

  // lane_array_ = *msg;
  is_lane_array_subscribed_ = true;

  processing();
}

void LaneSelectNode::callbackFromPoseStamped(const geometry_msgs::PoseStampedConstPtr &msg)
{
  current_pose_ = *msg;
  is_current_pose_subscribed_ = true;

  processing();
}

void LaneSelectNode::callbackFromTwistStamped(const geometry_msgs::TwistStampedConstPtr &msg)
{
  current_velocity_ = *msg;
  is_current_velocity_subscribed_ = true;

  processing();
}

void LaneSelectNode::run()
{
  ros::spin();
}

// distance between target 1 and target2 in 2-D
double getTwoDimensionalDistance(const geometry_msgs::Point &target1, const geometry_msgs::Point &target2)
{
  double distance = sqrt(pow(target1.x - target2.x, 2) + pow(target1.y - target2.y, 2));
  return distance;
}

void convertPointIntoRelativeCoordinate(const geometry_msgs::Point &input_point, const geometry_msgs::Pose &pose,
                                        geometry_msgs::Point *output_point)
{
  tf::Transform inverse;
  tf::poseMsgToTF(pose, inverse);
  tf::Transform transform = inverse.inverse();

  tf::Point p;
  pointMsgToTF(input_point, p);
  tf::Point tf_p = transform * p;
  pointTFToMsg(tf_p, *output_point);
}

double getRelativeAngle(const geometry_msgs::Pose &waypoint_pose, const geometry_msgs::Pose &current_pose)
{
  tf::Vector3 x_axis(1, 0, 0);
  tf::Transform waypoint_tfpose;
  tf::poseMsgToTF(waypoint_pose, waypoint_tfpose);
  tf::Vector3 waypoint_v = waypoint_tfpose.getBasis() * x_axis;
  tf::Transform current_tfpose;
  tf::poseMsgToTF(current_pose, current_tfpose);
  tf::Vector3 current_v = current_tfpose.getBasis() * x_axis;

  return current_v.angle(waypoint_v) * 180 / M_PI;
}

// get closest waypoint from current pose
int32_t getClosestWaypointNumber(const waypoint_follower::lane &current_lane, const geometry_msgs::Pose &current_pose,
                                 const geometry_msgs::Twist &current_velocity, const int32_t previous_number)
{
  if (current_lane.waypoints.empty())
  {
    return -1;
  }

  // ROS_INFO("number: %d",previous_number);
  std::vector<uint32_t> idx_vec;
  // if previous number is -1, search closest waypoint from waypoints in front of current pose
  if (previous_number == -1)
  {
    idx_vec.reserve(current_lane.waypoints.size());
    for (uint32_t i = 0; i < current_lane.waypoints.size(); i++)
    {
      geometry_msgs::Point converted_p;
      convertPointIntoRelativeCoordinate(current_lane.waypoints.at(i).pose.pose.position, current_pose, &converted_p);
      double angle = getRelativeAngle(current_lane.waypoints.at(i).pose.pose, current_pose);
      // ROS_INFO("angle: %lf",angle);
      if (converted_p.x > 0 && angle < 90)
      {
        idx_vec.push_back(i);
        // ROS_INFO("input idx: %d",i);
      }
    }
  }
  else
  {
    double ratio = 3;
    double minimum_dt = 2.0;
    double dt = current_velocity.linear.x * ratio > minimum_dt ? current_velocity.linear.x * ratio : minimum_dt;

    idx_vec.reserve(static_cast<uint32_t>(dt));

    auto range_max = static_cast<uint32_t>(previous_number + dt) < current_lane.waypoints.size()
                         ? static_cast<uint32_t>(previous_number + dt)
                         : current_lane.waypoints.size();
    for (uint32_t i = static_cast<uint32_t>(previous_number); i < range_max; i++)
    {
      geometry_msgs::Point converted_p;
      convertPointIntoRelativeCoordinate(current_lane.waypoints.at(i).pose.pose.position, current_pose, &converted_p);
      double angle = getRelativeAngle(current_lane.waypoints.at(i).pose.pose, current_pose);
      // ROS_INFO("angle: %lf",angle);
      if (converted_p.x > 0 && angle < 90)
      {
        idx_vec.push_back(i);
        // ROS_INFO("input idx: %d",i);
      }
    }
  }

  if (idx_vec.empty())
    return -1;

  std::vector<double> dist_vec;
  dist_vec.reserve(idx_vec.size());
  for (const auto &el : idx_vec)
  {
    double dt = getTwoDimensionalDistance(current_pose.position, current_lane.waypoints.at(el).pose.pose.position);
    dist_vec.push_back(dt);
    // ROS_INFO("dt: %lf",dt);
  }
  std::vector<double>::iterator itr = std::min_element(dist_vec.begin(), dist_vec.end());
  int32_t found_number = idx_vec.at(static_cast<uint32_t>(std::distance(dist_vec.begin(), itr)));
  // ROS_INFO("found number: %d",found_number);
  return found_number;
}

}  // lane_planner

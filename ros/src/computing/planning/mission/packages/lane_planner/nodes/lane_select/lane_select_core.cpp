/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "lane_select_core.h"

#include <algorithm>

namespace lane_planner
{
// Constructor
LaneSelectNode::LaneSelectNode()
  : private_nh_("~")
  , lane_array_id_(-1)
  , current_lane_idx_(-1)
  , right_lane_idx_(-1)
  , left_lane_idx_(-1)
  , is_lane_array_subscribed_(false)
  , is_current_pose_subscribed_(false)
  , is_current_velocity_subscribed_(false)
  , is_current_state_subscribed_(false)
  , is_config_subscribed_(false)
  , distance_threshold_(3.0)
  , lane_change_interval_(10.0)
  , lane_change_target_ratio_(2.0)
  , lane_change_target_minimum_(5.0)
  , vlength_hermite_curve_(10)
  , search_closest_waypoint_minimum_dt_(5)
  , current_state_("UNKNOWN")
{
  initForROS();
}

// Destructor
LaneSelectNode::~LaneSelectNode()
{
}

void LaneSelectNode::initForROS()
{
  // setup subscriber
  sub1_ = nh_.subscribe("traffic_waypoints_array", 1, &LaneSelectNode::callbackFromLaneArray, this);
  sub2_ = nh_.subscribe("current_pose", 1, &LaneSelectNode::callbackFromPoseStamped, this);
  sub3_ = nh_.subscribe("current_velocity", 1, &LaneSelectNode::callbackFromTwistStamped, this);
  sub4_ = nh_.subscribe("state", 1, &LaneSelectNode::callbackFromState, this);
  sub5_ = nh_.subscribe("/config/lane_select", 1, &LaneSelectNode::callbackFromConfig, this);
  sub6_ = nh_.subscribe("/decision_maker/state", 1, &LaneSelectNode::callbackFromDecisionMakerState, this);

  // setup publisher

  pub1_ = nh_.advertise<autoware_msgs::Lane>("base_waypoints", 1);
  pub2_ = nh_.advertise<std_msgs::Int32>("closest_waypoint", 1);
  pub3_ = nh_.advertise<std_msgs::Int32>("change_flag", 1);
  pub4_ = nh_.advertise<std_msgs::Int32>("/current_lane_id", 1);
  pub5_ = nh_.advertise<autoware_msgs::VehicleLocation>("vehicle_location", 1);

  vis_pub1_ = nh_.advertise<visualization_msgs::MarkerArray>("lane_select_marker", 1);

  // get from rosparam
  private_nh_.param<double>("lane_change_interval", lane_change_interval_, double(2));
  private_nh_.param<double>("distance_threshold", distance_threshold_, double(3.0));
  private_nh_.param<int>("search_closest_waypoint_minimum_dt", search_closest_waypoint_minimum_dt_, int(5));
}

bool LaneSelectNode::isAllTopicsSubscribed()
{
  if (!is_current_pose_subscribed_ || !is_lane_array_subscribed_ || !is_current_velocity_subscribed_)
  {
    ROS_WARN("Necessary topics are not subscribed yet. Waiting...");
    return false;
  }
  return true;
}

void LaneSelectNode::initForLaneSelect()
{
  if (!isAllTopicsSubscribed())
    return;

  // search closest waypoint number for each lanes
  if (!getClosestWaypointNumberForEachLanes())
  {
    publishClosestWaypoint(-1);
    publishVehicleLocation(-1, lane_array_id_);
    resetLaneIdx();
    return;
  }

  findCurrentLane();
  findNeighborLanes();
  updateChangeFlag();
  createLaneForChange();

  publishLane(std::get<0>(tuple_vec_.at(current_lane_idx_)));
  publishClosestWaypoint(std::get<1>(tuple_vec_.at(current_lane_idx_)));
  publishChangeFlag(std::get<2>(tuple_vec_.at(current_lane_idx_)));
  publishVehicleLocation(std::get<1>(tuple_vec_.at(current_lane_idx_)), lane_array_id_);
  publishVisualizer();

  resetSubscriptionFlag();
  return;
}

void LaneSelectNode::resetLaneIdx()
{
  current_lane_idx_ = -1;
  right_lane_idx_ = -1;
  left_lane_idx_ = -1;
  publishVisualizer();
}

void LaneSelectNode::resetSubscriptionFlag()
{
  is_current_pose_subscribed_ = false;
  is_current_velocity_subscribed_ = false;
  is_current_state_subscribed_ = false;
}

void LaneSelectNode::processing()
{
  if (!isAllTopicsSubscribed())
    return;

  // search closest waypoint number for each lanes
  if (!getClosestWaypointNumberForEachLanes())
  {
    publishClosestWaypoint(-1);
    publishVehicleLocation(-1, lane_array_id_);
    resetLaneIdx();
    return;
  }

  // if closest waypoint on current lane is -1,
  if (std::get<1>(tuple_vec_.at(current_lane_idx_)) == -1)
  {
    publishClosestWaypoint(-1);
    publishVehicleLocation(-1, lane_array_id_);
    resetLaneIdx();
    return;
  }

  findNeighborLanes();
  ROS_INFO("current_lane_idx: %d", current_lane_idx_);
  ROS_INFO("right_lane_idx: %d", right_lane_idx_);
  ROS_INFO("left_lane_idx: %d", left_lane_idx_);

  if (current_state_ == "LANE_CHANGE")
  {
    try
    {
      changeLane();
      std::get<1>(lane_for_change_) =
          getClosestWaypointNumber(std::get<0>(lane_for_change_), current_pose_.pose, current_velocity_.twist,
                                   std::get<1>(lane_for_change_), distance_threshold_, search_closest_waypoint_minimum_dt_);
      std::get<2>(lane_for_change_) = static_cast<ChangeFlag>(
          std::get<0>(lane_for_change_).waypoints.at(std::get<1>(lane_for_change_)).change_flag);
      ROS_INFO("closest: %d", std::get<1>(lane_for_change_));
      publishLane(std::get<0>(lane_for_change_));
      publishLaneID(std::get<0>(lane_for_change_));
      publishClosestWaypoint(std::get<1>(lane_for_change_));
      publishChangeFlag(std::get<2>(lane_for_change_));
      publishVehicleLocation(std::get<1>(lane_for_change_), lane_array_id_);
    }
    catch (std::out_of_range)
    {
      ROS_WARN("Failed to get closest waypoint num\n");
    }
  }
  else
  {
    updateChangeFlag();
    createLaneForChange();

    publishLane(std::get<0>(tuple_vec_.at(current_lane_idx_)));
    publishClosestWaypoint(std::get<1>(tuple_vec_.at(current_lane_idx_)));
    publishChangeFlag(std::get<2>(tuple_vec_.at(current_lane_idx_)));
    publishVehicleLocation(std::get<1>(tuple_vec_.at(current_lane_idx_)), lane_array_id_);
  }
  publishVisualizer();
  resetSubscriptionFlag();
}

int32_t LaneSelectNode::getClosestLaneChangeWaypointNumber(const std::vector<autoware_msgs::Waypoint> &wps,
                                                           int32_t cl_wp)
{
  for (uint32_t i = cl_wp; i < wps.size(); i++)
  {
    if (static_cast<ChangeFlag>(wps.at(i).change_flag) == ChangeFlag::right ||
        static_cast<ChangeFlag>(wps.at(i).change_flag) == ChangeFlag::left)
    {
      return i;
    }
  }
  return -1;
}

void LaneSelectNode::createLaneForChange()
{
  std::get<0>(lane_for_change_).waypoints.clear();
  std::get<0>(lane_for_change_).waypoints.shrink_to_fit();
  std::get<1>(lane_for_change_) = -1;

  const autoware_msgs::Lane &cur_lane = std::get<0>(tuple_vec_.at(current_lane_idx_));
  const int32_t &clst_wp = std::get<1>(tuple_vec_.at(current_lane_idx_));

  int32_t num_lane_change = getClosestLaneChangeWaypointNumber(cur_lane.waypoints, clst_wp);
  ROS_INFO("num_lane_change: %d", num_lane_change);
  if (num_lane_change < 0 || num_lane_change >= static_cast<int32_t>(cur_lane.waypoints.size()))
  {
    ROS_WARN("current lane doesn't have change flag");
    return;
  }

  if ((static_cast<ChangeFlag>(cur_lane.waypoints.at(num_lane_change).change_flag) == ChangeFlag::right &&
       right_lane_idx_ < 0) ||
      (static_cast<ChangeFlag>(cur_lane.waypoints.at(num_lane_change).change_flag) == ChangeFlag::left &&
       left_lane_idx_ < 0))
  {
    ROS_WARN("current lane doesn't have the lane for lane change");
    return;
  }

  double dt = getTwoDimensionalDistance(cur_lane.waypoints.at(num_lane_change).pose.pose.position,
                                        cur_lane.waypoints.at(clst_wp).pose.pose.position);
  double dt_by_vel = current_velocity_.twist.linear.x * lane_change_target_ratio_ > lane_change_target_minimum_ ?
                         current_velocity_.twist.linear.x * lane_change_target_ratio_ :
                         lane_change_target_minimum_;
  ROS_INFO("dt : %lf, dt_by_vel : %lf", dt, dt_by_vel);
  autoware_msgs::Lane &nghbr_lane =
      static_cast<ChangeFlag>(cur_lane.waypoints.at(num_lane_change).change_flag) == ChangeFlag::right ?
          std::get<0>(tuple_vec_.at(right_lane_idx_)) :
          std::get<0>(tuple_vec_.at(left_lane_idx_));
  const int32_t &nghbr_clst_wp =
      static_cast<ChangeFlag>(cur_lane.waypoints.at(num_lane_change).change_flag) == ChangeFlag::right ?
          std::get<1>(tuple_vec_.at(right_lane_idx_)) :
          std::get<1>(tuple_vec_.at(left_lane_idx_));

  int32_t target_num = -1;
  for (uint32_t i = nghbr_clst_wp; i < nghbr_lane.waypoints.size(); i++)
  {
    if (i == nghbr_lane.waypoints.size() - 1 ||
        dt + dt_by_vel < getTwoDimensionalDistance(nghbr_lane.waypoints.at(nghbr_clst_wp).pose.pose.position,
                                                   nghbr_lane.waypoints.at(i).pose.pose.position))
    {
      target_num = i;
      break;
    }
  }

  ROS_INFO("target_num : %d", target_num);
  if (target_num < 0)
    return;

  std::get<0>(lane_for_change_).header.stamp = nghbr_lane.header.stamp;
  std::vector<autoware_msgs::Waypoint> hermite_wps = generateHermiteCurveForROS(
      cur_lane.waypoints.at(num_lane_change).pose.pose, nghbr_lane.waypoints.at(target_num).pose.pose,
      cur_lane.waypoints.at(num_lane_change).twist.twist.linear.x, vlength_hermite_curve_);

  for (auto &&el : hermite_wps)
    el.change_flag = cur_lane.waypoints.at(num_lane_change).change_flag;

  std::get<0>(lane_for_change_).waypoints.reserve(nghbr_lane.waypoints.size() + hermite_wps.size());
  std::move(hermite_wps.begin(), hermite_wps.end(), std::back_inserter(std::get<0>(lane_for_change_).waypoints));
  auto itr = nghbr_lane.waypoints.begin();
  std::advance(itr, target_num);
  for (auto i = itr; i != nghbr_lane.waypoints.end(); i++)
  {
    if (getTwoDimensionalDistance(itr->pose.pose.position, i->pose.pose.position) < lane_change_interval_)
      i->change_flag = enumToInteger(ChangeFlag::straight);
    else
      break;
  }
  std::copy(itr, nghbr_lane.waypoints.end(), std::back_inserter(std::get<0>(lane_for_change_).waypoints));
}

void LaneSelectNode::updateChangeFlag()
{
  for (auto &el : tuple_vec_)
  {
    std::get<2>(el) = (std::get<1>(el) != -1) ?
                          static_cast<ChangeFlag>(std::get<0>(el).waypoints.at(std::get<1>(el)).change_flag) :
                          ChangeFlag::unknown;

    if (std::get<2>(el) == ChangeFlag::right && right_lane_idx_ == -1)
      std::get<2>(el) = ChangeFlag::unknown;
    else if (std::get<2>(el) == ChangeFlag::left && left_lane_idx_ == -1)
      std::get<2>(el) = ChangeFlag::unknown;

    ROS_INFO("change_flag: %d", enumToInteger(std::get<2>(el)));
  }
}

void LaneSelectNode::changeLane()
{
  if (std::get<2>(tuple_vec_.at(current_lane_idx_)) == ChangeFlag::right && right_lane_idx_ != -1 &&
      std::get<1>(tuple_vec_.at(right_lane_idx_)) != -1)
  {
    current_lane_idx_ = right_lane_idx_;
  }
  else if (std::get<2>(tuple_vec_.at(current_lane_idx_)) == ChangeFlag::left && left_lane_idx_ != -1 &&
           std::get<1>(tuple_vec_.at(left_lane_idx_)) != -1)
  {
    current_lane_idx_ = left_lane_idx_;
  }

  findNeighborLanes();
  return;
}

bool LaneSelectNode::getClosestWaypointNumberForEachLanes()
{
  for (auto &el : tuple_vec_)
  {
    std::get<1>(el) = getClosestWaypointNumber(std::get<0>(el), current_pose_.pose, current_velocity_.twist,
                                               std::get<1>(el), distance_threshold_, search_closest_waypoint_minimum_dt_);
    ROS_INFO("closest: %d", std::get<1>(el));
  }

  // confirm if all closest waypoint numbers are -1. If so, output warning
  int32_t accum = 0;
  for (const auto &el : tuple_vec_)
  {
    accum += std::get<1>(el);
  }
  if (accum == (-1) * static_cast<int32_t>(tuple_vec_.size()))
  {
    ROS_WARN("Cannot get closest waypoints. All closest waypoints are changed to -1...");
    return false;
  }

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
}

int32_t LaneSelectNode::findMostClosestLane(const std::vector<uint32_t> idx_vec, const geometry_msgs::Point p)
{
  std::vector<double> dist_vec;
  dist_vec.reserve(idx_vec.size());
  for (const auto &el : idx_vec)
  {
    int32_t closest_number = std::get<1>(tuple_vec_.at(el));
    dist_vec.push_back(
        getTwoDimensionalDistance(p, std::get<0>(tuple_vec_.at(el)).waypoints.at(closest_number).pose.pose.position));
  }
  std::vector<double>::iterator itr = std::min_element(dist_vec.begin(), dist_vec.end());
  return idx_vec.at(std::distance(dist_vec.begin(), itr));
}

void LaneSelectNode::findNeighborLanes()
{
  int32_t current_closest_num = std::get<1>(tuple_vec_.at(current_lane_idx_));
  const geometry_msgs::Pose &current_closest_pose =
      std::get<0>(tuple_vec_.at(current_lane_idx_)).waypoints.at(current_closest_num).pose.pose;

  std::vector<uint32_t> left_lane_idx_vec;
  left_lane_idx_vec.reserve(tuple_vec_.size());
  std::vector<uint32_t> right_lane_idx_vec;
  right_lane_idx_vec.reserve(tuple_vec_.size());
  for (uint32_t i = 0; i < tuple_vec_.size(); i++)
  {
    if (i == static_cast<uint32_t>(current_lane_idx_) || std::get<1>(tuple_vec_.at(i)) == -1)
      continue;

    int32_t target_num = std::get<1>(tuple_vec_.at(i));
    const geometry_msgs::Point &target_p = std::get<0>(tuple_vec_.at(i)).waypoints.at(target_num).pose.pose.position;

    geometry_msgs::Point converted_p = convertPointIntoRelativeCoordinate(target_p, current_closest_pose);

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
    left_lane_idx_ = findMostClosestLane(left_lane_idx_vec, current_closest_pose.position);
  else
    left_lane_idx_ = -1;

  if (!right_lane_idx_vec.empty())
    right_lane_idx_ = findMostClosestLane(right_lane_idx_vec, current_closest_pose.position);
  else
    right_lane_idx_ = -1;
}
visualization_msgs::Marker LaneSelectNode::createCurrentLaneMarker()
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time();
  marker.ns = "current_lane_marker";

  if (current_lane_idx_ == -1 || std::get<0>(tuple_vec_.at(current_lane_idx_)).waypoints.empty())
  {
    marker.action = visualization_msgs::Marker::DELETE;
    return marker;
  }

  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.05;

  std_msgs::ColorRGBA color_current;
  color_current.b = 1.0;
  color_current.g = 0.7;
  color_current.a = 1.0;
  marker.color = color_current;

  for (const auto &em : std::get<0>(tuple_vec_.at(current_lane_idx_)).waypoints)
    marker.points.push_back(em.pose.pose.position);

  return marker;
}

visualization_msgs::Marker LaneSelectNode::createRightLaneMarker()
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time();
  marker.ns = "right_lane_marker";

  if (right_lane_idx_ == -1 || std::get<0>(tuple_vec_.at(current_lane_idx_)).waypoints.empty())
  {
    marker.action = visualization_msgs::Marker::DELETE;
    return marker;
  }

  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.05;

  std_msgs::ColorRGBA color_neighbor;
  color_neighbor.r = 0.5;
  color_neighbor.b = 0.5;
  color_neighbor.g = 0.5;
  color_neighbor.a = 1.0;

  std_msgs::ColorRGBA color_neighbor_change;
  color_neighbor_change.b = 0.7;
  color_neighbor_change.g = 1.0;
  color_neighbor_change.a = 1.0;

  const ChangeFlag &change_flag = std::get<2>(tuple_vec_.at(current_lane_idx_));
  marker.color = change_flag == ChangeFlag::right ? color_neighbor_change : color_neighbor;

  for (const auto &em : std::get<0>(tuple_vec_.at(right_lane_idx_)).waypoints)
    marker.points.push_back(em.pose.pose.position);

  return marker;
}

visualization_msgs::Marker LaneSelectNode::createLeftLaneMarker()
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time();
  marker.ns = "left_lane_marker";

  if (left_lane_idx_ == -1 || std::get<0>(tuple_vec_.at(current_lane_idx_)).waypoints.empty())
  {
    marker.action = visualization_msgs::Marker::DELETE;
    return marker;
  }

  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.05;

  std_msgs::ColorRGBA color_neighbor;
  color_neighbor.r = 0.5;
  color_neighbor.b = 0.5;
  color_neighbor.g = 0.5;
  color_neighbor.a = 1.0;

  std_msgs::ColorRGBA color_neighbor_change;
  color_neighbor_change.b = 0.7;
  color_neighbor_change.g = 1.0;
  color_neighbor_change.a = 1.0;

  const ChangeFlag &change_flag = std::get<2>(tuple_vec_.at(current_lane_idx_));
  marker.color = change_flag == ChangeFlag::left ? color_neighbor_change : color_neighbor;

  for (const auto &em : std::get<0>(tuple_vec_.at((left_lane_idx_))).waypoints)
    marker.points.push_back(em.pose.pose.position);

  return marker;
}

visualization_msgs::Marker LaneSelectNode::createChangeLaneMarker()
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time();
  marker.ns = "change_lane_marker";

  if (std::get<0>(lane_for_change_).waypoints.empty())
  {
    marker.action = visualization_msgs::Marker::DELETE;
    return marker;
  }

  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.05;

  std_msgs::ColorRGBA color;
  color.r = 1.0;
  color.a = 1.0;

  std_msgs::ColorRGBA color_current;
  color_current.b = 1.0;
  color_current.g = 0.7;
  color_current.a = 1.0;

  marker.color = current_state_ == "LANE_CHANGE" ? color_current : color;
  for (const auto &em : std::get<0>(lane_for_change_).waypoints)
    marker.points.push_back(em.pose.pose.position);

  return marker;
}

visualization_msgs::Marker LaneSelectNode::createClosestWaypointsMarker()
{
  visualization_msgs::Marker marker;
  std_msgs::ColorRGBA color_closest_wp;
  color_closest_wp.r = 1.0;
  color_closest_wp.b = 1.0;
  color_closest_wp.g = 1.0;
  color_closest_wp.a = 1.0;

  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time();
  marker.ns = "closest_waypoints_marker";
  marker.type = visualization_msgs::Marker::POINTS;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.5;
  marker.color = color_closest_wp;

  marker.points.reserve(tuple_vec_.size());
  for (uint32_t i = 0; i < tuple_vec_.size(); i++)
  {
    if (std::get<1>(tuple_vec_.at(i)) == -1)
      continue;

    marker.points.push_back(
        std::get<0>(tuple_vec_.at(i)).waypoints.at(std::get<1>(tuple_vec_.at(i))).pose.pose.position);
  }

  return marker;
}

void LaneSelectNode::publishVisualizer()
{
  visualization_msgs::MarkerArray marker_array;
  marker_array.markers.push_back(createChangeLaneMarker());
  marker_array.markers.push_back(createCurrentLaneMarker());
  marker_array.markers.push_back(createRightLaneMarker());
  marker_array.markers.push_back(createLeftLaneMarker());
  marker_array.markers.push_back(createClosestWaypointsMarker());

  vis_pub1_.publish(marker_array);
}

void LaneSelectNode::publishLane(const autoware_msgs::Lane &lane)
{
  // publish global lane
  pub1_.publish(lane);
}

void LaneSelectNode::publishLaneID(const autoware_msgs::Lane &lane)
{
  std_msgs::Int32 msg;
  msg.data = lane.lane_id;
  pub4_.publish(msg);
}

void LaneSelectNode::publishClosestWaypoint(const int32_t clst_wp)
{
  // publish closest waypoint
  std_msgs::Int32 closest_waypoint;
  closest_waypoint.data = clst_wp;
  pub2_.publish(closest_waypoint);
}

void LaneSelectNode::publishChangeFlag(const ChangeFlag flag)
{
  std_msgs::Int32 change_flag;
  change_flag.data = enumToInteger(flag);
  pub3_.publish(change_flag);
}

void LaneSelectNode::publishVehicleLocation(const int32_t clst_wp, const int32_t larray_id)
{
  autoware_msgs::VehicleLocation vehicle_location;
  vehicle_location.header.stamp = ros::Time::now();
  vehicle_location.waypoint_index = clst_wp;
  vehicle_location.lane_array_id = larray_id;
  pub5_.publish(vehicle_location);
}

void LaneSelectNode::callbackFromLaneArray(const autoware_msgs::LaneArrayConstPtr &msg)
{
  tuple_vec_.clear();
  tuple_vec_.shrink_to_fit();
  tuple_vec_.reserve(msg->lanes.size());
  for (const auto &el : msg->lanes)
  {
    auto t = std::make_tuple(el, -1, ChangeFlag::unknown);
    tuple_vec_.push_back(t);
  }

  lane_array_id_ = msg->id;
  current_lane_idx_ = -1;
  right_lane_idx_ = -1;
  left_lane_idx_ = -1;
  is_lane_array_subscribed_ = true;

  if (current_lane_idx_ == -1)
    initForLaneSelect();
  else
    processing();
}

void LaneSelectNode::callbackFromPoseStamped(const geometry_msgs::PoseStampedConstPtr &msg)
{
  current_pose_ = *msg;
  is_current_pose_subscribed_ = true;

  if (current_lane_idx_ == -1)
    initForLaneSelect();
  else
    processing();
}

void LaneSelectNode::callbackFromTwistStamped(const geometry_msgs::TwistStampedConstPtr &msg)
{
  current_velocity_ = *msg;
  is_current_velocity_subscribed_ = true;

  if (current_lane_idx_ == -1)
    initForLaneSelect();
  else
    processing();
}

void LaneSelectNode::callbackFromState(const std_msgs::StringConstPtr &msg)
{
  current_state_ = msg->data;
  is_current_state_subscribed_ = true;

  if (current_lane_idx_ == -1)
    initForLaneSelect();
  else
    processing();
}
void LaneSelectNode::callbackFromDecisionMakerState(const std_msgs::StringConstPtr &msg)
{
  is_current_state_subscribed_ = true;

  if (msg->data.find("ChangeTo") != std::string::npos)
  {
    current_state_ = std::string("LANE_CHANGE");
    ;
  }
  else
  {
    current_state_ = msg->data;
  }

  if (current_lane_idx_ == -1)
    initForLaneSelect();
  else
    processing();
}

void LaneSelectNode::callbackFromConfig(const autoware_config_msgs::ConfigLaneSelectConstPtr &msg)
{
  distance_threshold_ = msg->distance_threshold_neighbor_lanes;
  lane_change_interval_ = msg->lane_change_interval;
  lane_change_target_ratio_ = msg->lane_change_target_ratio;
  lane_change_target_minimum_ = msg->lane_change_target_minimum;
  vlength_hermite_curve_ = msg->vector_length_hermite_curve;
  is_config_subscribed_ = true;

  if (current_lane_idx_ == -1)
    initForLaneSelect();
  else
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

geometry_msgs::Point convertPointIntoRelativeCoordinate(const geometry_msgs::Point &input_point,
                                                        const geometry_msgs::Pose &pose)
{
  tf::Transform inverse;
  tf::poseMsgToTF(pose, inverse);
  tf::Transform transform = inverse.inverse();

  tf::Point p;
  pointMsgToTF(input_point, p);
  tf::Point tf_p = transform * p;
  geometry_msgs::Point tf_point_msg;
  pointTFToMsg(tf_p, tf_point_msg);
  return tf_point_msg;
}

geometry_msgs::Point convertPointIntoWorldCoordinate(const geometry_msgs::Point &input_point,
                                                     const geometry_msgs::Pose &pose)
{
  tf::Transform inverse;
  tf::poseMsgToTF(pose, inverse);

  tf::Point p;
  pointMsgToTF(input_point, p);
  tf::Point tf_p = inverse * p;

  geometry_msgs::Point tf_point_msg;
  pointTFToMsg(tf_p, tf_point_msg);
  return tf_point_msg;
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
int32_t getClosestWaypointNumber(const autoware_msgs::Lane &current_lane, const geometry_msgs::Pose &current_pose,
                                 const geometry_msgs::Twist &current_velocity, const int32_t previous_number,
                                 const double distance_threshold, const int search_closest_waypoint_minimum_dt)
{
  if (current_lane.waypoints.empty())
    return -1;

  std::vector<uint32_t> idx_vec;
  // if previous number is -1, search closest waypoint from waypoints in front of current pose
  if (previous_number == -1)
  {
    idx_vec.reserve(current_lane.waypoints.size());
    for (uint32_t i = 0; i < current_lane.waypoints.size(); i++)
    {
      geometry_msgs::Point converted_p =
          convertPointIntoRelativeCoordinate(current_lane.waypoints.at(i).pose.pose.position, current_pose);
      double angle = getRelativeAngle(current_lane.waypoints.at(i).pose.pose, current_pose);
      if (converted_p.x > 0 && angle < 90)
        idx_vec.push_back(i);
    }
  }
  else
  {
    if (distance_threshold <
        getTwoDimensionalDistance(current_lane.waypoints.at(previous_number).pose.pose.position, current_pose.position))
    {
      ROS_WARN("Current_pose is far away from previous closest waypoint. Initilized...");
      return -1;
    }

    double ratio = 3;
    double dt = std::max(current_velocity.linear.x * ratio, static_cast<double>(search_closest_waypoint_minimum_dt));

    auto range_max = static_cast<uint32_t>(previous_number + dt) < current_lane.waypoints.size() ?
                         static_cast<uint32_t>(previous_number + dt) :
                         current_lane.waypoints.size();
    for (uint32_t i = static_cast<uint32_t>(previous_number); i < range_max; i++)
    {
      geometry_msgs::Point converted_p =
          convertPointIntoRelativeCoordinate(current_lane.waypoints.at(i).pose.pose.position, current_pose);
      double angle = getRelativeAngle(current_lane.waypoints.at(i).pose.pose, current_pose);
      if (converted_p.x > 0 && angle < 90)
        idx_vec.push_back(i);
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
  }
  std::vector<double>::iterator itr = std::min_element(dist_vec.begin(), dist_vec.end());
  int32_t found_number = idx_vec.at(static_cast<uint32_t>(std::distance(dist_vec.begin(), itr)));
  return found_number;
}

// let the linear equation be "ax + by + c = 0"
// if there are two points (x1,y1) , (x2,y2), a = "y2-y1, b = "(-1) * x2 - x1" ,c = "(-1) * (y2-y1)x1 + (x2-x1)y1"
bool getLinearEquation(geometry_msgs::Point start, geometry_msgs::Point end, double *a, double *b, double *c)
{
  //(x1, y1) = (start.x, star.y), (x2, y2) = (end.x, end.y)
  double sub_x = fabs(start.x - end.x);
  double sub_y = fabs(start.y - end.y);
  double error = pow(10, -5);  // 0.00001

  if (sub_x < error && sub_y < error)
  {
    ROS_INFO("two points are the same point!!");
    return false;
  }

  *a = end.y - start.y;
  *b = (-1) * (end.x - start.x);
  *c = (-1) * (end.y - start.y) * start.x + (end.x - start.x) * start.y;

  return true;
}
double getDistanceBetweenLineAndPoint(geometry_msgs::Point point, double a, double b, double c)
{
  double d = fabs(a * point.x + b * point.y + c) / sqrt(pow(a, 2) + pow(b, 2));

  return d;
}

}  // lane_planner

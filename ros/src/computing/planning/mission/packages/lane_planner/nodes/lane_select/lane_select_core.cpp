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
  , is_current_state_subscribed_(false)
  , last_change_time_(ros::Time::now())
  , current_change_flag_(ChangeFlag::unknown)
  , current_state_("UNKNOWN")
  , LANE_SIZE_(1.0)
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
  sub1_ = nh_.subscribe("traffic_waypoints_array", 100, &LaneSelectNode::callbackFromLaneArray, this);
  sub2_ = nh_.subscribe("current_pose", 100, &LaneSelectNode::callbackFromPoseStamped, this);
  sub3_ = nh_.subscribe("current_velocity", 100, &LaneSelectNode::callbackFromTwistStamped, this);
  sub4_ = nh_.subscribe("state", 100, &LaneSelectNode::callbackFromState, this);

  // setup publisher
  pub1_ = nh_.advertise<waypoint_follower::lane>("base_waypoints", 10);
  pub2_ = nh_.advertise<std_msgs::Int32>("closest_waypoint", 10);
  pub3_ = nh_.advertise<std_msgs::Int32>("change_flag", 10);
  vis_pub1_ = nh_.advertise<visualization_msgs::MarkerArray>("lane_select_marker", 10);

  // get from rosparam
  private_nh_.param<int32_t>("lane_change_interval", lane_change_interval_, int32_t(2));
  private_nh_.param<double>("distance_threshold", distance_threshold_, double(3.0));
}

void LaneSelectNode::processing()
{
  if (!is_current_pose_subscribed_ || !is_lane_array_subscribed_ || !is_current_velocity_subscribed_)
  {
    ROS_WARN("Necessary topics are not subscribed yet. Waiting...");
    return;
  }

  // search closest waypoint number for each lanes
  if (!getClosestWaypointNumberForEachLanes())
  {
    current_lane_idx_ = -1;
    right_lane_idx_ = -1;
    left_lane_idx_ = -1;
    publishVisualizer();
    return;
  }

  // if current_lane_idx_ = -1, find the lane index which has the most closest waypoint
  if (current_lane_idx_ == -1)
  {
    findCurrentLane();
    findNeighborLanes();
    publish();
    publishVisualizer();
    return;
  }

  // if closest waypoint on current lane is -1,
  if (std::get<1>(tuple_vec_.at(current_lane_idx_)) == -1)
  {
    current_lane_idx_ = -1;
    right_lane_idx_ = -1;
    left_lane_idx_ = -1;
    publishVisualizer();
    return;
  }

  if(right_lane_idx_ != -1)
  {
    if (std::get<1>(tuple_vec_.at(right_lane_idx_)) == -1)
      right_lane_idx_ = -1;
  }
  if(left_lane_idx_ != -1)
  {
    if (std::get<1>(tuple_vec_.at(left_lane_idx_)) == -1)
      left_lane_idx_ = -1;
  }

  ROS_INFO("current_lane_idx: %d", current_lane_idx_);
  ROS_INFO("right_lane_idx: %d", right_lane_idx_);
  ROS_INFO("left_lane_idx: %d", left_lane_idx_);
  ROS_INFO("current change_flag: %d", enumToInteger(std::get<2>(tuple_vec_.at(current_lane_idx_))));

  updateChangeFlag();
  if (current_state_ == "LANE_CHANGE")
    changeLane();

  publish();
  publishVisualizer();
}



void LaneSelectNode::updateChangeFlag()
{
  if(current_change_flag_ == ChangeFlag::unknown || current_change_flag_ == ChangeFlag::straight)
  {
    current_change_flag_ = std::get<2>(tuple_vec_.at(current_lane_idx_));

    if ((current_change_flag_ == ChangeFlag::left && left_lane_idx_ == -1) ||
        (current_change_flag_ == ChangeFlag::right && right_lane_idx_ == -1))
      current_change_flag_ = ChangeFlag::straight;
    return;
  }

  // if current change flag is right or left
  double a, b, c;

  if (std::get<1>(tuple_vec_.at(current_lane_idx_)) == 0 ||
                  std::get<1>(tuple_vec_.at(current_lane_idx_)) ==
                    static_cast<int32_t>(std::get<0>(tuple_vec_.at(current_lane_idx_)).waypoints.size()))
  {
    geometry_msgs::Point &closest_p = std::get<0>(tuple_vec_.at(current_lane_idx_))
                                          .waypoints.at(std::get<1>(tuple_vec_.at(current_lane_idx_)))
                                          .pose.pose.position;
    geometry_msgs::Point &front_of_closest_p = std::get<0>(tuple_vec_.at(current_lane_idx_))
                                                   .waypoints.at(std::get<1>(tuple_vec_.at(current_lane_idx_)) - 1)
                                                   .pose.pose.position;
    getLinearEquation(front_of_closest_p, closest_p, &a, &b, &c);
  }
  else
  {
    geometry_msgs::Point &closest_p = std::get<0>(tuple_vec_.at(current_lane_idx_))
                                          .waypoints.at(std::get<1>(tuple_vec_.at(current_lane_idx_)) - 1)
                                          .pose.pose.position;
    geometry_msgs::Point &front_of_closest_p = std::get<0>(tuple_vec_.at(current_lane_idx_))
                                                   .waypoints.at(std::get<1>(tuple_vec_.at(current_lane_idx_)))
                                                   .pose.pose.position;
    getLinearEquation(front_of_closest_p, closest_p, &a, &b, &c);
  }
  geometry_msgs::Point &current_point = current_pose_.pose.position;
  double d = getDistanceBetweenLineAndPoint(current_point, a, b, c);

  double threshold = 1.0;
  if(d < threshold)
  {
    current_change_flag_ = std::get<2>(tuple_vec_.at(current_lane_idx_));
  }
}

void LaneSelectNode::changeLane()
{
  ros::Time current_time = ros::Time::now();
  double dt = (current_time - last_change_time_).toSec();
  if (dt < lane_change_interval_)
    return;

  if (current_change_flag_ == ChangeFlag::right && right_lane_idx_ != -1)
  {
    if (std::get<1>(tuple_vec_.at(right_lane_idx_)) != -1)
    {
      current_lane_idx_ = right_lane_idx_;
      findNeighborLanes();
      last_change_time_ = ros::Time::now();
      return;
    }
  }

  if (current_change_flag_ == ChangeFlag::left && left_lane_idx_ != -1)
  {
    if (std::get<1>(tuple_vec_.at(left_lane_idx_)) != -1)
    {
      current_lane_idx_ = left_lane_idx_;
      findNeighborLanes();
      last_change_time_ = ros::Time::now();
      return;
    }
  }
}

bool LaneSelectNode::getClosestWaypointNumberForEachLanes()
{
  for (auto &el : tuple_vec_)
  {
    std::get<1>(el) = getClosestWaypointNumber(std::get<0>(el), current_pose_.pose, current_velocity_.twist,
                                               std::get<1>(el), distance_threshold_);
    ROS_INFO("closest: %d", std::get<1>(el));

    std::get<2>(el) = (std::get<1>(el) != -1)
                          ? static_cast<ChangeFlag>(std::get<0>(el).waypoints.at(std::get<1>(el)).change_flag)
                          : ChangeFlag::unknown;
    ROS_INFO("change_flag: %d", enumToInteger(std::get<2>(el)));
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
    left_lane_idx_ = findMostClosestLane(left_lane_idx_vec, current_closest_pose.position);
  else
    left_lane_idx_ = -1;

  if (!right_lane_idx_vec.empty())
    right_lane_idx_ = findMostClosestLane(right_lane_idx_vec, current_closest_pose.position);
  else
    right_lane_idx_ = -1;
}

std::unique_ptr<visualization_msgs::Marker> LaneSelectNode::createCurrentLaneMarker()
{
  std::unique_ptr<visualization_msgs::Marker> marker(new visualization_msgs::Marker);
  marker->header.frame_id = "map";
  marker->header.stamp = ros::Time();
  marker->ns = "current_lane_marker";

  if (current_lane_idx_ == -1 || std::get<0>(tuple_vec_.at(current_lane_idx_)).waypoints.empty())
  {
    marker->action = visualization_msgs::Marker::DELETE;
    return marker;
  }

  marker->type = visualization_msgs::Marker::LINE_STRIP;
  marker->action = visualization_msgs::Marker::ADD;
  marker->scale.x = 0.05;

  std_msgs::ColorRGBA color_current;
  color_current.b = 1.0;
  color_current.g = 0.7;
  color_current.a = 1.0;
  marker->color = color_current;

  marker->points = *createRectangleFromWaypoints(std::get<0>(tuple_vec_.at(current_lane_idx_)).waypoints, LANE_SIZE_);

  return marker;
}

std::unique_ptr<visualization_msgs::Marker> LaneSelectNode::createRightLaneMarker()
{
  std::unique_ptr<visualization_msgs::Marker> marker(new visualization_msgs::Marker);
  marker->header.frame_id = "map";
  marker->header.stamp = ros::Time();
  marker->ns = "right_lane_marker";

  if (right_lane_idx_ == -1 || std::get<0>(tuple_vec_.at(current_lane_idx_)).waypoints.empty())
  {
    marker->action = visualization_msgs::Marker::DELETE;
    return marker;
  }

  marker->type = visualization_msgs::Marker::LINE_STRIP;
  marker->action = visualization_msgs::Marker::ADD;
  marker->scale.x = 0.05;

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
  marker->color = change_flag == ChangeFlag::right ? color_neighbor_change : color_neighbor;

  marker->points = *createRectangleFromWaypoints(std::get<0>(tuple_vec_.at(right_lane_idx_)).waypoints, LANE_SIZE_);

  return marker;
}

std::unique_ptr<visualization_msgs::Marker> LaneSelectNode::createLeftLaneMarker()
{
  std::unique_ptr<visualization_msgs::Marker> marker(new visualization_msgs::Marker);
  marker->header.frame_id = "map";
  marker->header.stamp = ros::Time();
  marker->ns = "left_lane_marker";

  if (left_lane_idx_ == -1 || std::get<0>(tuple_vec_.at(current_lane_idx_)).waypoints.empty())
  {
    marker->action = visualization_msgs::Marker::DELETE;
    return marker;
  }

  marker->type = visualization_msgs::Marker::LINE_STRIP;
  marker->action = visualization_msgs::Marker::ADD;
  marker->scale.x = 0.05;

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
  marker->color = change_flag == ChangeFlag::left ? color_neighbor_change : color_neighbor;

  marker->points = *createRectangleFromWaypoints(std::get<0>(tuple_vec_.at(left_lane_idx_)).waypoints, LANE_SIZE_);

  return marker;
}

std::unique_ptr<visualization_msgs::Marker> LaneSelectNode::createClosestWaypointsMarker()
{
  std::unique_ptr<visualization_msgs::Marker> marker(new visualization_msgs::Marker);
  std_msgs::ColorRGBA color_closest_wp;
  color_closest_wp.r = 1.0;
  color_closest_wp.b = 1.0;
  color_closest_wp.g = 1.0;
  color_closest_wp.a = 1.0;

  marker->header.frame_id = "map";
  marker->header.stamp = ros::Time();
  marker->ns = "closest_waypoints_marker";
  marker->type = visualization_msgs::Marker::POINTS;
  marker->action = visualization_msgs::Marker::ADD;
  marker->scale.x = 0.5;
  marker->color = color_closest_wp;

  marker->points.reserve(tuple_vec_.size());
  for (uint32_t i = 0; i < tuple_vec_.size(); i++)
  {
    if (std::get<1>(tuple_vec_.at(i)) == -1)
      continue;

    marker->points.push_back(
        std::get<0>(tuple_vec_.at(i)).waypoints.at(std::get<1>(tuple_vec_.at(i))).pose.pose.position);
  }

  return marker;
}

std::unique_ptr<visualization_msgs::Marker> LaneSelectNode::createCurrentLaneFlagMarker()
{
  std::unique_ptr<visualization_msgs::Marker> marker(new visualization_msgs::Marker);
  marker->header.frame_id = "map";
  marker->header.stamp = ros::Time();
  marker->ns = "current_lane_flag_marker";

  if (current_lane_idx_ == -1)
  {
    marker->action = visualization_msgs::Marker::DELETE;
    return marker;
  }

  std_msgs::ColorRGBA red;
  red.r = 1.0;
  red.a = 1.0;

  marker->type = visualization_msgs::Marker::LINE_STRIP;
  marker->action = visualization_msgs::Marker::ADD;
  marker->scale.x = 0.05;
  marker->color = red;

  const int32_t &start = std::get<1>(tuple_vec_.at(current_lane_idx_));
  const size_t &end = std::get<0>(tuple_vec_.at(current_lane_idx_)).waypoints.size();
  const auto &wps = std::get<0>(tuple_vec_.at(current_lane_idx_)).waypoints;

  std::vector<waypoint_follower::waypoint> wps_extracted;
  for (uint32_t i = start; i < end; i++)
  {
    if (static_cast<ChangeFlag>(wps.at(i).change_flag) == ChangeFlag::right)
    {
      wps_extracted.push_back(wps.at(i));
      if (i == end - 1)
        break;

      if (static_cast<ChangeFlag>(wps.at(i + 1).change_flag) != ChangeFlag::right)
        break;
    }
    else if (static_cast<ChangeFlag>(wps.at(i).change_flag) == ChangeFlag::left)
    {
      wps_extracted.push_back(wps.at(i));
      if (i == end - 1)
        break;

      if (static_cast<ChangeFlag>(wps.at(i + 1).change_flag) != ChangeFlag::left)
        break;
    }
  }

  if (wps_extracted.empty())
  {
    marker->action = visualization_msgs::Marker::DELETE;
    return marker;
  }
  marker->points = *createRectangleFromWaypoints(wps_extracted, LANE_SIZE_ * 0.8);

  return marker;
}

std::unique_ptr<std::vector<geometry_msgs::Point>>
LaneSelectNode::createRectangleFromWaypoints(const std::vector<waypoint_follower::waypoint> &wps, const double &width)
{
  std::vector<std::tuple<geometry_msgs::Point, geometry_msgs::Point>> vertex;

  for (const auto &el : wps)
  {
    geometry_msgs::Point relative_p1;
    relative_p1.y = width / 2;
    geometry_msgs::Point relative_p2;
    relative_p2.y = -width / 2;
    vertex.push_back(std::make_tuple(*convertPointIntoWorldCoordinate(relative_p1, el.pose.pose),
                                     *convertPointIntoWorldCoordinate(relative_p2, el.pose.pose)));
  }

  std::unique_ptr<std::vector<geometry_msgs::Point>> rectangle(new std::vector<geometry_msgs::Point>);
  for (const auto &el : vertex)
    rectangle->push_back(std::get<0>(el));

  std::reverse(vertex.begin(), vertex.end());
  for (const auto &el : vertex)
    rectangle->push_back(std::get<1>(el));
  std::reverse(vertex.begin(), vertex.end());
  rectangle->push_back(std::get<0>(vertex.at(0)));

  return rectangle;
}

std::unique_ptr<visualization_msgs::Marker> LaneSelectNode::createCurrentLaneFlagArrowMarker()
{
  std::unique_ptr<visualization_msgs::Marker> marker(new visualization_msgs::Marker);

  marker->header.frame_id = "map";
  marker->header.stamp = ros::Time();
  marker->ns = "current_lane_flag_arrow_marker";

  if (current_lane_idx_ == -1)
  {
    marker->action = visualization_msgs::Marker::DELETE;
    return marker;
  }

  std_msgs::ColorRGBA red;
  red.r = 1.0;
  red.a = 1.0;

  marker->type = visualization_msgs::Marker::ARROW;
  marker->action = visualization_msgs::Marker::ADD;
  marker->color = red;
  marker->scale.x = 0.25;
  marker->scale.y = 0.5;

  const int32_t &start = std::get<1>(tuple_vec_.at(current_lane_idx_));
  const size_t &end = std::get<0>(tuple_vec_.at(current_lane_idx_)).waypoints.size();
  const auto &wps = std::get<0>(tuple_vec_.at(current_lane_idx_)).waypoints;

  std::vector<waypoint_follower::waypoint> wps_extracted;
  for (uint32_t i = start; i < end; i++)
  {
    if (static_cast<ChangeFlag>(wps.at(i).change_flag) == ChangeFlag::right)
    {
      wps_extracted.push_back(wps.at(i));
      if (i == end - 1)
        break;

      if (static_cast<ChangeFlag>(wps.at(i + 1).change_flag) != ChangeFlag::right)
        break;
    }
    else if (static_cast<ChangeFlag>(wps.at(i).change_flag) == ChangeFlag::left)
    {
      wps_extracted.push_back(wps.at(i));
      if (i == end - 1)
        break;

      if (static_cast<ChangeFlag>(wps.at(i + 1).change_flag) != ChangeFlag::left)
        break;
    }
  }

  if (wps_extracted.empty())
  {
    marker->action = visualization_msgs::Marker::DELETE;
    return marker;
  }
  uint32_t num = static_cast<uint32_t>(wps_extracted.size() / 2.0);
  geometry_msgs::Point relative_p1;
  relative_p1.y =
      static_cast<ChangeFlag>(wps_extracted.at(0).change_flag) == ChangeFlag::right ? -LANE_SIZE_ / 2 : LANE_SIZE_ / 2;
  marker->points.push_back(*convertPointIntoWorldCoordinate(relative_p1, wps_extracted.at(num).pose.pose));
  geometry_msgs::Point relative_p2;
  relative_p2.y = 3 * relative_p1.y;
  marker->points.push_back(*convertPointIntoWorldCoordinate(relative_p2, wps_extracted.at(num).pose.pose));

  return marker;
}

void LaneSelectNode::publishVisualizer()
{
  visualization_msgs::MarkerArray marker_array;

  marker_array.markers.push_back(*createCurrentLaneMarker());
  marker_array.markers.push_back(*createCurrentLaneFlagMarker());
  marker_array.markers.push_back(*createCurrentLaneFlagArrowMarker());
  marker_array.markers.push_back(*createRightLaneMarker());
  marker_array.markers.push_back(*createLeftLaneMarker());
  marker_array.markers.push_back(*createClosestWaypointsMarker());

  vis_pub1_.publish(marker_array);
}

void LaneSelectNode::publish()
{
  // publish current global lane
  waypoint_follower::lane global_lane;
  global_lane = std::get<0>(tuple_vec_.at(current_lane_idx_));
  pub1_.publish(global_lane);

  // publish closest waypoint
  std_msgs::Int32 closest_waypoint;
  closest_waypoint.data = std::get<1>(tuple_vec_.at(current_lane_idx_));
  pub2_.publish(closest_waypoint);

  std_msgs::Int32 change_flag;
  change_flag.data = enumToInteger(current_change_flag_);
  pub3_.publish(change_flag);

  is_current_pose_subscribed_ = false;
  is_current_velocity_subscribed_ = false;
  is_current_state_subscribed_ = false;
}

void LaneSelectNode::callbackFromLaneArray(const waypoint_follower::LaneArrayConstPtr &msg)
{
  tuple_vec_.clear();
  tuple_vec_.shrink_to_fit();
  tuple_vec_.reserve(msg->lanes.size());
  for (const auto &el : msg->lanes)
  {
    auto t = std::make_tuple(el, -1, ChangeFlag::unknown);
    tuple_vec_.push_back(t);
  }

  current_lane_idx_ = -1;
  right_lane_idx_ = -1;
  left_lane_idx_ = -1;
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

void LaneSelectNode::callbackFromState(const std_msgs::StringConstPtr &msg)
{
  current_state_ = msg->data;
  is_current_state_subscribed_ = true;

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

std::unique_ptr<geometry_msgs::Point> convertPointIntoWorldCoordinate(const geometry_msgs::Point &input_point,
                                                                      const geometry_msgs::Pose &pose)
{
  tf::Transform inverse;
  tf::poseMsgToTF(pose, inverse);

  tf::Point p;
  pointMsgToTF(input_point, p);
  tf::Point tf_p = inverse * p;

  std::unique_ptr<geometry_msgs::Point> tf_point_msg(new geometry_msgs::Point);
  pointTFToMsg(tf_p, *tf_point_msg);
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
int32_t getClosestWaypointNumber(const waypoint_follower::lane &current_lane, const geometry_msgs::Pose &current_pose,
                                 const geometry_msgs::Twist &current_velocity, const int32_t previous_number,
                                 const double distance_threshold)
{
  if (current_lane.waypoints.empty())
    return -1;

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
    if (distance_threshold <
        getTwoDimensionalDistance(current_lane.waypoints.at(previous_number).pose.pose.position, current_pose.position))
    {
      ROS_WARN("Current_pose is far away from previous closest waypoint. Initilized...");
      return -1;
    }

    double ratio = 3;
    double minimum_dt = 2.0;
    double dt = current_velocity.linear.x * ratio > minimum_dt ? current_velocity.linear.x * ratio : minimum_dt;

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

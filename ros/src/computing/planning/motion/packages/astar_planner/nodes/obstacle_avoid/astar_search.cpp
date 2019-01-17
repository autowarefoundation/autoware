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

#include "astar_search.h"

namespace astar_planner
{
AstarSearch::AstarSearch() : node_initialized_(false), upper_bound_distance_(-1)
{
  ros::NodeHandle private_nh_("~");
  private_nh_.param<bool>("use_2dnav_goal", use_2dnav_goal_, true);
  private_nh_.param<std::string>("map_frame", map_frame_, "map");
  private_nh_.param<int>("angle_size", angle_size_, 48);
  private_nh_.param<double>("minimum_turning_radius", minimum_turning_radius_, 5.1);
  private_nh_.param<int>("obstacle_threshold", obstacle_threshold_, 80);
  private_nh_.param<bool>("use_back", use_back_, true);
  private_nh_.param<double>("robot_length", robot_length_, 4.5);
  private_nh_.param<double>("robot_width", robot_width_, 1.75);
  private_nh_.param<double>("base2back", base2back_, 0.8);
  private_nh_.param<double>("curve_weight", curve_weight_, 1.05);
  private_nh_.param<double>("reverse_weight", reverse_weight_, 2.00);
  private_nh_.param<double>("distance_heuristic_weight", distance_heuristic_weight_, 1.00);
  private_nh_.param<double>("potential_weight", potential_weight_, 10.0);
  private_nh_.param<bool>("use_wavefront_heuristic", use_wavefront_heuristic_, false);
  private_nh_.param<bool>("use_potential_heuristic", use_potential_heuristic_, true);
  private_nh_.param<double>("time_limit", time_limit_, 10.0);
  private_nh_.param<double>("lateral_goal_range", lateral_goal_range_, 0.5);
  private_nh_.param<double>("longitudinal_goal_range", longitudinal_goal_range_, 2.0);
  private_nh_.param<double>("goal_angle_range", goal_angle_range_, 24.0);
  private_nh_.param<bool>("publish_marker", publish_marker_, false);

  createStateUpdateTableLocal(angle_size_);
}

AstarSearch::~AstarSearch()
{
}

void AstarSearch::initializeNode(const nav_msgs::OccupancyGrid &map)
{
  int height = map.info.height;
  int width = map.info.width;

  nodes_.resize(height);

  for (int i = 0; i < height; i++)
    nodes_[i].resize(width);

  for (int i = 0; i < height; i++)
    for (int j = 0; j < width; j++)
      nodes_[i][j].resize(angle_size_);

  node_initialized_ = true;
}

void AstarSearch::poseToIndex(const geometry_msgs::Pose &pose, int *index_x, int *index_y, int *index_theta)
{
  *index_x = pose.position.x / map_info_.resolution;
  *index_y = pose.position.y / map_info_.resolution;

  tf::Quaternion quat;
  tf::quaternionMsgToTF(pose.orientation, quat);
  double yaw = tf::getYaw(quat);
  if (yaw < 0)
    yaw += 2 * M_PI;

  // Descretize angle
  static double one_angle_range = 2 * M_PI / angle_size_;
  *index_theta = yaw / one_angle_range;
  *index_theta %= angle_size_;
}

// state update table for local hybrid astar
// five (for now) expansion to forward for each update
void AstarSearch::createStateUpdateTableLocal(int angle_size)
{
  // Vehicle moving for each angle
  state_update_table_.resize(angle_size);

  // the number of steering actions
  int max_action_num = 9;
  for (int i = 0; i < angle_size; i++)
    state_update_table_[i].reserve(max_action_num);

  // Minimum moving distance with one state update
  //     arc  = r                       * theta
  double step = minimum_turning_radius_ * (2.0 * M_PI / angle_size_);

  for (int i = 0; i < angle_size; i++)
  {
    double descretized_angle = 2.0 * M_PI / angle_size;
    double robot_angle = descretized_angle * i;

    // Calculate right and left circle
    // Robot moves along these circles
    double right_circle_center_x = minimum_turning_radius_ * std::sin(robot_angle);
    double right_circle_center_y = minimum_turning_radius_ * std::cos(robot_angle) * -1.0;
    double left_circle_center_x = right_circle_center_x * -1.0;
    double left_circle_center_y = right_circle_center_y * -1.0;

    NodeUpdate nu;

    // Calculate x and y shift to next state
    // forward
    nu.shift_x = step * std::cos(robot_angle);
    nu.shift_y = step * std::sin(robot_angle);
    nu.rotation = 0;
    nu.index_theta = 0;
    nu.step = step;
    nu.curve = false;
    nu.back = false;
    state_update_table_[i].emplace_back(nu);

    /*
    // forward right max
    nu.shift_x     = right_circle_center_x + minimum_turning_radius_ * std::cos(M_PI_2 + robot_angle -
    descretized_angle);
    nu.shift_y     = right_circle_center_y + minimum_turning_radius_ * std::sin(M_PI_2 + robot_angle -
    descretized_angle);
    nu.rotation    = descretized_angle * -1.0;
    nu.index_theta = -1;
    nu.step        = step;
    nu.curve       = true;
    nu.back        = false;
    state_update_table_[i].emplace_back(nu);

    // forward left max
    nu.shift_x     = left_circle_center_x + minimum_turning_radius_ * std::cos(-1.0 * M_PI_2 + robot_angle +
    descretized_angle);
    nu.shift_y     = left_circle_center_y + minimum_turning_radius_ * std::sin(-1.0 * M_PI_2 + robot_angle +
    descretized_angle);
    nu.rotation    = descretized_angle;
    nu.index_theta = 1;
    nu.step        = step;
    nu.curve       = true;
    nu.back        = false;
    state_update_table_[i].emplace_back(nu);
    */

    // forward right medium
    nu.shift_x =
        right_circle_center_x * 2 + minimum_turning_radius_ * 2 * std::cos(M_PI_2 + robot_angle - descretized_angle);
    nu.shift_y =
        right_circle_center_y * 2 + minimum_turning_radius_ * 2 * std::sin(M_PI_2 + robot_angle - descretized_angle);
    nu.rotation = descretized_angle * -1.0;
    nu.index_theta = -1;
    nu.step = step * 2;
    nu.curve = true;
    nu.back = false;
    state_update_table_[i].emplace_back(nu);

    // forward left medium
    nu.shift_x = left_circle_center_x * 2 +
                 minimum_turning_radius_ * 2 * std::cos(-1.0 * M_PI_2 + robot_angle + descretized_angle);
    nu.shift_y = left_circle_center_y * 2 +
                 minimum_turning_radius_ * 2 * std::sin(-1.0 * M_PI_2 + robot_angle + descretized_angle);
    nu.rotation = descretized_angle;
    nu.index_theta = 1;
    nu.step = step * 2;
    nu.curve = true;
    nu.back = false;
    state_update_table_[i].emplace_back(nu);

    /*
    // forward right max
    nu.shift_x     = right_circle_center_x + minimum_turning_radius_ * std::cos(M_PI_2 + robot_angle - descretized_angle
    * 2);
    nu.shift_y     = right_circle_center_y + minimum_turning_radius_ * std::sin(M_PI_2 + robot_angle - descretized_angle
    * 2);
    nu.rotation    = descretized_angle * 2 * -1.0;
    nu.index_theta = -2;
    nu.step        = step * 2;
    nu.curve       = true;
    nu.back        = false;
    state_update_table_[i].emplace_back(nu);

    // forward left max
    nu.shift_x     = left_circle_center_x + minimum_turning_radius_ * std::cos(-1.0 * M_PI_2 + robot_angle +
    descretized_angle * 2);
    nu.shift_y     = left_circle_center_y + minimum_turning_radius_ * std::sin(-1.0 * M_PI_2 + robot_angle +
    descretized_angle * 2);
    nu.rotation    = descretized_angle * 2;
    nu.index_theta = 2;
    nu.step        = step * 2;
    nu.curve       = true;
    nu.back        = false;
    state_update_table_[i].emplace_back(nu);
    */
  }
}
//---

bool AstarSearch::isOutOfRange(int index_x, int index_y)
{
  if (index_x < 0 || index_x >= static_cast<int>(map_info_.width) || index_y < 0 ||
      index_y >= static_cast<int>(map_info_.height))
    return true;

  return false;
}

void AstarSearch::displayFootprint(const nav_msgs::Path &path)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time();
  marker.ns = "footprint";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;

  marker.scale.x = robot_length_;
  marker.scale.y = robot_width_;
  marker.scale.z = 2.0;
  marker.color.a = 0.3;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.frame_locked = true;

  visualization_msgs::MarkerArray marker_array;
  for (const auto &p : path.poses)
  {
    marker.pose = p.pose;
    marker_array.markers.push_back(marker);
    marker.id += 1;
  }

  footprint_pub_.publish(marker_array);
}

void AstarSearch::setPath(const SimpleNode &goal)
{
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = map_frame_;
  path_.header = header;

  // From the goal node to the start node
  AstarNode *node = &nodes_[goal.index_y][goal.index_x][goal.index_theta];

  while (node != NULL)
  {
    // Set tf pose
    tf::Vector3 origin(node->x, node->y, 0);
    tf::Pose tf_pose;
    tf_pose.setOrigin(origin);
    tf_pose.setRotation(tf::createQuaternionFromYaw(node->theta));

    // Transform path to global frame
    tf_pose = map2ogm_ * tf_pose;

    // Set path as ros message
    geometry_msgs::PoseStamped ros_pose;
    tf::poseTFToMsg(tf_pose, ros_pose.pose);
    ros_pose.header = header;
    path_.poses.push_back(ros_pose);

    // To the next node
    node = node->parent;
  }

  // Reverse the vector to be start to goal order
  std::reverse(path_.poses.begin(), path_.poses.end());

  if (publish_marker_)
    displayFootprint(path_);
}

// Check if the next state is the goal
// Check lateral offset, longitudinal offset and angle
bool AstarSearch::isGoal(double x, double y, double theta)
{
  // To reduce computation time, we use square value for distance
  static const double lateral_goal_range =
      lateral_goal_range_ / 2.0;  // [meter], divide by 2 means we check left and right
  static const double longitudinal_goal_range =
      longitudinal_goal_range_ / 2.0;                                         // [meter], check only behind of the goal
  static const double goal_angle = M_PI * (goal_angle_range_ / 2.0) / 180.0;  // degrees -> radian

  // Calculate the node coordinate seen from the goal point
  tf::Point p(x, y, 0);
  geometry_msgs::Point relative_node_point = astar_planner::calcRelativeCoordinate(goal_pose_local_.pose, p);

  // Check Pose of goal
  if (relative_node_point.x < 0 &&  // shoud be behind of goal
      std::fabs(relative_node_point.x) < longitudinal_goal_range &&
      std::fabs(relative_node_point.y) < lateral_goal_range)
  {
    // Check the orientation of goal
    if (astar_planner::calcDiffOfRadian(goal_yaw_, theta) < goal_angle)
      return true;
  }

  return false;
}

bool AstarSearch::isObs(int index_x, int index_y)
{
  if (nodes_[index_y][index_x][0].status == STATUS::OBS)
    return true;

  return false;
}

bool AstarSearch::detectCollision(const SimpleNode &sn)
{
  // Define the robot as rectangle
  static double left = -1.0 * base2back_;
  static double right = robot_length_ - base2back_;
  static double top = robot_width_ / 2.0;
  static double bottom = -1.0 * robot_width_ / 2.0;
  static double resolution = map_info_.resolution;

  // Coordinate of base_link in OccupancyGrid frame
  static double one_angle_range = 2.0 * M_PI / angle_size_;
  double base_x = sn.index_x * resolution;
  double base_y = sn.index_y * resolution;
  double base_theta = sn.index_theta * one_angle_range;

  // Calculate cos and sin in advance
  double cos_theta = std::cos(base_theta);
  double sin_theta = std::sin(base_theta);

  // Convert each point to index and check if the node is Obstacle
  for (double x = left; x < right; x += resolution)
  {
    for (double y = top; y > bottom; y -= resolution)
    {
      // 2D point rotation
      int index_x = (x * cos_theta - y * sin_theta + base_x) / resolution;
      int index_y = (x * sin_theta + y * cos_theta + base_y) / resolution;

      if (isOutOfRange(index_x, index_y))
        return true;
      if (nodes_[index_y][index_x][0].status == STATUS::OBS)
        return true;
    }
  }

  return false;
}

bool AstarSearch::calcWaveFrontHeuristic(const SimpleNode &sn)
{
  // Set start point for wavefront search
  // This is goal for Astar search
  nodes_[sn.index_y][sn.index_x][0].hc = 0;
  WaveFrontNode wf_node(sn.index_x, sn.index_y, 1e-10);
  std::queue<WaveFrontNode> qu;
  qu.push(wf_node);

  // State update table for wavefront search
  // Nodes are expanded for each neighborhood cells (moore neighborhood)
  double resolution = map_info_.resolution;
  static std::vector<WaveFrontNode> updates = {
    astar_planner::getWaveFrontNode(0, 1, resolution), astar_planner::getWaveFrontNode(-1, 0, resolution),
    astar_planner::getWaveFrontNode(1, 0, resolution), astar_planner::getWaveFrontNode(0, -1, resolution),
    astar_planner::getWaveFrontNode(-1, 1, std::hypot(resolution, resolution)),
    astar_planner::getWaveFrontNode(1, 1, std::hypot(resolution, resolution)),
    astar_planner::getWaveFrontNode(-1, -1, std::hypot(resolution, resolution)),
    astar_planner::getWaveFrontNode(1, -1, std::hypot(resolution, resolution)),
  };

  // Get start index
  int start_index_x;
  int start_index_y;
  int start_index_theta;
  poseToIndex(start_pose_local_.pose, &start_index_x, &start_index_y, &start_index_theta);

  // Whether the robot can reach goal
  bool reachable = false;

  // Start wavefront search
  while (!qu.empty())
  {
    WaveFrontNode ref = qu.front();
    qu.pop();

    WaveFrontNode next;
    for (const auto &u : updates)
    {
      next.index_x = ref.index_x + u.index_x;
      next.index_y = ref.index_y + u.index_y;

      // out of range OR already visited OR obstacle node
      if (isOutOfRange(next.index_x, next.index_y) || nodes_[next.index_y][next.index_x][0].hc > 0 ||
          nodes_[next.index_y][next.index_x][0].status == STATUS::OBS)
        continue;

      // Take the size of robot into account
      if (detectCollisionWaveFront(next))
        continue;

      // Check if we can reach from start to goal
      if (next.index_x == start_index_x && next.index_y == start_index_y)
        reachable = true;

      // Set wavefront heuristic cost
      next.hc = ref.hc + u.hc;
      nodes_[next.index_y][next.index_x][0].hc = next.hc;

      qu.push(next);
    }
  }

  // End of search
  return reachable;
}

// Simple collidion detection for wavefront search
bool AstarSearch::detectCollisionWaveFront(const WaveFrontNode &ref)
{
  // Define the robot as square
  static double half = robot_width_ / 2;
  double robot_x = ref.index_x * map_info_.resolution;
  double robot_y = ref.index_y * map_info_.resolution;

  for (double y = half; y > -1.0 * half; y -= map_info_.resolution)
  {
    for (double x = -1.0 * half; x < half; x += map_info_.resolution)
    {
      int index_x = (robot_x + x) / map_info_.resolution;
      int index_y = (robot_y + y) / map_info_.resolution;

      if (isOutOfRange(index_x, index_y))
        return true;

      if (nodes_[index_y][index_x][0].status == STATUS::OBS)
        return true;
    }
  }

  return false;
}

void AstarSearch::reset()
{
  path_.poses.clear();
  debug_poses_.poses.clear();

  // Clear queue
  std::priority_queue<SimpleNode, std::vector<SimpleNode>, std::greater<SimpleNode>> empty;
  std::swap(openlist_, empty);

  ros::WallTime begin = ros::WallTime::now();

  // Reset node info here ...?
  for (size_t i = 0; i < map_info_.height; i++)
  {
    for (size_t j = 0; j < map_info_.width; j++)
    {
      for (int k = 0; k < angle_size_; k++)
      {
        // other values will be updated during the search
        nodes_[i][j][k].status = STATUS::NONE;
        nodes_[i][j][k].hc = 0;
      }
    }
  }

  ros::WallTime end = ros::WallTime::now();

  ROS_INFO("reset time: %lf [ms]", (end - begin).toSec() * 1000);
}

void AstarSearch::setMap(const nav_msgs::OccupancyGrid &map)
{
  map_info_ = map.info;

  std::string map_frame = map_frame_;
  std::string ogm_frame = map.header.frame_id;
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
  geometry_msgs::Pose ogm_in_map = astar_planner::transformPose(map_info_.origin, map2ogm_frame);
  tf::poseMsgToTF(ogm_in_map, map2ogm_);

  for (size_t i = 0; i < map.info.height; i++)
  {
    for (size_t j = 0; j < map.info.width; j++)
    {
      // Index of subscribing OccupancyGrid message
      size_t og_index = i * map.info.width + j;
      int cost = map.data[og_index];

      // hc is set to be 0 when reset()
      if (cost == 0)
        continue;

      if (use_potential_heuristic_)
      {
        // the cost more than threshold is regarded almost same as an obstacle
        // because of its very high cost
        if (cost > obstacle_threshold_)
          nodes_[i][j][0].status = STATUS::OBS;
        else
          nodes_[i][j][0].hc = cost * potential_weight_;
      }

      // obstacle or unknown area
      if (cost == 100 || cost < 0)
        nodes_[i][j][0].status = STATUS::OBS;
    }
  }
}

bool AstarSearch::setStartNode()
{
  // Get index of start pose
  int index_x;
  int index_y;
  int index_theta;
  poseToIndex(start_pose_local_.pose, &index_x, &index_y, &index_theta);
  SimpleNode start_sn(index_x, index_y, index_theta, 0, 0);

  // Check if start is valid
  if (isOutOfRange(index_x, index_y) || detectCollision(start_sn))
    return false;

  // Set start node
  AstarNode &start_node = nodes_[index_y][index_x][index_theta];
  start_node.x = start_pose_local_.pose.position.x;
  start_node.y = start_pose_local_.pose.position.y;
  start_node.theta = 2.0 * M_PI / angle_size_ * index_theta;
  start_node.gc = 0;
  start_node.move_distance = 0;
  start_node.back = false;
  start_node.status = STATUS::OPEN;
  start_node.parent = NULL;

  // set euclidean distance heuristic cost
  if (!use_wavefront_heuristic_ && !use_potential_heuristic_)
    start_node.hc = astar_planner::calcDistance(start_pose_local_.pose.position.x, start_pose_local_.pose.position.y,
                                                goal_pose_local_.pose.position.x, goal_pose_local_.pose.position.y) *
                    distance_heuristic_weight_;

  if (use_potential_heuristic_)
  {
    start_node.gc += start_node.hc;
    start_node.hc += astar_planner::calcDistance(start_pose_local_.pose.position.x, start_pose_local_.pose.position.y,
                                                 goal_pose_local_.pose.position.x, goal_pose_local_.pose.position.y) +
                     distance_heuristic_weight_;
  }

  // Push start node to openlist
  start_sn.cost = start_node.gc + start_node.hc;
  openlist_.push(start_sn);
  return true;
}

bool AstarSearch::setGoalNode()
{
  goal_yaw_ = astar_planner::modifyTheta(tf::getYaw(goal_pose_local_.pose.orientation));

  // Get index of goal pose
  int index_x;
  int index_y;
  int index_theta;
  poseToIndex(goal_pose_local_.pose, &index_x, &index_y, &index_theta);

  debug_poses_.header.frame_id = map_frame_;
  debug_poses_.poses.push_back(astar_planner::transformPose(goal_pose_local_.pose, map2ogm_));
  debug_pose_pub_.publish(debug_poses_);

  SimpleNode goal_sn(index_x, index_y, index_theta, 0, 0);

  // Check if goal is valid
  if (isOutOfRange(index_x, index_y) || detectCollision(goal_sn))
    return false;

  // Make multiple cells goals
  // createGoalList(goal_sn);

  // Calculate wavefront heuristic cost
  if (use_wavefront_heuristic_)
  {
    auto start = std::chrono::system_clock::now();

    bool wavefront_result = calcWaveFrontHeuristic(goal_sn);

    auto end = std::chrono::system_clock::now();
    auto usec = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    std::cout << "wavefront : " << usec / 1000.0 << "[msec]" << std::endl;

    if (!wavefront_result)
    {
      ROS_WARN("reachable is false...");
      return false;
    }
  }

  return true;
}

bool AstarSearch::search()
{
  ros::WallTime timer_begin = ros::WallTime::now();

  // Start A* search
  // If the openlist is empty, search failed
  while (!openlist_.empty())
  {
    // Check time and terminate if the search reaches the time limit
    ros::WallTime timer_end = ros::WallTime::now();
    double msec = (timer_end - timer_begin).toSec() * 1000.0;
    if (msec > time_limit_)
    {
      ROS_WARN("Exceed time limit of %lf [ms]", time_limit_);
      return false;
    }

    // Pop minimum cost node from openlist
    SimpleNode sn;
    sn = openlist_.top();
    openlist_.pop();

    // Expand nodes from this node
    AstarNode *current_node = &nodes_[sn.index_y][sn.index_x][sn.index_theta];
    current_node->status = STATUS::CLOSED;

    // Goal check
    if (isGoal(current_node->x, current_node->y, current_node->theta))
    {
      ROS_INFO("Search time: %lf [msec]", (timer_end - timer_begin).toSec() * 1000.0);

      setPath(sn);
      return true;
    }

    if (publish_marker_)
    {
      geometry_msgs::Pose p = astar_planner::xytToPoseMsg(current_node->x, current_node->y, current_node->theta);
      p = astar_planner::transformPose(p, map2ogm_);
      debug_poses_.poses.push_back(p);
    }

    // Expand nodes
    for (const auto &state : state_update_table_[sn.index_theta])
    {
      // Next state
      double next_x = current_node->x + state.shift_x;
      double next_y = current_node->y + state.shift_y;
      double next_theta = astar_planner::modifyTheta(current_node->theta + state.rotation);
      double move_cost = state.step;
      double move_distance = current_node->move_distance + state.step;

      // Increase reverse cost
      if (state.back != current_node->back)
        move_cost *= reverse_weight_;

      // Calculate index of the next state
      SimpleNode next;
      next.index_x = next_x / map_info_.resolution;
      next.index_y = next_y / map_info_.resolution;
      next.index_theta = sn.index_theta + state.index_theta;

      // Avoid invalid index
      next.index_theta = (next.index_theta + angle_size_) % angle_size_;

      // Check if the index is valid
      if (isOutOfRange(next.index_x, next.index_y) || isObs(next.index_x, next.index_y))
        continue;

      // prunning with upper bound
      if (upper_bound_distance_ > 0 && move_distance > upper_bound_distance_)
      {
        continue;
      }

      AstarNode *next_node = &nodes_[next.index_y][next.index_x][next.index_theta];
      double next_gc = current_node->gc + move_cost;
      double next_hc = nodes_[next.index_y][next.index_x][0].hc;  // wavefront or distance transform heuristic

      // increase the cost with euclidean distance
      if (use_potential_heuristic_)
      {
        next_gc += nodes_[next.index_y][next.index_x][0].hc;
        next_hc += astar_planner::calcDistance(next_x, next_y, goal_pose_local_.pose.position.x,
                                               goal_pose_local_.pose.position.y) *
                   distance_heuristic_weight_;
      }

      // increase the cost with euclidean distance
      if (!use_wavefront_heuristic_ && !use_potential_heuristic_)
        next_hc = astar_planner::calcDistance(next_x, next_y, goal_pose_local_.pose.position.x,
                                              goal_pose_local_.pose.position.y) *
                  distance_heuristic_weight_;

      // NONE
      if (next_node->status == STATUS::NONE)
      {
        next_node->status = STATUS::OPEN;
        next_node->x = next_x;
        next_node->y = next_y;
        next_node->theta = next_theta;
        next_node->gc = next_gc;
        next_node->hc = next_hc;
        next_node->move_distance = move_distance;
        next_node->back = state.back;
        next_node->parent = current_node;

        next.cost = next_node->gc + next_node->hc;
        openlist_.push(next);
        continue;
      }

      // OPEN or CLOSED
      if (next_node->status == STATUS::OPEN || next_node->status == STATUS::CLOSED)
      {
        if (next_gc < next_node->gc)
        {
          next_node->status = STATUS::OPEN;
          next_node->x = next_x;
          next_node->y = next_y;
          next_node->theta = next_theta;
          next_node->gc = next_gc;
          next_node->hc = next_hc;  // already calculated ?
          next_node->move_distance = move_distance;
          next_node->back = state.back;
          next_node->parent = current_node;

          next.cost = next_node->gc + next_node->hc;
          openlist_.push(next);
          continue;
        }
      }

    }  // state update
  }

  // Failed to find path
  ROS_INFO("Open list is empty...");
  return false;
}

bool AstarSearch::makePlan(const geometry_msgs::Pose &start_pose, const geometry_msgs::Pose &goal_pose,
                           const nav_msgs::OccupancyGrid &map, double upper_bound_distance)
{
  start_pose_local_.pose = start_pose;
  goal_pose_local_.pose = goal_pose;
  upper_bound_distance_ = upper_bound_distance;

  ROS_INFO("Prunning with upper bound %lf", upper_bound_distance_);

  ros::WallTime begin = ros::WallTime::now();
  setMap(map);
  ros::WallTime end = ros::WallTime::now();
  std::cout << "set map time: " << (end - begin).toSec() * 1000 << "[ms]" << std::endl;

  if (!setStartNode())
  {
    ROS_WARN("Invalid start pose!");
    return false;
  }

  if (!setGoalNode())
  {
    ROS_WARN("Invalid goal pose!");
    return false;
  }

  bool result = search();

  if (publish_marker_)
    debug_pose_pub_.publish(debug_poses_);

  return result;
}

// Allow two goals (reach goal2 via goal1)
bool AstarSearch::makePlan(const geometry_msgs::Pose &start_pose, const geometry_msgs::Pose &transit_pose,
                           const geometry_msgs::Pose &goal_pose, const nav_msgs::OccupancyGrid &map,
                           double upper_bound_distance)
{
  start_pose_local_.pose = start_pose;
  goal_pose_local_.pose = transit_pose;

  auto start = std::chrono::system_clock::now();
  setMap(map);
  auto end = std::chrono::system_clock::now();
  auto usec = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
  std::cout << "set map: " << usec / 1000.0 << std::endl;

  if (!setStartNode())
  {
    ROS_WARN("Invalid start pose!");
    return false;
  }

  if (!setGoalNode())
  {
    ROS_WARN("Invalid goal pose!");
    return false;
  }

  // First search from start to a transit point
  bool result = search();

  if (publish_marker_)
    debug_pose_pub_.publish(debug_poses_);

  if (!result)
    return false;

  // Prepare for the second search
  nav_msgs::Path start2transit = path_;
  reset();

  start_pose_local_.pose = transit_pose;
  goal_pose_local_.pose = goal_pose;

  if (!setStartNode())
  {
    ROS_WARN("Invalid start pose!");
    return false;
  }

  if (!setGoalNode())
  {
    ROS_WARN("Invalid goal pose!");
    return false;
  }

  // second search from a transit point to goal
  result = search();

  // join two paths
  path_.poses.reserve(start2transit.poses.size() + path_.poses.size());
  path_.poses.insert(path_.poses.begin(), start2transit.poses.begin(), start2transit.poses.end());

  if (publish_marker_)
    debug_pose_pub_.publish(debug_poses_);

  return result;
}

void AstarSearch::broadcastPathTF()
{
  tf::Transform transform;

  // Broadcast from start pose to goal pose
  for (int i = path_.poses.size() - 1; i >= 0; i--)
  {
    tf::Quaternion quat;
    tf::quaternionMsgToTF(path_.poses[i].pose.orientation, quat);
    transform.setOrigin(
        tf::Vector3(path_.poses[i].pose.position.x, path_.poses[i].pose.position.y, path_.poses[i].pose.position.z));
    transform.setRotation(quat);

    tf_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map", "/astar_path"));

    // sleep 0.1 [sec]
    usleep(100000);
  }
}

}  // namespace astar_planner

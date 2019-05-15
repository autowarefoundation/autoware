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

#include "astar_search/astar_search.h"

AstarSearch::AstarSearch()
{
  ros::NodeHandle private_nh_("~");

  // base configs
  private_nh_.param<bool>("use_back", use_back_, true);
  private_nh_.param<bool>("use_potential_heuristic", use_potential_heuristic_, true);
  private_nh_.param<bool>("use_wavefront_heuristic", use_wavefront_heuristic_, false);
  private_nh_.param<double>("time_limit", time_limit_, 5000.0);

  // robot configs
  private_nh_.param<double>("robot_length", robot_length_, 4.5);
  private_nh_.param<double>("robot_width", robot_width_, 1.75);
  private_nh_.param<double>("robot_base2back", robot_base2back_, 1.0);
  private_nh_.param<double>("minimum_turning_radius", minimum_turning_radius_, 6.0);

  // search configs
  private_nh_.param<int>("theta_size", theta_size_, 48);
  private_nh_.param<double>("angle_goal_range", angle_goal_range_, 6.0);
  private_nh_.param<double>("curve_weight", curve_weight_, 1.2);
  private_nh_.param<double>("reverse_weight", reverse_weight_, 2.00);
  private_nh_.param<double>("lateral_goal_range", lateral_goal_range_, 0.5);
  private_nh_.param<double>("longitudinal_goal_range", longitudinal_goal_range_, 2.0);

  // costmap configs
  private_nh_.param<int>("obstacle_threshold", obstacle_threshold_, 100);
  private_nh_.param<double>("potential_weight", potential_weight_, 10.0);
  private_nh_.param<double>("distance_heuristic_weight", distance_heuristic_weight_, 1.0);

  createStateUpdateTable();
}

AstarSearch::~AstarSearch()
{
}

// state update table for hybrid astar
void AstarSearch::createStateUpdateTable()
{
  // Vehicle moving for each angle
  state_update_table_.resize(theta_size_);
  double dtheta = 2.0 * M_PI / theta_size_;

  // Minimum moving distance with one state update
  //     arc  = r                       * theta
  double step = minimum_turning_radius_ * dtheta;

  for (int i = 0; i < theta_size_; i++)
  {
    double theta = dtheta * i;

    // Calculate right and left circle
    // Robot moves along these circles
    double right_circle_center_x = minimum_turning_radius_ * std::sin(theta);
    double right_circle_center_y = minimum_turning_radius_ * -std::cos(theta);
    double left_circle_center_x = -right_circle_center_x;
    double left_circle_center_y = -right_circle_center_y;

    // Calculate x and y shift to next state
    NodeUpdate nu;

    // forward
    nu.shift_x = step * std::cos(theta);
    nu.shift_y = step * std::sin(theta);
    nu.rotation = 0.0;
    nu.index_theta = 0;
    nu.step = step;
    nu.curve = false;
    nu.back = false;
    state_update_table_[i].emplace_back(nu);

    // forward right
    nu.shift_x = right_circle_center_x + minimum_turning_radius_ * std::cos(M_PI_2 + theta - dtheta);
    nu.shift_y = right_circle_center_y + minimum_turning_radius_ * std::sin(M_PI_2 + theta - dtheta);
    nu.rotation = -dtheta;
    nu.index_theta = -1;
    nu.step = step;
    nu.curve = true;
    nu.back = false;
    state_update_table_[i].emplace_back(nu);

    // forward left
    nu.shift_x = left_circle_center_x + minimum_turning_radius_ * std::cos(-M_PI_2 + theta + dtheta);
    nu.shift_y = left_circle_center_y + minimum_turning_radius_ * std::sin(-M_PI_2 + theta + dtheta);
    nu.rotation = dtheta;
    nu.index_theta = 1;
    nu.step = step;
    nu.curve = true;
    nu.back = false;
    state_update_table_[i].emplace_back(nu);

    if (use_back_)
    {
      // backward
      nu.shift_x = step * std::cos(theta) * -1.0;
      nu.shift_y = step * std::sin(theta) * -1.0;
      nu.rotation = 0;
      nu.index_theta = 0;
      nu.step = step;
      nu.curve = false;
      nu.back = true;
      state_update_table_[i].emplace_back(nu);

      // backward right
      nu.shift_x = right_circle_center_x + minimum_turning_radius_ * std::cos(M_PI_2 + theta + dtheta);
      nu.shift_y = right_circle_center_y + minimum_turning_radius_ * std::sin(M_PI_2 + theta + dtheta);
      nu.rotation = dtheta;
      nu.index_theta = 1;
      nu.step = step;
      nu.curve = true;
      nu.back = true;
      state_update_table_[i].emplace_back(nu);

      // backward left
      nu.shift_x = left_circle_center_x + minimum_turning_radius_ * std::cos(-1.0 * M_PI_2 + theta - dtheta);
      nu.shift_y = left_circle_center_y + minimum_turning_radius_ * std::sin(-1.0 * M_PI_2 + theta - dtheta);
      nu.rotation = dtheta * -1.0;
      nu.index_theta = -1;
      nu.step = step;
      nu.curve = true;
      nu.back = true;
      state_update_table_[i].emplace_back(nu);
    }
  }
}

void AstarSearch::initialize(const nav_msgs::OccupancyGrid& costmap)
{
  costmap_ = costmap;

  int height = costmap_.info.height;
  int width = costmap_.info.width;

  // size initialization
  nodes_.resize(height);
  for (int i = 0; i < height; i++)
  {
    nodes_[i].resize(width);
  }
  for (int i = 0; i < height; i++)
  {
    for (int j = 0; j < width; j++)
    {
      nodes_[i][j].resize(theta_size_);
    }
  }

  // cost initialization
  for (int i = 0; i < height; i++)
  {
    for (int j = 0; j < width; j++)
    {
      // Index of subscribing OccupancyGrid message
      int og_index = i * width + j;
      int cost = costmap_.data[og_index];

      // hc is set to be 0 when reset()
      if (cost == 0)
      {
        continue;
      }

      // obstacle or unknown area
      if (cost < 0 || obstacle_threshold_ <= cost)
      {
        nodes_[i][j][0].status = STATUS::OBS;
      }

      // the cost more than threshold is regarded almost same as an obstacle
      // because of its very high cost
      if (use_potential_heuristic_)
      {
        nodes_[i][j][0].hc = cost * potential_weight_;
      }
    }
  }
}

bool AstarSearch::makePlan(const geometry_msgs::Pose& start_pose, const geometry_msgs::Pose& goal_pose)
{
  if (!setStartNode(start_pose))
  {
    // ROS_WARN_STREAM("Invalid start pose");
    return false;
  }

  if (!setGoalNode(goal_pose))
  {
    // ROS_WARN_STREAM("Invalid goal pose");
    return false;
  }

  return search();
}

bool AstarSearch::setStartNode(const geometry_msgs::Pose& start_pose)
{
  // Get index of start pose
  int index_x, index_y, index_theta;
  start_pose_local_.pose = start_pose;
  poseToIndex(start_pose_local_.pose, &index_x, &index_y, &index_theta);
  SimpleNode start_sn(index_x, index_y, index_theta, 0, 0);

  // Check if start is valid
  if (isOutOfRange(index_x, index_y) || detectCollision(start_sn))
  {
    return false;
  }

  // Set start node
  AstarNode& start_node = nodes_[index_y][index_x][index_theta];
  start_node.x = start_pose_local_.pose.position.x;
  start_node.y = start_pose_local_.pose.position.y;
  start_node.theta = 2.0 * M_PI / theta_size_ * index_theta;
  start_node.gc = 0;
  start_node.move_distance = 0;
  start_node.back = false;
  start_node.status = STATUS::OPEN;
  start_node.parent = NULL;

  // set euclidean distance heuristic cost
  if (!use_wavefront_heuristic_ && !use_potential_heuristic_)
  {
    start_node.hc = calcDistance(start_pose_local_.pose.position.x, start_pose_local_.pose.position.y,
                                 goal_pose_local_.pose.position.x, goal_pose_local_.pose.position.y) *
                    distance_heuristic_weight_;
  }
  else if (use_potential_heuristic_)
  {
    start_node.gc += start_node.hc;
    start_node.hc += calcDistance(start_pose_local_.pose.position.x, start_pose_local_.pose.position.y,
                                  goal_pose_local_.pose.position.x, goal_pose_local_.pose.position.y) +
                     distance_heuristic_weight_;
  }

  // Push start node to openlist
  start_sn.cost = start_node.gc + start_node.hc;
  openlist_.push(start_sn);

  return true;
}

bool AstarSearch::setGoalNode(const geometry_msgs::Pose& goal_pose)
{
  goal_pose_local_.pose = goal_pose;
  goal_yaw_ = modifyTheta(tf::getYaw(goal_pose_local_.pose.orientation));

  // Get index of goal pose
  int index_x, index_y, index_theta;
  poseToIndex(goal_pose_local_.pose, &index_x, &index_y, &index_theta);
  SimpleNode goal_sn(index_x, index_y, index_theta, 0, 0);

  // Check if goal is valid
  if (isOutOfRange(index_x, index_y) || detectCollision(goal_sn))
  {
    return false;
  }

  // Calculate wavefront heuristic cost
  if (use_wavefront_heuristic_)
  {
    // auto start = std::chrono::system_clock::now();
    bool wavefront_result = calcWaveFrontHeuristic(goal_sn);
    // auto end = std::chrono::system_clock::now();
    // auto usec = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    // std::cout << "wavefront : " << usec / 1000.0 << "[msec]" << std::endl;

    if (!wavefront_result)
    {
      // ROS_WARN("Reachable is false...");
      return false;
    }
  }

  return true;
}

void AstarSearch::poseToIndex(const geometry_msgs::Pose& pose, int* index_x, int* index_y, int* index_theta)
{
  tf::Transform orig_tf;
  tf::poseMsgToTF(costmap_.info.origin, orig_tf);
  geometry_msgs::Pose pose2d = transformPose(pose, orig_tf.inverse());

  *index_x = pose2d.position.x / costmap_.info.resolution;
  *index_y = pose2d.position.y / costmap_.info.resolution;

  tf::Quaternion quat;
  tf::quaternionMsgToTF(pose2d.orientation, quat);
  double yaw = tf::getYaw(quat);
  if (yaw < 0)
    yaw += 2.0 * M_PI;

  // Descretize angle
  static double one_angle_range = 2.0 * M_PI / theta_size_;
  *index_theta = yaw / one_angle_range;
  *index_theta %= theta_size_;
}

void AstarSearch::pointToIndex(const geometry_msgs::Point& point, int* index_x, int* index_y)
{
  geometry_msgs::Pose pose;
  pose.position = point;
  int index_theta;
  poseToIndex(pose, index_x, index_y, &index_theta);
}

bool AstarSearch::isOutOfRange(int index_x, int index_y)
{
  if (index_x < 0 || index_x >= static_cast<int>(costmap_.info.width) || index_y < 0 ||
      index_y >= static_cast<int>(costmap_.info.height))
  {
    return true;
  }
  return false;
}

bool AstarSearch::search()
{
  ros::WallTime begin = ros::WallTime::now();

  // Start A* search
  // If the openlist is empty, search failed
  while (!openlist_.empty())
  {
    // Check time and terminate if the search reaches the time limit
    ros::WallTime now = ros::WallTime::now();
    double msec = (now - begin).toSec() * 1000.0;
    if (msec > time_limit_)
    {
      // ROS_WARN("Exceed time limit of %lf [ms]", time_limit_);
      return false;
    }

    // Pop minimum cost node from openlist
    SimpleNode top_sn = openlist_.top();
    openlist_.pop();

    // Expand nodes from this node
    AstarNode* current_an = &nodes_[top_sn.index_y][top_sn.index_x][top_sn.index_theta];
    current_an->status = STATUS::CLOSED;

    // Goal check
    if (isGoal(current_an->x, current_an->y, current_an->theta))
    {
      // ROS_INFO("Search time: %lf [msec]", (now - begin).toSec() * 1000.0);
      setPath(top_sn);
      return true;
    }

    // Expand nodes
    for (const auto& state : state_update_table_[top_sn.index_theta])
    {
      // Next state
      double next_x = current_an->x + state.shift_x;
      double next_y = current_an->y + state.shift_y;
      double next_theta = modifyTheta(current_an->theta + state.rotation);
      double move_cost = state.step;
      double move_distance = current_an->move_distance + state.step;

      // Increase reverse cost
      if (state.back != current_an->back)
        move_cost *= reverse_weight_;

      // Calculate index of the next state
      SimpleNode next_sn;
      geometry_msgs::Point next_pos;
      next_pos.x = next_x;
      next_pos.y = next_y;
      pointToIndex(next_pos, &next_sn.index_x, &next_sn.index_y);
      next_sn.index_theta = top_sn.index_theta + state.index_theta;

      // Avoid invalid index
      next_sn.index_theta = (next_sn.index_theta + theta_size_) % theta_size_;

      // Check if the index is valid
      if (isOutOfRange(next_sn.index_x, next_sn.index_y) || detectCollision(next_sn))
      {
        continue;
      }

      AstarNode* next_an = &nodes_[next_sn.index_y][next_sn.index_x][next_sn.index_theta];
      double next_gc = current_an->gc + move_cost;
      double next_hc = nodes_[next_sn.index_y][next_sn.index_x][0].hc;  // wavefront or distance transform heuristic

      // increase the cost with euclidean distance
      if (use_potential_heuristic_)
      {
        next_gc += nodes_[next_sn.index_y][next_sn.index_x][0].hc;
        next_hc += calcDistance(next_x, next_y, goal_pose_local_.pose.position.x, goal_pose_local_.pose.position.y) *
                   distance_heuristic_weight_;
      }

      // increase the cost with euclidean distance
      if (!use_wavefront_heuristic_ && !use_potential_heuristic_)
      {
        next_hc = calcDistance(next_x, next_y, goal_pose_local_.pose.position.x, goal_pose_local_.pose.position.y) *
                  distance_heuristic_weight_;
      }

      // NONE
      if (next_an->status == STATUS::NONE)
      {
        next_an->status = STATUS::OPEN;
        next_an->x = next_x;
        next_an->y = next_y;
        next_an->theta = next_theta;
        next_an->gc = next_gc;
        next_an->hc = next_hc;
        next_an->move_distance = move_distance;
        next_an->back = state.back;
        next_an->parent = current_an;
        next_sn.cost = next_an->gc + next_an->hc;
        openlist_.push(next_sn);
        continue;
      }

      // OPEN or CLOSED
      if (next_an->status == STATUS::OPEN || next_an->status == STATUS::CLOSED)
      {
        if (next_gc < next_an->gc)
        {
          next_an->status = STATUS::OPEN;
          next_an->x = next_x;
          next_an->y = next_y;
          next_an->theta = next_theta;
          next_an->gc = next_gc;
          next_an->hc = next_hc;  // already calculated ?
          next_an->move_distance = move_distance;
          next_an->back = state.back;
          next_an->parent = current_an;
          next_sn.cost = next_an->gc + next_an->hc;
          openlist_.push(next_sn);
          continue;
        }
      }
    }  // state update
  }

  // Failed to find path
  // ROS_INFO("Open list is empty...");
  return false;
}

void AstarSearch::setPath(const SimpleNode& goal)
{
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = costmap_.header.frame_id;
  path_.header = header;

  // From the goal node to the start node
  AstarNode* node = &nodes_[goal.index_y][goal.index_x][goal.index_theta];

  while (node != NULL)
  {
    // Set tf pose
    tf::Vector3 origin(node->x, node->y, 0);
    tf::Pose tf_pose;
    tf_pose.setOrigin(origin);
    tf_pose.setRotation(tf::createQuaternionFromYaw(node->theta));

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
  static const double goal_angle = M_PI * (angle_goal_range_ / 2.0) / 180.0;  // degrees -> radian

  // Calculate the node coordinate seen from the goal point
  tf::Point p(x, y, 0);
  geometry_msgs::Point relative_node_point = calcRelativeCoordinate(goal_pose_local_.pose, p);

  // Check Pose of goal
  if (relative_node_point.x < 0 &&  // shoud be behind of goal
      std::fabs(relative_node_point.x) < longitudinal_goal_range &&
      std::fabs(relative_node_point.y) < lateral_goal_range)
  {
    // Check the orientation of goal
    if (calcDiffOfRadian(goal_yaw_, theta) < goal_angle)
    {
      return true;
    }
  }

  return false;
}

bool AstarSearch::isObs(int index_x, int index_y)
{
  if (nodes_[index_y][index_x][0].status == STATUS::OBS)
  {
    return true;
  }

  return false;
}

bool AstarSearch::detectCollision(const SimpleNode& sn)
{
  // Define the robot as rectangle
  static double left = -1.0 * robot_base2back_;
  static double right = robot_length_ - robot_base2back_;
  static double top = robot_width_ / 2.0;
  static double bottom = -1.0 * robot_width_ / 2.0;
  static double resolution = costmap_.info.resolution;

  // Coordinate of base_link in OccupancyGrid frame
  static double one_angle_range = 2.0 * M_PI / theta_size_;
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
      {
        return true;
      }
      else if (nodes_[index_y][index_x][0].status == STATUS::OBS)
      {
        return true;
      }
    }
  }

  return false;
}

bool AstarSearch::calcWaveFrontHeuristic(const SimpleNode& sn)
{
  // Set start point for wavefront search
  // This is goal for Astar search
  nodes_[sn.index_y][sn.index_x][0].hc = 0;
  WaveFrontNode wf_node(sn.index_x, sn.index_y, 1e-10);
  std::queue<WaveFrontNode> qu;
  qu.push(wf_node);

  // State update table for wavefront search
  // Nodes are expanded for each neighborhood cells (moore neighborhood)
  double resolution = costmap_.info.resolution;
  static std::vector<WaveFrontNode> updates = {
    getWaveFrontNode(0, 1, resolution),
    getWaveFrontNode(-1, 0, resolution),
    getWaveFrontNode(1, 0, resolution),
    getWaveFrontNode(0, -1, resolution),
    getWaveFrontNode(-1, 1, std::hypot(resolution, resolution)),
    getWaveFrontNode(1, 1, std::hypot(resolution, resolution)),
    getWaveFrontNode(-1, -1, std::hypot(resolution, resolution)),
    getWaveFrontNode(1, -1, std::hypot(resolution, resolution)),
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
    for (const auto& u : updates)
    {
      next.index_x = ref.index_x + u.index_x;
      next.index_y = ref.index_y + u.index_y;

      // out of range OR already visited OR obstacle node
      if (isOutOfRange(next.index_x, next.index_y) || nodes_[next.index_y][next.index_x][0].hc > 0 ||
          nodes_[next.index_y][next.index_x][0].status == STATUS::OBS)
      {
        continue;
      }

      // Take the size of robot into account
      if (detectCollisionWaveFront(next))
      {
        continue;
      }

      // Check if we can reach from start to goal
      if (next.index_x == start_index_x && next.index_y == start_index_y)
      {
        reachable = true;
      }

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
bool AstarSearch::detectCollisionWaveFront(const WaveFrontNode& ref)
{
  // Define the robot as square
  static double half = robot_width_ / 2;
  double robot_x = ref.index_x * costmap_.info.resolution;
  double robot_y = ref.index_y * costmap_.info.resolution;

  for (double y = half; y > -1.0 * half; y -= costmap_.info.resolution)
  {
    for (double x = -1.0 * half; x < half; x += costmap_.info.resolution)
    {
      int index_x = (robot_x + x) / costmap_.info.resolution;
      int index_y = (robot_y + y) / costmap_.info.resolution;

      if (isOutOfRange(index_x, index_y))
      {
        return true;
      }

      if (nodes_[index_y][index_x][0].status == STATUS::OBS)
      {
        return true;
      }
    }
  }

  return false;
}

void AstarSearch::reset()
{
  path_.poses.clear();

  // Clear queue
  std::priority_queue<SimpleNode, std::vector<SimpleNode>, std::greater<SimpleNode>> empty;
  std::swap(openlist_, empty);

  // ros::WallTime begin = ros::WallTime::now();

  // Reset node info here ...?
  for (size_t i = 0; i < costmap_.info.height; i++)
  {
    for (size_t j = 0; j < costmap_.info.width; j++)
    {
      for (int k = 0; k < theta_size_; k++)
      {
        // other values will be updated during the search
        nodes_[i][j][k].status = STATUS::NONE;
        nodes_[i][j][k].hc = 0;
      }
    }
  }

  // ros::WallTime end = ros::WallTime::now();

  // ROS_INFO("Reset time: %lf [ms]", (end - begin).toSec() * 1000);
}

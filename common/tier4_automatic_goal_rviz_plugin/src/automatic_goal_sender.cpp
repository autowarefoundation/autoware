// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "automatic_goal_sender.hpp"

namespace automatic_goal
{
AutowareAutomaticGoalSender::AutowareAutomaticGoalSender() : Node("automatic_goal_sender")
{
}

void AutowareAutomaticGoalSender::init()
{
  loadParams(this);
  initCommunication(this);
  loadGoalsList(goals_list_file_path_);
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&AutowareAutomaticGoalSender::updateAutoExecutionTimerTick, this));

  // Print info
  RCLCPP_INFO_STREAM(get_logger(), "GoalsList has been loaded from: " << goals_list_file_path_);
  for (auto const & goal : goals_achieved_)
    RCLCPP_INFO_STREAM(get_logger(), "Loaded: " << goal.second.first);
  RCLCPP_INFO_STREAM(
    get_logger(), "Achieved goals will be saved in: " << goals_achieved_file_path_);
}

void AutowareAutomaticGoalSender::loadParams(rclcpp::Node * node)
{
  namespace fs = std::filesystem;
  node->declare_parameter("goals_list_file_path", "");
  node->declare_parameter("goals_achieved_dir_path", "");
  // goals_list
  goals_list_file_path_ = node->get_parameter("goals_list_file_path").as_string();
  if (!fs::exists(goals_list_file_path_) || !fs::is_regular_file(goals_list_file_path_))
    throw std::invalid_argument(
      "goals_list_file_path parameter - file path is invalid: " + goals_list_file_path_);
  // goals_achieved
  goals_achieved_file_path_ = node->get_parameter("goals_achieved_dir_path").as_string();
  if (!fs::exists(goals_achieved_file_path_) || fs::is_regular_file(goals_achieved_file_path_))
    throw std::invalid_argument(
      "goals_achieved_dir_path - directory path is invalid: " + goals_achieved_file_path_);
  goals_achieved_file_path_ += "goals_achieved.log";
}

void AutowareAutomaticGoalSender::initCommunication(rclcpp::Node * node)
{
  // Executing
  sub_operation_mode_ = node->create_subscription<OperationModeState>(
    "/api/operation_mode/state", rclcpp::QoS{1}.transient_local(),
    std::bind(&AutowareAutomaticGoalSender::onOperationMode, this, std::placeholders::_1));

  cli_change_to_autonomous_ =
    node->create_client<ChangeOperationMode>("/api/operation_mode/change_to_autonomous");

  cli_change_to_stop_ =
    node->create_client<ChangeOperationMode>("/api/operation_mode/change_to_stop");

  // Planning
  sub_route_ = node->create_subscription<RouteState>(
    "/api/routing/state", rclcpp::QoS{1}.transient_local(),
    std::bind(&AutowareAutomaticGoalSender::onRoute, this, std::placeholders::_1));

  cli_clear_route_ = node->create_client<ClearRoute>("/api/routing/clear_route");

  cli_set_route_ = node->create_client<SetRoutePoints>("/api/routing/set_route_points");
}

// Sub
void AutowareAutomaticGoalSender::onRoute(const RouteState::ConstSharedPtr msg)
{
  if (msg->state == RouteState::UNSET && state_ == State::CLEARING)
    state_ = State::CLEARED;
  else if (msg->state == RouteState::SET && state_ == State::PLANNING)
    state_ = State::PLANNED;
  else if (msg->state == RouteState::ARRIVED && state_ == State::STARTED)
    state_ = State::ARRIVED;
  onRouteUpdated(msg);
}

void AutowareAutomaticGoalSender::onOperationMode(const OperationModeState::ConstSharedPtr msg)
{
  if (msg->mode == OperationModeState::STOP && state_ == State::INITIALIZING)
    state_ = State::EDITING;
  else if (msg->mode == OperationModeState::STOP && state_ == State::STOPPING)
    state_ = State::STOPPED;
  else if (msg->mode == OperationModeState::AUTONOMOUS && state_ == State::STARTING)
    state_ = State::STARTED;
  onOperationModeUpdated(msg);
}

// Update
void AutowareAutomaticGoalSender::updateGoalsList()
{
  unsigned i = 0;
  for (const auto & goal : goals_list_) {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2);
    tf2::Quaternion tf2_quat;
    tf2::convert(goal->pose.orientation, tf2_quat);
    ss << "G" << i << " (" << goal->pose.position.x << ", ";
    ss << goal->pose.position.y << ", " << tf2::getYaw(tf2_quat) << ")";
    goals_achieved_.insert({i++, std::make_pair(ss.str(), 0)});
  }
  onGoalListUpdated();
}

void AutowareAutomaticGoalSender::updateAutoExecutionTimerTick()
{
  auto goal = goals_achieved_[current_goal_].first;

  if (state_ == State::INITIALIZING) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 3000, "Initializing... waiting for OperationModeState::STOP");

  } else if (state_ == State::EDITING) {  // skip the editing step by default
    state_ = State::AUTO_NEXT;

  } else if (state_ == State::AUTO_NEXT) {  // plan to next goal
    RCLCPP_INFO_STREAM(get_logger(), goal << ": Goal set as the next. Planning in progress...");
    if (callPlanToGoalIndex(cli_set_route_, current_goal_)) state_ = State::PLANNING;

  } else if (state_ == State::PLANNED) {  // start plan to next goal
    RCLCPP_INFO_STREAM(get_logger(), goal << ": Route has been planned. Route starting...");
    if (callService<ChangeOperationMode>(cli_change_to_autonomous_)) state_ = State::STARTING;

  } else if (state_ == State::STARTED) {
    RCLCPP_INFO_STREAM_THROTTLE(get_logger(), *get_clock(), 5000, goal << ": Driving to the goal.");

  } else if (state_ == State::ARRIVED) {  // clear plan after achieving next goal, goal++
    RCLCPP_INFO_STREAM(
      get_logger(), goal << ": Goal has been ACHIEVED " << ++goals_achieved_[current_goal_].second
                         << " times. Route clearing...");
    updateAchievedGoalsFile(current_goal_);
    if (callService<ClearRoute>(cli_clear_route_)) state_ = State::CLEARING;

  } else if (state_ == State::CLEARED) {
    RCLCPP_INFO_STREAM(get_logger(), goal << ": Route has been cleared.");
    current_goal_++;
    current_goal_ = current_goal_ % goals_list_.size();
    state_ = State::AUTO_NEXT;

  } else if (state_ == State::STOPPED) {
    RCLCPP_WARN_STREAM(
      get_logger(), goal << ": The execution has been stopped unexpectedly! Restarting...");
    if (callService<ChangeOperationMode>(cli_change_to_autonomous_)) state_ = State::STARTING;

  } else if (state_ == State::ERROR) {
    timer_->cancel();
    throw std::runtime_error(goal + ": Error encountered during execution!");
  }
}

// File
void AutowareAutomaticGoalSender::loadGoalsList(const std::string & file_path)
{
  YAML::Node node = YAML::LoadFile(file_path);
  goals_list_.clear();
  for (auto && goal : node) {
    std::shared_ptr<PoseStamped> pose = std::make_shared<PoseStamped>();
    pose->header.frame_id = "map";
    pose->header.stamp = rclcpp::Time();
    pose->pose.position.x = goal["position_x"].as<double>();
    pose->pose.position.y = goal["position_y"].as<double>();
    pose->pose.position.z = goal["position_z"].as<double>();
    pose->pose.orientation.x = goal["orientation_x"].as<double>();
    pose->pose.orientation.y = goal["orientation_y"].as<double>();
    pose->pose.orientation.z = goal["orientation_z"].as<double>();
    pose->pose.orientation.w = goal["orientation_w"].as<double>();
    goals_list_.push_back(pose);
  }
  resetAchievedGoals();
  updateGoalsList();
}

void AutowareAutomaticGoalSender::updateAchievedGoalsFile(const unsigned goal_index)
{
  if (!goals_achieved_file_path_.empty()) {
    std::ofstream out(goals_achieved_file_path_, std::fstream::app);
    std::stringstream ss;
    ss << "[" << getTimestamp() << "] Achieved: " << goals_achieved_[goal_index].first;
    ss << ", Current number of achievements: " << goals_achieved_[goal_index].second << "\n";
    out << ss.str();
    out.close();
  }
}

void AutowareAutomaticGoalSender::resetAchievedGoals()
{
  goals_achieved_.clear();
  if (!goals_achieved_file_path_.empty()) {
    std::ofstream out(goals_achieved_file_path_, std::fstream::app);
    out << "[" << getTimestamp()
        << "] GoalsList was loaded from a file or a goal was removed - counters have been reset\n";
    out.close();
  }
}
}  // namespace automatic_goal

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::shared_ptr<automatic_goal::AutowareAutomaticGoalSender> node{nullptr};
  try {
    node = std::make_shared<automatic_goal::AutowareAutomaticGoalSender>();
    node->init();
  } catch (const std::exception & e) {
    fprintf(stderr, "%s Exiting...\n", e.what());
    return 1;
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

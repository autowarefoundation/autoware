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

#ifndef AUTOMATIC_GOAL_SENDER_HPP_
#define AUTOMATIC_GOAL_SENDER_HPP_
#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_adapi_v1_msgs/msg/route_state.hpp>
#include <autoware_adapi_v1_msgs/srv/change_operation_mode.hpp>
#include <autoware_adapi_v1_msgs/srv/clear_route.hpp>
#include <autoware_adapi_v1_msgs/srv/set_route_points.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/utils.h>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace automatic_goal
{
enum class AutomaticGoalState {
  INITIALIZING,
  EDITING,
  PLANNING,
  PLANNED,
  STARTING,
  STARTED,
  ARRIVED,
  AUTO_NEXT,
  STOPPING,
  STOPPED,
  CLEARING,
  CLEARED,
  ERROR,
};

class AutowareAutomaticGoalSender : public rclcpp::Node
{
  using State = AutomaticGoalState;
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using OperationModeState = autoware_adapi_v1_msgs::msg::OperationModeState;
  using ChangeOperationMode = autoware_adapi_v1_msgs::srv::ChangeOperationMode;
  using RouteState = autoware_adapi_v1_msgs::msg::RouteState;
  using SetRoutePoints = autoware_adapi_v1_msgs::srv::SetRoutePoints;
  using ClearRoute = autoware_adapi_v1_msgs::srv::ClearRoute;

public:
  AutowareAutomaticGoalSender();
  void init();

protected:
  void initCommunication(rclcpp::Node * node);
  // Calls
  bool callPlanToGoalIndex(
    const rclcpp::Client<SetRoutePoints>::SharedPtr client, const unsigned goal_index)
  {
    if (!client->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "SetRoutePoints client is unavailable");
      return false;
    }

    auto req = std::make_shared<SetRoutePoints::Request>();
    req->header = goals_list_.at(goal_index)->header;
    req->goal = goals_list_.at(goal_index)->pose;
    client->async_send_request(
      req, [this](typename rclcpp::Client<SetRoutePoints>::SharedFuture result) {
        if (result.get()->status.code != 0) state_ = State::ERROR;
        printCallResult<SetRoutePoints>(result);
        onCallResult();
      });
    return true;
  }
  template <typename T>
  bool callService(const typename rclcpp::Client<T>::SharedPtr client)
  {
    if (!client->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "Client is unavailable");
      return false;
    }

    auto req = std::make_shared<typename T::Request>();
    client->async_send_request(req, [this](typename rclcpp::Client<T>::SharedFuture result) {
      if (result.get()->status.code != 0) state_ = State::ERROR;
      printCallResult<T>(result);
      onCallResult();
    });
    return true;
  }
  template <typename T>
  void printCallResult(typename rclcpp::Client<T>::SharedFuture result)
  {
    if (result.get()->status.code != 0) {
      RCLCPP_ERROR(
        get_logger(), "Service type \"%s\" status: %d, %s", typeid(T).name(),
        result.get()->status.code, result.get()->status.message.c_str());
    } else {
      RCLCPP_DEBUG(
        get_logger(), "Service type \"%s\" status: %d, %s", typeid(T).name(),
        result.get()->status.code, result.get()->status.message.c_str());
    }
  }

  // Update
  void updateGoalsList();
  virtual void updateAutoExecutionTimerTick();

  // File
  void loadGoalsList(const std::string & file_path);
  void updateAchievedGoalsFile(const unsigned goal_index);
  void resetAchievedGoals();
  static std::string getTimestamp()
  {
    char buffer[128];
    std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::strftime(&buffer[0], sizeof(buffer), "%Y-%m-%d %H:%M:%S", std::localtime(&now));
    return std::string{buffer};
  }

  // Sub
  void onRoute(const RouteState::ConstSharedPtr msg);
  void onOperationMode(const OperationModeState::ConstSharedPtr msg);

  // Interface
  virtual void onRouteUpdated(const RouteState::ConstSharedPtr) {}
  virtual void onOperationModeUpdated(const OperationModeState::ConstSharedPtr) {}
  virtual void onCallResult() {}
  virtual void onGoalListUpdated() {}

  // Cli
  rclcpp::Client<ChangeOperationMode>::SharedPtr cli_change_to_autonomous_{nullptr};
  rclcpp::Client<ChangeOperationMode>::SharedPtr cli_change_to_stop_{nullptr};
  rclcpp::Client<ClearRoute>::SharedPtr cli_clear_route_{nullptr};
  rclcpp::Client<SetRoutePoints>::SharedPtr cli_set_route_{nullptr};

  // Containers
  unsigned current_goal_{0};
  State state_{State::INITIALIZING};
  std::vector<PoseStamped::ConstSharedPtr> goals_list_{};
  std::map<unsigned, std::pair<std::string, unsigned>> goals_achieved_{};
  std::string goals_achieved_file_path_{};

private:
  void loadParams(rclcpp::Node * node);

  // Sub
  rclcpp::Subscription<RouteState>::SharedPtr sub_route_{nullptr};
  rclcpp::Subscription<OperationModeState>::SharedPtr sub_operation_mode_{nullptr};

  // Containers
  std::string goals_list_file_path_{};
  rclcpp::TimerBase::SharedPtr timer_{nullptr};
};
}  // namespace automatic_goal
#endif  // AUTOMATIC_GOAL_SENDER_HPP_

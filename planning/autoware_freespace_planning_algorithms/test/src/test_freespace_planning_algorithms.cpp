// Copyright 2021 Tier IV, Inc. All rights reserved.
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

#include "autoware/freespace_planning_algorithms/abstract_algorithm.hpp"
#include "autoware/freespace_planning_algorithms/astar_search.hpp"
#include "autoware/freespace_planning_algorithms/rrtstar.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rcpputils/filesystem_helper.hpp>
#include <rosbag2_cpp/writer.hpp>

#include <std_msgs/msg/float64.hpp>

#include <gtest/gtest.h>
#include <rcutils/time.h>
#include <tf2/utils.h>

#include <algorithm>
#include <array>
#include <fstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace fpa = autoware::freespace_planning_algorithms;

const double length_lexus = 5.5;
const double width_lexus = 2.75;
const double base_length_lexus = 3.0;
const double max_steering_lexus = 0.7;
const fpa::VehicleShape vehicle_shape =
  fpa::VehicleShape(length_lexus, width_lexus, base_length_lexus, max_steering_lexus, 1.5);
const double pi = 3.1415926;
const std::array<double, 3> start_pose{5.5, 4., pi * 0.5};
const std::array<double, 3> goal_pose1{8.0, 26.3, pi * 1.5};   // easiest
const std::array<double, 3> goal_pose2{15.0, 11.6, pi * 0.5};  // second easiest
const std::array<double, 3> goal_pose3{18.4, 26.3, pi * 1.5};  // third easiest
const std::array<double, 3> goal_pose4{25.0, 26.3, pi * 1.5};  // most difficult
const std::array<std::array<double, 3>, 4> goal_poses{
  goal_pose1, goal_pose2, goal_pose3, goal_pose4};

geometry_msgs::msg::Pose create_pose_msg(std::array<double, 3> pose3d)
{
  geometry_msgs::msg::Pose pose{};
  tf2::Quaternion quat{};
  quat.setRPY(0, 0, pose3d[2]);
  tf2::convert(quat, pose.orientation);
  pose.position.x = pose3d[0];
  pose.position.y = pose3d[1];
  pose.position.z = 0.0;
  return pose;
}

std_msgs::msg::Float64 create_float_msg(double val)
{
  std_msgs::msg::Float64 msg;
  msg.data = val;
  return msg;
}

nav_msgs::msg::OccupancyGrid construct_cost_map(
  size_t width, size_t height, double resolution, size_t n_padding)
{
  nav_msgs::msg::OccupancyGrid costmap_msg{};

  // create info
  costmap_msg.info.width = width;
  costmap_msg.info.height = height;
  costmap_msg.info.resolution = resolution;

  // create data
  const size_t n_elem = width * height;
  for (size_t i = 0; i < n_elem; ++i) {
    costmap_msg.data.push_back(0.0);
  }

  for (size_t i = 0; i < n_padding; ++i) {
    // fill left
    for (size_t j = width * i; j <= width * (i + 1); ++j) {
      costmap_msg.data[j] = 100.0;
    }
    // fill right
    for (size_t j = width * (height - n_padding + i); j <= width * (height - n_padding + i + 1);
         ++j) {
      costmap_msg.data[j] = 100.0;
    }
  }

  for (size_t i = 0; i < height; ++i) {
    // fill bottom
    for (size_t j = i * width; j <= i * width + n_padding; ++j) {
      costmap_msg.data[j] = 100.0;
    }
    for (size_t j = (i + 1) * width - n_padding; j <= (i + 1) * width; ++j) {
      costmap_msg.data[j] = 100.0;
    }
  }

  for (size_t i = 0; i < height; ++i) {
    for (size_t j = 0; j < width; ++j) {
      const double x = j * resolution;
      const double y = i * resolution;
      // wall
      if (8.0 < x && x < 28.0 && 9.0 < y && y < 9.5) {
        costmap_msg.data[i * width + j] = 100.0;
      }

      // car1
      if (10.0 < x && x < 10.0 + width_lexus && 22.0 < y && y < 22.0 + length_lexus) {
        costmap_msg.data[i * width + j] = 100.0;
      }

      // car2
      if (13.5 < x && x < 13.5 + width_lexus && 22.0 < y && y < 22.0 + length_lexus) {
        costmap_msg.data[i * width + j] = 100.0;
      }

      // car3
      if (20.0 < x && x < 20.0 + width_lexus && 22.0 < y && y < 22.0 + length_lexus) {
        costmap_msg.data[i * width + j] = 100.0;
      }

      // car4
      if (10.0 < x && x < 10.0 + width_lexus && 10.0 < y && y < 10.0 + length_lexus) {
        costmap_msg.data[i * width + j] = 100.0;
      }
    }
  }
  return costmap_msg;

  return costmap_msg;
}

template <typename MessageT>
void add_message_to_rosbag(
  rosbag2_cpp::Writer & writer, const MessageT & message, const std::string & name,
  const std::string & type)
{
  rclcpp::SerializedMessage serialized_msg;
  rclcpp::Serialization<MessageT> serialization;
  serialization.serialize_message(&message, &serialized_msg);

  rosbag2_storage::TopicMetadata tm;
  tm.name = name;
  tm.type = type;
  tm.serialization_format = "cdr";
  writer.create_topic(tm);

  auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  auto ret = rcutils_system_time_now(&bag_message->time_stamp);
  if (ret != RCL_RET_OK) {
    RCLCPP_ERROR(rclcpp::get_logger("saveToBag"), "couldn't assign time rosbag message");
  }

  bag_message->topic_name = tm.name;
  bag_message->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
    &serialized_msg.get_rcl_serialized_message(), [](rcutils_uint8_array_t * /* data */) {});
  writer.write(bag_message);
}

fpa::PlannerCommonParam get_default_planner_params()
{
  // set problem configuration
  const double time_limit = 10 * 1000.0;
  const double max_turning_ratio = 0.5;
  const int turning_steps = 1;

  const int theta_size = 144;

  const double curve_weight = 0.5;
  const double reverse_weight = 1.0;
  const double direction_change_weight = 1.5;

  const double lateral_goal_range = 0.5;
  const double longitudinal_goal_range = 2.0;
  const double angle_goal_range = 6.0;
  const int obstacle_threshold = 100;

  return fpa::PlannerCommonParam{
    time_limit,
    theta_size,
    curve_weight,
    reverse_weight,
    direction_change_weight,
    lateral_goal_range,
    longitudinal_goal_range,
    angle_goal_range,
    max_turning_ratio,
    turning_steps,
    obstacle_threshold};
}

std::unique_ptr<fpa::AbstractPlanningAlgorithm> configure_astar(bool use_multi)
{
  auto planner_common_param = get_default_planner_params();
  if (use_multi) {
    planner_common_param.turning_steps = 3;
  }

  // configure astar param
  const std::string search_method = "forward";
  const bool only_behind_solutions = false;
  const bool use_back = true;
  const bool adapt_expansion_distance = true;
  const double expansion_distance = 0.4;
  const double near_goal_distance = 4.0;
  const double distance_heuristic_weight = 2.0;
  const double smoothness_weight = 0.5;
  const double obstacle_distance_weight = 1.7;
  const double goal_lat_distance_weight = 1.0;
  const auto astar_param = fpa::AstarParam{
    search_method,
    only_behind_solutions,
    use_back,
    adapt_expansion_distance,
    expansion_distance,
    near_goal_distance,
    distance_heuristic_weight,
    smoothness_weight,
    obstacle_distance_weight,
    goal_lat_distance_weight};

  auto algo = std::make_unique<fpa::AstarSearch>(planner_common_param, vehicle_shape, astar_param);
  return algo;
}

std::unique_ptr<fpa::AbstractPlanningAlgorithm> configure_rrtstar(bool informed, bool update)
{
  auto planner_common_param = get_default_planner_params();

  // configure rrtstar param
  const double mu = 12.0;
  const double margin = 0.2;
  const double max_planning_time = 200;
  const auto rrtstar_param = fpa::RRTStarParam{update, informed, max_planning_time, mu, margin};
  auto algo = std::make_unique<fpa::RRTStar>(planner_common_param, vehicle_shape, rrtstar_param);
  return algo;
}

enum AlgorithmType {
  ASTAR_SINGLE,
  ASTAR_MULTI,
  RRTSTAR_FASTEST,
  RRTSTAR_UPDATE,
  RRTSTAR_INFORMED_UPDATE,
};
// cspell: ignore fpalgos
std::unordered_map<AlgorithmType, std::string> rosbag_dir_prefix_table(
  {{ASTAR_SINGLE, "fpalgos-astar_single"},
   {ASTAR_MULTI, "fpalgos-astar_multi"},
   {RRTSTAR_FASTEST, "fpalgos-rrtstar_fastest"},
   {RRTSTAR_UPDATE, "fpalgos-rrtstar_update"},
   {RRTSTAR_INFORMED_UPDATE, "fpalgos-rrtstar_informed_update"}});

bool test_algorithm(enum AlgorithmType algo_type, bool dump_rosbag = false)
{
  std::unique_ptr<fpa::AbstractPlanningAlgorithm> algo;
  if (algo_type == AlgorithmType::ASTAR_SINGLE) {
    algo = configure_astar(true);
  } else if (algo_type == AlgorithmType::ASTAR_MULTI) {
    algo = configure_astar(false);
  } else if (algo_type == AlgorithmType::RRTSTAR_FASTEST) {
    algo = configure_rrtstar(false, false);
  } else if (algo_type == AlgorithmType::RRTSTAR_UPDATE) {
    algo = configure_rrtstar(false, true);
  } else if (algo_type == AlgorithmType::RRTSTAR_INFORMED_UPDATE) {
    algo = configure_rrtstar(true, true);
  } else {
    throw std::runtime_error("invalid algorithm time");
  }

  // All algorithms have the same interface.
  const auto costmap_msg = construct_cost_map(150, 150, 0.2, 10);
  bool success_all = true;  // if any local test below fails, overwrite this function

  rclcpp::Clock clock{RCL_SYSTEM_TIME};
  for (size_t i = 0; i < goal_poses.size(); ++i) {
    const auto goal_pose = goal_poses.at(i);

    algo->setMap(costmap_msg);
    double msec;
    double cost;

    if (algo_type == RRTSTAR_FASTEST || algo_type == RRTSTAR_UPDATE) {
      std::cout << "measuring average performance ..." << std::endl;
      const size_t N_mc = (algo_type == RRTSTAR_UPDATE ? 5 : 100);
      double time_sum = 0.0;
      double cost_sum = 0.0;
      for (size_t j = 0; j < N_mc; j++) {
        const rclcpp::Time begin = clock.now();
        if (!algo->makePlan(create_pose_msg(start_pose), create_pose_msg(goal_pose))) {
          success_all = false;
          std::cout << "plan fail" << std::endl;
          continue;
        }
        const rclcpp::Time now = clock.now();
        time_sum += (now - begin).seconds() * 1000.0;
        cost_sum += algo->getWaypoints().compute_length();
      }
      msec = time_sum / N_mc;  // average performance
      cost = cost_sum / N_mc;

    } else {
      const rclcpp::Time begin = clock.now();
      if (!algo->makePlan(create_pose_msg(start_pose), create_pose_msg(goal_pose))) {
        success_all = false;
        std::cout << "plan fail" << std::endl;
        continue;
      }
      const rclcpp::Time now = clock.now();
      msec = (now - begin).seconds() * 1000.0;
      cost = algo->getWaypoints().compute_length();
    }

    std::cout << "plan success : " << msec << "[msec]"
              << ", solution cost : " << cost << std::endl;
    const auto result = algo->getWaypoints();
    geometry_msgs::msg::PoseArray trajectory;
    for (const auto & pose : result.waypoints) {
      trajectory.poses.push_back(pose.pose.pose);
    }
    if (algo->hasObstacleOnTrajectory(trajectory)) {
      std::cout << "not feasible trajectory" << std::endl;
      success_all = false;
    }

    if (dump_rosbag) {
      // dump rosbag for visualization using python script
      const std::string dir_name =
        "/tmp/" + rosbag_dir_prefix_table[algo_type] + "-case" + std::to_string(i);

      rcpputils::fs::remove_all(dir_name);

      rosbag2_storage::StorageOptions storage_options;
      storage_options.uri = dir_name;
      storage_options.storage_id = "sqlite3";

      rosbag2_cpp::ConverterOptions converter_options;
      converter_options.input_serialization_format = "cdr";
      converter_options.output_serialization_format = "cdr";

      rosbag2_cpp::Writer writer(std::make_unique<rosbag2_cpp::writers::SequentialWriter>());
      writer.open(storage_options, converter_options);

      add_message_to_rosbag(
        writer, create_float_msg(vehicle_shape.length), "vehicle_length", "std_msgs/msg/Float64");
      add_message_to_rosbag(
        writer, create_float_msg(vehicle_shape.width), "vehicle_width", "std_msgs/msg/Float64");
      add_message_to_rosbag(
        writer, create_float_msg(vehicle_shape.base2back), "vehicle_base2back",
        "std_msgs/msg/Float64");

      add_message_to_rosbag(writer, costmap_msg, "costmap", "nav_msgs/msg/OccupancyGrid");
      add_message_to_rosbag(writer, create_pose_msg(start_pose), "start", "geometry_msgs/msg/Pose");
      add_message_to_rosbag(writer, create_pose_msg(goal_pose), "goal", "geometry_msgs/msg/Pose");
      add_message_to_rosbag(writer, trajectory, "trajectory", "geometry_msgs/msg/PoseArray");
      add_message_to_rosbag(writer, create_float_msg(msec), "elapsed_time", "std_msgs/msg/Float64");
    }
  }
  return success_all;
}

TEST(AstarSearchTestSuite, SingleCurvature)
{
  EXPECT_TRUE(test_algorithm(AlgorithmType::ASTAR_SINGLE));
}

TEST(AstarSearchTestSuite, MultiCurvature)
{
  EXPECT_TRUE(test_algorithm(AlgorithmType::ASTAR_MULTI));
}

TEST(RRTStarTestSuite, Fastest)
{
  EXPECT_TRUE(test_algorithm(AlgorithmType::RRTSTAR_FASTEST));
}

TEST(RRTStarTestSuite, Update)
{
  EXPECT_TRUE(test_algorithm(AlgorithmType::RRTSTAR_UPDATE));
}

TEST(RRTStarTestSuite, InformedUpdate)
{
  EXPECT_TRUE(test_algorithm(AlgorithmType::RRTSTAR_INFORMED_UPDATE));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

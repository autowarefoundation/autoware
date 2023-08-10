// Copyright 2023 TIER IV, Inc.
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

#include "map_projection_loader/map_projection_loader.hpp"

#include "map_projection_loader/load_info_from_lanelet2_map.hpp"

#include <yaml-cpp/yaml.h>

#include <fstream>

tier4_map_msgs::msg::MapProjectorInfo load_info_from_yaml(const std::string & filename)
{
  YAML::Node data = YAML::LoadFile(filename);

  tier4_map_msgs::msg::MapProjectorInfo msg;
  msg.type = data["type"].as<std::string>();
  if (msg.type == "MGRS") {
    msg.mgrs_grid = data["mgrs_grid"].as<std::string>();
  } else if (msg.type == "UTM") {
    msg.map_origin.latitude = data["map_origin"]["latitude"].as<double>();
    msg.map_origin.longitude = data["map_origin"]["longitude"].as<double>();
  } else if (msg.type == "local") {
    ;  // do nothing
  } else {
    throw std::runtime_error(
      "Invalid map projector type. Currently supported types: MGRS, UTM, and local");
  }
  return msg;
}

MapProjectionLoader::MapProjectionLoader() : Node("map_projection_loader")
{
  std::string yaml_filename = this->declare_parameter<std::string>("map_projector_info_path");
  std::string lanelet2_map_filename = this->declare_parameter<std::string>("lanelet2_map_path");
  std::ifstream file(yaml_filename);

  tier4_map_msgs::msg::MapProjectorInfo msg;

  bool use_yaml_file = file.is_open();
  if (use_yaml_file) {
    RCLCPP_INFO(this->get_logger(), "Load %s", yaml_filename.c_str());
    msg = load_info_from_yaml(yaml_filename);
  } else {
    RCLCPP_INFO(this->get_logger(), "Load %s", lanelet2_map_filename.c_str());
    RCLCPP_WARN(
      this->get_logger(),
      "DEPRECATED WARNING: Loading map projection info from lanelet2 map may soon be deleted. "
      "Please use map_projector_info.yaml instead. For more info, visit "
      "https://github.com/autowarefoundation/autoware.universe/blob/main/map/map_projection_loader/"
      "README.md");
    msg = load_info_from_lanelet2_map(lanelet2_map_filename);
  }

  // Publish the message
  publisher_ = this->create_publisher<tier4_map_msgs::msg::MapProjectorInfo>(
    "~/map_projector_info", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());
  publisher_->publish(msg);
}

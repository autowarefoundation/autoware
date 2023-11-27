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

#ifndef OBJECTS_OF_INTEREST_MARKER_INTERFACE__OBJECTS_OF_INTEREST_MARKER_INTERFACE_HPP_
#define OBJECTS_OF_INTEREST_MARKER_INTERFACE__OBJECTS_OF_INTEREST_MARKER_INTERFACE_HPP_
#include "objects_of_interest_marker_interface/coloring.hpp"
#include "objects_of_interest_marker_interface/marker_data.hpp"
#include "objects_of_interest_marker_interface/marker_utils.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_object.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <string>
#include <vector>

namespace objects_of_interest_marker_interface
{
class ObjectsOfInterestMarkerInterface
{
public:
  /**
   * @brief Constructor
   * @param node Node that publishes marker
   * @param name Module name
   */
  ObjectsOfInterestMarkerInterface(rclcpp::Node * node, const std::string & name);

  /**
   * @brief Insert object data to visualize
   * @param pose Object pose
   * @param shape Object shape
   * @param color_name Color name
   */
  void insertObjectData(
    const geometry_msgs::msg::Pose & pose, const autoware_auto_perception_msgs::msg::Shape & shape,
    const ColorName & color_name);

  /**
   * @brief Insert object data to visualize with custom color data
   * @param pose Object pose
   * @param shape Object shape
   * @param color Color data with alpha
   */
  void insertObjectDataWithCustomColor(
    const geometry_msgs::msg::Pose & pose, const autoware_auto_perception_msgs::msg::Shape & shape,
    const std_msgs::msg::ColorRGBA & color);

  /**
   * @brief Publish interest objects marker
   */
  void publishMarkerArray();

  /**
   * @brief Set height offset of markers
   * @param offset Height offset of markers
   */
  void setHeightOffset(const double offset);

  /**
   * @brief Get color data from color name
   * @param color_name Color name
   * @param alpha Alpha
   */
  static std_msgs::msg::ColorRGBA getColor(const ColorName & color_name, const float alpha = 0.99f);

  /**
   * @brief Get module name including this interface
   */
  std::string getName() const { return name_; }

  /**
   * @brief Get height offset
   */
  double getHeightOffset() const { return height_offset_; }

private:
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_marker_;

  double height_offset_{0.5};
  std::vector<ObjectMarkerData> obj_marker_data_array_;

  std::string name_;
  std::string topic_namespace_ = "/planning/debug/objects_of_interest";
};

}  // namespace objects_of_interest_marker_interface

#endif  // OBJECTS_OF_INTEREST_MARKER_INTERFACE__OBJECTS_OF_INTEREST_MARKER_INTERFACE_HPP_

// Copyright 2022 TIER IV, Inc.
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

#include "heatmap_visualizer/utils.hpp"

#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/math/normalization.hpp>
#include <tier4_autoware_utils/math/unit_conversion.hpp>

#include <tf2/utils.h>

#include <cmath>
#include <numeric>

namespace heatmap_visualizer
{
using tier4_autoware_utils::normalizeRadian;

int discretizeAngle(const double theta, const int theta_size)
{
  const double one_angle_range = 2.0 * M_PI / theta_size;
  return static_cast<int>(std::rint(normalizeRadian(theta, 0.0) / one_angle_range)) % theta_size;
}

IndexXYT pose2index(
  const nav_msgs::msg::OccupancyGrid & costmap, const geometry_msgs::msg::Pose & pose_local,
  const int theta_size)
{
  const int index_x = pose_local.position.x / costmap.info.resolution;
  const int index_y = pose_local.position.y / costmap.info.resolution;
  const double theta = tf2::getYaw(pose_local.orientation);
  const int index_theta = discretizeAngle(theta, theta_size);
  return {index_x, index_y, index_theta};
}

void setHeatmapToBuffer(
  const autoware_auto_perception_msgs::msg::DetectedObject & obj,
  const nav_msgs::msg::OccupancyGrid & heatmap, std::vector<float> * data_buffer,
  const bool use_confidence)
{
  int theta_size = 48;
  IndexXYT indexPose{pose2index(heatmap, obj.kinematics.pose_with_covariance.pose, theta_size)};
  int mapWidth = heatmap.info.width;
  int mapHeight = heatmap.info.height;
  int indexX = indexPose.x + mapWidth / 2;
  int indexY = indexPose.y + mapHeight / 2;

  float score;
  if (use_confidence) {
    score = obj.existence_probability;
  } else {
    score = 1.0;
  }

  try {
    data_buffer->at(indexY * mapWidth + indexX) += score;
  } catch (const std::out_of_range & e) {
    RCLCPP_ERROR(rclcpp::get_logger("setHeatmapToBuffer"), e.what());
  }
}

void setHeatmapToOccupancyGrid(
  const std::vector<float> & data_buffer, nav_msgs::msg::OccupancyGrid * heatmap)
{
  float max_value = *std::max_element(data_buffer.begin(), data_buffer.end());
  float min_value = *std::min_element(data_buffer.begin(), data_buffer.end());
  if (max_value == min_value) {
    return;
  }

  for (size_t i = 0; i < heatmap->data.size(); ++i) {
    heatmap->data[i] =
      static_cast<uint8_t>(100 * (data_buffer[i] - min_value) / (max_value - min_value));
  }
  return;
}

uint8_t getSemanticType(const std::string & class_name)
{
  if (class_name == "CAR") {
    return Label::CAR;
  } else if (class_name == "TRUCK") {
    return Label::TRUCK;
  } else if (class_name == "BUS") {
    return Label::BUS;
  } else if (class_name == "TRAILER") {
    return Label::TRAILER;
  } else if (class_name == "BICYCLE") {
    return Label::BICYCLE;
  } else if (class_name == "MOTORBIKE") {
    return Label::MOTORCYCLE;
  } else if (class_name == "PEDESTRIAN") {
    return Label::PEDESTRIAN;
  } else {
    return Label::UNKNOWN;
  }
}

std::string getClassName(const uint8_t label)
{
  if (label == Label::CAR) {
    return "CAR";
  } else if (label == Label::TRUCK) {
    return "TRUCK";
  } else if (label == Label::BUS) {
    return "BUS";
  } else if (label == Label::TRAILER) {
    return "TRAILER";
  } else if (label == Label::BICYCLE) {
    return "BICYCLE";
  } else if (label == Label::MOTORCYCLE) {
    return "MOTORBIKE";
  } else if (label == Label::PEDESTRIAN) {
    return "PEDESTRIAN";
  } else {
    return "UNKNOWN";
  }
}

}  // namespace heatmap_visualizer

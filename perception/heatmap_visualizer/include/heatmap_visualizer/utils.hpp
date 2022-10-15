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

#ifndef HEATMAP_VISUALIZER__UTILS_HPP_
#define HEATMAP_VISUALIZER__UTILS_HPP_

#include <autoware_auto_perception_msgs/msg/detected_object.hpp>
#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>
#include <autoware_auto_perception_msgs/msg/object_classification.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <string>
#include <vector>

namespace heatmap_visualizer
{
using Label = autoware_auto_perception_msgs::msg::ObjectClassification;

/**
 * @brief IndexXYT object
 *
 */
struct IndexXYT
{
  int x;
  int y;
  int theta;
};  // struct IndexXYT

/**
 * @brief
 *
 * @param theta
 * @param theta_size
 * @return int
 */
int discretizeAngle(const double theta, const int theta_size);

/**
 * @brief
 *
 * @param costmap
 * @param pose_local
 * @param theta_size
 * @return IndexXYT
 */
IndexXYT pose2index(
  const nav_msgs::msg::OccupancyGrid & costmap, const geometry_msgs::msg::Pose & pose_local,
  const int theta_size);

/**
 * @brief Set confidence score or 1 to box position to data buffer.
 *
 * @param obj
 * @param heatmap
 * @param data_buffer
 * @param use_confidence
 */
void setHeatmapToBuffer(
  const autoware_auto_perception_msgs::msg::DetectedObject & obj,
  const nav_msgs::msg::OccupancyGrid & heatmap, std::vector<float> * data_buffer,
  const bool use_confidence);

/**
 * @brief Set the Heatmap To Occupancy Grid object
 *
 * @param data_buffer
 * @param heatmap
 */
void setHeatmapToOccupancyGrid(
  const std::vector<float> & data_buffer, nav_msgs::msg::OccupancyGrid * heatmap);

/**
 * @brief Get the Semantic Type object
 *
 * @param class_name
 * @return uint8_t
 */
uint8_t getSemanticType(const std::string & class_name);

/**
 * @brief Get the Class Name object
 *
 * @param label
 * @return std::string
 */
std::string getClassName(const uint8_t label);

}  // namespace heatmap_visualizer

#endif  // HEATMAP_VISUALIZER__UTILS_HPP_

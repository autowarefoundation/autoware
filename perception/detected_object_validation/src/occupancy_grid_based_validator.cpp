// Copyright 2022 Tier IV, Inc.
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

#include "detected_object_validation/occupancy_grid_based_validator/occupancy_grid_based_validator.hpp"

#include <object_recognition_utils/object_classification.hpp>
#include <object_recognition_utils/object_recognition_utils.hpp>
#include <tier4_autoware_utils/geometry/boost_polygon_utils.hpp>

#include <boost/optional.hpp>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace occupancy_grid_based_validator
{
using Shape = autoware_auto_perception_msgs::msg::Shape;
using Polygon2d = tier4_autoware_utils::Polygon2d;

OccupancyGridBasedValidator::OccupancyGridBasedValidator(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("occupancy_grid_based_validator", node_options),
  objects_sub_(this, "~/input/detected_objects", rclcpp::QoS{1}.get_rmw_qos_profile()),
  occ_grid_sub_(this, "~/input/occupancy_grid_map", rclcpp::QoS{1}.get_rmw_qos_profile()),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_),
  sync_(SyncPolicy(10), objects_sub_, occ_grid_sub_)
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  sync_.registerCallback(
    std::bind(&OccupancyGridBasedValidator::onObjectsAndOccGrid, this, _1, _2));
  objects_pub_ = create_publisher<autoware_auto_perception_msgs::msg::DetectedObjects>(
    "~/output/objects", rclcpp::QoS{1});

  mean_threshold_ = declare_parameter<float>("mean_threshold", 0.6);
  enable_debug_ = declare_parameter<bool>("enable_debug", false);
}

void OccupancyGridBasedValidator::onObjectsAndOccGrid(
  const autoware_auto_perception_msgs::msg::DetectedObjects::ConstSharedPtr & input_objects,
  const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & input_occ_grid)
{
  autoware_auto_perception_msgs::msg::DetectedObjects output;
  output.header = input_objects->header;

  // Transform to occ grid frame
  autoware_auto_perception_msgs::msg::DetectedObjects transformed_objects;
  if (!object_recognition_utils::transformObjects(
        *input_objects, input_occ_grid->header.frame_id, tf_buffer_, transformed_objects))
    return;

  // Convert ros data type to cv::Mat
  cv::Mat occ_grid = fromOccupancyGrid(*input_occ_grid);

  // Get vehicle mask image and calculate mean within mask.
  for (size_t i = 0; i < transformed_objects.objects.size(); ++i) {
    const auto & transformed_object = transformed_objects.objects.at(i);
    const auto & object = input_objects->objects.at(i);
    const auto & label = object.classification.front().label;
    if (object_recognition_utils::isCarLikeVehicle(label)) {
      auto mask = getMask(*input_occ_grid, transformed_object);
      const float mean = mask ? cv::mean(occ_grid, mask.value())[0] * 0.01 : 1.0;
      if (mean_threshold_ < mean) output.objects.push_back(object);
    } else {
      output.objects.push_back(object);
    }
  }

  objects_pub_->publish(output);

  if (enable_debug_) showDebugImage(*input_occ_grid, transformed_objects, occ_grid);
}

std::optional<cv::Mat> OccupancyGridBasedValidator::getMask(
  const nav_msgs::msg::OccupancyGrid & occupancy_grid,
  const autoware_auto_perception_msgs::msg::DetectedObject & object)
{
  cv::Mat mask = cv::Mat::zeros(occupancy_grid.info.height, occupancy_grid.info.width, CV_8UC1);
  return getMask(occupancy_grid, object, mask);
}

std::optional<cv::Mat> OccupancyGridBasedValidator::getMask(
  const nav_msgs::msg::OccupancyGrid & occupancy_grid,
  const autoware_auto_perception_msgs::msg::DetectedObject & object, cv::Mat mask)
{
  const auto & resolution = occupancy_grid.info.resolution;
  const auto & origin = occupancy_grid.info.origin;
  std::vector<cv::Point> pixel_vertices;
  Polygon2d poly2d =
    tier4_autoware_utils::toPolygon2d(object.kinematics.pose_with_covariance.pose, object.shape);

  bool is_polygon_within_image = true;
  for (const auto & p : poly2d.outer()) {
    const float px = (p.x() - origin.position.x) / resolution;
    const float py = (p.y() - origin.position.y) / resolution;
    const bool is_point_within_image = (0 <= px && px < mask.cols && 0 <= py && py < mask.rows);

    if (!is_point_within_image) is_polygon_within_image = false;

    pixel_vertices.push_back(cv::Point2f(px, py));
  }

  if (is_polygon_within_image && !pixel_vertices.empty()) {
    cv::fillConvexPoly(mask, pixel_vertices, cv::Scalar(255));
    return mask;
  } else {
    return std::nullopt;
  }
}

cv::Mat OccupancyGridBasedValidator::fromOccupancyGrid(
  const nav_msgs::msg::OccupancyGrid & occupancy_grid)
{
  cv::Mat cv_occ_grid =
    cv::Mat::zeros(occupancy_grid.info.height, occupancy_grid.info.width, CV_8UC1);
  for (size_t i = 0; i < occupancy_grid.data.size(); ++i) {
    size_t y = i / occupancy_grid.info.width;
    size_t x = i % occupancy_grid.info.width;
    const auto & data = occupancy_grid.data[i];
    cv_occ_grid.at<unsigned char>(y, x) =
      std::min(std::max(data, static_cast<signed char>(0)), static_cast<signed char>(50)) * 2;
  }
  return cv_occ_grid;
}

void OccupancyGridBasedValidator::showDebugImage(
  const nav_msgs::msg::OccupancyGrid & ros_occ_grid,
  const autoware_auto_perception_msgs::msg::DetectedObjects & objects, const cv::Mat & occ_grid)
{
  cv::namedWindow("removed_objects_image", cv::WINDOW_NORMAL);
  cv::namedWindow("passed_objects_image", cv::WINDOW_NORMAL);
  cv::Mat removed_objects_image = occ_grid.clone();
  cv::Mat passed_objects_image = occ_grid.clone();

  // Get vehicle mask image and calculate mean within mask.
  for (const auto & object : objects.objects) {
    const auto & label = object.classification.front().label;
    if (object_recognition_utils::isCarLikeVehicle(label)) {
      auto mask = getMask(ros_occ_grid, object);
      const float mean = mask ? cv::mean(occ_grid, mask.value())[0] * 0.01 : 1.0;
      if (mean_threshold_ < mean) {
        auto mask = getMask(ros_occ_grid, object, passed_objects_image);
        if (mask) passed_objects_image = mask.value();
      } else {
        auto mask = getMask(ros_occ_grid, object, removed_objects_image);
        if (mask) removed_objects_image = mask.value();
      }
    }
  }
  cv::imshow("removed_objects_image", removed_objects_image);
  cv::imshow("passed_objects_image", passed_objects_image);
  cv::waitKey(2);
}

}  // namespace occupancy_grid_based_validator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(occupancy_grid_based_validator::OccupancyGridBasedValidator)

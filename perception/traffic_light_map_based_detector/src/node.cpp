// Copyright 2020 Tier IV, Inc.
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
/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
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
 *
 * Authors: Yukihiro Saito
 *
 */

#include "traffic_light_map_based_detector/node.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <lanelet2_extension/visualization/visualization.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <autoware_auto_perception_msgs/msg/traffic_light_roi.hpp>

#include <lanelet2_core/Exceptions.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace
{
cv::Point2d calcRawImagePointFromPoint3D(
  const image_geometry::PinholeCameraModel & pinhole_camera_model, const cv::Point3d & point3d)
{
  cv::Point2d rectified_image_point = pinhole_camera_model.project3dToPixel(point3d);
  return pinhole_camera_model.unrectifyPoint(rectified_image_point);
}

cv::Point2d calcRawImagePointFromPoint3D(
  const image_geometry::PinholeCameraModel & pinhole_camera_model,
  const geometry_msgs::msg::Point & point3d)
{
  return calcRawImagePointFromPoint3D(
    pinhole_camera_model, cv::Point3d(point3d.x, point3d.y, point3d.z));
}

void roundInImageFrame(
  const image_geometry::PinholeCameraModel & pinhole_camera_model, cv::Point2d & point)
{
  const sensor_msgs::msg::CameraInfo camera_info = pinhole_camera_model.cameraInfo();
  point.x =
    std::max(std::min(point.x, static_cast<double>(static_cast<int>(camera_info.width) - 1)), 0.0);
  point.y =
    std::max(std::min(point.y, static_cast<double>(static_cast<int>(camera_info.height) - 1)), 0.0);
}
}  // namespace

namespace traffic_light
{
MapBasedDetector::MapBasedDetector(const rclcpp::NodeOptions & node_options)
: Node("traffic_light_map_based_detector", node_options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  using std::placeholders::_1;

  // subscribers
  map_sub_ = create_subscription<autoware_auto_mapping_msgs::msg::HADMapBin>(
    "~/input/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&MapBasedDetector::mapCallback, this, _1));
  camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    "~/input/camera_info", rclcpp::SensorDataQoS(),
    std::bind(&MapBasedDetector::cameraInfoCallback, this, _1));
  route_sub_ = create_subscription<autoware_planning_msgs::msg::LaneletRoute>(
    "~/input/route", rclcpp::QoS{1}.transient_local(),
    std::bind(&MapBasedDetector::routeCallback, this, _1));

  // publishers
  roi_pub_ = this->create_publisher<autoware_auto_perception_msgs::msg::TrafficLightRoiArray>(
    "~/output/rois", 1);
  viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/markers", 1);

  // parameter declaration needs default values: are 0.0 goof defaults for this?
  config_.max_vibration_pitch = declare_parameter<double>("max_vibration_pitch", 0.0);
  config_.max_vibration_yaw = declare_parameter<double>("max_vibration_yaw", 0.0);
  config_.max_vibration_height = declare_parameter<double>("max_vibration_height", 0.0);
  config_.max_vibration_width = declare_parameter<double>("max_vibration_width", 0.0);
  config_.max_vibration_depth = declare_parameter<double>("max_vibration_depth", 0.0);
}

void MapBasedDetector::cameraInfoCallback(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr input_msg)
{
  if (all_traffic_lights_ptr_ == nullptr && route_traffic_lights_ptr_ == nullptr) {
    return;
  }

  image_geometry::PinholeCameraModel pinhole_camera_model;
  pinhole_camera_model.fromCameraInfo(*input_msg);

  autoware_auto_perception_msgs::msg::TrafficLightRoiArray output_msg;
  output_msg.header = input_msg->header;

  /* Camera pose */
  geometry_msgs::msg::PoseStamped camera_pose_stamped;
  try {
    geometry_msgs::msg::TransformStamped transform;
    transform = tf_buffer_.lookupTransform(
      "map", input_msg->header.frame_id, input_msg->header.stamp,
      rclcpp::Duration::from_seconds(0.2));
    camera_pose_stamped.header = input_msg->header;
    camera_pose_stamped.pose = tier4_autoware_utils::transform2pose(transform.transform);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "cannot get transform from map frame to camera frame");
    return;
  }

  /*
   * visible_traffic_lights : for each traffic light in map check if in range and in view angle of
   * camera
   */
  std::vector<lanelet::ConstLineString3d> visible_traffic_lights;
  // If get a route, use only traffic lights on the route.
  if (route_traffic_lights_ptr_ != nullptr) {
    getVisibleTrafficLights(
      *route_traffic_lights_ptr_, camera_pose_stamped.pose, pinhole_camera_model,
      visible_traffic_lights);
    // If don't get a route, use the traffic lights around ego vehicle.
  } else if (all_traffic_lights_ptr_ != nullptr) {
    getVisibleTrafficLights(
      *all_traffic_lights_ptr_, camera_pose_stamped.pose, pinhole_camera_model,
      visible_traffic_lights);
    // This shouldn't run.
  } else {
    return;
  }

  /*
   * Get the ROI from the lanelet and the intrinsic matrix of camera to determine where it appears
   * in image.
   */
  for (const auto & traffic_light : visible_traffic_lights) {
    autoware_auto_perception_msgs::msg::TrafficLightRoi tl_roi;
    if (!getTrafficLightRoi(
          camera_pose_stamped.pose, pinhole_camera_model, traffic_light, config_, tl_roi)) {
      continue;
    }
    output_msg.rois.push_back(tl_roi);
  }
  roi_pub_->publish(output_msg);
  publishVisibleTrafficLights(camera_pose_stamped, visible_traffic_lights, viz_pub_);
}

bool MapBasedDetector::getTrafficLightRoi(
  const geometry_msgs::msg::Pose & camera_pose,
  const image_geometry::PinholeCameraModel & pinhole_camera_model,
  const lanelet::ConstLineString3d traffic_light, const Config & config,
  autoware_auto_perception_msgs::msg::TrafficLightRoi & tl_roi)
{
  const double tl_height = traffic_light.attributeOr("height", 0.0);
  const auto & tl_left_down_point = traffic_light.front();
  const auto & tl_right_down_point = traffic_light.back();

  tf2::Transform tf_map2camera(
    tf2::Quaternion(
      camera_pose.orientation.x, camera_pose.orientation.y, camera_pose.orientation.z,
      camera_pose.orientation.w),
    tf2::Vector3(camera_pose.position.x, camera_pose.position.y, camera_pose.position.z));
  // id
  tl_roi.id = traffic_light.id();

  // for roi.x_offset and roi.y_offset
  {
    tf2::Transform tf_map2tl(
      tf2::Quaternion(0, 0, 0, 1),
      tf2::Vector3(
        tl_left_down_point.x(), tl_left_down_point.y(), tl_left_down_point.z() + tl_height));
    tf2::Transform tf_camera2tl;
    tf_camera2tl = tf_map2camera.inverse() * tf_map2tl;
    // max vibration
    const double max_vibration_x =
      std::sin(config.max_vibration_yaw * 0.5) * tf_camera2tl.getOrigin().z() +
      config.max_vibration_width * 0.5;
    const double max_vibration_y =
      std::sin(config.max_vibration_pitch * 0.5) * tf_camera2tl.getOrigin().z() +
      config.max_vibration_height * 0.5;
    const double max_vibration_z = config.max_vibration_depth * 0.5;
    // target position in camera coordinate
    geometry_msgs::msg::Point point3d;
    point3d.x = tf_camera2tl.getOrigin().x() - max_vibration_x;
    point3d.y = tf_camera2tl.getOrigin().y() - max_vibration_y;
    point3d.z = tf_camera2tl.getOrigin().z() - max_vibration_z;
    if (point3d.z <= 0.0) {
      return false;
    }
    cv::Point2d point2d = calcRawImagePointFromPoint3D(pinhole_camera_model, point3d);
    roundInImageFrame(pinhole_camera_model, point2d);
    tl_roi.roi.x_offset = point2d.x;
    tl_roi.roi.y_offset = point2d.y;
  }

  // for roi.width and roi.height
  {
    tf2::Transform tf_map2tl(
      tf2::Quaternion(0, 0, 0, 1),
      tf2::Vector3(tl_right_down_point.x(), tl_right_down_point.y(), tl_right_down_point.z()));
    tf2::Transform tf_camera2tl;
    tf_camera2tl = tf_map2camera.inverse() * tf_map2tl;
    // max vibration
    const double max_vibration_x =
      std::sin(config.max_vibration_yaw * 0.5) * tf_camera2tl.getOrigin().z() +
      config.max_vibration_width * 0.5;
    const double max_vibration_y =
      std::sin(config.max_vibration_pitch * 0.5) * tf_camera2tl.getOrigin().z() +
      config.max_vibration_height * 0.5;
    const double max_vibration_z = config.max_vibration_depth * 0.5;
    // target position in camera coordinate
    geometry_msgs::msg::Point point3d;
    point3d.x = tf_camera2tl.getOrigin().x() + max_vibration_x;
    point3d.y = tf_camera2tl.getOrigin().y() + max_vibration_y;
    point3d.z = tf_camera2tl.getOrigin().z() - max_vibration_z;
    if (point3d.z <= 0.0) {
      return false;
    }
    cv::Point2d point2d = calcRawImagePointFromPoint3D(pinhole_camera_model, point3d);
    roundInImageFrame(pinhole_camera_model, point2d);
    tl_roi.roi.width = point2d.x - tl_roi.roi.x_offset;
    tl_roi.roi.height = point2d.y - tl_roi.roi.y_offset;
    if (tl_roi.roi.width < 1 || tl_roi.roi.height < 1) {
      return false;
    }
  }
  return true;
}

void MapBasedDetector::mapCallback(
  const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr input_msg)
{
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();

  lanelet::utils::conversion::fromBinMsg(*input_msg, lanelet_map_ptr_);
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_ptr_);
  std::vector<lanelet::AutowareTrafficLightConstPtr> all_lanelet_traffic_lights =
    lanelet::utils::query::autowareTrafficLights(all_lanelets);
  all_traffic_lights_ptr_ = std::make_shared<MapBasedDetector::TrafficLightSet>();
  for (auto tl_itr = all_lanelet_traffic_lights.begin(); tl_itr != all_lanelet_traffic_lights.end();
       ++tl_itr) {
    lanelet::AutowareTrafficLightConstPtr tl = *tl_itr;

    auto lights = tl->trafficLights();
    for (auto lsp : lights) {
      if (!lsp.isLineString()) {  // traffic lights must be linestrings
        continue;
      }
      all_traffic_lights_ptr_->insert(static_cast<lanelet::ConstLineString3d>(lsp));
    }
  }
}

void MapBasedDetector::routeCallback(
  const autoware_planning_msgs::msg::LaneletRoute::ConstSharedPtr input_msg)
{
  if (lanelet_map_ptr_ == nullptr) {
    RCLCPP_WARN(get_logger(), "cannot set traffic light in route because don't receive map");
    return;
  }
  lanelet::ConstLanelets route_lanelets;
  for (const auto & segment : input_msg->segments) {
    for (const auto & primitive : segment.primitives) {
      try {
        route_lanelets.push_back(lanelet_map_ptr_->laneletLayer.get(primitive.id));
      } catch (const lanelet::NoSuchPrimitiveError & ex) {
        RCLCPP_ERROR(get_logger(), "%s", ex.what());
        return;
      }
    }
  }
  std::vector<lanelet::AutowareTrafficLightConstPtr> route_lanelet_traffic_lights =
    lanelet::utils::query::autowareTrafficLights(route_lanelets);
  route_traffic_lights_ptr_ = std::make_shared<MapBasedDetector::TrafficLightSet>();
  for (auto tl_itr = route_lanelet_traffic_lights.begin();
       tl_itr != route_lanelet_traffic_lights.end(); ++tl_itr) {
    lanelet::AutowareTrafficLightConstPtr tl = *tl_itr;

    auto lights = tl->trafficLights();
    for (auto lsp : lights) {
      if (!lsp.isLineString()) {  // traffic lights must be linestrings
        continue;
      }
      route_traffic_lights_ptr_->insert(static_cast<lanelet::ConstLineString3d>(lsp));
    }
  }
}

void MapBasedDetector::getVisibleTrafficLights(
  const MapBasedDetector::TrafficLightSet & all_traffic_lights,
  const geometry_msgs::msg::Pose & camera_pose,
  const image_geometry::PinholeCameraModel & pinhole_camera_model,
  std::vector<lanelet::ConstLineString3d> & visible_traffic_lights)
{
  for (const auto & traffic_light : all_traffic_lights) {
    const auto & tl_left_down_point = traffic_light.front();
    const auto & tl_right_down_point = traffic_light.back();
    const double tl_height = traffic_light.attributeOr("height", 0.0);

    // check distance range
    geometry_msgs::msg::Point tl_central_point;
    tl_central_point.x = (tl_right_down_point.x() + tl_left_down_point.x()) / 2.0;
    tl_central_point.y = (tl_right_down_point.y() + tl_left_down_point.y()) / 2.0;
    tl_central_point.z = (tl_right_down_point.z() + tl_left_down_point.z() + tl_height) / 2.0;
    constexpr double max_distance_range = 200.0;
    if (!isInDistanceRange(tl_central_point, camera_pose.position, max_distance_range)) {
      continue;
    }

    // check angle range
    const double tl_yaw = tier4_autoware_utils::normalizeRadian(
      std::atan2(
        tl_right_down_point.y() - tl_left_down_point.y(),
        tl_right_down_point.x() - tl_left_down_point.x()) +
      M_PI_2);
    constexpr double max_angle_range = tier4_autoware_utils::deg2rad(40.0);

    // get direction of z axis
    tf2::Vector3 camera_z_dir(0, 0, 1);
    tf2::Matrix3x3 camera_rotation_matrix(tf2::Quaternion(
      camera_pose.orientation.x, camera_pose.orientation.y, camera_pose.orientation.z,
      camera_pose.orientation.w));
    camera_z_dir = camera_rotation_matrix * camera_z_dir;
    double camera_yaw = std::atan2(camera_z_dir.y(), camera_z_dir.x());
    camera_yaw = tier4_autoware_utils::normalizeRadian(camera_yaw);
    if (!isInAngleRange(tl_yaw, camera_yaw, max_angle_range)) {
      continue;
    }

    // check within image frame
    tf2::Transform tf_map2camera(
      tf2::Quaternion(
        camera_pose.orientation.x, camera_pose.orientation.y, camera_pose.orientation.z,
        camera_pose.orientation.w),
      tf2::Vector3(camera_pose.position.x, camera_pose.position.y, camera_pose.position.z));
    tf2::Transform tf_map2tl(
      tf2::Quaternion(0, 0, 0, 1),
      tf2::Vector3(tl_central_point.x, tl_central_point.y, tl_central_point.z));
    tf2::Transform tf_camera2tl;
    tf_camera2tl = tf_map2camera.inverse() * tf_map2tl;

    geometry_msgs::msg::Point camera2tl_point;
    camera2tl_point.x = tf_camera2tl.getOrigin().x();
    camera2tl_point.y = tf_camera2tl.getOrigin().y();
    camera2tl_point.z = tf_camera2tl.getOrigin().z();
    if (!isInImageFrame(pinhole_camera_model, camera2tl_point)) {
      continue;
    }
    visible_traffic_lights.push_back(traffic_light);
  }
}

bool MapBasedDetector::isInDistanceRange(
  const geometry_msgs::msg::Point & tl_point, const geometry_msgs::msg::Point & camera_point,
  const double max_distance_range) const
{
  const double sq_dist = (tl_point.x - camera_point.x) * (tl_point.x - camera_point.x) +
                         (tl_point.y - camera_point.y) * (tl_point.y - camera_point.y);
  return sq_dist < (max_distance_range * max_distance_range);
}

bool MapBasedDetector::isInAngleRange(
  const double & tl_yaw, const double & camera_yaw, const double max_angle_range) const
{
  Eigen::Vector2d vec1, vec2;
  vec1 << std::cos(tl_yaw), std::sin(tl_yaw);
  vec2 << std::cos(camera_yaw), std::sin(camera_yaw);
  const double diff_angle = std::acos(vec1.dot(vec2));
  return std::fabs(diff_angle) < max_angle_range;
}

bool MapBasedDetector::isInImageFrame(
  const image_geometry::PinholeCameraModel & pinhole_camera_model,
  const geometry_msgs::msg::Point & point) const
{
  if (point.z <= 0.0) {
    return false;
  }

  cv::Point2d point2d = calcRawImagePointFromPoint3D(pinhole_camera_model, point);
  if (0 <= point2d.x && point2d.x < pinhole_camera_model.cameraInfo().width) {
    if (0 <= point2d.y && point2d.y < pinhole_camera_model.cameraInfo().height) {
      return true;
    }
  }
  return false;
}

void MapBasedDetector::publishVisibleTrafficLights(
  const geometry_msgs::msg::PoseStamped camera_pose_stamped,
  const std::vector<lanelet::ConstLineString3d> & visible_traffic_lights,
  const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub)
{
  visualization_msgs::msg::MarkerArray output_msg;
  for (const auto & traffic_light : visible_traffic_lights) {
    const auto & tl_left_down_point = traffic_light.front();
    const auto & tl_right_down_point = traffic_light.back();
    const double tl_height = traffic_light.attributeOr("height", 0.0);
    const int id = traffic_light.id();

    geometry_msgs::msg::Point tl_central_point;
    tl_central_point.x = (tl_right_down_point.x() + tl_left_down_point.x()) / 2.0;
    tl_central_point.y = (tl_right_down_point.y() + tl_left_down_point.y()) / 2.0;
    tl_central_point.z = (tl_right_down_point.z() + tl_left_down_point.z() + tl_height) / 2.0;

    visualization_msgs::msg::Marker marker;

    tf2::Transform tf_map2camera(
      tf2::Quaternion(
        camera_pose_stamped.pose.orientation.x, camera_pose_stamped.pose.orientation.y,
        camera_pose_stamped.pose.orientation.z, camera_pose_stamped.pose.orientation.w),
      tf2::Vector3(
        camera_pose_stamped.pose.position.x, camera_pose_stamped.pose.position.y,
        camera_pose_stamped.pose.position.z));
    tf2::Transform tf_map2tl(
      tf2::Quaternion(0, 0, 0, 1),
      tf2::Vector3(tl_central_point.x, tl_central_point.y, tl_central_point.z));
    tf2::Transform tf_camera2tl;
    tf_camera2tl = tf_map2camera.inverse() * tf_map2tl;

    marker.header = camera_pose_stamped.header;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.ns = std::string("beam");
    marker.scale.x = 0.05;
    marker.action = visualization_msgs::msg::Marker::MODIFY;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    geometry_msgs::msg::Point point;
    point.x = 0.0;
    point.y = 0.0;
    point.z = 0.0;
    marker.points.push_back(point);
    point.x = tf_camera2tl.getOrigin().x();
    point.y = tf_camera2tl.getOrigin().y();
    point.z = tf_camera2tl.getOrigin().z();
    marker.points.push_back(point);

    marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    marker.color.a = 0.999;  // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    output_msg.markers.push_back(marker);
  }
  pub->publish(output_msg);
}
}  // namespace traffic_light

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(traffic_light::MapBasedDetector)

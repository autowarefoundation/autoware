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

#include "yabloc_pose_initializer/camera/camera_pose_initializer.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <cv_bridge/cv_bridge.h>

#include <filesystem>

namespace yabloc
{
CameraPoseInitializer::CameraPoseInitializer()
: Node("camera_pose_initializer"), angle_resolution_(declare_parameter<int>("angle_resolution"))
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  const rclcpp::QoS map_qos = rclcpp::QoS(1).transient_local().reliable();
  service_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  marker_module_ = std::make_unique<initializer::MarkerModule>(this);
  projector_module_ = std::make_unique<initializer::ProjectorModule>(this);

  // Subscriber
  auto on_map = std::bind(&CameraPoseInitializer::on_map, this, _1);
  auto on_image = [this](Image::ConstSharedPtr msg) -> void { latest_image_msg_ = msg; };
  sub_map_ = create_subscription<HADMapBin>("~/input/vector_map", map_qos, on_map);
  sub_image_ = create_subscription<Image>("~/input/image_raw", 10, on_image);

  // Server
  auto on_service = std::bind(&CameraPoseInitializer::on_service, this, _1, _2);
  align_server_ = create_service<RequestPoseAlignment>("~/yabloc_align_srv", on_service);

  const std::string model_path = declare_parameter<std::string>("model_path");
  RCLCPP_INFO_STREAM(get_logger(), "segmentation model path: " + model_path);
  if (std::filesystem::exists(model_path)) {
    semantic_segmentation_ = std::make_unique<SemanticSegmentation>(model_path);
  } else {
    semantic_segmentation_ = nullptr;
    SemanticSegmentation::print_error_message(get_logger());
  }
}

cv::Mat bitwise_and_3ch(const cv::Mat src1, const cv::Mat src2)
{
  std::vector<cv::Mat> src1_array;
  std::vector<cv::Mat> src2_array;
  cv::split(src1, src1_array);
  cv::split(src2, src2_array);
  std::vector<cv::Mat> dst_array;
  for (int i = 0; i < 3; i++) {
    cv::Mat dst;
    cv::bitwise_and(src1_array.at(i), src2_array.at(i), dst);
    dst_array.push_back(dst);
  }
  cv::Mat merged;
  cv::merge(dst_array, merged);
  return merged;
}

int count_non_zero(cv::Mat image_3ch)
{
  std::vector<cv::Mat> images;
  cv::split(image_3ch, images);
  int count = 0;
  for (int i = 0; i < 3; i++) {
    count += cv::countNonZero(images.at(i));
  }
  return count;
}

std::optional<double> CameraPoseInitializer::estimate_pose(
  const Eigen::Vector3f & position, const double yaw_angle_rad, const double yaw_std_rad)
{
  if (!projector_module_->define_project_func()) {
    return std::nullopt;
  }
  if (!lane_image_) {
    RCLCPP_WARN_STREAM(get_logger(), "vector map is not ready ");
    return std::nullopt;
  }
  // TODO(KYabuuchi) check latest_image_msg's time stamp, too
  if (!latest_image_msg_.has_value()) {
    RCLCPP_WARN_STREAM(get_logger(), "source image is not ready");
    return std::nullopt;
  }

  cv::Mat segmented_image;
  {
    if (semantic_segmentation_) {
      const cv::Mat src_image =
        cv_bridge::toCvCopy(*latest_image_msg_.value(), sensor_msgs::image_encodings::BGR8)->image;
      segmented_image = semantic_segmentation_->inference(src_image);
    } else {
      RCLCPP_WARN_STREAM(get_logger(), "segmentation service failed unexpectedly");
      // NOTE: Even if the segmentation service fails, the function will still return the
      // yaw_angle_rad as it is and complete the initialization. The service fails
      // when the DNN model is not downloaded. Ideally, initialization should rely on
      // segmentation, but this implementation allows initialization even in cases where network
      // connectivity is not available.
      return yaw_angle_rad;
    }
  }

  geometry_msgs::msg::Pose query_pose;
  query_pose.position.x = position.x();
  query_pose.position.y = position.y();
  query_pose.position.z = position.z();

  lanelet::ConstLanelets current_lanelets;
  std::optional<double> lane_angle_rad = std::nullopt;
  if (lanelet::utils::query::getCurrentLanelets(const_lanelets_, query_pose, &current_lanelets)) {
    lane_angle_rad = lanelet::utils::getLaneletAngle(current_lanelets.front(), query_pose.position);
  }

  cv::Mat projected_image = projector_module_->project_image(segmented_image);
  cv::Mat vector_map_image = lane_image_->create_vector_map_image(position);

  std::vector<float> scores;
  std::vector<float> angles_rad;

  for (int i = -angle_resolution_; i < angle_resolution_; i++) {
    const double angle_rad =
      yaw_angle_rad + yaw_std_rad * static_cast<double>(i) / static_cast<double>(angle_resolution_);
    const double angle_deg = angle_rad * 180. / M_PI;

    cv::Mat rot = cv::getRotationMatrix2D(cv::Point2f(400, 400), angle_deg, 1);
    cv::Mat rotated_image;
    cv::warpAffine(projected_image, rotated_image, rot, vector_map_image.size());
    cv::Mat dst = bitwise_and_3ch(rotated_image, vector_map_image);

    // consider lanelet direction
    float gain = 1;
    if (lane_angle_rad) {
      gain = 2 + std::cos((lane_angle_rad.value() - angle_rad) / 2.0);
    }
    // If count_non_zero() returns 0 everywhere, the orientation is chosen by the only gain
    const float score = gain * (1 + count_non_zero(dst));

    scores.push_back(score);
    angles_rad.push_back(angle_rad);
  }

  marker_module_->publish_marker(scores, angles_rad, position);

  const size_t max_index =
    std::distance(scores.begin(), std::max_element(scores.begin(), scores.end()));
  return angles_rad.at(max_index);
}

void CameraPoseInitializer::on_map(const HADMapBin & msg)
{
  lanelet::LaneletMapPtr lanelet_map(new lanelet::LaneletMap);
  lanelet::utils::conversion::fromBinMsg(msg, lanelet_map);
  lane_image_ = std::make_unique<LaneImage>(lanelet_map);

  const_lanelets_.clear();
  for (auto l : lanelet_map->laneletLayer) {
    const_lanelets_.push_back(l);
  }
}

void CameraPoseInitializer::on_service(
  const RequestPoseAlignment::Request::SharedPtr request,
  RequestPoseAlignment::Response::SharedPtr response)
{
  RCLCPP_INFO_STREAM(get_logger(), "CameraPoseInitializer on_service");

  const auto query_pos_with_cov = request->pose_with_covariance;
  const auto query_pos = request->pose_with_covariance.pose.pose.position;
  const auto orientation = request->pose_with_covariance.pose.pose.orientation;
  const double yaw_std_rad = std::sqrt(query_pos_with_cov.pose.covariance.at(35));
  const Eigen::Vector3f pos_vec3f(query_pos.x, query_pos.y, query_pos.z);
  RCLCPP_INFO_STREAM(get_logger(), "Given initial position " << pos_vec3f.transpose());

  // Estimate orientation
  const double initial_yaw_angle_rad = 2 * std::atan2(orientation.z, orientation.w);
  const auto yaw_angle_rad_opt = estimate_pose(pos_vec3f, initial_yaw_angle_rad, yaw_std_rad);
  if (yaw_angle_rad_opt.has_value()) {
    response->success = true;
    response->pose_with_covariance =
      create_rectified_initial_pose(pos_vec3f, yaw_angle_rad_opt.value(), query_pos_with_cov);
  } else {
    response->success = false;
  }
}

CameraPoseInitializer::PoseCovStamped CameraPoseInitializer::create_rectified_initial_pose(
  const Eigen::Vector3f & pos, double yaw_angle_rad, const PoseCovStamped & src_msg)
{
  PoseCovStamped msg = src_msg;
  msg.pose.pose.position.x = pos.x();
  msg.pose.pose.position.y = pos.y();
  msg.pose.pose.position.z = pos.z();
  msg.pose.pose.orientation.w = std::cos(yaw_angle_rad / 2.);
  msg.pose.pose.orientation.x = 0;
  msg.pose.pose.orientation.y = 0;
  msg.pose.pose.orientation.z = std::sin(yaw_angle_rad / 2.);
  return msg;
}

}  // namespace yabloc

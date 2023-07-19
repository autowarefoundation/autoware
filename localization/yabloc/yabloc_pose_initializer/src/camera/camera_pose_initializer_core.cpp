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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

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

  // Client
  segmentation_client_ = create_client<SegmentationSrv>(
    "~/semantic_segmentation_srv", rmw_qos_profile_services_default, service_callback_group_);

  // Server
  auto on_service = std::bind(&CameraPoseInitializer::on_service, this, _1, _2);
  align_server_ = create_service<RequestPoseAlignment>("~/yabloc_align_srv", on_service);

  using namespace std::literals::chrono_literals;
  while (!segmentation_client_->wait_for_service(1s) && rclcpp::ok()) {
    RCLCPP_INFO_STREAM(get_logger(), "Waiting for " << segmentation_client_->get_service_name());
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

int count_nonzero(cv::Mat image_3ch)
{
  std::vector<cv::Mat> images;
  cv::split(image_3ch, images);
  int count = 0;
  for (int i = 0; i < 3; i++) {
    count += cv::countNonZero(images.at(i));
  }
  return count;
}

bool CameraPoseInitializer::estimate_pose(
  const Eigen::Vector3f & position, double & yaw_angle_rad, double yaw_std_rad)
{
  if (!projector_module_->define_project_func()) {
    return false;
  }
  if (!lane_image_) {
    RCLCPP_WARN_STREAM(get_logger(), "vector map is not ready ");
    return false;
  }
  // TODO(KYabuuchi) check latest_image_msg's time stamp, too
  if (!latest_image_msg_.has_value()) {
    RCLCPP_WARN_STREAM(get_logger(), "source image is not ready");
    return false;
  }

  Image segmented_image;
  {
    // Call semantic segmentation service
    auto request = std::make_shared<SegmentationSrv::Request>();
    request->src_image = *latest_image_msg_.value();
    auto result_future = segmentation_client_->async_send_request(request);
    using namespace std::literals::chrono_literals;
    std::future_status status = result_future.wait_for(1000ms);
    if (status == std::future_status::ready) {
      segmented_image = result_future.get()->dst_image;
    } else {
      RCLCPP_ERROR_STREAM(get_logger(), "segmentation service exited unexpectedly");
      return false;
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
    const float score = gain * count_nonzero(dst);

    // DEBUG:
    constexpr bool imshow = false;
    if (imshow) {
      cv::Mat show_image;
      cv::hconcat(std::vector<cv::Mat>{rotated_image, vector_map_image, dst}, show_image);
      cv::imshow("and operator", show_image);
      cv::waitKey(50);
    }

    scores.push_back(score);
    angles_rad.push_back(angle_rad);
  }

  {
    size_t max_index =
      std::distance(scores.begin(), std::max_element(scores.begin(), scores.end()));
    yaw_angle_rad = angles_rad.at(max_index);
  }

  marker_module_->publish_marker(scores, angles_rad, position);

  return true;
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

  response->success = false;

  const auto query_pos_with_cov = request->pose_with_covariance;
  const auto query_pos = request->pose_with_covariance.pose.pose.position;
  const auto orientation = request->pose_with_covariance.pose.pose.orientation;
  const double yaw_std_rad = std::sqrt(query_pos_with_cov.pose.covariance.at(35));
  const Eigen::Vector3f pos_vec3f(query_pos.x, query_pos.y, query_pos.z);
  RCLCPP_INFO_STREAM(get_logger(), "Given initial position " << pos_vec3f.transpose());

  // Estimate orientation
  const auto header = request->pose_with_covariance.header;
  double yaw_angle_rad = 2 * std::atan2(orientation.z, orientation.w);
  if (estimate_pose(pos_vec3f, yaw_angle_rad, yaw_std_rad)) {
    response->success = true;
    response->pose_with_covariance =
      create_rectified_initial_pose(pos_vec3f, yaw_angle_rad, query_pos_with_cov);
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

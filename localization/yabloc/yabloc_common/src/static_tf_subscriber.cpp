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

#include "yabloc_common/static_tf_subscriber.hpp"

namespace yabloc::common
{
StaticTfSubscriber::StaticTfSubscriber(rclcpp::Clock::SharedPtr clock)
{
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(clock);
  transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

std::optional<Sophus::SE3f> StaticTfSubscriber::se3f(
  const std::string & frame_id, const std::string & parent_frame_id)
{
  std::optional<Eigen::Affine3f> opt_aff = (*this)(frame_id, parent_frame_id);
  if (!opt_aff.has_value()) return std::nullopt;

  Sophus::SE3f se3f(opt_aff->rotation(), opt_aff->translation());
  return se3f;
}

std::optional<Eigen::Affine3f> StaticTfSubscriber::operator()(
  const std::string & frame_id, const std::string & parent_frame_id)
{
  std::optional<Eigen::Affine3f> extrinsic{std::nullopt};
  try {
    geometry_msgs::msg::TransformStamped ts =
      tf_buffer_->lookupTransform(parent_frame_id, frame_id, tf2::TimePointZero);
    Eigen::Vector3f p;
    p.x() = static_cast<float>(ts.transform.translation.x);
    p.y() = static_cast<float>(ts.transform.translation.y);
    p.z() = static_cast<float>(ts.transform.translation.z);

    Eigen::Quaternionf q;
    q.w() = static_cast<float>(ts.transform.rotation.w);
    q.x() = static_cast<float>(ts.transform.rotation.x);
    q.y() = static_cast<float>(ts.transform.rotation.y);
    q.z() = static_cast<float>(ts.transform.rotation.z);
    extrinsic = Eigen::Affine3f::Identity();
    extrinsic->translation() = p;
    extrinsic->matrix().topLeftCorner(3, 3) = q.toRotationMatrix();
  } catch (const tf2::TransformException & ex) {
  }
  return extrinsic;
}

}  // namespace yabloc::common

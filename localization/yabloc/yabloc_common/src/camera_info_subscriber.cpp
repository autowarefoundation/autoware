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

#include "yabloc_common/camera_info_subscriber.hpp"

namespace yabloc::common
{
CameraInfoSubscriber::CameraInfoSubscriber(rclcpp::Node * node)
{
  rclcpp::QoS qos(10);
  auto callback = [this](const CameraInfo & msg) -> void { opt_info_ = msg; };
  sub_info_ = node->create_subscription<CameraInfo>("~/input/camera_info", qos, callback);
}

bool CameraInfoSubscriber::is_camera_info_ready() const
{
  return opt_info_.has_value();
}

bool CameraInfoSubscriber::is_camera_info_nullopt() const
{
  return !(opt_info_.has_value());
}

std::string CameraInfoSubscriber::get_frame_id() const
{
  return opt_info_->header.frame_id;
}

Eigen::Vector2i CameraInfoSubscriber::size() const
{
  if (!opt_info_.has_value()) {
    throw std::runtime_error("camera_info is not ready but it's accessed");
  }
  return {opt_info_->width, opt_info_->height};
}

Eigen::Matrix3f CameraInfoSubscriber::intrinsic() const
{
  if (!opt_info_.has_value()) {
    throw std::runtime_error("camera_info is not ready but it's accessed");
  }
  const Eigen::Matrix3d kd_t = Eigen::Map<const Eigen::Matrix<double, 3, 3>>(opt_info_->k.data());
  return kd_t.cast<float>().transpose();
}

}  // namespace yabloc::common

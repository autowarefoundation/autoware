// Copyright 2015-2019 Autoware Foundation
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

#include "ndt_scan_matcher/tf2_listener_module.hpp"

#include <tier4_autoware_utils/geometry/geometry.hpp>

geometry_msgs::msg::TransformStamped identity_transform_stamped(
  const builtin_interfaces::msg::Time & timestamp, const std::string & header_frame_id,
  const std::string & child_frame_id)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = timestamp;
  transform.header.frame_id = header_frame_id;
  transform.child_frame_id = child_frame_id;
  transform.transform.rotation = tier4_autoware_utils::createQuaternion(0.0, 0.0, 0.0, 1.0);
  transform.transform.translation = tier4_autoware_utils::createTranslation(0.0, 0.0, 0.0);
  return transform;
}

Tf2ListenerModule::Tf2ListenerModule(rclcpp::Node * node)
: logger_(node->get_logger()), tf2_buffer_(node->get_clock()), tf2_listener_(tf2_buffer_)
{
}

bool Tf2ListenerModule::get_transform(
  const builtin_interfaces::msg::Time & timestamp, const std::string & target_frame,
  const std::string & source_frame, const TransformStamped::SharedPtr & transform_stamped_ptr) const
{
  const TransformStamped identity =
    identity_transform_stamped(timestamp, target_frame, source_frame);

  if (target_frame == source_frame) {
    *transform_stamped_ptr = identity;
    return true;
  }

  try {
    *transform_stamped_ptr =
      tf2_buffer_.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(logger_, "%s", ex.what());
    RCLCPP_ERROR(logger_, "Please publish TF %s to %s", target_frame.c_str(), source_frame.c_str());

    *transform_stamped_ptr = identity;
    return false;
  }
  return true;
}

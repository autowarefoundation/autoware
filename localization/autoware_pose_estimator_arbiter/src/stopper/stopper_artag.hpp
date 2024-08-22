// Copyright 2023 Autoware Foundation
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

#ifndef STOPPER__STOPPER_ARTAG_HPP_
#define STOPPER__STOPPER_ARTAG_HPP_

#include "stopper/base_stopper.hpp"

#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <memory>
namespace autoware::pose_estimator_arbiter::stopper
{
class StopperArTag : public BaseStopper
{
  using Image = sensor_msgs::msg::Image;
  using SetBool = std_srvs::srv::SetBool;

public:
  explicit StopperArTag(rclcpp::Node * node, const std::shared_ptr<const SharedData> & shared_data)
  : BaseStopper(node, shared_data)
  {
    ar_tag_is_enabled_ = true;
    pub_image_ = node->create_publisher<Image>("~/output/artag/image", rclcpp::SensorDataQoS());

    // Register callback
    shared_data_->artag_input_image.register_callback([this](Image::ConstSharedPtr msg) -> void {
      if (ar_tag_is_enabled_) {
        pub_image_->publish(*msg);
      }
    });
  }

  void set_enable(bool enabled) override { ar_tag_is_enabled_ = enabled; }

protected:
  rclcpp::CallbackGroup::SharedPtr service_callback_group_;

private:
  bool ar_tag_is_enabled_;
  rclcpp::Publisher<Image>::SharedPtr pub_image_;
};
}  // namespace autoware::pose_estimator_arbiter::stopper

#endif  // STOPPER__STOPPER_ARTAG_HPP_

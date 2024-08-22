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

#ifndef SHARED_DATA_HPP_
#define SHARED_DATA_HPP_

#include <autoware_adapi_v1_msgs/msg/localization_initialization_state.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <memory>
#include <optional>
#include <vector>

namespace autoware::pose_estimator_arbiter
{
template <typename T>
struct CallbackInvokingVariable
{
  CallbackInvokingVariable() = default;

  explicit CallbackInvokingVariable(T initial_data) : value_(initial_data) {}

  // Set data and invoke all callbacks
  void set_and_invoke(T new_value)
  {
    value_ = new_value;

    // Call all callbacks with the new value
    for (const auto & callback : callbacks_) {
      callback(value_.value());
    }
  }

  // Same as std::optional::has_value()
  bool has_value() const { return value_.has_value(); }

  // Same as std::optional::value()
  T operator()() const { return value_.value(); }

  // Register callback function which is invoked when set_and_invoke() is called
  void register_callback(std::function<void(T)> callback) const { callbacks_.push_back(callback); }

  // Create subscription callback function which is used as below:
  //   auto subscriber = create_subscription<T>("topic_name", 10,
  //   callback_invoking_variable.create_callback());
  auto create_callback()
  {
    return std::bind(&CallbackInvokingVariable::set_and_invoke, this, std::placeholders::_1);
  }

private:
  // The latest data
  std::optional<T> value_{std::nullopt};

  // These functions are expected not to change the value variable
  mutable std::vector<std::function<void(T)>> callbacks_;
};

// This structure is handed to several modules as shared_ptr so that all modules can access data.
struct SharedData
{
public:
  using Image = sensor_msgs::msg::Image;
  using PoseCovStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using HADMapBin = autoware_map_msgs::msg::LaneletMapBin;
  using InitializationState = autoware_adapi_v1_msgs::msg::LocalizationInitializationState;

  SharedData() = default;

  // Used for stoppers
  CallbackInvokingVariable<PoseCovStamped::ConstSharedPtr> eagleye_output_pose_cov;
  CallbackInvokingVariable<Image::ConstSharedPtr> artag_input_image;
  CallbackInvokingVariable<PointCloud2::ConstSharedPtr> ndt_input_points;
  CallbackInvokingVariable<Image::ConstSharedPtr> yabloc_input_image;

  // Used for switch rule
  CallbackInvokingVariable<PoseCovStamped::ConstSharedPtr> localization_pose_cov;
  CallbackInvokingVariable<PointCloud2::ConstSharedPtr> point_cloud_map;
  CallbackInvokingVariable<HADMapBin::ConstSharedPtr> vector_map;
  CallbackInvokingVariable<InitializationState::ConstSharedPtr> initialization_state{
    std::make_shared<InitializationState>(
      InitializationState{}.set__state(InitializationState::UNINITIALIZED))};
};

}  // namespace autoware::pose_estimator_arbiter
#endif  // SHARED_DATA_HPP_

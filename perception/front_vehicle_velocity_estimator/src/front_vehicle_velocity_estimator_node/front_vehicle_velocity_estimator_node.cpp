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

#include "front_vehicle_velocity_estimator/front_vehicle_velocity_estimator_node.hpp"

#include "rclcpp/rclcpp.hpp"

#include <memory>
#include <string>
#include <vector>

using namespace std::literals;
using std::chrono::duration;
using std::chrono::duration_cast;
using std::chrono::nanoseconds;

namespace
{
template <class T>
bool update_param(
  const std::vector<rclcpp::Parameter> & params, const std::string & name, T & value)
{
  const auto itr = std::find_if(
    params.cbegin(), params.cend(),
    [&name](const rclcpp::Parameter & p) { return p.get_name() == name; });

  // Not found
  if (itr == params.cend()) {
    return false;
  }

  value = itr->template get_value<T>();
  return true;
}
}  // namespace

namespace front_vehicle_velocity_estimator
{

FrontVehicleVelocityEstimatorNode::FrontVehicleVelocityEstimatorNode(
  const rclcpp::NodeOptions & node_options)
: Node("front_vehicle_velocity_estimator", node_options)
{
  // Parameter Server
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&FrontVehicleVelocityEstimatorNode::onSetParam, this, std::placeholders::_1));

  // Node Parameter
  node_param_.update_rate_hz = declare_parameter<double>("node_params.update_rate_hz", 10.0);

  // Core Parameter
  core_param_.moving_average_num = declare_parameter<int>("core_params.moving_average_num", 1);
  core_param_.threshold_pointcloud_z_high =
    declare_parameter<float>("core_params.threshold_pointcloud_z_high", 1.0f);
  core_param_.threshold_pointcloud_z_low =
    declare_parameter<float>("core_params.threshold_pointcloud_z_low", 0.6f);
  core_param_.threshold_relative_velocity =
    declare_parameter<double>("core_params.threshold_relative_velocity", 10.0);
  core_param_.threshold_absolute_velocity =
    declare_parameter<double>("core_params.threshold_absolute_velocity", 20.0);

  // Core
  front_vehicle_velocity_estimator_ = std::make_unique<FrontVehicleVelocityEstimator>(get_logger());
  front_vehicle_velocity_estimator_->setParam(core_param_);

  // Subscriber
  sub_pointcloud_.subscribe(
    this, "~/input/pointcloud", rclcpp::SensorDataQoS().get_rmw_qos_profile());
  sub_objects_.subscribe(this, "~/input/objects", rclcpp::QoS{1}.get_rmw_qos_profile());
  sub_odometry_.subscribe(this, "~/input/odometry", rclcpp::QoS{1}.get_rmw_qos_profile());

  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  sync_ptr_ = std::make_shared<Sync>(SyncPolicy(20), sub_pointcloud_, sub_objects_, sub_odometry_);
  sync_ptr_->registerCallback(
    std::bind(&FrontVehicleVelocityEstimatorNode::onData, this, _1, _2, _3));

  // Publisher
  pub_objects_ = create_publisher<DetectedObjects>("~/output/objects", 1);
  pub_nearest_neighbor_pointcloud_ =
    create_publisher<PointCloud2>("~/debug/nearest_neighbor_pointcloud", 1);
}

void FrontVehicleVelocityEstimatorNode::onData(
  const PointCloud2::ConstSharedPtr pointcloud_msg,
  const DetectedObjects::ConstSharedPtr object_msg, const Odometry::ConstSharedPtr odometry_msg)
{
  pointcloud_data_ = pointcloud_msg;
  objects_data_ = object_msg;
  odometry_data_ = odometry_msg;

  if (!isDataReady()) {
    return;
  }

  if (!odometry_data_) {
    // If odometry data does not come, publish original objects
    pub_objects_->publish(*objects_data_);
  } else {
    // Set input data
    input_.objects = objects_data_;
    input_.pointcloud = pointcloud_data_;
    input_.odometry = odometry_data_;

    // Update
    output_ = front_vehicle_velocity_estimator_->update(input_);

    // Publish
    pub_objects_->publish(output_.objects);
    pub_nearest_neighbor_pointcloud_->publish(output_.nearest_neighbor_pointcloud);
  }
}

rcl_interfaces::msg::SetParametersResult FrontVehicleVelocityEstimatorNode::onSetParam(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;

  try {
    // Node Parameter
    {
      auto & p = node_param_;

      // Update params
      update_param(params, "node_params.update_rate_hz", p.update_rate_hz);
    }

    // Core Parameter
    {
      auto & p = core_param_;

      // Update params
      update_param(params, "core_params.moving_average_num", p.moving_average_num);
      update_param(
        params, "core_params.threshold_pointcloud_z_high", p.threshold_pointcloud_z_high);
      update_param(params, "core_params.threshold_pointcloud_z_low", p.threshold_pointcloud_z_low);
      update_param(
        params, "core_params.threshold_relative_velocity", p.threshold_relative_velocity);
      update_param(
        params, "core_params.threshold_absolute_velocity", p.threshold_absolute_velocity);

      // Set parameter to instance
      if (front_vehicle_velocity_estimator_) {
        front_vehicle_velocity_estimator_->setParam(core_param_);
      }
    }
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    result.successful = false;
    result.reason = e.what();
    return result;
  }

  result.successful = true;
  result.reason = "success";
  return result;
}

bool FrontVehicleVelocityEstimatorNode::isDataReady()
{
  if (!objects_data_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "waiting for objects msg...");
    return false;
  }
  if (!pointcloud_data_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "waiting for pointcloud msg...");
    return false;
  }
  return true;
}
}  // namespace front_vehicle_velocity_estimator

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(front_vehicle_velocity_estimator::FrontVehicleVelocityEstimatorNode)

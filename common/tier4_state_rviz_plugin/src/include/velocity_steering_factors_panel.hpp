//
//  Copyright 2023 TIER IV, Inc. All rights reserved.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//

#ifndef VELOCITY_STEERING_FACTORS_PANEL_HPP_
#define VELOCITY_STEERING_FACTORS_PANEL_HPP_

#include <QGroupBox>
#include <QLabel>
#include <QLayout>
#include <QPushButton>
#include <QSpinBox>
#include <QTableWidget>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include <autoware_adapi_v1_msgs/msg/planning_behavior.hpp>
#include <autoware_adapi_v1_msgs/msg/steering_factor_array.hpp>
#include <autoware_adapi_v1_msgs/msg/velocity_factor_array.hpp>

#include <memory>

namespace rviz_plugins
{
class VelocitySteeringFactorsPanel : public rviz_common::Panel
{
  using PlanningBehavior = autoware_adapi_v1_msgs::msg::PlanningBehavior;
  using VelocityFactorArray = autoware_adapi_v1_msgs::msg::VelocityFactorArray;
  using VelocityFactor = autoware_adapi_v1_msgs::msg::VelocityFactor;
  using SteeringFactorArray = autoware_adapi_v1_msgs::msg::SteeringFactorArray;
  using SteeringFactor = autoware_adapi_v1_msgs::msg::SteeringFactor;

  Q_OBJECT

public:
  explicit VelocitySteeringFactorsPanel(QWidget * parent = nullptr);
  void onInitialize() override;

protected:
  // Layout
  QGroupBox * makeVelocityFactorsGroup();
  QGroupBox * makeSteeringFactorsGroup();

  rclcpp::Node::SharedPtr raw_node_;

  // Planning
  QTableWidget * velocity_factors_table_{nullptr};
  QTableWidget * steering_factors_table_{nullptr};

  rclcpp::Subscription<VelocityFactorArray>::SharedPtr sub_velocity_factors_;
  rclcpp::Subscription<SteeringFactorArray>::SharedPtr sub_steering_factors_;

  void onVelocityFactors(const VelocityFactorArray::ConstSharedPtr msg);
  void onSteeringFactors(const SteeringFactorArray::ConstSharedPtr msg);
};
}  // namespace rviz_plugins

#endif  // VELOCITY_STEERING_FACTORS_PANEL_HPP_

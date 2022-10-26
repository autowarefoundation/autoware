//
//  Copyright 2022 Tier IV, Inc. All rights reserved.
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

#include "manual_controller.hpp"

#include <QHBoxLayout>
#include <QString>
#include <QTimer>
#include <QVBoxLayout>
#include <rviz_common/display_context.hpp>

#include <memory>
#include <string>

using std::placeholders::_1;

namespace rviz_plugins
{

double lowpassFilter(
  const double current_value, const double prev_value, double cutoff, const double dt)
{
  const double tau = 1.0 / (2.0 * M_PI * cutoff);
  const double a = tau / (dt + tau);
  return prev_value * a + (1.0 - a) * current_value;
}

ManualController::ManualController(QWidget * parent) : rviz_common::Panel(parent)
{
  auto * state_layout = new QHBoxLayout;
  {
    // Enable Button
    enable_button_ptr_ = new QPushButton("Enable Manual Control");
    connect(enable_button_ptr_, SIGNAL(clicked()), SLOT(onClickEnableButton()));
    state_layout->addWidget(enable_button_ptr_);

    // Gate Mode
    auto * gate_prefix_label_ptr = new QLabel("GATE: ");
    gate_prefix_label_ptr->setAlignment(Qt::AlignRight);
    gate_mode_label_ptr_ = new QLabel("INIT");
    gate_mode_label_ptr_->setAlignment(Qt::AlignCenter);
    state_layout->addWidget(gate_prefix_label_ptr);
    state_layout->addWidget(gate_mode_label_ptr_);

    // Engage Status
    auto * engage_prefix_label_ptr = new QLabel("Engage: ");
    engage_prefix_label_ptr->setAlignment(Qt::AlignRight);
    engage_status_label_ptr_ = new QLabel("INIT");
    engage_status_label_ptr_->setAlignment(Qt::AlignCenter);
    state_layout->addWidget(engage_prefix_label_ptr);
    state_layout->addWidget(engage_status_label_ptr_);

    // Gear
    auto * gear_prefix_label_ptr = new QLabel("GEAR: ");
    gear_prefix_label_ptr->setAlignment(Qt::AlignRight);
    gear_label_ptr_ = new QLabel("INIT");
    gear_label_ptr_->setAlignment(Qt::AlignCenter);
    state_layout->addWidget(gear_prefix_label_ptr);
    state_layout->addWidget(gear_label_ptr_);
  }

  auto * cruise_velocity_layout = new QHBoxLayout();
  // Velocity Limit
  {
    cruise_velocity_button_ptr_ = new QPushButton("Set Cruise Velocity");
    cruise_velocity_input_ = new QSpinBox();
    cruise_velocity_input_->setRange(-100.0, 100.0);
    cruise_velocity_input_->setValue(0.0);
    cruise_velocity_input_->setSingleStep(5.0);
    connect(cruise_velocity_button_ptr_, SIGNAL(clicked()), this, SLOT(onClickCruiseVelocity()));
    cruise_velocity_layout->addWidget(cruise_velocity_button_ptr_);
    cruise_velocity_layout->addWidget(cruise_velocity_input_);
    cruise_velocity_layout->addWidget(new QLabel("  [km/h]"));
  }

  steering_slider_ptr_ = new QDial();
  steering_slider_ptr_->setRange(-90, 90);
  steering_slider_ptr_->setValue(0.0);
  connect(steering_slider_ptr_, SIGNAL(valueChanged(int)), this, SLOT(onManualSteering()));
  steering_angle_ptr_ = new QLabel();
  cruise_velocity_layout->addWidget(new QLabel("steering "));
  cruise_velocity_layout->addWidget(steering_slider_ptr_);
  cruise_velocity_layout->addWidget(steering_angle_ptr_);
  cruise_velocity_layout->addWidget(new QLabel("  [deg]"));

  // Layout
  auto * v_layout = new QVBoxLayout;
  v_layout->addLayout(state_layout);
  v_layout->addLayout(cruise_velocity_layout);
  setLayout(v_layout);

  auto * timer = new QTimer(this);
  connect(timer, &QTimer::timeout, this, &ManualController::update);
  timer->start(30);
}

void ManualController::update()
{
  if (!raw_node_) return;
  AckermannControlCommand ackermann;
  {
    ackermann.stamp = raw_node_->get_clock()->now();
    ackermann.lateral.steering_tire_angle = steering_angle_;
    ackermann.longitudinal.speed = cruise_velocity_;
    if (current_acceleration_) {
      /**
       * @brief Calculate desired acceleration by simple BackSteppingControl
       *  V = 0.5*(v-v_des)^2 >= 0
       *  D[V] = (D[v] - a_des)*(v-v_des) <=0
       *  a_des = k_const *(v - v_des) + a (k < 0 )
       */
      const double k = -0.5;
      const double v = current_velocity_;
      const double v_des = cruise_velocity_;
      const double a = *current_acceleration_;
      const double a_des = k * (v - v_des) + a;
      ackermann.longitudinal.acceleration = std::clamp(a_des, -1.0, 1.0);
    }
  }
  GearCommand gear_cmd;
  {
    const double eps = 0.001;
    if (ackermann.longitudinal.speed > eps) {
      gear_cmd.command = GearCommand::DRIVE;
    } else if (ackermann.longitudinal.speed < -eps && current_velocity_ < eps) {
      gear_cmd.command = GearCommand::REVERSE;
      ackermann.longitudinal.acceleration *= -1.0;
    } else {
      gear_cmd.command = GearCommand::PARK;
    }
  }
  pub_control_command_->publish(ackermann);
  pub_gear_cmd_->publish(gear_cmd);
}

void ManualController::onManualSteering()
{
  const double scale_factor = -0.25;
  steering_angle_ = scale_factor * steering_slider_ptr_->sliderPosition() * M_PI / 180.0;
  const QString steering_string =
    QString::fromStdString(std::to_string(steering_angle_ * 180.0 / M_PI));
  steering_angle_ptr_->setText(steering_string);
}

void ManualController::onClickCruiseVelocity()
{
  cruise_velocity_ = cruise_velocity_input_->value() / 3.6;
}

void ManualController::onInitialize()
{
  raw_node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  sub_gate_mode_ = raw_node_->create_subscription<GateMode>(
    "/control/current_gate_mode", 10, std::bind(&ManualController::onGateMode, this, _1));

  sub_velocity_ = raw_node_->create_subscription<VelocityReport>(
    "/vehicle/status/velocity_status", 1, std::bind(&ManualController::onVelocity, this, _1));

  sub_engage_ = raw_node_->create_subscription<Engage>(
    "/api/autoware/get/engage", 10, std::bind(&ManualController::onEngageStatus, this, _1));

  sub_gear_ = raw_node_->create_subscription<GearReport>(
    "/vehicle/status/gear_status", 10, std::bind(&ManualController::onGear, this, _1));

  client_engage_ = raw_node_->create_client<EngageSrv>(
    "/api/autoware/set/engage", rmw_qos_profile_services_default);

  pub_gate_mode_ = raw_node_->create_publisher<GateMode>("/control/gate_mode_cmd", rclcpp::QoS(1));

  pub_control_command_ = raw_node_->create_publisher<AckermannControlCommand>(
    "/external/selected/control_cmd", rclcpp::QoS(1));

  pub_gear_cmd_ = raw_node_->create_publisher<GearCommand>("/external/selected/gear_cmd", 1);
}

void ManualController::onGateMode(const tier4_control_msgs::msg::GateMode::ConstSharedPtr msg)
{
  switch (msg->data) {
    case tier4_control_msgs::msg::GateMode::AUTO:
      gate_mode_label_ptr_->setText("Not Ready");
      gate_mode_label_ptr_->setStyleSheet("background-color: #00FF00;");
      break;

    case tier4_control_msgs::msg::GateMode::EXTERNAL:
      gate_mode_label_ptr_->setText("Ready");
      gate_mode_label_ptr_->setStyleSheet("background-color: #FFFF00;");
      break;

    default:
      gate_mode_label_ptr_->setText("UNKNOWN");
      gate_mode_label_ptr_->setStyleSheet("background-color: #FF0000;");
      break;
  }
}
void ManualController::onEngageStatus(const Engage::ConstSharedPtr msg)
{
  current_engage_ = msg->engage;
  if (current_engage_) {
    engage_status_label_ptr_->setText(QString::fromStdString("Ready"));
    engage_status_label_ptr_->setStyleSheet("background-color: #FFFF00;");
  } else {
    engage_status_label_ptr_->setText(QString::fromStdString("Not Ready"));
    engage_status_label_ptr_->setStyleSheet("background-color: #00FF00;");
  }
}

void ManualController::onVelocity(const VelocityReport::ConstSharedPtr msg)
{
  current_velocity_ = msg->longitudinal_velocity;
  if (previous_velocity_) {
    const double cutoff = 10.0;
    const double dt = 1.0 / 10.0;
    const double acc = (current_velocity_ - *previous_velocity_) / dt;
    if (!current_acceleration_) {
      current_acceleration_ = std::make_unique<double>(acc);
    } else {
      current_acceleration_ =
        std::make_unique<double>(lowpassFilter(acc, *current_acceleration_, cutoff, dt));
    }
  }
  previous_velocity_ = std::make_unique<double>(msg->longitudinal_velocity);
  prev_stamp_ = rclcpp::Time(msg->header.stamp);
}

void ManualController::onGear(const GearReport::ConstSharedPtr msg)
{
  switch (msg->report) {
    case GearReport::PARK:
      gear_label_ptr_->setText("P");
      break;
    case GearReport::REVERSE:
      gear_label_ptr_->setText("R");
      break;
    case GearReport::DRIVE:
      gear_label_ptr_->setText("D");
      break;
    case GearReport::LOW:
      gear_label_ptr_->setText("L");
      break;
  }
}

void ManualController::onClickEnableButton()
{
  // gate mode
  {
    pub_gate_mode_->publish(tier4_control_msgs::build<GateMode>().data(GateMode::EXTERNAL));
  }
  // engage
  {
    auto req = std::make_shared<EngageSrv::Request>();
    req->engage = true;
    RCLCPP_INFO(raw_node_->get_logger(), "client request");
    if (!client_engage_->service_is_ready()) {
      RCLCPP_INFO(raw_node_->get_logger(), "client is unavailable");
      return;
    }
    client_engage_->async_send_request(
      req, []([[maybe_unused]] rclcpp::Client<EngageSrv>::SharedFuture result) {});
  }
}

}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::ManualController, rviz_common::Panel)

//
//  Copyright 2020 TIER IV, Inc. All rights reserved.
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

#include "state_panel.hpp"

#include <QGridLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QString>
#include <QVBoxLayout>
#include <rviz_common/display_context.hpp>

#include <memory>
#include <string>

namespace tier4_adapi_rviz_plugins
{

StatePanel::StatePanel(QWidget * parent) : rviz_common::Panel(parent)
{
  // Gear
  auto * gear_prefix_label_ptr = new QLabel("GEAR: ");
  gear_prefix_label_ptr->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  gear_label_ptr_ = new QLabel("INIT");
  gear_label_ptr_->setAlignment(Qt::AlignCenter);
  auto * gear_layout = new QHBoxLayout;
  gear_layout->addWidget(gear_prefix_label_ptr);
  gear_layout->addWidget(gear_label_ptr_);

  // Velocity Limit
  velocity_limit_button_ptr_ = new QPushButton("Send Velocity Limit");
  pub_velocity_limit_input_ = new QSpinBox();
  pub_velocity_limit_input_->setRange(-100.0, 100.0);
  pub_velocity_limit_input_->setValue(0.0);
  pub_velocity_limit_input_->setSingleStep(5.0);
  connect(velocity_limit_button_ptr_, SIGNAL(clicked()), this, SLOT(onClickVelocityLimit()));

  // Emergency Button
  emergency_button_ptr_ = new QPushButton("Set Emergency");
  connect(emergency_button_ptr_, SIGNAL(clicked()), this, SLOT(onClickEmergencyButton()));

  // Layout
  auto * v_layout = new QVBoxLayout;
  auto * velocity_limit_layout = new QHBoxLayout();
  v_layout->addWidget(makeOperationModeGroup());
  v_layout->addWidget(makeControlModeGroup());
  {
    auto * h_layout = new QHBoxLayout();
    h_layout->addWidget(makeRoutingGroup());
    h_layout->addWidget(makeLocalizationGroup());
    h_layout->addWidget(makeMotionGroup());
    h_layout->addWidget(makeFailSafeGroup());
    v_layout->addLayout(h_layout);
  }

  v_layout->addLayout(gear_layout);
  velocity_limit_layout->addWidget(velocity_limit_button_ptr_);
  velocity_limit_layout->addWidget(pub_velocity_limit_input_);
  velocity_limit_layout->addWidget(new QLabel("  [km/h]"));
  velocity_limit_layout->addWidget(emergency_button_ptr_);
  v_layout->addLayout(velocity_limit_layout);
  setLayout(v_layout);
}

QGroupBox * StatePanel::makeOperationModeGroup()
{
  auto * group = new QGroupBox("OperationMode");
  auto * grid = new QGridLayout;

  operation_mode_label_ptr_ = new QLabel("INIT");
  operation_mode_label_ptr_->setAlignment(Qt::AlignCenter);
  operation_mode_label_ptr_->setStyleSheet("border:1px solid black;");
  grid->addWidget(operation_mode_label_ptr_, 0, 0, 0, 1);

  auto_button_ptr_ = new QPushButton("AUTO");
  auto_button_ptr_->setCheckable(true);
  connect(auto_button_ptr_, SIGNAL(clicked()), SLOT(onClickAutonomous()));
  grid->addWidget(auto_button_ptr_, 0, 1);

  stop_button_ptr_ = new QPushButton("STOP");
  stop_button_ptr_->setCheckable(true);
  connect(stop_button_ptr_, SIGNAL(clicked()), SLOT(onClickStop()));
  grid->addWidget(stop_button_ptr_, 0, 2);

  local_button_ptr_ = new QPushButton("LOCAL");
  local_button_ptr_->setCheckable(true);
  connect(local_button_ptr_, SIGNAL(clicked()), SLOT(onClickLocal()));
  grid->addWidget(local_button_ptr_, 1, 1);

  remote_button_ptr_ = new QPushButton("REMOTE");
  remote_button_ptr_->setCheckable(true);
  connect(remote_button_ptr_, SIGNAL(clicked()), SLOT(onClickRemote()));
  grid->addWidget(remote_button_ptr_, 1, 2);

  group->setLayout(grid);
  return group;
}

QGroupBox * StatePanel::makeControlModeGroup()
{
  auto * group = new QGroupBox("AutowareControl");
  auto * grid = new QGridLayout;

  control_mode_label_ptr_ = new QLabel("INIT");
  control_mode_label_ptr_->setAlignment(Qt::AlignCenter);
  control_mode_label_ptr_->setStyleSheet("border:1px solid black;");
  grid->addWidget(control_mode_label_ptr_, 0, 0);

  enable_button_ptr_ = new QPushButton("Enable");
  enable_button_ptr_->setCheckable(true);
  connect(enable_button_ptr_, SIGNAL(clicked()), SLOT(onClickAutowareControl()));
  grid->addWidget(enable_button_ptr_, 0, 1);

  disable_button_ptr_ = new QPushButton("Disable");
  disable_button_ptr_->setCheckable(true);
  connect(disable_button_ptr_, SIGNAL(clicked()), SLOT(onClickDirectControl()));
  grid->addWidget(disable_button_ptr_, 0, 2);

  group->setLayout(grid);
  return group;
}

QGroupBox * StatePanel::makeRoutingGroup()
{
  auto * group = new QGroupBox("Routing");
  auto * grid = new QGridLayout;

  routing_label_ptr_ = new QLabel("INIT");
  routing_label_ptr_->setAlignment(Qt::AlignCenter);
  routing_label_ptr_->setStyleSheet("border:1px solid black;");
  grid->addWidget(routing_label_ptr_, 0, 0);

  clear_route_button_ptr_ = new QPushButton("Clear Route");
  clear_route_button_ptr_->setCheckable(true);
  connect(clear_route_button_ptr_, SIGNAL(clicked()), SLOT(onClickClearRoute()));
  grid->addWidget(clear_route_button_ptr_, 1, 0);

  group->setLayout(grid);
  return group;
}

QGroupBox * StatePanel::makeLocalizationGroup()
{
  auto * group = new QGroupBox("Localization");
  auto * grid = new QGridLayout;

  localization_label_ptr_ = new QLabel("INIT");
  localization_label_ptr_->setAlignment(Qt::AlignCenter);
  localization_label_ptr_->setStyleSheet("border:1px solid black;");
  grid->addWidget(localization_label_ptr_, 0, 0);

  init_by_gnss_button_ptr_ = new QPushButton("Init by GNSS");
  connect(init_by_gnss_button_ptr_, SIGNAL(clicked()), SLOT(onClickInitByGnss()));
  grid->addWidget(init_by_gnss_button_ptr_, 1, 0);

  group->setLayout(grid);
  return group;
}

QGroupBox * StatePanel::makeMotionGroup()
{
  auto * group = new QGroupBox("Motion");
  auto * grid = new QGridLayout;

  motion_label_ptr_ = new QLabel("INIT");
  motion_label_ptr_->setAlignment(Qt::AlignCenter);
  motion_label_ptr_->setStyleSheet("border:1px solid black;");
  grid->addWidget(motion_label_ptr_, 0, 0);

  accept_start_button_ptr_ = new QPushButton("Accept Start");
  accept_start_button_ptr_->setCheckable(true);
  connect(accept_start_button_ptr_, SIGNAL(clicked()), SLOT(onClickAcceptStart()));
  grid->addWidget(accept_start_button_ptr_, 1, 0);

  group->setLayout(grid);
  return group;
}

QGroupBox * StatePanel::makeFailSafeGroup()
{
  auto * group = new QGroupBox("FailSafe");
  auto * grid = new QGridLayout;

  mrm_state_label_ptr_ = new QLabel("INIT");
  mrm_state_label_ptr_->setAlignment(Qt::AlignCenter);
  mrm_state_label_ptr_->setStyleSheet("border:1px solid black;");
  grid->addWidget(mrm_state_label_ptr_, 0, 0);

  mrm_behavior_label_ptr_ = new QLabel("INIT");
  mrm_behavior_label_ptr_->setAlignment(Qt::AlignCenter);
  mrm_behavior_label_ptr_->setStyleSheet("border:1px solid black;");
  grid->addWidget(mrm_behavior_label_ptr_, 1, 0);

  group->setLayout(grid);
  return group;
}

void StatePanel::onInitialize()
{
  using std::placeholders::_1;

  raw_node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  // Operation Mode
  sub_operation_mode_ = raw_node_->create_subscription<OperationModeState>(
    "/api/operation_mode/state", rclcpp::QoS{1}.transient_local(),
    std::bind(&StatePanel::onOperationMode, this, _1));

  client_change_to_autonomous_ =
    raw_node_->create_client<ChangeOperationMode>("/api/operation_mode/change_to_autonomous");

  client_change_to_stop_ =
    raw_node_->create_client<ChangeOperationMode>("/api/operation_mode/change_to_stop");

  client_change_to_local_ =
    raw_node_->create_client<ChangeOperationMode>("/api/operation_mode/change_to_local");

  client_change_to_remote_ =
    raw_node_->create_client<ChangeOperationMode>("/api/operation_mode/change_to_remote");

  client_enable_autoware_control_ =
    raw_node_->create_client<ChangeOperationMode>("/api/operation_mode/enable_autoware_control");

  client_enable_direct_control_ =
    raw_node_->create_client<ChangeOperationMode>("/api/operation_mode/disable_autoware_control");

  // Routing
  sub_route_ = raw_node_->create_subscription<RouteState>(
    "/api/routing/state", rclcpp::QoS{1}.transient_local(),
    std::bind(&StatePanel::onRoute, this, _1));

  client_clear_route_ = raw_node_->create_client<ClearRoute>("/api/routing/clear_route");

  // Localization
  sub_localization_ = raw_node_->create_subscription<LocalizationInitializationState>(
    "/api/localization/initialization_state", rclcpp::QoS{1}.transient_local(),
    std::bind(&StatePanel::onLocalization, this, _1));

  client_init_by_gnss_ =
    raw_node_->create_client<InitializeLocalization>("/api/localization/initialize");

  // Motion
  sub_motion_ = raw_node_->create_subscription<MotionState>(
    "/api/motion/state", rclcpp::QoS{1}.transient_local(),
    std::bind(&StatePanel::onMotion, this, _1));

  client_accept_start_ = raw_node_->create_client<AcceptStart>("/api/motion/accept_start");

  // FailSafe
  sub_mrm_ = raw_node_->create_subscription<MRMState>(
    "/api/fail_safe/mrm_state", rclcpp::QoS{1}.transient_local(),
    std::bind(&StatePanel::onMRMState, this, _1));

  // Others
  sub_gear_ = raw_node_->create_subscription<autoware_vehicle_msgs::msg::GearReport>(
    "/vehicle/status/gear_status", 10, std::bind(&StatePanel::onShift, this, _1));

  sub_emergency_ = raw_node_->create_subscription<tier4_external_api_msgs::msg::Emergency>(
    "/api/autoware/get/emergency", 10, std::bind(&StatePanel::onEmergencyStatus, this, _1));

  client_emergency_stop_ = raw_node_->create_client<tier4_external_api_msgs::srv::SetEmergency>(
    "/api/autoware/set/emergency");

  pub_velocity_limit_ = raw_node_->create_publisher<tier4_planning_msgs::msg::VelocityLimit>(
    "/planning/scenario_planning/max_velocity_default", rclcpp::QoS{1}.transient_local());
}

void StatePanel::onOperationMode(const OperationModeState::ConstSharedPtr msg)
{
  auto changeButtonState = [this](
                             QPushButton * button, const bool is_desired_mode_available,
                             const uint8_t current_mode = OperationModeState::UNKNOWN,
                             const uint8_t desired_mode = OperationModeState::STOP) {
    if (is_desired_mode_available && current_mode != desired_mode) {
      activateButton(button);
    } else {
      deactivateButton(button);
    }
  };

  QString text = "";
  QString style_sheet = "";
  // Operation Mode
  switch (msg->mode) {
    case OperationModeState::AUTONOMOUS:
      text = "AUTONOMOUS";
      style_sheet = "background-color: #00FF00;";  // green
      break;

    case OperationModeState::LOCAL:
      text = "LOCAL";
      style_sheet = "background-color: #FFFF00;";  // yellow
      break;

    case OperationModeState::REMOTE:
      text = "REMOTE";
      style_sheet = "background-color: #FFFF00;";  // yellow
      break;

    case OperationModeState::STOP:
      text = "STOP";
      style_sheet = "background-color: #FFA500;";  // orange
      break;

    default:
      text = "UNKNOWN";
      style_sheet = "background-color: #FF0000;";  // red
      break;
  }

  if (msg->is_in_transition) {
    text += "\n(TRANSITION)";
  }

  updateLabel(operation_mode_label_ptr_, text, style_sheet);

  // Control Mode
  if (msg->is_autoware_control_enabled) {
    updateLabel(control_mode_label_ptr_, "Enable", "background-color: #00FF00;");  // green
  } else {
    updateLabel(control_mode_label_ptr_, "Disable", "background-color: #FFFF00;");  // yellow
  }

  // Button
  changeButtonState(
    auto_button_ptr_, msg->is_autonomous_mode_available, msg->mode, OperationModeState::AUTONOMOUS);
  changeButtonState(
    stop_button_ptr_, msg->is_stop_mode_available, msg->mode, OperationModeState::STOP);
  changeButtonState(
    local_button_ptr_, msg->is_local_mode_available, msg->mode, OperationModeState::LOCAL);
  changeButtonState(
    remote_button_ptr_, msg->is_remote_mode_available, msg->mode, OperationModeState::REMOTE);

  changeButtonState(enable_button_ptr_, !msg->is_autoware_control_enabled);
  changeButtonState(disable_button_ptr_, msg->is_autoware_control_enabled);
}

void StatePanel::onRoute(const RouteState::ConstSharedPtr msg)
{
  QString text = "";
  QString style_sheet = "";
  switch (msg->state) {
    case RouteState::UNSET:
      text = "UNSET";
      style_sheet = "background-color: #FFFF00;";  // yellow
      break;

    case RouteState::SET:
      text = "SET";
      style_sheet = "background-color: #00FF00;";  // green
      break;

    case RouteState::ARRIVED:
      text = "ARRIVED";
      style_sheet = "background-color: #FFA500;";  // orange
      break;

    case RouteState::CHANGING:
      text = "CHANGING";
      style_sheet = "background-color: #FFFF00;";  // yellow
      break;

    default:
      text = "UNKNOWN";
      style_sheet = "background-color: #FF0000;";  // red
      break;
  }

  updateLabel(routing_label_ptr_, text, style_sheet);

  if (msg->state == RouteState::SET) {
    activateButton(clear_route_button_ptr_);
  } else {
    deactivateButton(clear_route_button_ptr_);
  }
}

void StatePanel::onLocalization(const LocalizationInitializationState::ConstSharedPtr msg)
{
  QString text = "";
  QString style_sheet = "";
  switch (msg->state) {
    case LocalizationInitializationState::UNINITIALIZED:
      text = "UNINITIALIZED";
      style_sheet = "background-color: #FFFF00;";  // yellow
      break;

    case LocalizationInitializationState::INITIALIZING:
      text = "INITIALIZING";
      style_sheet = "background-color: #FFA500;";  // orange
      break;

    case LocalizationInitializationState::INITIALIZED:
      text = "INITIALIZED";
      style_sheet = "background-color: #00FF00;";  // green
      break;

    default:
      text = "UNKNOWN";
      style_sheet = "background-color: #FF0000;";  // red
      break;
  }

  updateLabel(localization_label_ptr_, text, style_sheet);
}

void StatePanel::onMotion(const MotionState::ConstSharedPtr msg)
{
  QString text = "";
  QString style_sheet = "";
  switch (msg->state) {
    case MotionState::STARTING:
      text = "STARTING";
      style_sheet = "background-color: #FFFF00;";  // yellow
      break;

    case MotionState::STOPPED:
      text = "STOPPED";
      style_sheet = "background-color: #FFA500;";  // orange
      break;

    case MotionState::MOVING:
      text = "MOVING";
      style_sheet = "background-color: #00FF00;";  // green
      break;

    default:
      text = "UNKNOWN";
      style_sheet = "background-color: #FF0000;";  // red
      break;
  }

  updateLabel(motion_label_ptr_, text, style_sheet);

  if (msg->state == MotionState::STARTING) {
    activateButton(accept_start_button_ptr_);
  } else {
    deactivateButton(accept_start_button_ptr_);
  }
}

void StatePanel::onMRMState(const MRMState::ConstSharedPtr msg)
{
  // state
  {
    QString text = "";
    QString style_sheet = "";
    switch (msg->state) {
      case MRMState::NONE:
        text = "NONE";
        style_sheet = "background-color: #00FF00;";  // green
        break;

      case MRMState::MRM_OPERATING:
        text = "MRM_OPERATING";
        style_sheet = "background-color: #FFA500;";  // orange
        break;

      case MRMState::MRM_SUCCEEDED:
        text = "MRM_SUCCEEDED";
        style_sheet = "background-color: #FFFF00;";  // yellow
        break;

      case MRMState::MRM_FAILED:
        text = "MRM_FAILED";
        style_sheet = "background-color: #FF0000;";  // red
        break;

      default:
        text = "UNKNOWN";
        style_sheet = "background-color: #FF0000;";  // red
        break;
    }

    updateLabel(mrm_state_label_ptr_, text, style_sheet);
  }

  // behavior
  {
    QString text = "";
    QString style_sheet = "";
    switch (msg->behavior) {
      case MRMState::NONE:
        text = "NONE";
        style_sheet = "background-color: #00FF00;";  // green
        break;

      case MRMState::PULL_OVER:
        text = "PULL_OVER";
        style_sheet = "background-color: #FFFF00;";  // yellow
        break;

      case MRMState::COMFORTABLE_STOP:
        text = "COMFORTABLE_STOP";
        style_sheet = "background-color: #FFFF00;";  // yellow
        break;

      case MRMState::EMERGENCY_STOP:
        text = "EMERGENCY_STOP";
        style_sheet = "background-color: #FFA500;";  // orange
        break;

      default:
        text = "UNKNOWN";
        style_sheet = "background-color: #FF0000;";  // red
        break;
    }

    updateLabel(mrm_behavior_label_ptr_, text, style_sheet);
  }
}

void StatePanel::onShift(const autoware_vehicle_msgs::msg::GearReport::ConstSharedPtr msg)
{
  switch (msg->report) {
    case autoware_vehicle_msgs::msg::GearReport::PARK:
      gear_label_ptr_->setText("PARKING");
      break;
    case autoware_vehicle_msgs::msg::GearReport::REVERSE:
      gear_label_ptr_->setText("REVERSE");
      break;
    case autoware_vehicle_msgs::msg::GearReport::DRIVE:
      gear_label_ptr_->setText("DRIVE");
      break;
    case autoware_vehicle_msgs::msg::GearReport::NEUTRAL:
      gear_label_ptr_->setText("NEUTRAL");
      break;
    case autoware_vehicle_msgs::msg::GearReport::LOW:
      gear_label_ptr_->setText("LOW");
      break;
  }
}

void StatePanel::onEmergencyStatus(
  const tier4_external_api_msgs::msg::Emergency::ConstSharedPtr msg)
{
  current_emergency_ = msg->emergency;
  if (msg->emergency) {
    emergency_button_ptr_->setText(QString::fromStdString("Clear Emergency"));
    emergency_button_ptr_->setStyleSheet("background-color: #FF0000;");
  } else {
    emergency_button_ptr_->setText(QString::fromStdString("Set Emergency"));
    emergency_button_ptr_->setStyleSheet("background-color: #00FF00;");
  }
}

void StatePanel::onClickVelocityLimit()
{
  auto velocity_limit = std::make_shared<tier4_planning_msgs::msg::VelocityLimit>();
  velocity_limit->stamp = raw_node_->now();
  velocity_limit->max_velocity = pub_velocity_limit_input_->value() / 3.6;
  pub_velocity_limit_->publish(*velocity_limit);
}

void StatePanel::onClickAutonomous()
{
  callServiceWithoutResponse<ChangeOperationMode>(client_change_to_autonomous_);
}
void StatePanel::onClickStop()
{
  callServiceWithoutResponse<ChangeOperationMode>(client_change_to_stop_);
}
void StatePanel::onClickLocal()
{
  callServiceWithoutResponse<ChangeOperationMode>(client_change_to_local_);
}
void StatePanel::onClickRemote()
{
  callServiceWithoutResponse<ChangeOperationMode>(client_change_to_remote_);
}
void StatePanel::onClickAutowareControl()
{
  callServiceWithoutResponse<ChangeOperationMode>(client_enable_autoware_control_);
}
void StatePanel::onClickDirectControl()
{
  callServiceWithoutResponse<ChangeOperationMode>(client_enable_direct_control_);
}

void StatePanel::onClickClearRoute()
{
  callServiceWithoutResponse<ClearRoute>(client_clear_route_);
}

void StatePanel::onClickInitByGnss()
{
  callServiceWithoutResponse<InitializeLocalization>(client_init_by_gnss_);
}

void StatePanel::onClickAcceptStart()
{
  callServiceWithoutResponse<AcceptStart>(client_accept_start_);
}

void StatePanel::onClickEmergencyButton()
{
  using tier4_external_api_msgs::msg::ResponseStatus;
  using tier4_external_api_msgs::srv::SetEmergency;

  auto request = std::make_shared<SetEmergency::Request>();
  request->emergency = !current_emergency_;

  RCLCPP_INFO(raw_node_->get_logger(), request->emergency ? "Set Emergency" : "Clear Emergency");

  client_emergency_stop_->async_send_request(
    request, [this](rclcpp::Client<SetEmergency>::SharedFuture result) {
      const auto & response = result.get();
      if (response->status.code == ResponseStatus::SUCCESS) {
        RCLCPP_INFO(raw_node_->get_logger(), "service succeeded");
      } else {
        RCLCPP_WARN(
          raw_node_->get_logger(), "service failed: %s", response->status.message.c_str());
      }
    });
}

}  // namespace tier4_adapi_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(tier4_adapi_rviz_plugins::StatePanel, rviz_common::Panel)

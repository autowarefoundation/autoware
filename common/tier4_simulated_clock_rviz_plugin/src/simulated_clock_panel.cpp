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

#include "simulated_clock_panel.hpp"

#include <qt5/QtWidgets/QGridLayout>
#include <qt5/QtWidgets/QHBoxLayout>
#include <qt5/QtWidgets/QLabel>
#include <qt5/QtWidgets/QWidget>
#include <rclcpp/duration.hpp>
#include <rviz_common/display_context.hpp>

#include <chrono>
#include <string>

namespace rviz_plugins
{
SimulatedClockPanel::SimulatedClockPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  pause_button_ = new QPushButton("Pause");
  pause_button_->setToolTip("Freeze ROS time.");
  pause_button_->setCheckable(true);

  publishing_rate_input_ = new QSpinBox();
  publishing_rate_input_->setRange(1, 1000);
  publishing_rate_input_->setSingleStep(1);
  publishing_rate_input_->setValue(100);
  publishing_rate_input_->setSuffix("Hz");

  clock_speed_input_ = new QDoubleSpinBox();
  clock_speed_input_->setRange(0.0, 10.0);
  clock_speed_input_->setSingleStep(0.1);
  clock_speed_input_->setValue(1.0);
  clock_speed_input_->setSuffix(" X real time");

  step_button_ = new QPushButton("Step");
  step_button_->setToolTip("Pause and steps the simulation clock");
  step_time_input_ = new QSpinBox();
  step_time_input_->setRange(1, 999);
  step_time_input_->setValue(1);
  step_unit_combo_ = new QComboBox();
  step_unit_combo_->addItems({"s", "ms", "µs", "ns"});

  auto * layout = new QGridLayout(this);
  auto * step_layout = new QHBoxLayout();
  auto * clock_layout = new QHBoxLayout();
  auto * clock_box = new QWidget();
  auto * step_box = new QWidget();
  clock_box->setLayout(clock_layout);
  step_box->setLayout(step_layout);
  layout->addWidget(pause_button_, 0, 0);
  layout->addWidget(step_button_, 1, 0);
  clock_layout->addWidget(new QLabel("Speed:"));
  clock_layout->addWidget(clock_speed_input_);
  clock_layout->addWidget(new QLabel("Rate:"));
  clock_layout->addWidget(publishing_rate_input_);
  step_layout->addWidget(step_time_input_);
  step_layout->addWidget(step_unit_combo_);
  layout->addWidget(clock_box, 0, 1, 1, 2);
  layout->addWidget(step_box, 1, 1, 1, 2);
  layout->setContentsMargins(0, 0, 20, 0);
  prev_published_time_ = std::chrono::system_clock::now();

  connect(publishing_rate_input_, SIGNAL(valueChanged(int)), this, SLOT(onRateChanged(int)));
  connect(step_button_, SIGNAL(clicked()), this, SLOT(onStepClicked()));
}

void SimulatedClockPanel::onInitialize()
{
  raw_node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  clock_pub_ = raw_node_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::QoS(1));
  createWallTimer();
}

void SimulatedClockPanel::onRateChanged(int new_rate)
{
  (void)new_rate;
  pub_timer_->cancel();
  createWallTimer();
}

void SimulatedClockPanel::onStepClicked()
{
  using std::chrono::duration_cast, std::chrono::seconds, std::chrono::milliseconds,
    std::chrono::microseconds, std::chrono::nanoseconds;
  pause_button_->setChecked(true);
  const auto step_time = step_time_input_->value();
  const auto unit = step_unit_combo_->currentText();
  nanoseconds step_duration_ns{};
  if (unit == "s") {
    step_duration_ns += duration_cast<nanoseconds>(seconds(step_time));
  } else if (unit == "ms") {
    step_duration_ns += duration_cast<nanoseconds>(milliseconds(step_time));
  } else if (unit == "µs") {
    step_duration_ns += duration_cast<nanoseconds>(microseconds(step_time));
  } else if (unit == "ns") {
    step_duration_ns += duration_cast<nanoseconds>(nanoseconds(step_time));
  }
  addTimeToClock(step_duration_ns);
}

void SimulatedClockPanel::createWallTimer()
{
  // convert rate from Hz to milliseconds
  const auto period =
    std::chrono::milliseconds(static_cast<int64_t>(1e3 / publishing_rate_input_->value()));
  pub_timer_ = raw_node_->create_wall_timer(period, [&]() { onTimer(); });
}

void SimulatedClockPanel::onTimer()
{
  if (!pause_button_->isChecked()) {
    const auto duration_since_prev_clock = std::chrono::system_clock::now() - prev_published_time_;
    const auto speed_adjusted_duration = duration_since_prev_clock * clock_speed_input_->value();
    addTimeToClock(std::chrono::duration_cast<std::chrono::nanoseconds>(speed_adjusted_duration));
  }
  clock_pub_->publish(clock_msg_);
  prev_published_time_ = std::chrono::system_clock::now();
}

void SimulatedClockPanel::addTimeToClock(std::chrono::nanoseconds time_to_add_ns)
{
  constexpr auto one_sec = std::chrono::seconds(1);
  constexpr auto one_sec_ns = std::chrono::nanoseconds(one_sec);
  while (time_to_add_ns >= one_sec) {
    time_to_add_ns -= one_sec;
    clock_msg_.clock.sec += 1;
  }
  clock_msg_.clock.nanosec += time_to_add_ns.count();
  if (clock_msg_.clock.nanosec >= one_sec_ns.count()) {
    clock_msg_.clock.sec += 1;
    clock_msg_.clock.nanosec = clock_msg_.clock.nanosec - one_sec_ns.count();
  }
}

}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::SimulatedClockPanel, rviz_common::Panel)

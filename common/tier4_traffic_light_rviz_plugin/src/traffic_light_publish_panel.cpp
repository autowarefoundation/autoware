//
//  Copyright 2022 TIER IV, Inc. All rights reserved.
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

#include "traffic_light_publish_panel.hpp"

#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QString>
#include <QStringList>
#include <QVBoxLayout>
#include <rviz_common/display_context.hpp>

#include <memory>
#include <string>

namespace rviz_plugins
{
TrafficLightPublishPanel::TrafficLightPublishPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  // Publish Rate
  publishing_rate_input_ = new QSpinBox();
  publishing_rate_input_->setRange(1, 100);
  publishing_rate_input_->setSingleStep(1);
  publishing_rate_input_->setValue(10);
  publishing_rate_input_->setSuffix("Hz");

  // Traffic Light ID
  traffic_light_id_input_ = new QSpinBox();
  traffic_light_id_input_->setRange(0, 999999);
  traffic_light_id_input_->setValue(0);

  // Traffic Light Color
  light_color_combo_ = new QComboBox();
  light_color_combo_->addItems(
    {"RED", "AMBER", "GREEN", "WHITE", "LEFT_ARROW", "RIGHT_ARROW", "UP_ARROW", "DOWN_ARROW",
     "DOWN_LEFT_ARROW", "DOWN_RIGHT_ARROW", "FLASHING", "UNKNOWN"});

  // Set Traffic Signals Button
  set_button_ = new QPushButton("SET");

  // Reset Traffic Signals Button
  reset_button_ = new QPushButton("RESET");

  // Publish Traffic Signals Button
  publish_button_ = new QPushButton("PUBLISH");

  auto vertical_header = new QHeaderView(Qt::Vertical);
  vertical_header->hide();
  auto horizontal_header = new QHeaderView(Qt::Horizontal);
  horizontal_header->setSectionResizeMode(QHeaderView::Stretch);

  traffic_table_ = new QTableWidget();
  traffic_table_->setColumnCount(2);
  traffic_table_->setHorizontalHeaderLabels({"ID", "Status"});
  traffic_table_->setVerticalHeader(vertical_header);
  traffic_table_->setHorizontalHeader(horizontal_header);

  connect(publishing_rate_input_, SIGNAL(valueChanged(int)), this, SLOT(onRateChanged(int)));
  connect(set_button_, SIGNAL(clicked()), SLOT(onSetTrafficLightState()));
  connect(reset_button_, SIGNAL(clicked()), SLOT(onResetTrafficLightState()));
  connect(publish_button_, SIGNAL(clicked()), SLOT(onPublishTrafficLightState()));

  auto * h_layout_1 = new QHBoxLayout;
  h_layout_1->addWidget(new QLabel("Rate: "));
  h_layout_1->addWidget(publishing_rate_input_);
  h_layout_1->addWidget(new QLabel("Traffic Light ID: "));
  h_layout_1->addWidget(traffic_light_id_input_);

  auto * h_layout_2 = new QHBoxLayout;
  h_layout_2->addWidget(new QLabel("Traffic Light Status: "));
  h_layout_2->addWidget(light_color_combo_);

  auto * v_layout = new QVBoxLayout;
  v_layout->addLayout(h_layout_1);
  v_layout->addLayout(h_layout_2);
  v_layout->addWidget(set_button_);
  v_layout->addWidget(reset_button_);
  v_layout->addWidget(publish_button_);

  auto * h_layout_3 = new QHBoxLayout;
  h_layout_3->addLayout(v_layout);
  h_layout_3->addWidget(traffic_table_);

  setLayout(h_layout_3);
}

void TrafficLightPublishPanel::onSetTrafficLightState()
{
  const auto traffic_light_id = traffic_light_id_input_->value();
  const auto color = light_color_combo_->currentText();

  TrafficLight traffic_light;
  traffic_light.confidence = 1.0;

  if (color == "RED") {
    traffic_light.color = TrafficLight::RED;
  } else if (color == "AMBER") {
    traffic_light.color = TrafficLight::AMBER;
  } else if (color == "GREEN") {
    traffic_light.color = TrafficLight::GREEN;
  } else if (color == "WHITE") {
    traffic_light.color = TrafficLight::WHITE;
  } else if (color == "LEFT_ARROW") {
    traffic_light.color = TrafficLight::LEFT_ARROW;
  } else if (color == "RIGHT_ARROW") {
    traffic_light.color = TrafficLight::RIGHT_ARROW;
  } else if (color == "UP_ARROW") {
    traffic_light.color = TrafficLight::UP_ARROW;
  } else if (color == "DOWN_ARROW") {
    traffic_light.color = TrafficLight::DOWN_ARROW;
  } else if (color == "DOWN_LEFT_ARROW") {
    traffic_light.color = TrafficLight::DOWN_LEFT_ARROW;
  } else if (color == "DOWN_RIGHT_ARROW") {
    traffic_light.color = TrafficLight::DOWN_RIGHT_ARROW;
  } else if (color == "FLASHING") {
    traffic_light.color = TrafficLight::FLASHING;
  } else if (color == "UNKNOWN") {
    traffic_light.color = TrafficLight::UNKNOWN;
  }

  TrafficSignal traffic_signal;
  traffic_signal.lights.push_back(traffic_light);
  traffic_signal.map_primitive_id = traffic_light_id;

  for (auto & signal : extra_traffic_signals_.signals) {
    if (signal.map_primitive_id == traffic_light_id) {
      signal = traffic_signal;
      return;
    }
  }

  extra_traffic_signals_.signals.push_back(traffic_signal);
}

void TrafficLightPublishPanel::onResetTrafficLightState()
{
  extra_traffic_signals_.signals.clear();
  enable_publish_ = false;

  publish_button_->setText("PUBLISH");
  publish_button_->setStyleSheet("background-color: #FFFFFF");
}

void TrafficLightPublishPanel::onPublishTrafficLightState()
{
  enable_publish_ = true;

  publish_button_->setText("PUBLISHING...");
  publish_button_->setStyleSheet("background-color: #FFBF00");
}

void TrafficLightPublishPanel::onInitialize()
{
  raw_node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  pub_traffic_signals_ = raw_node_->create_publisher<TrafficSignalArray>(
    "/perception/traffic_light_recognition/traffic_signals", rclcpp::QoS(1));

  createWallTimer();

  enable_publish_ = false;
}

void TrafficLightPublishPanel::onRateChanged(int new_rate)
{
  (void)new_rate;
  pub_timer_->cancel();
  createWallTimer();
}

void TrafficLightPublishPanel::createWallTimer()
{
  // convert rate from Hz to milliseconds
  const auto period =
    std::chrono::milliseconds(static_cast<int64_t>(1e3 / publishing_rate_input_->value()));
  pub_timer_ = raw_node_->create_wall_timer(period, [&]() { onTimer(); });
}

void TrafficLightPublishPanel::onTimer()
{
  if (enable_publish_) {
    extra_traffic_signals_.header.stamp = rclcpp::Clock().now();
    pub_traffic_signals_->publish(extra_traffic_signals_);
  }

  traffic_table_->setRowCount(extra_traffic_signals_.signals.size());

  if (extra_traffic_signals_.signals.empty()) {
    return;
  }

  for (size_t i = 0; i < extra_traffic_signals_.signals.size(); ++i) {
    const auto & signal = extra_traffic_signals_.signals.at(i);

    if (signal.lights.empty()) {
      continue;
    }

    auto id_label = new QLabel(QString::number(signal.map_primitive_id));
    id_label->setAlignment(Qt::AlignCenter);

    auto color_label = new QLabel();
    color_label->setAlignment(Qt::AlignCenter);

    const auto & light = signal.lights.front();
    switch (light.color) {
      case TrafficLight::RED:
        color_label->setText("RED");
        color_label->setStyleSheet("background-color: #FF0000;");
        break;
      case TrafficLight::AMBER:
        color_label->setText("AMBER");
        color_label->setStyleSheet("background-color: #FFBF00;");
        break;
      case TrafficLight::GREEN:
        color_label->setText("GREEN");
        color_label->setStyleSheet("background-color: #7CFC00;");
        break;
      case TrafficLight::WHITE:
        color_label->setText("WHITE");
        color_label->setStyleSheet("background-color: #FFFFFF;");
        break;
      case TrafficLight::LEFT_ARROW:
        color_label->setText("LEFT_ARROW");
        color_label->setStyleSheet("background-color: #7CFC00;");
        break;
      case TrafficLight::RIGHT_ARROW:
        color_label->setText("RIGHT_ARROW");
        color_label->setStyleSheet("background-color: #7CFC00;");
        break;
      case TrafficLight::UP_ARROW:
        color_label->setText("UP_ARROW");
        color_label->setStyleSheet("background-color: #7CFC00;");
        break;
      case TrafficLight::DOWN_ARROW:
        color_label->setText("DOWN_ARROW");
        color_label->setStyleSheet("background-color: #7CFC00;");
        break;
      case TrafficLight::DOWN_LEFT_ARROW:
        color_label->setText("DOWN_LEFT_ARROW");
        color_label->setStyleSheet("background-color: #7CFC00;");
        break;
      case TrafficLight::DOWN_RIGHT_ARROW:
        color_label->setText("DOWN_RIGHT_ARROW");
        color_label->setStyleSheet("background-color: #7CFC00;");
        break;
      case TrafficLight::FLASHING:
        color_label->setText("FLASHING");
        color_label->setStyleSheet("background-color: #7CFC00;");
        break;
      case TrafficLight::UNKNOWN:
        color_label->setText("UNKNOWN");
        color_label->setStyleSheet("background-color: #808080;");
        break;
      default:
        break;
    }

    traffic_table_->setCellWidget(i, 0, id_label);
    traffic_table_->setCellWidget(i, 1, color_label);
  }
}

}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::TrafficLightPublishPanel, rviz_common::Panel)

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

#ifndef TRAFFIC_LIGHT_PUBLISH_PANEL_HPP_
#define TRAFFIC_LIGHT_PUBLISH_PANEL_HPP_

#ifndef Q_MOC_RUN
#include <qt5/QtWidgets/QComboBox>
#include <qt5/QtWidgets/QPushButton>
#include <qt5/QtWidgets/QSpinBox>
#include <qt5/QtWidgets/QTableWidget>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <rviz_common/panel.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>
#endif

#include <set>

namespace rviz_plugins
{

using autoware_map_msgs::msg::LaneletMapBin;
using autoware_perception_msgs::msg::TrafficLightElement;
using autoware_perception_msgs::msg::TrafficLightGroup;
using autoware_perception_msgs::msg::TrafficLightGroupArray;
class TrafficLightPublishPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit TrafficLightPublishPanel(QWidget * parent = nullptr);
  void onInitialize() override;

public Q_SLOTS:
  void onRateChanged(int new_rate);
  void onSetTrafficLightState();
  void onResetTrafficLightState();
  void onPublishTrafficLightState();

protected:
  void onTimer();
  void createWallTimer();
  void onVectorMap(const LaneletMapBin::ConstSharedPtr msg);

  rclcpp::Node::SharedPtr raw_node_;
  rclcpp::TimerBase::SharedPtr pub_timer_;
  rclcpp::Publisher<TrafficLightGroupArray>::SharedPtr pub_traffic_signals_;
  rclcpp::Subscription<LaneletMapBin>::SharedPtr sub_vector_map_;

  QSpinBox * publishing_rate_input_;
  QComboBox * traffic_light_id_input_;
  QDoubleSpinBox * traffic_light_confidence_input_;
  QComboBox * light_color_combo_;
  QComboBox * light_shape_combo_;
  QComboBox * light_status_combo_;
  QPushButton * set_button_;
  QPushButton * reset_button_;
  QPushButton * publish_button_;
  QTableWidget * traffic_table_;

  TrafficLightGroupArray extra_traffic_signals_;

  bool enable_publish_{false};
  std::set<int> traffic_light_ids_;
  bool received_vector_map_{false};
};

}  // namespace rviz_plugins

#endif  // TRAFFIC_LIGHT_PUBLISH_PANEL_HPP_

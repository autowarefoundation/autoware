// Copyright 2021 Tier IV, Inc.
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

#ifndef AUTOWARE_DATETIME_PANEL_HPP_
#define AUTOWARE_DATETIME_PANEL_HPP_

#include <rviz_common/display_context.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

class QLineEdit;

class AutowareDateTimePanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit AutowareDateTimePanel(QWidget * parent = nullptr);
  void update();
  void onInitialize() override;

private:
  QLineEdit * ros_time_label_;
  QLineEdit * wall_time_label_;

protected:
  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node_;
};

#endif  // AUTOWARE_DATETIME_PANEL_HPP_

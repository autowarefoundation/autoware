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

#ifndef TARGET_OBJECT_TYPE_PANEL_HPP_
#define TARGET_OBJECT_TYPE_PANEL_HPP_

#include <QPushButton>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

class TargetObjectTypePanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit TargetObjectTypePanel(QWidget * parent = 0);

protected:
  QTableWidget * matrix_widget_;
  std::shared_ptr<rclcpp::Node> node_;
  std::vector<std::string> modules_;
  std::vector<std::string> targets_;

  struct ParamNameEnableObject
  {
    std::string node;
    std::string ns;
    std::unordered_map<std::string, std::string> name;
  };
  std::unordered_map<std::string, ParamNameEnableObject> param_names_;

private slots:
  void onReloadButtonClicked();

private:
  QPushButton * reload_button_;

  void updateMatrix();
  void setParameters();
};

#endif  // TARGET_OBJECT_TYPE_PANEL_HPP_

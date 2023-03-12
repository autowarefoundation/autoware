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

#ifndef AUTOMATIC_GOAL_PANEL_HPP_
#define AUTOMATIC_GOAL_PANEL_HPP_

#include "automatic_goal_sender.hpp"

#include <QFileDialog>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLayout>
#include <QListWidget>
#include <QMessageBox>
#include <QPainter>
#include <QPixmap>
#include <QPushButton>
#include <QString>
#include <QTimer>
#include <QVBoxLayout>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/panel.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_adapi_v1_msgs/msg/route_state.hpp>
#include <autoware_adapi_v1_msgs/srv/change_operation_mode.hpp>
#include <autoware_adapi_v1_msgs/srv/clear_route.hpp>
#include <autoware_adapi_v1_msgs/srv/set_route_points.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2/utils.h>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <memory>
#include <string>
#include <utility>

namespace rviz_plugins
{
class AutowareAutomaticGoalPanel : public rviz_common::Panel,
                                   public automatic_goal::AutowareAutomaticGoalSender
{
  using State = automatic_goal::AutomaticGoalState;
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using MarkerArray = visualization_msgs::msg::MarkerArray;
  using OperationModeState = autoware_adapi_v1_msgs::msg::OperationModeState;
  using ChangeOperationMode = autoware_adapi_v1_msgs::srv::ChangeOperationMode;
  using RouteState = autoware_adapi_v1_msgs::msg::RouteState;
  using SetRoutePoints = autoware_adapi_v1_msgs::srv::SetRoutePoints;
  using ClearRoute = autoware_adapi_v1_msgs::srv::ClearRoute;
  Q_OBJECT

public:
  explicit AutowareAutomaticGoalPanel(QWidget * parent = nullptr);
  void onInitialize() override;

public Q_SLOTS:  // NOLINT for Qt
  void onToggleLoopList(bool checked);
  void onToggleAutoMode(bool checked);
  void onToggleSaveGoalsAchievement(bool checked);
  void onClickPlan();
  void onClickStart();
  void onClickStop();
  void onClickClearRoute();
  void onClickRemove();
  void onClickLoadListFromFile();
  void onClickSaveListToFile();

private:
  // Override
  void updateAutoExecutionTimerTick() override;
  void onRouteUpdated(const RouteState::ConstSharedPtr msg) override;
  void onOperationModeUpdated(const OperationModeState::ConstSharedPtr msg) override;
  void onCallResult() override;
  void onGoalListUpdated() override;

  // Inputs
  void onAppendGoal(const PoseStamped::ConstSharedPtr pose);

  // Visual updates
  void updateGUI();
  void updateGoalIcon(const unsigned goal_index, const QColor & color);
  void publishMarkers();
  void showMessageBox(const QString & string);
  void disableAutomaticMode() { automatic_mode_btn_ptr_->setChecked(false); }
  static void activateButton(QAbstractButton * btn) { btn->setEnabled(true); }
  static void deactivateButton(QAbstractButton * btn) { btn->setEnabled(false); }
  static void updateLabel(QLabel * label, const QString & text, const QString & style_sheet)
  {
    label->setText(text);
    label->setStyleSheet(style_sheet);
  }
  // File
  void saveGoalsList(const std::string & file);

  // Pub/Sub
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_marker_{nullptr};
  rclcpp::Subscription<PoseStamped>::SharedPtr sub_append_goal_{nullptr};

  // Containers
  rclcpp::Node::SharedPtr raw_node_{nullptr};
  bool is_automatic_mode_on_{false};
  bool is_loop_list_on_{false};

  // QT Containers
  QGroupBox * makeGoalsListGroup();
  QGroupBox * makeRoutingGroup();
  QGroupBox * makeEngagementGroup();
  QTimer * qt_timer_{nullptr};
  QListWidget * goals_list_widget_ptr_{nullptr};
  QLabel * routing_label_ptr_{nullptr};
  QLabel * operation_mode_label_ptr_{nullptr};
  QLabel * engagement_label_ptr_{nullptr};
  QPushButton * loop_list_btn_ptr_{nullptr};
  QPushButton * goals_achieved_btn_ptr_{nullptr};
  QPushButton * load_file_btn_ptr_{nullptr};
  QPushButton * save_file_btn_ptr_{nullptr};
  QPushButton * automatic_mode_btn_ptr_{nullptr};
  QPushButton * remove_selected_goal_btn_ptr_{nullptr};
  QPushButton * clear_route_btn_ptr_{nullptr};
  QPushButton * plan_btn_ptr_{nullptr};
  QPushButton * start_btn_ptr_{nullptr};
  QPushButton * stop_btn_ptr_{nullptr};
};
}  // namespace rviz_plugins

#endif  // AUTOMATIC_GOAL_PANEL_HPP_

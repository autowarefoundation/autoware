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

#include "automatic_goal_panel.hpp"

namespace rviz_plugins
{
AutowareAutomaticGoalPanel::AutowareAutomaticGoalPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  qt_timer_ = new QTimer(this);
  connect(
    qt_timer_, &QTimer::timeout, this, &AutowareAutomaticGoalPanel::updateAutoExecutionTimerTick);

  auto * h_layout = new QHBoxLayout(this);
  auto * v_layout = new QVBoxLayout(this);
  h_layout->addWidget(makeGoalsListGroup());
  v_layout->addWidget(makeEngagementGroup());
  v_layout->addWidget(makeRoutingGroup());
  h_layout->addLayout(v_layout);
  setLayout(h_layout);
}

// Layouts
QGroupBox * AutowareAutomaticGoalPanel::makeGoalsListGroup()
{
  auto * group = new QGroupBox("GoalsList", this);
  auto * grid = new QGridLayout(group);

  load_file_btn_ptr_ = new QPushButton("Load from file", group);
  connect(load_file_btn_ptr_, SIGNAL(clicked()), SLOT(onClickLoadListFromFile()));
  grid->addWidget(load_file_btn_ptr_, 0, 0);

  save_file_btn_ptr_ = new QPushButton("Save to file", group);
  connect(save_file_btn_ptr_, SIGNAL(clicked()), SLOT(onClickSaveListToFile()));
  grid->addWidget(save_file_btn_ptr_, 1, 0);

  goals_list_widget_ptr_ = new QListWidget(group);
  goals_list_widget_ptr_->setStyleSheet("border:1px solid black;");
  grid->addWidget(goals_list_widget_ptr_, 2, 0);

  remove_selected_goal_btn_ptr_ = new QPushButton("Remove selected", group);
  connect(remove_selected_goal_btn_ptr_, SIGNAL(clicked()), SLOT(onClickRemove()));
  grid->addWidget(remove_selected_goal_btn_ptr_, 3, 0);

  loop_list_btn_ptr_ = new QPushButton("Loop list", group);
  loop_list_btn_ptr_->setCheckable(true);
  connect(loop_list_btn_ptr_, SIGNAL(toggled(bool)), SLOT(onToggleLoopList(bool)));
  grid->addWidget(loop_list_btn_ptr_, 4, 0);

  goals_achieved_btn_ptr_ = new QPushButton("Saving achieved goals to file", group);
  goals_achieved_btn_ptr_->setCheckable(true);
  connect(goals_achieved_btn_ptr_, SIGNAL(toggled(bool)), SLOT(onToggleSaveGoalsAchievement(bool)));
  grid->addWidget(goals_achieved_btn_ptr_, 5, 0);

  group->setLayout(grid);
  return group;
}

QGroupBox * AutowareAutomaticGoalPanel::makeRoutingGroup()
{
  auto * group = new QGroupBox("Routing", this);
  auto * grid = new QGridLayout(group);

  routing_label_ptr_ = new QLabel("INIT", group);
  routing_label_ptr_->setMinimumSize(100, 25);
  routing_label_ptr_->setAlignment(Qt::AlignCenter);
  routing_label_ptr_->setStyleSheet("border:1px solid black;");
  grid->addWidget(routing_label_ptr_, 0, 0);

  clear_route_btn_ptr_ = new QPushButton("Clear planned route", group);
  connect(clear_route_btn_ptr_, &QPushButton::clicked, [this]() { onClickClearRoute(); });
  grid->addWidget(clear_route_btn_ptr_, 1, 0);
  group->setLayout(grid);

  group->setLayout(grid);
  return group;
}

QGroupBox * AutowareAutomaticGoalPanel::makeEngagementGroup()
{
  auto * group = new QGroupBox("Engagement", this);
  auto * grid = new QGridLayout(group);

  engagement_label_ptr_ = new QLabel("INITIALIZING", group);
  engagement_label_ptr_->setMinimumSize(100, 25);
  engagement_label_ptr_->setAlignment(Qt::AlignCenter);
  engagement_label_ptr_->setStyleSheet("border:1px solid black;");
  grid->addWidget(engagement_label_ptr_, 0, 0);

  automatic_mode_btn_ptr_ = new QPushButton("Send goals automatically", group);
  automatic_mode_btn_ptr_->setCheckable(true);

  connect(automatic_mode_btn_ptr_, SIGNAL(toggled(bool)), SLOT(onToggleAutoMode(bool)));
  grid->addWidget(automatic_mode_btn_ptr_, 1, 0);

  plan_btn_ptr_ = new QPushButton("Plan to selected goal", group);
  connect(plan_btn_ptr_, &QPushButton::clicked, [this] { onClickPlan(); });
  grid->addWidget(plan_btn_ptr_, 2, 0);

  start_btn_ptr_ = new QPushButton("Start current plan", group);
  connect(start_btn_ptr_, &QPushButton::clicked, [this] { onClickStart(); });
  grid->addWidget(start_btn_ptr_, 3, 0);

  stop_btn_ptr_ = new QPushButton("Stop current plan", group);
  connect(stop_btn_ptr_, SIGNAL(clicked()), SLOT(onClickStop()));
  grid->addWidget(stop_btn_ptr_, 4, 0);
  group->setLayout(grid);

  group->setLayout(grid);
  return group;
}

void AutowareAutomaticGoalPanel::showMessageBox(const QString & string)
{
  QMessageBox msg_box(this);
  msg_box.setText(string);
  msg_box.exec();
}

// Slots
void AutowareAutomaticGoalPanel::onInitialize()
{
  raw_node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  pub_marker_ = raw_node_->create_publisher<MarkerArray>("~/automatic_goal/markers", 0);
  sub_append_goal_ = raw_node_->create_subscription<PoseStamped>(
    "~/automatic_goal/goal", 5,
    std::bind(&AutowareAutomaticGoalPanel::onAppendGoal, this, std::placeholders::_1));
  initCommunication(raw_node_.get());
}

void AutowareAutomaticGoalPanel::onToggleLoopList(bool checked)
{
  is_loop_list_on_ = checked;
  updateGUI();
}

void AutowareAutomaticGoalPanel::onToggleSaveGoalsAchievement(bool checked)
{
  if (checked) {
    QString file_name = QFileDialog::getSaveFileName(
      this, tr("Save File with  GoalsList"), "/tmp/goals_achieved.log",
      tr("Achieved goals (*.log)"));
    goals_achieved_file_path_ = file_name.toStdString();
  } else {
    goals_achieved_file_path_ = "";
  }
  updateGUI();
}

void AutowareAutomaticGoalPanel::onToggleAutoMode(bool checked)
{
  if (checked && goals_list_widget_ptr_->selectedItems().count() != 1) {
    showMessageBox("Select the first goal in GoalsList");
    automatic_mode_btn_ptr_->setChecked(false);
  } else {
    if (checked) current_goal_ = goals_list_widget_ptr_->currentRow();
    is_automatic_mode_on_ = checked;
    is_automatic_mode_on_ ? qt_timer_->start(1000) : qt_timer_->stop();
    onClickClearRoute();  // here will be set State::AUTO_NEXT or State::EDITING;
  }
}

void AutowareAutomaticGoalPanel::onClickPlan()
{
  if (goals_list_widget_ptr_->selectedItems().count() != 1) {
    showMessageBox("Select a goal in GoalsList");
    return;
  }

  if (callPlanToGoalIndex(cli_set_route_, goals_list_widget_ptr_->currentRow())) {
    state_ = State::PLANNING;
    updateGUI();
  }
}

void AutowareAutomaticGoalPanel::onClickStart()
{
  if (callService<ChangeOperationMode>(cli_change_to_autonomous_)) {
    state_ = State::STARTING;
    updateGUI();
  }
}

void AutowareAutomaticGoalPanel::onClickStop()
{
  // if ERROR is set then the ego is already stopped
  if (state_ == State::ERROR) {
    state_ = State::STOPPED;
    updateGUI();
  } else if (callService<ChangeOperationMode>(cli_change_to_stop_)) {
    state_ = State::STOPPING;
    updateGUI();
  }
}

void AutowareAutomaticGoalPanel::onClickClearRoute()
{
  if (callService<ClearRoute>(cli_clear_route_)) {
    state_ = State::CLEARING;
    updateGUI();
  }
}

void AutowareAutomaticGoalPanel::onClickRemove()
{
  if (static_cast<unsigned>(goals_list_widget_ptr_->currentRow()) < goals_list_.size())
    goals_list_.erase(goals_list_.begin() + goals_list_widget_ptr_->currentRow());
  resetAchievedGoals();
  updateGUI();
  updateGoalsList();
}

void AutowareAutomaticGoalPanel::onClickLoadListFromFile()
{
  QString file_name = QFileDialog::getOpenFileName(
    this, tr("Open File with GoalsList"), "/tmp", tr("Goal lists (*.yaml)"));
  if (file_name.count() > 0) loadGoalsList(file_name.toStdString());
}

void AutowareAutomaticGoalPanel::onClickSaveListToFile()
{
  if (!goals_list_.empty()) {
    QString file_name = QFileDialog::getSaveFileName(
      this, tr("Save File with  GoalsList"), "/tmp/goals_list.yaml", tr("Goal lists (*.yaml)"));
    if (file_name.count() > 0) saveGoalsList(file_name.toStdString());
  }
}

// Inputs
void AutowareAutomaticGoalPanel::onAppendGoal(const PoseStamped::ConstSharedPtr pose)
{
  if (state_ == State::EDITING) {
    goals_list_.push_back(pose);
    updateGoalsList();
    updateGUI();
  }
}

// Override
void AutowareAutomaticGoalPanel::onCallResult()
{
  updateGUI();
}
void AutowareAutomaticGoalPanel::onGoalListUpdated()
{
  goals_list_widget_ptr_->clear();
  for (auto const & goal : goals_achieved_) {
    auto * item =
      new QListWidgetItem(QString::fromStdString(goal.second.first), goals_list_widget_ptr_);
    goals_list_widget_ptr_->addItem(item);
    updateGoalIcon(goals_list_widget_ptr_->count() - 1, QColor("lightGray"));
  }
  publishMarkers();
}
void AutowareAutomaticGoalPanel::onOperationModeUpdated(const OperationModeState::ConstSharedPtr)
{
  updateGUI();
}
void AutowareAutomaticGoalPanel::onRouteUpdated(const RouteState::ConstSharedPtr msg)
{
  std::pair<QString, QString> style;
  if (msg->state == RouteState::UNSET)
    style = std::make_pair("UNSET", "background-color: #FFFF00;");  // yellow
  else if (msg->state == RouteState::SET)
    style = std::make_pair("SET", "background-color: #00FF00;");  // green
  else if (msg->state == RouteState::ARRIVED)
    style = std::make_pair("ARRIVED", "background-color: #FFA500;");  // orange
  else if (msg->state == RouteState::CHANGING)
    style = std::make_pair("CHANGING", "background-color: #FFFF00;");  // yellow
  else
    style = std::make_pair("UNKNOWN", "background-color: #FF0000;");  // red

  updateLabel(routing_label_ptr_, style.first, style.second);
  updateGUI();
}

void AutowareAutomaticGoalPanel::updateAutoExecutionTimerTick()
{
  if (is_automatic_mode_on_) {
    if (state_ == State::AUTO_NEXT) {
      // end if loop is off
      if (current_goal_ >= goals_list_.size() && !is_loop_list_on_) {
        disableAutomaticMode();
        return;
      }
      // plan to next goal
      current_goal_ = current_goal_ % goals_list_.size();
      if (callPlanToGoalIndex(cli_set_route_, current_goal_)) {
        state_ = State::PLANNING;
        updateGUI();
      }
    } else if (state_ == State::PLANNED) {
      updateGoalIcon(current_goal_, QColor("yellow"));
      onClickStart();
    } else if (state_ == State::ARRIVED) {
      goals_achieved_[current_goal_].second++;
      updateAchievedGoalsFile(current_goal_);
      updateGoalIcon(current_goal_++, QColor("green"));
      onClickClearRoute();  // will be set AUTO_NEXT as next state_
    } else if (state_ == State::STOPPED || state_ == State::ERROR) {
      disableAutomaticMode();
    }
  }
}

// Visual updates
void AutowareAutomaticGoalPanel::updateGUI()
{
  deactivateButton(automatic_mode_btn_ptr_);
  deactivateButton(remove_selected_goal_btn_ptr_);
  deactivateButton(clear_route_btn_ptr_);
  deactivateButton(plan_btn_ptr_);
  deactivateButton(start_btn_ptr_);
  deactivateButton(stop_btn_ptr_);
  deactivateButton(load_file_btn_ptr_);
  deactivateButton(save_file_btn_ptr_);
  deactivateButton(loop_list_btn_ptr_);
  deactivateButton(goals_achieved_btn_ptr_);

  std::pair<QString, QString> style;
  switch (state_) {
    case State::EDITING:
      activateButton(load_file_btn_ptr_);
      if (!goals_list_.empty()) {
        activateButton(goals_achieved_btn_ptr_);
        activateButton(plan_btn_ptr_);
        activateButton(remove_selected_goal_btn_ptr_);
        activateButton(automatic_mode_btn_ptr_);
        activateButton(save_file_btn_ptr_);
        activateButton(loop_list_btn_ptr_);
      }
      style = std::make_pair("EDITING", "background-color: #FFFF00;");
      break;
    case State::PLANNED:
      activateButton(start_btn_ptr_);
      activateButton(clear_route_btn_ptr_);
      activateButton(save_file_btn_ptr_);
      style = std::make_pair("PLANNED", "background-color: #00FF00;");
      break;
    case State::STARTED:
      activateButton(stop_btn_ptr_);
      activateButton(save_file_btn_ptr_);
      style = std::make_pair("STARTED", "background-color: #00FF00;");
      break;
    case State::STOPPED:
      activateButton(start_btn_ptr_);
      activateButton(automatic_mode_btn_ptr_);
      activateButton(clear_route_btn_ptr_);
      activateButton(save_file_btn_ptr_);
      style = std::make_pair("STOPPED", "background-color: #00FF00;");
      break;
    case State::ARRIVED:
      if (!is_automatic_mode_on_) onClickClearRoute();  // will be set EDITING as next state_
      break;
    case State::CLEARED:
      is_automatic_mode_on_ ? state_ = State::AUTO_NEXT : state_ = State::EDITING;
      updateGUI();
      break;
    case State::ERROR:
      activateButton(stop_btn_ptr_);
      if (!goals_list_.empty()) activateButton(save_file_btn_ptr_);
      style = std::make_pair("ERROR", "background-color: #FF0000;");
      break;
    case State::PLANNING:
      activateButton(clear_route_btn_ptr_);
      style = std::make_pair("PLANNING", "background-color: #FFA500;");
      break;
    case State::STARTING:
      style = std::make_pair("STARTING", "background-color: #FFA500;");
      break;
    case State::STOPPING:
      style = std::make_pair("STOPPING", "background-color: #FFA500;");
      break;
    case State::CLEARING:
      style = std::make_pair("CLEARING", "background-color: #FFA500;");
      break;
    default:
      break;
  }

  automatic_mode_btn_ptr_->setStyleSheet("");
  loop_list_btn_ptr_->setStyleSheet("");
  goals_achieved_btn_ptr_->setStyleSheet("");
  if (is_automatic_mode_on_) automatic_mode_btn_ptr_->setStyleSheet("background-color: green");
  if (is_loop_list_on_) loop_list_btn_ptr_->setStyleSheet("background-color: green");
  if (!goals_achieved_file_path_.empty())
    goals_achieved_btn_ptr_->setStyleSheet("background-color: green");

  updateLabel(engagement_label_ptr_, style.first, style.second);
}

void AutowareAutomaticGoalPanel::updateGoalIcon(const unsigned goal_index, const QColor & color)
{
  QPixmap pixmap(24, 24);
  pixmap.fill(color);
  QPainter painter(&pixmap);
  painter.setPen(QColor("black"));
  painter.setFont(QFont("fixed-width", 10));
  QString text = QString::number(goals_achieved_[goal_index].second);
  painter.drawText(QRect(0, 0, 24, 24), Qt::AlignCenter, text);
  QIcon icon(pixmap);
  goals_list_widget_ptr_->item(static_cast<int>(goal_index))->setIcon(icon);
}

void AutowareAutomaticGoalPanel::publishMarkers()
{
  MarkerArray text_array;
  MarkerArray arrow_array;
  // Clear existing
  visualization_msgs::msg::Marker marker;
  marker.header.stamp = rclcpp::Time();
  marker.ns = "names";
  marker.id = 0;
  marker.action = visualization_msgs::msg::Marker::DELETEALL;
  text_array.markers.push_back(marker);
  marker.ns = "poses";
  arrow_array.markers.push_back(marker);
  pub_marker_->publish(text_array);
  pub_marker_->publish(arrow_array);
  text_array.markers.clear();
  arrow_array.markers.clear();
  // Publish current
  for (unsigned i = 0; i < goals_list_.size(); i++) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = rclcpp::Time();
    marker.ns = "names";
    marker.id = static_cast<int>(i);
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = goals_list_[i]->pose;
    marker.lifetime = rclcpp::Duration(0, 0);
    marker.scale.x = 1.6;
    marker.scale.y = 1.6;
    marker.scale.z = 1.6;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    marker.frame_locked = false;
    marker.text = "G" + std::to_string(i);
    text_array.markers.push_back(marker);
    marker.ns = "poses";
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    arrow_array.markers.push_back(marker);
  }
  pub_marker_->publish(text_array);
  pub_marker_->publish(arrow_array);
}

// File
void AutowareAutomaticGoalPanel::saveGoalsList(const std::string & file_path)
{
  YAML::Node node;
  for (unsigned i = 0; i < goals_list_.size(); ++i) {
    node[i]["position_x"] = goals_list_[i]->pose.position.x;
    node[i]["position_y"] = goals_list_[i]->pose.position.y;
    node[i]["position_z"] = goals_list_[i]->pose.position.z;
    node[i]["orientation_x"] = goals_list_[i]->pose.orientation.x;
    node[i]["orientation_y"] = goals_list_[i]->pose.orientation.y;
    node[i]["orientation_z"] = goals_list_[i]->pose.orientation.z;
    node[i]["orientation_w"] = goals_list_[i]->pose.orientation.w;
  }
  std::ofstream file_out(file_path);
  file_out << node;
  file_out.close();
}

}  // namespace rviz_plugins
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::AutowareAutomaticGoalPanel, rviz_common::Panel)

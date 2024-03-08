//  Copyright 2024 TIER IV, Inc. All rights reserved.
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

#include "metrics_visualize_panel.hpp"

#include <rviz_common/display_context.hpp>

#include <X11/Xlib.h>

#include <algorithm>
#include <optional>
#include <string>
#include <vector>

namespace rviz_plugins
{
MetricsVisualizePanel::MetricsVisualizePanel(QWidget * parent)
: rviz_common::Panel(parent), grid_(new QGridLayout())
{
  setLayout(grid_);
  topic_selector_ = new QComboBox();
  for (const auto & topic : topics_) {
    topic_selector_->addItem(QString::fromStdString(topic));
  }
  grid_->addWidget(topic_selector_, 0, 0, 1, -1);
  connect(topic_selector_, SIGNAL(currentIndexChanged(int)), this, SLOT(onTopicChanged()));
}

void MetricsVisualizePanel::onInitialize()
{
  using std::placeholders::_1;

  raw_node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  for (const auto & topic_name : topics_) {
    const auto callback = [this, topic_name](const DiagnosticArray::ConstSharedPtr msg) {
      this->onMetrics(msg, topic_name);
    };
    const auto subscription =
      raw_node_->create_subscription<DiagnosticArray>(topic_name, rclcpp::QoS{1}, callback);
    subscriptions_[topic_name] = subscription;
  }

  const auto period = std::chrono::milliseconds(static_cast<int64_t>(1e3 / 10));
  timer_ = raw_node_->create_wall_timer(period, [&]() { onTimer(); });
}

void MetricsVisualizePanel::updateWidgetVisibility(
  const std::string & target_topic, const bool show)
{
  for (const auto & [topic_name, metric_widgets_pair] : topic_widgets_map_) {
    const bool is_target_topic = (topic_name == target_topic);
    if ((!is_target_topic && show) || (is_target_topic && !show)) {
      continue;
    }
    for (const auto & [metric, widgets] : metric_widgets_pair) {
      widgets.first->setVisible(show);
      widgets.second->setVisible(show);
    }
  }
}

void MetricsVisualizePanel::showCurrentTopicWidgets()
{
  const std::string current_topic = topic_selector_->currentText().toStdString();
  updateWidgetVisibility(current_topic, true);
}

void MetricsVisualizePanel::hideInactiveTopicWidgets()
{
  const std::string current_topic = topic_selector_->currentText().toStdString();
  updateWidgetVisibility(current_topic, false);
}

void MetricsVisualizePanel::onTopicChanged()
{
  std::lock_guard<std::mutex> message_lock(mutex_);
  hideInactiveTopicWidgets();
  showCurrentTopicWidgets();
}

void MetricsVisualizePanel::onTimer()
{
  std::lock_guard<std::mutex> message_lock(mutex_);

  for (auto & [name, metric] : metrics_) {
    metric.updateGraph();
    metric.updateTable();
  }
}

void MetricsVisualizePanel::onMetrics(
  const DiagnosticArray::ConstSharedPtr & msg, const std::string & topic_name)
{
  std::lock_guard<std::mutex> message_lock(mutex_);

  const auto time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
  constexpr size_t GRAPH_COL_SIZE = 5;

  for (const auto & status : msg->status) {
    const size_t num_current_metrics = topic_widgets_map_[topic_name].size();
    if (metrics_.count(status.name) == 0) {
      const auto metric = Metric(status);
      metrics_.emplace(status.name, metric);

      // Calculate grid position
      const size_t row = num_current_metrics / GRAPH_COL_SIZE * 2 +
                         1;  // start from 1 to leave space for the topic selector
      const size_t col = num_current_metrics % GRAPH_COL_SIZE;

      // Get the widgets from the metric
      const auto tableWidget = metric.getTable();
      const auto chartViewWidget = metric.getChartView();

      // Add the widgets to the grid layout
      grid_->addWidget(tableWidget, row, col);
      grid_->setRowStretch(row, false);
      grid_->addWidget(chartViewWidget, row + 1, col);
      grid_->setRowStretch(row + 1, true);
      grid_->setColumnStretch(col, true);

      // Also add the widgets to the graph_widgets_ vector for easy removal later
      topic_widgets_map_[topic_name][status.name] = std::make_pair(tableWidget, chartViewWidget);
    }

    metrics_.at(status.name).updateData(time, status);
  }
}

}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::MetricsVisualizePanel, rviz_common::Panel)

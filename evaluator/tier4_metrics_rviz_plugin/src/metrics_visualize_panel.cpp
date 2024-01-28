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
}

void MetricsVisualizePanel::onInitialize()
{
  using std::placeholders::_1;

  raw_node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  sub_ = raw_node_->create_subscription<DiagnosticArray>(
    "/diagnostic/planning_evaluator/metrics", rclcpp::QoS{1},
    std::bind(&MetricsVisualizePanel::onMetrics, this, _1));

  const auto period = std::chrono::milliseconds(static_cast<int64_t>(1e3 / 10));
  timer_ = raw_node_->create_wall_timer(period, [&]() { onTimer(); });
}

void MetricsVisualizePanel::onTimer()
{
  std::lock_guard<std::mutex> message_lock(mutex_);

  for (auto & [name, metric] : metrics_) {
    metric.updateGraph();
    metric.updateTable();
  }
}

void MetricsVisualizePanel::onMetrics(const DiagnosticArray::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> message_lock(mutex_);

  const auto time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

  constexpr size_t GRAPH_COL_SIZE = 5;
  for (size_t i = 0; i < msg->status.size(); ++i) {
    const auto & status = msg->status.at(i);

    if (metrics_.count(status.name) == 0) {
      auto metric = Metric(status);
      metrics_.emplace(status.name, metric);
      grid_->addWidget(metric.getTable(), i / GRAPH_COL_SIZE * 2, i % GRAPH_COL_SIZE);
      grid_->setRowStretch(i / GRAPH_COL_SIZE * 2, false);
      grid_->addWidget(metric.getChartView(), i / GRAPH_COL_SIZE * 2 + 1, i % GRAPH_COL_SIZE);
      grid_->setRowStretch(i / GRAPH_COL_SIZE * 2 + 1, true);
      grid_->setColumnStretch(i % GRAPH_COL_SIZE, true);
    }

    metrics_.at(status.name).updateData(time, status);
  }
}

}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::MetricsVisualizePanel, rviz_common::Panel)

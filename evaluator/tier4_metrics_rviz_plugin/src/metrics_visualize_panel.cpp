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
  // Initialize the main tab widget
  tab_widget_ = new QTabWidget();

  // Create and configure the "All Metrics" tab
  QWidget * all_metrics_widget = new QWidget();
  grid_ = new QGridLayout(all_metrics_widget);  // Apply grid layout to the widget directly
  all_metrics_widget->setLayout(grid_);

  // Add topic selector combobox
  topic_selector_ = new QComboBox();
  for (const auto & topic : topics_) {
    topic_selector_->addItem(QString::fromStdString(topic));
  }
  grid_->addWidget(topic_selector_, 0, 0, 1, -1);  // Add topic selector to the grid layout
  connect(topic_selector_, SIGNAL(currentIndexChanged(int)), this, SLOT(onTopicChanged()));

  tab_widget_->addTab(
    all_metrics_widget, "All Metrics");  // Add "All Metrics" tab to the tab widget

  // Create and configure the "Specific Metrics" tab
  QWidget * specific_metrics_widget = new QWidget();
  QVBoxLayout * specific_metrics_layout = new QVBoxLayout();
  specific_metrics_widget->setLayout(specific_metrics_layout);

  // Add specific metric selector combobox
  specific_metric_selector_ = new QComboBox();
  specific_metrics_layout->addWidget(specific_metric_selector_);
  connect(
    specific_metric_selector_, SIGNAL(currentIndexChanged(int)), this,
    SLOT(onSpecificMetricChanged()));

  // Add clear button
  QPushButton * clear_button = new QPushButton("Clear");
  specific_metrics_layout->addWidget(clear_button);
  connect(clear_button, &QPushButton::clicked, this, &MetricsVisualizePanel::onClearButtonClicked);

  // Add chart view for specific metrics
  specific_metric_chart_view_ = new QChartView();
  specific_metrics_layout->addWidget(specific_metric_chart_view_);

  tab_widget_->addTab(
    specific_metrics_widget, "Specific Metrics");  // Add "Specific Metrics" tab to the tab widget

  // Set the main layout of the panel
  QVBoxLayout * main_layout = new QVBoxLayout();
  main_layout->addWidget(tab_widget_);
  setLayout(main_layout);
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

void MetricsVisualizePanel::updateSelectedMetric(const std::string & metric_name)
{
  std::lock_guard<std::mutex> message_lock(mutex_);

  for (const auto & [topic, msg] : current_msg_map_) {
    const auto time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    for (const auto & status : msg->status) {
      if (metric_name == status.name) {
        selected_metrics_ = {metric_name, Metric(status)};
        selected_metrics_->second.updateData(time, status);
        return;
      }
    }
  }
}

void MetricsVisualizePanel::updateViews()
{
  if (!selected_metrics_) {
    return;
  }

  Metric & metric = selected_metrics_->second;
  specific_metric_chart_view_->setChart(metric.getChartView()->chart());
  auto * specific_metrics_widget = dynamic_cast<QWidget *>(tab_widget_->widget(1));
  auto * specific_metrics_layout = dynamic_cast<QVBoxLayout *>(specific_metrics_widget->layout());
  specific_metrics_layout->removeWidget(specific_metric_table_);
  specific_metric_table_ = metric.getTable();
  QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
  sizePolicy.setHeightForWidth(specific_metric_table_->sizePolicy().hasHeightForWidth());
  specific_metric_table_->setSizePolicy(sizePolicy);
  specific_metrics_layout->insertWidget(1, specific_metric_table_);
}

void MetricsVisualizePanel::onSpecificMetricChanged()
{
  const auto selected_metrics_str = specific_metric_selector_->currentText().toStdString();
  updateSelectedMetric(selected_metrics_str);
  updateViews();
}

void MetricsVisualizePanel::onClearButtonClicked()
{
  if (!selected_metrics_) {
    return;
  }
  updateSelectedMetric(selected_metrics_->first);
  updateViews();
}

void MetricsVisualizePanel::onTimer()
{
  std::lock_guard<std::mutex> message_lock(mutex_);

  for (auto & [name, metric] : metrics_) {
    metric.updateGraph();
    metric.updateTable();
  }

  if (selected_metrics_) {
    selected_metrics_->second.updateGraph();
    selected_metrics_->second.updateTable();
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
                         2;  // start from 2 to leave space for the topic selector and tab widget
      const size_t col = num_current_metrics % GRAPH_COL_SIZE;

      // Get the widgets from the metric
      const auto tableWidget = metric.getTable();
      const auto chartViewWidget = metric.getChartView();

      // Get the layout for the "All Metrics" tab
      auto all_metrics_widget = dynamic_cast<QWidget *>(tab_widget_->widget(0));
      QGridLayout * all_metrics_layout = dynamic_cast<QGridLayout *>(all_metrics_widget->layout());

      // Add the widgets to the "All Metrics" tab layout
      all_metrics_layout->addWidget(tableWidget, row, col);
      all_metrics_layout->setRowStretch(row, false);
      all_metrics_layout->addWidget(chartViewWidget, row + 1, col);
      all_metrics_layout->setRowStretch(row + 1, true);
      all_metrics_layout->setColumnStretch(col, true);

      // Also add the widgets to the topic_widgets_map_ for easy management
      topic_widgets_map_[topic_name][status.name] = std::make_pair(tableWidget, chartViewWidget);
    }
    metrics_.at(status.name).updateData(time, status);

    // update selected metrics
    const auto selected_metrics_str = specific_metric_selector_->currentText().toStdString();
    if (selected_metrics_str == status.name) {
      if (selected_metrics_) {
        selected_metrics_->second.updateData(time, status);
      }
    }
  }

  // Update the specific metric selector
  QSignalBlocker blocker(specific_metric_selector_);
  for (const auto & status : msg->status) {
    if (specific_metric_selector_->findText(QString::fromStdString(status.name)) == -1) {
      specific_metric_selector_->addItem(QString::fromStdString(status.name));
    }
  }

  // save the message for metrics selector
  current_msg_map_[topic_name] = msg;
}

}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::MetricsVisualizePanel, rviz_common::Panel)

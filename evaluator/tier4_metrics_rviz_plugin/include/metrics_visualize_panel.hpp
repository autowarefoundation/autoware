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

#ifndef METRICS_VISUALIZE_PANEL_HPP_
#define METRICS_VISUALIZE_PANEL_HPP_

#ifndef Q_MOC_RUN
#include <QChartView>
#include <QColor>
#include <QComboBox>
#include <QGridLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineSeries>
#include <QPainter>
#include <QPushButton>
#include <QTableWidget>
#include <QVBoxLayout>
#endif

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include <iostream>
#include <limits>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace rviz_plugins
{

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;
using QtCharts::QChart;
using QtCharts::QChartView;
using QtCharts::QLineSeries;

struct Metric
{
public:
  explicit Metric(const DiagnosticStatus & status) : chart(new QChartView), table(new QTableWidget)
  {
    init(status);
  }

  void init(const DiagnosticStatus & status)
  {
    QStringList header{};

    {
      auto label = new QLabel;
      label->setAlignment(Qt::AlignCenter);
      label->setText(QString::fromStdString(status.name));
      labels.emplace("metric_name", label);

      header.push_back("metric_name");
    }

    for (const auto & [key, value] : status.values) {
      auto label = new QLabel;
      label->setAlignment(Qt::AlignCenter);
      labels.emplace(key, label);

      auto plot = new QLineSeries;
      plot->setName(QString::fromStdString(key));
      plots.emplace(key, plot);
      chart->chart()->addSeries(plot);
      chart->chart()->createDefaultAxes();

      header.push_back(QString::fromStdString(key));
    }

    {
      chart->chart()->setTitle(QString::fromStdString(status.name));
      chart->chart()->legend()->setVisible(true);
      chart->chart()->legend()->detachFromChart();
      chart->setRenderHint(QPainter::Antialiasing);
    }

    {
      table->setColumnCount(status.values.size() + 1);
      table->setHorizontalHeaderLabels(header);
      table->verticalHeader()->hide();
      table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
      table->setRowCount(1);
      table->setSizeAdjustPolicy(QAbstractScrollArea::AdjustToContents);
    }
  }

  void updateData(const double time, const DiagnosticStatus & status)
  {
    for (const auto & [key, value] : status.values) {
      const double data = std::stod(value);
      labels.at(key)->setText(QString::fromStdString(toString(data)));
      plots.at(key)->append(time, data);
      updateMinMax(data);
    }

    {
      const auto area = chart->chart()->plotArea();
      const auto rect = chart->chart()->legend()->rect();
      chart->chart()->legend()->setGeometry(
        QRectF(area.x(), area.y(), area.width(), rect.height()));
      chart->chart()->axes(Qt::Horizontal).front()->setRange(time - 100.0, time);
    }

    {
      table->setCellWidget(0, 0, labels.at("metric_name"));
    }

    for (size_t i = 0; i < status.values.size(); ++i) {
      table->setCellWidget(0, i + 1, labels.at(status.values.at(i).key));
    }
  }

  void updateMinMax(double data)
  {
    if (data < y_range_min) {
      y_range_min = data > 0.0 ? 0.9 * data : 1.1 * data;
      chart->chart()->axes(Qt::Vertical).front()->setMin(y_range_min);
    }

    if (data > y_range_max) {
      y_range_max = data > 0.0 ? 1.1 * data : 0.9 * data;
      chart->chart()->axes(Qt::Vertical).front()->setMax(y_range_max);
    }
  }

  void updateTable() { table->update(); }

  void updateGraph() { chart->update(); }

  QChartView * getChartView() const { return chart; }

  QTableWidget * getTable() const { return table; }

  std::unordered_map<std::string, QLabel *> getLabels() const { return labels; }

private:
  static std::optional<std::string> getValue(const DiagnosticStatus & status, std::string && key)
  {
    const auto itr = std::find_if(
      status.values.begin(), status.values.end(),
      [&](const auto & value) { return value.key == key; });

    if (itr == status.values.end()) {
      return std::nullopt;
    }

    return itr->value;
  }

  static std::string toString(const double & value)
  {
    std::stringstream ss;
    ss << std::scientific << std::setprecision(2) << value;
    return ss.str();
  }

  QChartView * chart;
  QTableWidget * table;

  std::unordered_map<std::string, QLabel *> labels;
  std::unordered_map<std::string, QLineSeries *> plots;

  double y_range_min{std::numeric_limits<double>::max()};
  double y_range_max{std::numeric_limits<double>::lowest()};
};

class MetricsVisualizePanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit MetricsVisualizePanel(QWidget * parent = nullptr);
  void onInitialize() override;

private Q_SLOTS:
  // Slot functions triggered by UI events
  void onTopicChanged();
  void onSpecificMetricChanged();
  void onClearButtonClicked();
  void onTabChanged();

private:
  // ROS 2 node and subscriptions for handling metrics data
  rclcpp::Node::SharedPtr raw_node_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unordered_map<std::string, rclcpp::Subscription<DiagnosticArray>::SharedPtr> subscriptions_;

  // Topics from which metrics are collected
  std::vector<std::string> topics_ = {
    "/diagnostic/planning_evaluator/metrics", "/diagnostic/perception_online_evaluator/metrics"};

  // Timer and metrics message callback
  void onTimer();
  void onMetrics(const DiagnosticArray::ConstSharedPtr & msg, const std::string & topic_name);

  // Functions to update UI based on selected metrics
  void updateViews();
  void updateSelectedMetric(const std::string & metric_name);

  // UI components
  QGridLayout * grid_;
  QComboBox * topic_selector_;
  QTabWidget * tab_widget_;

  // "Specific Metrics" tab components
  QComboBox * specific_metric_selector_;
  QChartView * specific_metric_chart_view_;
  QTableWidget * specific_metric_table_;

  // Selected metrics data
  std::optional<std::pair<std::string, Metric>> selected_metrics_;

  // Cache for received messages by topics
  std::unordered_map<std::string, DiagnosticArray::ConstSharedPtr> current_msg_map_;

  // Mapping from topics to metrics widgets (tables and charts)
  std::unordered_map<
    std::string, std::unordered_map<std::string, std::pair<QTableWidget *, QChartView *>>>
    topic_widgets_map_;

  // Synchronization
  std::mutex mutex_;

  // Stored metrics data
  std::unordered_map<std::string, Metric> metrics_;

  // Utility functions for managing widget visibility based on topics
  void updateWidgetVisibility(const std::string & target_topic, const bool show);
  void showCurrentTopicWidgets();
  void hideInactiveTopicWidgets();
};
}  // namespace rviz_plugins

#endif  // METRICS_VISUALIZE_PANEL_HPP_

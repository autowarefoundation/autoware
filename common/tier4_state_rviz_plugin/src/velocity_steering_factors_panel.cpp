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

#include "velocity_steering_factors_panel.hpp"

#include <QGridLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QString>
#include <QVBoxLayout>
#include <rviz_common/display_context.hpp>

#include <memory>
#include <string>

namespace rviz_plugins
{
VelocitySteeringFactorsPanel::VelocitySteeringFactorsPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  // Layout
  auto * v_layout = new QVBoxLayout;
  v_layout->addWidget(makeVelocityFactorsGroup());
  v_layout->addWidget(makeSteeringFactorsGroup());
  setLayout(v_layout);
}

QGroupBox * VelocitySteeringFactorsPanel::makeVelocityFactorsGroup()
{
  auto * group = new QGroupBox("VelocityFactors");
  auto * grid = new QGridLayout;

  auto vertical_header = new QHeaderView(Qt::Vertical);
  vertical_header->hide();
  auto horizontal_header = new QHeaderView(Qt::Horizontal);
  horizontal_header->setSectionResizeMode(QHeaderView::Stretch);

  auto header_labels = QStringList({"Type", "Status", "Distance [m]", "Detail"});
  velocity_factors_table_ = new QTableWidget();
  velocity_factors_table_->setColumnCount(header_labels.size());
  velocity_factors_table_->setHorizontalHeaderLabels(header_labels);
  velocity_factors_table_->setVerticalHeader(vertical_header);
  velocity_factors_table_->setHorizontalHeader(horizontal_header);
  grid->addWidget(velocity_factors_table_, 0, 0);

  group->setLayout(grid);
  return group;
}

QGroupBox * VelocitySteeringFactorsPanel::makeSteeringFactorsGroup()
{
  auto * group = new QGroupBox("SteeringFactors");
  auto * grid = new QGridLayout;

  auto vertical_header = new QHeaderView(Qt::Vertical);
  vertical_header->hide();
  auto horizontal_header = new QHeaderView(Qt::Horizontal);
  horizontal_header->setSectionResizeMode(QHeaderView::Stretch);

  auto header_labels =
    QStringList({"Type", "Status", "Distance.1 [m]", "Distance.2 [m]", "Direction", "Detail"});
  steering_factors_table_ = new QTableWidget();
  steering_factors_table_->setColumnCount(header_labels.size());
  steering_factors_table_->setHorizontalHeaderLabels(header_labels);
  steering_factors_table_->setVerticalHeader(vertical_header);
  steering_factors_table_->setHorizontalHeader(horizontal_header);
  grid->addWidget(steering_factors_table_, 1, 0);

  group->setLayout(grid);
  return group;
}

void VelocitySteeringFactorsPanel::onInitialize()
{
  using std::placeholders::_1;

  raw_node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  // Planning
  sub_velocity_factors_ = raw_node_->create_subscription<VelocityFactorArray>(
    "/api/planning/velocity_factors", 10,
    std::bind(&VelocitySteeringFactorsPanel::onVelocityFactors, this, _1));

  sub_steering_factors_ = raw_node_->create_subscription<SteeringFactorArray>(
    "/api/planning/steering_factors", 10,
    std::bind(&VelocitySteeringFactorsPanel::onSteeringFactors, this, _1));
}

void VelocitySteeringFactorsPanel::onVelocityFactors(const VelocityFactorArray::ConstSharedPtr msg)
{
  velocity_factors_table_->clearContents();
  velocity_factors_table_->setRowCount(msg->factors.size());

  for (std::size_t i = 0; i < msg->factors.size(); i++) {
    const auto & e = msg->factors.at(i);

    // behavior
    {
      auto label = new QLabel(e.behavior.empty() ? "UNKNOWN" : e.behavior.c_str());
      label->setAlignment(Qt::AlignCenter);
      velocity_factors_table_->setCellWidget(i, 0, label);
    }

    // status
    {
      auto label = new QLabel();
      switch (e.status) {
        case VelocityFactor::APPROACHING:
          label->setText("APPROACHING");
          break;
        case VelocityFactor::STOPPED:
          label->setText("STOPPED");
          break;
        default:
          label->setText("UNKNOWN");
          break;
      }
      label->setAlignment(Qt::AlignCenter);
      velocity_factors_table_->setCellWidget(i, 1, label);
    }

    // distance
    {
      auto label = new QLabel();
      std::stringstream ss;
      ss << std::fixed << std::setprecision(2) << e.distance;
      label->setText(QString::fromStdString(ss.str()));
      label->setAlignment(Qt::AlignCenter);
      velocity_factors_table_->setCellWidget(i, 2, label);
    }

    // detail
    {
      auto label = new QLabel(QString::fromStdString(e.detail));
      label->setAlignment(Qt::AlignCenter);
      velocity_factors_table_->setCellWidget(i, 3, label);
    }
  }
  velocity_factors_table_->update();
}

void VelocitySteeringFactorsPanel::onSteeringFactors(const SteeringFactorArray::ConstSharedPtr msg)
{
  steering_factors_table_->clearContents();
  steering_factors_table_->setRowCount(msg->factors.size());

  for (std::size_t i = 0; i < msg->factors.size(); i++) {
    const auto & e = msg->factors.at(i);

    // behavior
    {
      auto label = new QLabel(e.behavior.empty() ? "UNKNOWN" : e.behavior.c_str());
      label->setAlignment(Qt::AlignCenter);
      steering_factors_table_->setCellWidget(i, 0, label);
    }

    // status
    {
      auto label = new QLabel();
      switch (e.status) {
        case SteeringFactor::APPROACHING:
          label->setText("APPROACHING");
          break;
        case SteeringFactor::TURNING:
          label->setText("TURNING");
          break;
        default:
          label->setText("UNKNOWN");
          break;
      }
      label->setAlignment(Qt::AlignCenter);
      steering_factors_table_->setCellWidget(i, 1, label);
    }

    // distance.1
    {
      auto label = new QLabel();
      std::stringstream ss;
      ss << std::fixed << std::setprecision(2) << e.distance.front();
      label->setText(QString::fromStdString(ss.str()));
      label->setAlignment(Qt::AlignCenter);
      steering_factors_table_->setCellWidget(i, 2, label);
    }

    // distance.2
    {
      auto label = new QLabel();
      std::stringstream ss;
      ss << std::fixed << std::setprecision(2) << e.distance.back();
      label->setText(QString::fromStdString(ss.str()));
      label->setAlignment(Qt::AlignCenter);
      steering_factors_table_->setCellWidget(i, 3, label);
    }

    // Direction
    {
      auto label = new QLabel();
      switch (e.direction) {
        case SteeringFactor::LEFT:
          label->setText("LEFT");
          break;
        case SteeringFactor::RIGHT:
          label->setText("RIGHT");
          break;
        case SteeringFactor::STRAIGHT:
          label->setText("STRAIGHT");
          break;
        default:
          label->setText("UNKNOWN");
          break;
      }
      label->setAlignment(Qt::AlignCenter);
      steering_factors_table_->setCellWidget(i, 4, label);
    }

    // detail
    {
      auto label = new QLabel(QString::fromStdString(e.detail));
      label->setAlignment(Qt::AlignCenter);
      steering_factors_table_->setCellWidget(i, 5, label);
    }
  }
  steering_factors_table_->update();
}
}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::VelocitySteeringFactorsPanel, rviz_common::Panel)

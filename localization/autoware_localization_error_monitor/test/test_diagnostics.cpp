// Copyright 2023 Autoware Foundation
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

#include "diagnostics_helper.hpp"

#include <gtest/gtest.h>

TEST(TestLocalizationErrorMonitorDiagnostics, CheckLocalizationAccuracy)
{
  diagnostic_msgs::msg::DiagnosticStatus stat;

  const double warn_ellipse_size = 0.8;
  const double error_ellipse_size = 1.0;

  double ellipse_size = 0.0;
  stat = autoware::localization_error_monitor::check_localization_accuracy(
    ellipse_size, warn_ellipse_size, error_ellipse_size);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::OK);

  ellipse_size = 0.7;
  stat = autoware::localization_error_monitor::check_localization_accuracy(
    ellipse_size, warn_ellipse_size, error_ellipse_size);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::OK);

  ellipse_size = 0.8;
  stat = autoware::localization_error_monitor::check_localization_accuracy(
    ellipse_size, warn_ellipse_size, error_ellipse_size);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);

  ellipse_size = 0.9;
  stat = autoware::localization_error_monitor::check_localization_accuracy(
    ellipse_size, warn_ellipse_size, error_ellipse_size);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);

  ellipse_size = 1.0;
  stat = autoware::localization_error_monitor::check_localization_accuracy(
    ellipse_size, warn_ellipse_size, error_ellipse_size);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
}

TEST(TestLocalizationErrorMonitorDiagnostics, CheckLocalizationAccuracyLateralDirection)
{
  diagnostic_msgs::msg::DiagnosticStatus stat;

  const double warn_ellipse_size = 0.25;
  const double error_ellipse_size = 0.3;

  double ellipse_size = 0.0;
  stat = autoware::localization_error_monitor::check_localization_accuracy_lateral_direction(
    ellipse_size, warn_ellipse_size, error_ellipse_size);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::OK);

  ellipse_size = 0.24;
  stat = autoware::localization_error_monitor::check_localization_accuracy_lateral_direction(
    ellipse_size, warn_ellipse_size, error_ellipse_size);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::OK);

  ellipse_size = 0.25;
  stat = autoware::localization_error_monitor::check_localization_accuracy_lateral_direction(
    ellipse_size, warn_ellipse_size, error_ellipse_size);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);

  ellipse_size = 0.29;
  stat = autoware::localization_error_monitor::check_localization_accuracy_lateral_direction(
    ellipse_size, warn_ellipse_size, error_ellipse_size);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);

  ellipse_size = 0.3;
  stat = autoware::localization_error_monitor::check_localization_accuracy_lateral_direction(
    ellipse_size, warn_ellipse_size, error_ellipse_size);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
}

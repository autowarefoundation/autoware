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

#include "localization_error_monitor/diagnostics.hpp"

#include <gtest/gtest.h>

TEST(TestLocalizationErrorMonitorDiagnostics, CheckLocalizationAccuracy)
{
  diagnostic_msgs::msg::DiagnosticStatus stat;

  const double warn_ellipse_size = 0.8;
  const double error_ellipse_size = 1.0;

  double ellipse_size = 0.0;
  stat = checkLocalizationAccuracy(ellipse_size, warn_ellipse_size, error_ellipse_size);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::OK);

  ellipse_size = 0.7;
  stat = checkLocalizationAccuracy(ellipse_size, warn_ellipse_size, error_ellipse_size);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::OK);

  ellipse_size = 0.8;
  stat = checkLocalizationAccuracy(ellipse_size, warn_ellipse_size, error_ellipse_size);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);

  ellipse_size = 0.9;
  stat = checkLocalizationAccuracy(ellipse_size, warn_ellipse_size, error_ellipse_size);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);

  ellipse_size = 1.0;
  stat = checkLocalizationAccuracy(ellipse_size, warn_ellipse_size, error_ellipse_size);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
}

TEST(TestLocalizationErrorMonitorDiagnostics, CheckLocalizationAccuracyLateralDirection)
{
  diagnostic_msgs::msg::DiagnosticStatus stat;

  const double warn_ellipse_size = 0.25;
  const double error_ellipse_size = 0.3;

  double ellipse_size = 0.0;
  stat =
    checkLocalizationAccuracyLateralDirection(ellipse_size, warn_ellipse_size, error_ellipse_size);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::OK);

  ellipse_size = 0.24;
  stat =
    checkLocalizationAccuracyLateralDirection(ellipse_size, warn_ellipse_size, error_ellipse_size);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::OK);

  ellipse_size = 0.25;
  stat =
    checkLocalizationAccuracyLateralDirection(ellipse_size, warn_ellipse_size, error_ellipse_size);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);

  ellipse_size = 0.29;
  stat =
    checkLocalizationAccuracyLateralDirection(ellipse_size, warn_ellipse_size, error_ellipse_size);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);

  ellipse_size = 0.3;
  stat =
    checkLocalizationAccuracyLateralDirection(ellipse_size, warn_ellipse_size, error_ellipse_size);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
}

TEST(TestLocalizationErrorMonitorDiagnostics, MergeDiagnosticStatus)
{
  diagnostic_msgs::msg::DiagnosticStatus merged_stat;
  std::vector<diagnostic_msgs::msg::DiagnosticStatus> stat_array(2);

  stat_array.at(0).level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  stat_array.at(0).message = "OK";
  stat_array.at(1).level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  stat_array.at(1).message = "OK";
  merged_stat = mergeDiagnosticStatus(stat_array);
  EXPECT_EQ(merged_stat.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_EQ(merged_stat.message, "OK");

  stat_array.at(0).level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
  stat_array.at(0).message = "WARN0";
  stat_array.at(1).level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  stat_array.at(1).message = "OK";
  merged_stat = mergeDiagnosticStatus(stat_array);
  EXPECT_EQ(merged_stat.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
  EXPECT_EQ(merged_stat.message, "WARN0");

  stat_array.at(0).level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  stat_array.at(0).message = "OK";
  stat_array.at(1).level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
  stat_array.at(1).message = "WARN1";
  merged_stat = mergeDiagnosticStatus(stat_array);
  EXPECT_EQ(merged_stat.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
  EXPECT_EQ(merged_stat.message, "WARN1");

  stat_array.at(0).level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
  stat_array.at(0).message = "WARN0";
  stat_array.at(1).level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
  stat_array.at(1).message = "WARN1";
  merged_stat = mergeDiagnosticStatus(stat_array);
  EXPECT_EQ(merged_stat.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
  EXPECT_EQ(merged_stat.message, "WARN0; WARN1");

  stat_array.at(0).level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  stat_array.at(0).message = "OK";
  stat_array.at(1).level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  stat_array.at(1).message = "ERROR1";
  merged_stat = mergeDiagnosticStatus(stat_array);
  EXPECT_EQ(merged_stat.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_EQ(merged_stat.message, "ERROR1");

  stat_array.at(0).level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
  stat_array.at(0).message = "WARN0";
  stat_array.at(1).level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  stat_array.at(1).message = "ERROR1";
  merged_stat = mergeDiagnosticStatus(stat_array);
  EXPECT_EQ(merged_stat.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_EQ(merged_stat.message, "WARN0; ERROR1");

  stat_array.at(0).level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  stat_array.at(0).message = "ERROR0";
  stat_array.at(1).level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  stat_array.at(1).message = "ERROR1";
  merged_stat = mergeDiagnosticStatus(stat_array);
  EXPECT_EQ(merged_stat.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_EQ(merged_stat.message, "ERROR0; ERROR1");
}

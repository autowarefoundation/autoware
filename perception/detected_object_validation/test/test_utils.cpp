// Copyright 2024 TIER IV, Inc.
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

#include "autoware/detected_object_validation/utils/utils.hpp"

#include "autoware_perception_msgs/msg/object_classification.hpp"

#include <gtest/gtest.h>

using autoware::detected_object_validation::utils::FilterTargetLabel;
using AutowareLabel = autoware_perception_msgs::msg::ObjectClassification;

FilterTargetLabel createFilterTargetAll()
{
  FilterTargetLabel filter;
  filter.UNKNOWN = true;
  filter.CAR = true;
  filter.TRUCK = true;
  filter.BUS = true;
  filter.TRAILER = true;
  filter.MOTORCYCLE = true;
  filter.BICYCLE = true;
  filter.PEDESTRIAN = true;
  return filter;
}

FilterTargetLabel createFilterTargetVehicle()
{
  FilterTargetLabel filter;
  filter.UNKNOWN = false;
  filter.CAR = true;
  filter.TRUCK = true;
  filter.BUS = true;
  filter.TRAILER = true;
  filter.MOTORCYCLE = false;
  filter.BICYCLE = false;
  filter.PEDESTRIAN = false;
  return filter;
}

TEST(IsTargetTest, AllTarget)
{
  auto filter = createFilterTargetAll();
  auto label = AutowareLabel::CAR;
  EXPECT_TRUE(filter.isTarget(label));
}

TEST(IsTargetTest, VehicleTarget)
{
  auto filter = createFilterTargetVehicle();

  auto car_label = AutowareLabel::CAR;
  EXPECT_TRUE(filter.isTarget(car_label));

  auto unknown_label = AutowareLabel::UNKNOWN;
  EXPECT_FALSE(filter.isTarget(unknown_label));
}

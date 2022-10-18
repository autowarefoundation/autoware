// Copyright 2022 TIER IV, Inc.
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

#include "perception_utils/object_classification.hpp"

#include <gtest/gtest.h>

constexpr double epsilon = 1e-06;

namespace
{
autoware_auto_perception_msgs::msg::ObjectClassification createObjectClassification(
  const std::uint8_t label, const double probability)
{
  autoware_auto_perception_msgs::msg::ObjectClassification classification;
  classification.label = label;
  classification.probability = probability;

  return classification;
}

}  // namespace

TEST(object_classification, test_getHighestProbLabel)
{
  using autoware_auto_perception_msgs::msg::ObjectClassification;
  using perception_utils::getHighestProbLabel;

  {  // empty
    std::vector<autoware_auto_perception_msgs::msg::ObjectClassification> classifications;
    std::uint8_t label = getHighestProbLabel(classifications);
    EXPECT_EQ(label, ObjectClassification::UNKNOWN);
  }

  {  // normal case
    std::vector<autoware_auto_perception_msgs::msg::ObjectClassification> classifications;
    classifications.push_back(createObjectClassification(ObjectClassification::CAR, 0.5));
    classifications.push_back(createObjectClassification(ObjectClassification::TRUCK, 0.8));
    classifications.push_back(createObjectClassification(ObjectClassification::BUS, 0.7));

    std::uint8_t label = getHighestProbLabel(classifications);
    EXPECT_EQ(label, ObjectClassification::TRUCK);
  }

  {  // labels with the same probability
    std::vector<autoware_auto_perception_msgs::msg::ObjectClassification> classifications;
    classifications.push_back(createObjectClassification(ObjectClassification::CAR, 0.8));
    classifications.push_back(createObjectClassification(ObjectClassification::TRUCK, 0.8));
    classifications.push_back(createObjectClassification(ObjectClassification::BUS, 0.7));

    std::uint8_t label = getHighestProbLabel(classifications);
    EXPECT_EQ(label, ObjectClassification::CAR);
  }
}

TEST(object_classification, test_getHighestProbClassification)
{
  using autoware_auto_perception_msgs::msg::ObjectClassification;
  using perception_utils::getHighestProbClassification;

  {  // empty
    std::vector<autoware_auto_perception_msgs::msg::ObjectClassification> classifications;
    auto classification = getHighestProbClassification(classifications);
    EXPECT_EQ(classification.label, ObjectClassification::UNKNOWN);
    EXPECT_DOUBLE_EQ(classification.probability, 0.0);
  }

  {  // normal case
    std::vector<autoware_auto_perception_msgs::msg::ObjectClassification> classifications;
    classifications.push_back(createObjectClassification(ObjectClassification::CAR, 0.5));
    classifications.push_back(createObjectClassification(ObjectClassification::TRUCK, 0.8));
    classifications.push_back(createObjectClassification(ObjectClassification::BUS, 0.7));

    auto classification = getHighestProbClassification(classifications);
    EXPECT_EQ(classification.label, ObjectClassification::TRUCK);
    EXPECT_NEAR(classification.probability, 0.8, epsilon);
  }

  {  // labels with the same probability
    std::vector<autoware_auto_perception_msgs::msg::ObjectClassification> classifications;
    classifications.push_back(createObjectClassification(ObjectClassification::CAR, 0.8));
    classifications.push_back(createObjectClassification(ObjectClassification::TRUCK, 0.8));
    classifications.push_back(createObjectClassification(ObjectClassification::BUS, 0.7));

    auto classification = getHighestProbClassification(classifications);
    EXPECT_EQ(classification.label, ObjectClassification::CAR);
    EXPECT_NEAR(classification.probability, 0.8, epsilon);
  }
}

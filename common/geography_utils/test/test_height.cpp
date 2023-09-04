// Copyright 2023 TIER IV, Inc.
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

#include <geography_utils/height.hpp>

#include <gtest/gtest.h>

#include <stdexcept>
#include <string>

// Test case to verify if same source and target datums return original height
TEST(GeographyUtils, SameSourceTargetDatum)
{
  const double height = 10.0;
  const double latitude = 35.0;
  const double longitude = 139.0;
  const std::string datum = "WGS84";

  double converted_height =
    geography_utils::convert_height(height, latitude, longitude, datum, datum);

  EXPECT_DOUBLE_EQ(height, converted_height);
}

// Test case to verify valid source and target datums
TEST(GeographyUtils, ValidSourceTargetDatum)
{
  // Calculated with
  // https://www.unavco.org/software/geodetic-utilities/geoid-height-calculator/geoid-height-calculator.html
  const double height = 10.0;
  const double latitude = 35.0;
  const double longitude = 139.0;
  const double target_height = -30.18;

  double converted_height =
    geography_utils::convert_height(height, latitude, longitude, "WGS84", "EGM2008");

  EXPECT_NEAR(target_height, converted_height, 0.1);
}

// Test case to verify invalid source and target datums
TEST(GeographyUtils, InvalidSourceTargetDatum)
{
  const double height = 10.0;
  const double latitude = 35.0;
  const double longitude = 139.0;

  EXPECT_THROW(
    geography_utils::convert_height(height, latitude, longitude, "INVALID1", "INVALID2"),
    std::invalid_argument);
}

// Test case to verify invalid source datums
TEST(GeographyUtils, InvalidSourceDatum)
{
  const double height = 10.0;
  const double latitude = 35.0;
  const double longitude = 139.0;

  EXPECT_THROW(
    geography_utils::convert_height(height, latitude, longitude, "INVALID1", "WGS84"),
    std::invalid_argument);
}

// Test case to verify invalid target datums
TEST(GeographyUtils, InvalidTargetDatum)
{
  const double height = 10.0;
  const double latitude = 35.0;
  const double longitude = 139.0;

  EXPECT_THROW(
    geography_utils::convert_height(height, latitude, longitude, "WGS84", "INVALID2"),
    std::invalid_argument);
}

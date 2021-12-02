// Copyright 2015-2019 Autoware Foundation
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

#include "lanelet2_extension/projection/mgrs_projector.hpp"

#include <gtest/gtest.h>
#include <math.h>

class TestSuite : public ::testing::Test
{
public:
  TestSuite() {}
  ~TestSuite() {}
};

TEST(TestSuite, ForwardProjection)
{
  lanelet::projection::MGRSProjector projector;
  // lat/lon in Tokyo
  lanelet::GPSPoint gps_point;
  gps_point.lat = 35.652832;
  gps_point.lon = 139.839478;
  gps_point.ele = 12.3456789;
  lanelet::BasicPoint3d mgrs_point = projector.forward(gps_point);

  // projected z value should not change
  ASSERT_DOUBLE_EQ(mgrs_point.z(), gps_point.ele)
    << "Forward projected z value should be " << gps_point.ele;

  // https://www.movable-type.co.uk/scripts/latlong-utm-mgrs.html
  // round the projected value to mm since the above reference only gives value
  // in mm precision
  ASSERT_EQ(projector.getProjectedMGRSGrid(), "54SUE") << "Projected grid should be "
                                                       << "54SUE";
  double rounded_x_mm = round(mgrs_point.x() * 1000) / 1000.0;
  ASSERT_DOUBLE_EQ(rounded_x_mm, 94946.081) << "Forward projected x value should be " << 94946.081;
  double rounded_y_mm = round(mgrs_point.y() * 1000) / 1000.0;
  ASSERT_DOUBLE_EQ(rounded_y_mm, 46063.748) << "Forward projected y value should be " << 46063.748;
}

TEST(TestSuite, ReverseProjection)
{
  lanelet::projection::MGRSProjector projector;
  lanelet::BasicPoint3d mgrs_point;
  mgrs_point.x() = 94946.0;
  mgrs_point.y() = 46063.0;
  mgrs_point.z() = 12.3456789;

  projector.setMGRSCode("54SUE");
  lanelet::GPSPoint gps_point = projector.reverse(mgrs_point);

  // projected z value should not change
  ASSERT_DOUBLE_EQ(gps_point.ele, mgrs_point.z())
    << "Reverse projected z value should be " << mgrs_point.z();

  // https://www.movable-type.co.uk/scripts/latlong-utm-mgrs.html
  // round the projected value since the above reference only gives value up to
  // precision of 1e-8
  double rounded_lat = round(gps_point.lat * 1e8) / 1e8;
  ASSERT_DOUBLE_EQ(rounded_lat, 35.65282525)
    << "Reverse projected latitude value should be " << 35.65282525;
  double rounded_lon = round(gps_point.lon * 1e8) / 1e8;
  ASSERT_DOUBLE_EQ(rounded_lon, 139.83947721)
    << "Reverse projected longitude value should be " << 139.83947721;
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

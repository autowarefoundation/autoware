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

#include <geography_utils/projection.hpp>

#include <gtest/gtest.h>

#include <stdexcept>
#include <string>

TEST(GeographyUtilsProjection, ProjectForwardToMGRS)
{
  // source point
  geographic_msgs::msg::GeoPoint geo_point;
  geo_point.latitude = 35.62426;
  geo_point.longitude = 139.74252;
  geo_point.altitude = 10.0;

  // target point
  geometry_msgs::msg::Point local_point;
  local_point.x = 86128.0;
  local_point.y = 43002.0;
  local_point.z = 10.0;

  // projector info
  tier4_map_msgs::msg::MapProjectorInfo projector_info;
  projector_info.projector_type = tier4_map_msgs::msg::MapProjectorInfo::MGRS;
  projector_info.mgrs_grid = "54SUE";
  projector_info.vertical_datum = tier4_map_msgs::msg::MapProjectorInfo::WGS84;

  // conversion
  const geometry_msgs::msg::Point converted_point =
    geography_utils::project_forward(geo_point, projector_info);

  EXPECT_NEAR(converted_point.x, local_point.x, 1.0);
  EXPECT_NEAR(converted_point.y, local_point.y, 1.0);
  EXPECT_NEAR(converted_point.z, local_point.z, 1.0);
}

TEST(GeographyUtilsProjection, ProjectReverseFromMGRS)
{
  // source point
  geometry_msgs::msg::Point local_point;
  local_point.x = 86128.0;
  local_point.y = 43002.0;
  local_point.z = 10.0;

  // target point
  geographic_msgs::msg::GeoPoint geo_point;
  geo_point.latitude = 35.62426;
  geo_point.longitude = 139.74252;
  geo_point.altitude = 10.0;

  // projector info
  tier4_map_msgs::msg::MapProjectorInfo projector_info;
  projector_info.projector_type = tier4_map_msgs::msg::MapProjectorInfo::MGRS;
  projector_info.mgrs_grid = "54SUE";
  projector_info.vertical_datum = tier4_map_msgs::msg::MapProjectorInfo::WGS84;

  // conversion
  const geographic_msgs::msg::GeoPoint converted_point =
    geography_utils::project_reverse(local_point, projector_info);

  EXPECT_NEAR(converted_point.latitude, geo_point.latitude, 0.0001);
  EXPECT_NEAR(converted_point.longitude, geo_point.longitude, 0.0001);
  EXPECT_NEAR(converted_point.altitude, geo_point.altitude, 0.0001);
}

TEST(GeographyUtilsProjection, ProjectForwardAndReverseMGRS)
{
  // source point
  geographic_msgs::msg::GeoPoint geo_point;
  geo_point.latitude = 35.62426;
  geo_point.longitude = 139.74252;
  geo_point.altitude = 10.0;

  // projector info
  tier4_map_msgs::msg::MapProjectorInfo projector_info;
  projector_info.projector_type = tier4_map_msgs::msg::MapProjectorInfo::MGRS;
  projector_info.mgrs_grid = "54SUE";
  projector_info.vertical_datum = tier4_map_msgs::msg::MapProjectorInfo::WGS84;

  // conversion
  const geometry_msgs::msg::Point converted_local_point =
    geography_utils::project_forward(geo_point, projector_info);
  const geographic_msgs::msg::GeoPoint converted_geo_point =
    geography_utils::project_reverse(converted_local_point, projector_info);

  EXPECT_NEAR(converted_geo_point.latitude, geo_point.latitude, 0.0001);
  EXPECT_NEAR(converted_geo_point.longitude, geo_point.longitude, 0.0001);
  EXPECT_NEAR(converted_geo_point.altitude, geo_point.altitude, 0.0001);
}

TEST(GeographyUtilsProjection, ProjectForwardToLocalCartesianUTMOrigin)
{
  // source point
  geographic_msgs::msg::GeoPoint geo_point;
  geo_point.latitude = 35.62406;
  geo_point.longitude = 139.74252;
  geo_point.altitude = 10.0;

  // target point
  geometry_msgs::msg::Point local_point;
  local_point.x = 0.0;
  local_point.y = -22.18;
  local_point.z = 20.0;

  // projector info
  tier4_map_msgs::msg::MapProjectorInfo projector_info;
  projector_info.projector_type = tier4_map_msgs::msg::MapProjectorInfo::LOCAL_CARTESIAN_UTM;
  projector_info.vertical_datum = tier4_map_msgs::msg::MapProjectorInfo::WGS84;
  projector_info.map_origin.latitude = 35.62426;
  projector_info.map_origin.longitude = 139.74252;
  projector_info.map_origin.altitude = -10.0;

  // conversion
  const geometry_msgs::msg::Point converted_point =
    geography_utils::project_forward(geo_point, projector_info);

  EXPECT_NEAR(converted_point.x, local_point.x, 1.0);
  EXPECT_NEAR(converted_point.y, local_point.y, 1.0);
  EXPECT_NEAR(converted_point.z, local_point.z, 1.0);
}

TEST(GeographyUtilsProjection, ProjectForwardAndReverseLocalCartesianUTMOrigin)
{
  // source point
  geographic_msgs::msg::GeoPoint geo_point;
  geo_point.latitude = 35.62426;
  geo_point.longitude = 139.74252;
  geo_point.altitude = 10.0;

  // projector info
  tier4_map_msgs::msg::MapProjectorInfo projector_info;
  projector_info.projector_type = tier4_map_msgs::msg::MapProjectorInfo::LOCAL_CARTESIAN_UTM;
  projector_info.vertical_datum = tier4_map_msgs::msg::MapProjectorInfo::WGS84;
  projector_info.map_origin.latitude = 35.0;
  projector_info.map_origin.longitude = 139.0;
  projector_info.map_origin.altitude = 0.0;

  // conversion
  const geometry_msgs::msg::Point converted_local_point =
    geography_utils::project_forward(geo_point, projector_info);
  const geographic_msgs::msg::GeoPoint converted_geo_point =
    geography_utils::project_reverse(converted_local_point, projector_info);

  EXPECT_NEAR(converted_geo_point.latitude, geo_point.latitude, 0.0001);
  EXPECT_NEAR(converted_geo_point.longitude, geo_point.longitude, 0.0001);
  EXPECT_NEAR(converted_geo_point.altitude, geo_point.altitude, 0.0001);
}

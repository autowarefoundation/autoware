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

#ifndef YABLOC_COMMON__GROUND_SERVER__POLYGON_OPERATION_HPP_
#define YABLOC_COMMON__GROUND_SERVER__POLYGON_OPERATION_HPP_

#include <lanelet2_core/LaneletMap.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace yabloc::ground_server
{
pcl::PointCloud<pcl::PointXYZ> sample_from_polygons(const lanelet::PolygonLayer & polygons);

pcl::PointCloud<pcl::PointXYZ> fill_points_in_polygon(
  const pcl::PointCloud<pcl::PointXYZ> & src_cloud);
}  // namespace yabloc::ground_server

#endif  // YABLOC_COMMON__GROUND_SERVER__POLYGON_OPERATION_HPP_

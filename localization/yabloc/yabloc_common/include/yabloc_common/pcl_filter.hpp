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

#ifndef YABLOC_COMMON__PCL_FILTER_HPP_
#define YABLOC_COMMON__PCL_FILTER_HPP_

#include <pcl/PointIndices.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>

namespace yabloc::common
{
template <typename T>
pcl::PointCloud<T> extract(const pcl::PointCloud<T> & cloud, pcl::PointIndices::Ptr indices_ptr)
{
  pcl::ExtractIndices<T> extract;
  extract.setIndices(indices_ptr);
  extract.setInputCloud(cloud);

  pcl::PointCloud<T> extracted_cloud;
  extract.filter(extracted_cloud);
  return extracted_cloud;
}
}  // namespace yabloc::common

#endif  // YABLOC_COMMON__PCL_FILTER_HPP_

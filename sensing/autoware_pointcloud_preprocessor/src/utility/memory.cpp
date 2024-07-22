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

#include "autoware/pointcloud_preprocessor/utility/memory.hpp"

#include <autoware_point_types/types.hpp>

namespace autoware::pointcloud_preprocessor::utils
{
bool is_data_layout_compatible_with_point_xyzi(const sensor_msgs::msg::PointCloud2 & input)
{
  using PointIndex = autoware_point_types::PointXYZIIndex;
  using autoware_point_types::PointXYZI;
  if (input.fields.size() < 4) {
    return false;
  }
  bool same_layout = true;
  const auto & field_x = input.fields.at(static_cast<size_t>(PointIndex::X));
  same_layout &= field_x.name == "x";
  same_layout &= field_x.offset == offsetof(PointXYZI, x);
  same_layout &= field_x.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_x.count == 1;
  const auto & field_y = input.fields.at(static_cast<size_t>(PointIndex::Y));
  same_layout &= field_y.name == "y";
  same_layout &= field_y.offset == offsetof(PointXYZI, y);
  same_layout &= field_y.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_y.count == 1;
  const auto & field_z = input.fields.at(static_cast<size_t>(PointIndex::Z));
  same_layout &= field_z.name == "z";
  same_layout &= field_z.offset == offsetof(PointXYZI, z);
  same_layout &= field_z.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_z.count == 1;
  const auto & field_intensity = input.fields.at(static_cast<size_t>(PointIndex::Intensity));
  same_layout &= field_intensity.name == "intensity";
  same_layout &= field_intensity.offset == offsetof(PointXYZI, intensity);
  same_layout &= field_intensity.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_intensity.count == 1;
  return same_layout;
}

bool is_data_layout_compatible_with_point_xyzirc(const sensor_msgs::msg::PointCloud2 & input)
{
  using PointIndex = autoware_point_types::PointXYZIRCIndex;
  using autoware_point_types::PointXYZIRC;
  if (input.fields.size() < 6) {
    return false;
  }
  bool same_layout = true;
  const auto & field_x = input.fields.at(static_cast<size_t>(PointIndex::X));
  same_layout &= field_x.name == "x";
  same_layout &= field_x.offset == offsetof(PointXYZIRC, x);
  same_layout &= field_x.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_x.count == 1;
  const auto & field_y = input.fields.at(static_cast<size_t>(PointIndex::Y));
  same_layout &= field_y.name == "y";
  same_layout &= field_y.offset == offsetof(PointXYZIRC, y);
  same_layout &= field_y.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_y.count == 1;
  const auto & field_z = input.fields.at(static_cast<size_t>(PointIndex::Z));
  same_layout &= field_z.name == "z";
  same_layout &= field_z.offset == offsetof(PointXYZIRC, z);
  same_layout &= field_z.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_z.count == 1;
  const auto & field_intensity = input.fields.at(static_cast<size_t>(PointIndex::Intensity));
  same_layout &= field_intensity.name == "intensity";
  same_layout &= field_intensity.offset == offsetof(PointXYZIRC, intensity);
  same_layout &= field_intensity.datatype == sensor_msgs::msg::PointField::UINT8;
  same_layout &= field_intensity.count == 1;
  const auto & field_return_type = input.fields.at(static_cast<size_t>(PointIndex::ReturnType));
  same_layout &= field_return_type.name == "return_type";
  same_layout &= field_return_type.offset == offsetof(PointXYZIRC, return_type);
  same_layout &= field_return_type.datatype == sensor_msgs::msg::PointField::UINT8;
  same_layout &= field_return_type.count == 1;
  const auto & field_ring = input.fields.at(static_cast<size_t>(PointIndex::Channel));
  same_layout &= field_ring.name == "channel";
  same_layout &= field_ring.offset == offsetof(PointXYZIRC, channel);
  same_layout &= field_ring.datatype == sensor_msgs::msg::PointField::UINT16;
  same_layout &= field_ring.count == 1;

  return same_layout;
}

bool is_data_layout_compatible_with_point_xyziradrt(const sensor_msgs::msg::PointCloud2 & input)
{
  using PointIndex = autoware_point_types::PointXYZIRADRTIndex;
  using autoware_point_types::PointXYZIRADRT;
  if (input.fields.size() < 9) {
    return false;
  }
  bool same_layout = true;
  const auto & field_x = input.fields.at(static_cast<size_t>(PointIndex::X));
  same_layout &= field_x.name == "x";
  same_layout &= field_x.offset == offsetof(PointXYZIRADRT, x);
  same_layout &= field_x.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_x.count == 1;
  const auto & field_y = input.fields.at(static_cast<size_t>(PointIndex::Y));
  same_layout &= field_y.name == "y";
  same_layout &= field_y.offset == offsetof(PointXYZIRADRT, y);
  same_layout &= field_y.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_y.count == 1;
  const auto & field_z = input.fields.at(static_cast<size_t>(PointIndex::Z));
  same_layout &= field_z.name == "z";
  same_layout &= field_z.offset == offsetof(PointXYZIRADRT, z);
  same_layout &= field_z.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_z.count == 1;
  const auto & field_intensity = input.fields.at(static_cast<size_t>(PointIndex::Intensity));
  same_layout &= field_intensity.name == "intensity";
  same_layout &= field_intensity.offset == offsetof(PointXYZIRADRT, intensity);
  same_layout &= field_intensity.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_intensity.count == 1;
  const auto & field_ring = input.fields.at(static_cast<size_t>(PointIndex::Ring));
  same_layout &= field_ring.name == "ring";
  same_layout &= field_ring.offset == offsetof(PointXYZIRADRT, ring);
  same_layout &= field_ring.datatype == sensor_msgs::msg::PointField::UINT16;
  same_layout &= field_ring.count == 1;
  const auto & field_azimuth = input.fields.at(static_cast<size_t>(PointIndex::Azimuth));
  same_layout &= field_azimuth.name == "azimuth";
  same_layout &= field_azimuth.offset == offsetof(PointXYZIRADRT, azimuth);
  same_layout &= field_azimuth.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_azimuth.count == 1;
  const auto & field_distance = input.fields.at(static_cast<size_t>(PointIndex::Distance));
  same_layout &= field_distance.name == "distance";
  same_layout &= field_distance.offset == offsetof(PointXYZIRADRT, distance);
  same_layout &= field_distance.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_distance.count == 1;
  const auto & field_return_type = input.fields.at(static_cast<size_t>(PointIndex::ReturnType));
  same_layout &= field_return_type.name == "return_type";
  same_layout &= field_return_type.offset == offsetof(PointXYZIRADRT, return_type);
  same_layout &= field_return_type.datatype == sensor_msgs::msg::PointField::UINT8;
  same_layout &= field_return_type.count == 1;
  const auto & field_time_stamp = input.fields.at(static_cast<size_t>(PointIndex::TimeStamp));
  same_layout &= field_time_stamp.name == "time_stamp";
  same_layout &= field_time_stamp.offset == offsetof(PointXYZIRADRT, time_stamp);
  same_layout &= field_time_stamp.datatype == sensor_msgs::msg::PointField::FLOAT64;
  same_layout &= field_time_stamp.count == 1;
  return same_layout;
}

bool is_data_layout_compatible_with_point_xyzircaedt(const sensor_msgs::msg::PointCloud2 & input)
{
  using PointIndex = autoware_point_types::PointXYZIRCAEDTIndex;
  using autoware_point_types::PointXYZIRCAEDT;
  if (input.fields.size() != 10) {
    return false;
  }
  bool same_layout = true;
  const auto & field_x = input.fields.at(static_cast<size_t>(PointIndex::X));
  same_layout &= field_x.name == "x";
  same_layout &= field_x.offset == offsetof(PointXYZIRCAEDT, x);
  same_layout &= field_x.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_x.count == 1;
  const auto & field_y = input.fields.at(static_cast<size_t>(PointIndex::Y));
  same_layout &= field_y.name == "y";
  same_layout &= field_y.offset == offsetof(PointXYZIRCAEDT, y);
  same_layout &= field_y.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_y.count == 1;
  const auto & field_z = input.fields.at(static_cast<size_t>(PointIndex::Z));
  same_layout &= field_z.name == "z";
  same_layout &= field_z.offset == offsetof(PointXYZIRCAEDT, z);
  same_layout &= field_z.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_z.count == 1;
  const auto & field_intensity = input.fields.at(static_cast<size_t>(PointIndex::Intensity));
  same_layout &= field_intensity.name == "intensity";
  same_layout &= field_intensity.offset == offsetof(PointXYZIRCAEDT, intensity);
  same_layout &= field_intensity.datatype == sensor_msgs::msg::PointField::UINT8;
  same_layout &= field_intensity.count == 1;
  const auto & field_return_type = input.fields.at(static_cast<size_t>(PointIndex::ReturnType));
  same_layout &= field_return_type.name == "return_type";
  same_layout &= field_return_type.offset == offsetof(PointXYZIRCAEDT, return_type);
  same_layout &= field_return_type.datatype == sensor_msgs::msg::PointField::UINT8;
  same_layout &= field_return_type.count == 1;
  const auto & field_ring = input.fields.at(static_cast<size_t>(PointIndex::Channel));
  same_layout &= field_ring.name == "channel";
  same_layout &= field_ring.offset == offsetof(PointXYZIRCAEDT, channel);
  same_layout &= field_ring.datatype == sensor_msgs::msg::PointField::UINT16;
  same_layout &= field_ring.count == 1;
  const auto & field_azimuth = input.fields.at(static_cast<size_t>(PointIndex::Azimuth));
  same_layout &= field_azimuth.name == "azimuth";
  same_layout &= field_azimuth.offset == offsetof(PointXYZIRCAEDT, azimuth);
  same_layout &= field_azimuth.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_azimuth.count == 1;
  const auto & field_elevation = input.fields.at(static_cast<size_t>(PointIndex::Elevation));
  same_layout &= field_elevation.name == "elevation";
  same_layout &= field_elevation.offset == offsetof(PointXYZIRCAEDT, elevation);
  same_layout &= field_elevation.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_elevation.count == 1;
  const auto & field_distance = input.fields.at(static_cast<size_t>(PointIndex::Distance));
  same_layout &= field_distance.name == "distance";
  same_layout &= field_distance.offset == offsetof(PointXYZIRCAEDT, distance);
  same_layout &= field_distance.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_distance.count == 1;
  const auto & field_time_stamp = input.fields.at(static_cast<size_t>(PointIndex::TimeStamp));
  same_layout &= field_time_stamp.name == "time_stamp";
  same_layout &= field_time_stamp.offset == offsetof(PointXYZIRCAEDT, time_stamp);
  same_layout &= field_time_stamp.datatype == sensor_msgs::msg::PointField::UINT32;
  same_layout &= field_time_stamp.count == 1;
  return same_layout;
}

}  // namespace autoware::pointcloud_preprocessor::utils

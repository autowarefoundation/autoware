/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "autoware_lidar_localizer/util/data_structs.h"

#include <pcl/filters/voxel_grid.h>

Eigen::Matrix4f convertToEigenMatrix4f(const Pose &pose) {
  const Eigen::Translation3f translation(pose.x, pose.y, pose.z);
  const Eigen::AngleAxisf rotation_x(pose.roll, Eigen::Vector3f::UnitX());
  const Eigen::AngleAxisf rotation_y(pose.pitch, Eigen::Vector3f::UnitY());
  const Eigen::AngleAxisf rotation_z(pose.yaw, Eigen::Vector3f::UnitZ());
  const Eigen::Matrix4f m = (translation * rotation_z * rotation_y * rotation_x).matrix();
  return m;
}

Pose convertToPose(const Eigen::Matrix4f &m) {
  Pose pose;
  pose.x = m(0, 3);
  pose.y = m(1, 3);
  pose.z = m(2, 3);

  // reference to tf::getEulerYPR()
  if (std::fabs(m(2, 0)) >= 1) {
    pose.yaw = 0;
    if (m(2, 0) < 0) {
      pose.pitch = M_PI / 2.0;
      pose.roll = std::atan2(m(0, 1), m(0, 2));
    }
    else {
      pose.pitch = -M_PI / 2.0;
      pose.roll = std::atan2(-m(0, 1), -m(0, 2));
    }
  }
  else {
    pose.pitch = -std::asin(m(2, 0));
    pose.roll = std::atan2(m(2, 1) / std::cos(pose.pitch),
                           m(2, 2) / std::cos(pose.pitch));
    pose.yaw = std::atan2(m(1, 0) / std::cos(pose.pitch),
                          m(0, 0) / std::cos(pose.pitch));
  }

  return pose;
}

Pose transformToPose(const Eigen::Matrix4f &rotation_m, const Eigen::Matrix4f &base_m) {
  Eigen::Matrix4f trans_m = rotation_m * base_m;
  return convertToPose(trans_m);
}

Pose transformToPose(const Eigen::Matrix4f &rotation_m, const Pose &base_pose) {
  Eigen::Matrix4f base_m = convertToEigenMatrix4f(base_pose);
  return transformToPose(rotation_m, base_m);
}

Pose transformToPose(const Pose &rotation_pose, const Eigen::Matrix4f &base_m) {
  Eigen::Matrix4f rotation_m = convertToEigenMatrix4f(rotation_pose);
  return transformToPose(rotation_m, base_m);
}

Pose transformToPose(const Pose &rotation_pose, const Pose &base_pose) {
  Eigen::Matrix4f rotation_m = convertToEigenMatrix4f(rotation_pose);
  Eigen::Matrix4f base_m = convertToEigenMatrix4f(base_pose);
  return transformToPose(rotation_m, base_m);
}

Pose convertPoseIntoRelativeCoordinate(const Pose &target_pose,
                                       const Pose &reference_pose) {
  Eigen::Matrix4f target_eigen = convertToEigenMatrix4f(target_pose);
  Eigen::Matrix4f reference_eigen = convertToEigenMatrix4f(reference_pose);

  Eigen::Matrix4f trans_eigen = reference_eigen.inverse() * target_eigen;

  return convertToPose(trans_eigen);
}

template <class PointType>
void addPointCloud(const boost::shared_ptr<pcl::PointCloud<PointType>> &input_ptr,
                   const boost::shared_ptr<pcl::PointCloud<PointType>> &output_ptr) {

  if(input_ptr == nullptr || output_ptr == nullptr) {
      return;
  }

  const auto need_points_size = output_ptr->points.size() + input_ptr->points.size();
  if (output_ptr->points.capacity() < need_points_size) {
    const auto reverse_size = need_points_size * 2;
    output_ptr->points.reserve(reverse_size);
  }
  std::cout << __func__ << " "  << output_ptr->points.size() << std::endl;
  *output_ptr += *input_ptr;
  std::cout << __func__ << " "  << output_ptr->points.size() << std::endl;
}
template void addPointCloud(const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &input_ptr,
                            const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &output_ptr);
template void addPointCloud(const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> &input_ptr,
                            const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> &output_ptr);


template <class PointType>
void donwsamplePointCloud(const boost::shared_ptr<pcl::PointCloud<PointType>> &input_ptr,
                          const boost::shared_ptr<pcl::PointCloud<PointType>> &output_ptr,
                          const double leaf_size_x, const double leaf_size_y, const double leaf_size_z) {
  pcl::VoxelGrid<PointType> voxel_grid_filter;
  voxel_grid_filter.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);
  voxel_grid_filter.setInputCloud(input_ptr);
  voxel_grid_filter.filter(*output_ptr);
}
template void donwsamplePointCloud(const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &input_ptr,
                                   const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &output_ptr,
                                   const double leaf_size_x, const double leaf_size_y, const double leaf_size_z);
template void donwsamplePointCloud(const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> &input_ptr,
                                   const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> &output_ptr,
                                   const double leaf_size_x, const double leaf_size_y, const double leaf_size_z);

template <class PointType>
void donwsamplePointCloud(const boost::shared_ptr<pcl::PointCloud<PointType>> &input_ptr,
                          const boost::shared_ptr<pcl::PointCloud<PointType>> &output_ptr,
                          const double leaf_size) {
  donwsamplePointCloud(input_ptr, output_ptr, leaf_size, leaf_size, leaf_size);
}
template void donwsamplePointCloud(const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &input_ptr,
                                   const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &output_ptr,
                                   const double leaf_size);
template void donwsamplePointCloud(const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> &input_ptr,
                                   const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> &output_ptr,
                                   const double leaf_size);

template <class PointType>
void passThroughPointCloud(const boost::shared_ptr<pcl::PointCloud<PointType>> &input_ptr,
                           const boost::shared_ptr<pcl::PointCloud<PointType>> &output_ptr,
                           const double x, const double y, const double width) {

  if(input_ptr == nullptr || output_ptr == nullptr) {
    return;
  }

  output_ptr->points.reserve(output_ptr->width);
  for (const auto &point : input_ptr->points) {
    if (point.x >= x && point.x <= x + width && point.y >= y &&
        point.y <= y + width) {
      output_ptr->points.push_back(point);
    }
  }
  output_ptr->width = output_ptr->points.size();
  output_ptr->height = 1;
}
template void passThroughPointCloud(const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &input_ptr,
                                    const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &output_ptr,
                                    const double x, const double y, const double width);
template void passThroughPointCloud(const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> &input_ptr,
                                    const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> &output_ptr,
                                    const double x, const double y, const double width);

template <class PointType>
void limitPointCloudRange(const boost::shared_ptr<pcl::PointCloud<PointType>> &input_ptr,
                          const boost::shared_ptr<pcl::PointCloud<PointType>> &output_ptr,
                          const double min_range_meter, const double max_range_meter) {

  if(input_ptr == nullptr || output_ptr == nullptr) {
    return;
  }

  double r = 0;
  for (const auto &p : input_ptr->points) {
    r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
    if (r > min_range_meter && r < max_range_meter) {
      output_ptr->push_back(p);
    }
  }
}
template void limitPointCloudRange(const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &input_ptr,
                                   const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &output_ptr,
                                   const double min_range_meter, const double max_range_meter);
template void limitPointCloudRange(const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> &input_ptr,
                                   const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> &output_ptr,
                                   const double min_range_meter, const double max_range_meter);

template <class PointType>
double calcMaxX(const boost::shared_ptr<pcl::PointCloud<PointType>> &cloud) {

  if (cloud == nullptr || cloud->empty()) {
    return 0;
  }

  double x =
      std::max_element(cloud->begin(), cloud->end(), [](const PointType &lhs,
                                                        const PointType &rhs) {
        return lhs.x < rhs.x;
      })->x;
  return x;
}
template double calcMaxX(const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud);
template double calcMaxX(const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> &cloud);

template <class PointType>
double calcMinX(const boost::shared_ptr<pcl::PointCloud<PointType>> &cloud) {

  if (cloud == nullptr || cloud->empty()) {
    return 0;
  }

  double x =
      std::min_element(cloud->begin(), cloud->end(), [](const PointType &lhs,
                                                        const PointType &rhs) {
        return lhs.x < rhs.x;
      })->x;
  return x;
}
template double calcMinX(const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud);
template double calcMinX(const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> &cloud);

template <class PointType>
double calcMaxY(const boost::shared_ptr<pcl::PointCloud<PointType>> &cloud) {

  if (cloud == nullptr || cloud->empty()) {
    return 0;
  }

  double y =
      std::max_element(cloud->begin(), cloud->end(), [](const PointType &lhs,
                                                        const PointType &rhs) {
        return lhs.y < rhs.y;
      })->y;
  return y;
}
template double calcMaxY(const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud);
template double calcMaxY(const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> &cloud);

template <class PointType>
double calcMinY(const boost::shared_ptr<pcl::PointCloud<PointType>> &cloud) {

  if (cloud == nullptr || cloud->empty()) {
    return 0;
  }

  double y =
      std::min_element(cloud->begin(), cloud->end(), [](const PointType &lhs,
                                                        const PointType &rhs) {
        return lhs.y < rhs.y;
      })->y;
  return y;
}
template double calcMinY(const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud);
template double calcMinY(const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> &cloud);

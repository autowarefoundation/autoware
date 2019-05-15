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

#ifndef UTIL_FUNCTIONS_H
#define UTIL_FUNCTIONS_H

#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include "data_structs.h"

Eigen::Matrix4f convertToEigenMatrix4f(const Pose &pose);
Pose convertToPose(const Eigen::Matrix4f &m);
Pose transformToPose(const Eigen::Matrix4f &rotation_m, const Eigen::Matrix4f &base_m);
Pose transformToPose(const Eigen::Matrix4f &rotation_m, const Pose &base_pose);
Pose transformToPose(const Pose &rotation_pose, const Eigen::Matrix4f &base_m);
Pose transformToPose(const Pose &rotation_pose, const Pose &base_pose);
Pose convertPoseIntoRelativeCoordinate(const Pose &target_pose,
                                       const Pose &reference_pose);

template <class PointType>
void addPointCloud(const boost::shared_ptr<pcl::PointCloud<PointType>> &input_ptr,
                   const boost::shared_ptr<pcl::PointCloud<PointType>> &output_ptr);

template <class PointType>
void donwsamplePointCloud(const boost::shared_ptr<pcl::PointCloud<PointType>> &input_ptr,
                          const boost::shared_ptr<pcl::PointCloud<PointType>> &output_ptr,
                          const double leaf_size_x, const double leaf_size_y, const double leaf_size_z);

template <class PointType>
void donwsamplePointCloud(const boost::shared_ptr<pcl::PointCloud<PointType>> &input_ptr,
                          const boost::shared_ptr<pcl::PointCloud<PointType>> &output_ptr,
                          const double leaf_size);

template <class PointType>
void passThroughPointCloud(const boost::shared_ptr<pcl::PointCloud<PointType>> &input_ptr,
                           const boost::shared_ptr<pcl::PointCloud<PointType>> &output_ptr,
                           const double x, const double y, const double width);

template <class PointType>
void limitPointCloudRange(const boost::shared_ptr<pcl::PointCloud<PointType>> &input_ptr,
                          const boost::shared_ptr<pcl::PointCloud<PointType>> &output_ptr,
                          const double min_range_meter, const double max_range_meter);

template <class PointType>
double calcMaxX(const boost::shared_ptr<pcl::PointCloud<PointType>> &cloud);

template <class PointType>
double calcMinX(const boost::shared_ptr<pcl::PointCloud<PointType>> &cloud);

template <class PointType>
double calcMaxY(const boost::shared_ptr<pcl::PointCloud<PointType>> &cloud);

template <class PointType>
double calcMinY(const boost::shared_ptr<pcl::PointCloud<PointType>> &cloud);

#endif

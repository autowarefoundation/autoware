// Copyright 2020 Tier IV, Inc.
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

#ifndef OBJECT_ASSOCIATION_MERGER__DATA_ASSOCIATION_HPP_
#define OBJECT_ASSOCIATION_MERGER__DATA_ASSOCIATION_HPP_

#include <list>
#include <unordered_map>
#include <vector>

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
class DataAssociation
{
private:
  double getDistance(
    const geometry_msgs::msg::Point & point0, const geometry_msgs::msg::Point & point1);
  geometry_msgs::msg::Point getCentroid(const sensor_msgs::msg::PointCloud2 & pointcloud);
  Eigen::MatrixXi can_assign_matrix_;
  Eigen::MatrixXd max_dist_matrix_;
  Eigen::MatrixXd max_area_matrix_;
  Eigen::MatrixXd min_area_matrix_;
  const double score_threshold_;

public:
  DataAssociation();
  bool assign(
    const Eigen::MatrixXd & src, std::unordered_map<int, int> & direct_assignment,
    std::unordered_map<int, int> & reverse_assignment);
  Eigen::MatrixXd calcScoreMatrix(
    const autoware_auto_perception_msgs::msg::DetectedObjects & object0,
    const autoware_auto_perception_msgs::msg::DetectedObjects & object1);
  virtual ~DataAssociation() {}
};

#endif  // OBJECT_ASSOCIATION_MERGER__DATA_ASSOCIATION_HPP_

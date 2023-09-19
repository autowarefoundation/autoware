// Copyright 2020-2023 TIER IV, Inc.
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
/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
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

/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef LIDAR_APOLLO_INSTANCE_SEGMENTATION__CLUSTER2D_HPP_
#define LIDAR_APOLLO_INSTANCE_SEGMENTATION__CLUSTER2D_HPP_

#include "disjoint_set.hpp"
#include "util.hpp"

#include <std_msgs/msg/header.hpp>
#include <tier4_perception_msgs/msg/detected_object_with_feature.hpp>
#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>

#include <pcl/PointIndices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <memory>
#include <vector>

namespace lidar_apollo_instance_segmentation
{
enum MetaType {
  META_UNKNOWN,
  META_SMALL_MOT,
  META_BIG_MOT,
  META_NON_MOT,
  META_PEDESTRIAN,
  MAX_META_TYPE
};

struct Obstacle
{
  std::vector<int> grids;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr;
  float score;
  float height;
  float heading;
  MetaType meta_type;
  std::vector<float> meta_type_probabilities;

  Obstacle() : score(0.0), height(-5.0), heading(0.0), meta_type(META_UNKNOWN)
  {
    cloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>);
    meta_type_probabilities.assign(MAX_META_TYPE, 0.0);
  }
};

class Cluster2D
{
public:
  Cluster2D(const int rows, const int cols, const float range);

  ~Cluster2D() {}

  void cluster(
    const float * inferred_data, const pcl::PointCloud<pcl::PointXYZI>::Ptr & pc_ptr,
    const pcl::PointIndices & valid_indices, float objectness_thresh,
    bool use_all_grids_for_clustering);

  void filter(const float * inferred_data);
  void classify(const float * inferred_data);

  void getObjects(
    const float confidence_thresh, const float height_thresh, const int min_pts_num,
    tier4_perception_msgs::msg::DetectedObjectsWithFeature & objects,
    const std_msgs::msg::Header & in_header);

  tier4_perception_msgs::msg::DetectedObjectWithFeature obstacleToObject(
    const Obstacle & in_obstacle, const std_msgs::msg::Header & in_header);

private:
  int rows_;
  int cols_;
  int size_;
  float range_;
  float scale_;
  float inv_res_x_;
  float inv_res_y_;
  std::vector<int> point2grid_;
  std::vector<Obstacle> obstacles_;
  std::vector<int> id_img_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr pc_ptr_;
  const std::vector<int> * valid_indices_in_pc_ = nullptr;

  struct Node
  {
    Node * center_node;
    Node * parent;
    char node_rank;
    char traversed;
    bool is_center;
    bool is_object;
    int point_num;
    int obstacle_id;

    Node()
    {
      center_node = nullptr;
      parent = nullptr;
      node_rank = 0;
      traversed = 0;
      is_center = false;
      is_object = false;
      point_num = 0;
      obstacle_id = -1;
    }
  };

  inline bool IsValidRowCol(int row, int col) const { return IsValidRow(row) && IsValidCol(col); }

  inline bool IsValidRow(int row) const { return row >= 0 && row < rows_; }

  inline bool IsValidCol(int col) const { return col >= 0 && col < cols_; }

  inline int RowCol2Grid(int row, int col) const { return row * cols_ + col; }

  void traverse(Node * x);
};
}  // namespace lidar_apollo_instance_segmentation

#endif  // LIDAR_APOLLO_INSTANCE_SEGMENTATION__CLUSTER2D_HPP_

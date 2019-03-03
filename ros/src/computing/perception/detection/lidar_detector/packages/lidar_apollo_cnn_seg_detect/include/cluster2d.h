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

#ifndef CLUSTER2D_H
#define CLUSTER2D_H

#include <vector>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include "caffe/caffe.hpp"

#include "util.h"
#include "disjoint_set.h"

#include <autoware_msgs/DetectedObject.h>
#include <autoware_msgs/DetectedObjectArray.h>

#include <std_msgs/Header.h>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"

enum ObjectType
{
  UNKNOWN = 0,
  UNKNOWN_MOVABLE = 1,
  UNKNOWN_UNMOVABLE = 2,
  PEDESTRIAN = 3,
  BICYCLE = 4,
  VEHICLE = 5,
  MAX_OBJECT_TYPE = 6,
};

enum MetaType
{
  META_UNKNOWN,
  META_SMALLMOT,
  META_BIGMOT,
  META_NONMOT,
  META_PEDESTRIAN,
  MAX_META_TYPE
};


struct Obstacle
{
  std::vector<int> grids;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr;
  float score;
  float height;
  MetaType meta_type;
  std::vector<float> meta_type_probs;

  Obstacle() : score(0.0), height(-5.0), meta_type(META_UNKNOWN)
  {
    cloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>);
    meta_type_probs.assign(MAX_META_TYPE, 0.0);
  }

  std::string GetTypeString() const
  {
    switch (meta_type)
    {
      case META_UNKNOWN:
        return "unknown";
      case META_SMALLMOT:
        return "car";
      case META_BIGMOT:
        return "car";
      case META_NONMOT:
        return "bike";
      case META_PEDESTRIAN:
        return "pedestrian";
      default:
        return "unknown";
    }
  }
};

class Cluster2D
{
public:
  Cluster2D() = default;

  ~Cluster2D() = default;

  bool init(int rows, int cols, float range);

  void cluster(const caffe::Blob<float> &category_pt_blob,
               const caffe::Blob<float> &instance_pt_blob,
               const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_ptr,
               const pcl::PointIndices &valid_indices,
               float objectness_thresh, bool use_all_grids_for_clustering);

  void filter(const caffe::Blob<float> &confidence_pt_blob,
              const caffe::Blob<float> &height_pt_blob);

  void classify(const caffe::Blob<float> &classify_pt_blob);

  void getObjects(const float confidence_thresh,
                  const float height_thresh,
                  const int min_pts_num,
                  autoware_msgs::DetectedObjectArray &objects,
                  const std_msgs::Header &in_header);

  autoware_msgs::DetectedObject obstacleToObject(const Obstacle &in_obstacle,
                                                 const std_msgs::Header &in_header);

private:
  int rows_;
  int cols_;
  int grids_;
  float range_;
  float scale_;
  float inv_res_x_;
  float inv_res_y_;
  std::vector<int> point2grid_;
  std::vector<Obstacle> obstacles_;
  std::vector<int> id_img_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr pc_ptr_;
  const std::vector<int> *valid_indices_in_pc_ = nullptr;

  struct Node
  {
    Node *center_node;
    Node *parent;
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

  inline bool IsValidRowCol(int row, int col) const
  {
    return IsValidRow(row) && IsValidCol(col);
  }

  inline bool IsValidRow(int row) const
  {
    return row >= 0 && row < rows_;
  }

  inline bool IsValidCol(int col) const
  {
    return col >= 0 && col < cols_;
  }

  inline int RowCol2Grid(int row, int col) const
  {
    return row * cols_ + col;
  }

  void traverse(Node *x);

  ObjectType getObjectType(const MetaType meta_type_id);
};

#endif //CLUSTER_2D_H

// Copyright 2017-2022 Arm Ltd., TierIV, Autoware Foundation, The Apollo Authors
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

#ifndef LIDAR_APOLLO_SEGMENTATION_TVM__CLUSTER2D_HPP_
#define LIDAR_APOLLO_SEGMENTATION_TVM__CLUSTER2D_HPP_

#include <lidar_apollo_segmentation_tvm/disjoint_set.hpp>
#include <lidar_apollo_segmentation_tvm/util.hpp>
#include <lidar_apollo_segmentation_tvm/visibility_control.hpp>

#include <autoware_auto_perception_msgs/msg/point_clusters.hpp>
#include <tier4_perception_msgs/msg/detected_object_with_feature.hpp>
#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstdint>
#include <memory>
#include <vector>

namespace autoware
{
namespace perception
{
namespace lidar_apollo_segmentation_tvm
{

/// \brief Internal obstacle classification categories.
enum class MetaType {
  META_UNKNOWN,
  META_SMALL_MOT,
  META_BIG_MOT,
  META_NON_MOT,
  META_PEDESTRIAN,
  MAX_META_TYPE
};

/// \brief Internal obstacle representation.
struct Obstacle
{
  std::vector<int32_t> grids;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr;
  float score;
  float height;
  float heading;
  MetaType meta_type;
  std::vector<float> meta_type_probabilities;

  Obstacle() : score(0.0), height(-5.0), heading(0.0), meta_type(MetaType::META_UNKNOWN)
  {
    cloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>);
    meta_type_probabilities.assign(static_cast<int>(MetaType::MAX_META_TYPE), 0.0);
  }
};

/// \brief Handle the ouput of the CNN-based prediction by obtaining information on individual
///        cells.
class LIDAR_APOLLO_SEGMENTATION_TVM_LOCAL Cluster2D
{
public:
  /// \brief Constructor
  /// \param[in] rows The number of rows in the cluster.
  /// \param[in] cols The number of columns in the cluster.
  /// \param[in] range Scaling factor.
  explicit Cluster2D(int32_t rows, int32_t cols, float range);

  /// \brief Construct a directed graph and search the connected components for candidate object
  ///        clusters.
  /// \param[in] inferred_data Prediction information from the neural network inference.
  /// \param[in] pc_ptr Input point cloud.
  /// \param[in] valid_indices Indices of the points to consider in the point cloud.
  /// \param[in] objectness_thresh Threshold for filtering out non-object cells.
  /// \param[in] use_all_grids_for_clustering
  void cluster(
    const float * inferred_data, const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & pc_ptr,
    const pcl::PointIndices & valid_indices, float objectness_thresh,
    bool use_all_grids_for_clustering);

  /// \brief Populate the fields of obstacles_ elements.
  /// \param[in] inferred_data Prediction information from the neural network inference.
  void filter(const float * inferred_data);

  /// \brief Assign a classification type to the obstacles_ elements.
  /// \param[in] inferred_data Prediction information from the neural network inference.
  void classify(const float * inferred_data);

  /// \brief Remove the candidate clusters that don't meet the parameters' requirements.
  /// \param[in] confidence_thresh The detection confidence score threshold.
  /// \param[in] height_thresh If it is non-negative, the points that are higher than the predicted
  ///                          object height by height_thresh are filtered out.
  /// \param[in] min_pts_num The candidate clusters with less than min_pts_num points are removed.
  /// \return The detected objects.
  std::shared_ptr<tier4_perception_msgs::msg::DetectedObjectsWithFeature> getObjects(
    float confidence_thresh, float height_thresh, int32_t min_pts_num);

  /// \brief Transform an obstacle from the internal representation to the external one.
  /// \param[in] in_obstacle
  /// \return Output obstacle.
  tier4_perception_msgs::msg::DetectedObjectWithFeature obstacleToObject(
    const Obstacle & in_obstacle) const;

private:
  const int32_t rows_;
  const int32_t cols_;
  const int32_t siz_;
  const float range_;
  const float scale_;
  const float inv_res_x_;
  const float inv_res_y_;
  std::vector<int32_t> point2grid_;
  std::vector<Obstacle> obstacles_;
  std::vector<int32_t> id_img_;

  pcl::PointCloud<pcl::PointXYZI>::ConstPtr pc_ptr_;
  const std::vector<int32_t> * valid_indices_in_pc_ = nullptr;

  /// \brief Node of a directed graph.
  struct Node
  {
    Node * center_node;
    Node * parent;
    int8_t node_rank;
    int8_t traversed;
    bool is_center;
    bool is_object;
    int32_t point_num;
    int32_t obstacle_id;

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

  /// \brief Check whether a signed row and column values are valid array indices.
  inline bool IsValidRowCol(int32_t row, int32_t col) const
  {
    return IsValidRow(row) && IsValidCol(col);
  }

  /// \brief Check whether a signed row value is a valid array index.
  inline bool IsValidRow(int32_t row) const { return row >= 0 && row < rows_; }

  /// \brief Check whether a signed column value is a valid array index.
  inline bool IsValidCol(int32_t col) const { return col >= 0 && col < cols_; }

  /// \brief Transform a row and column coordinate to a linear grid index.
  inline int32_t RowCol2Grid(int32_t row, int32_t col) const { return row * cols_ + col; }

  /// \brief Traverse the directed graph until visiting a node.
  /// \param[in] x Node to visit.
  void traverse(Node * x) const;
};
}  // namespace lidar_apollo_segmentation_tvm
}  // namespace perception
}  // namespace autoware
#endif  // LIDAR_APOLLO_SEGMENTATION_TVM__CLUSTER2D_HPP_

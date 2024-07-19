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
//
//
// Author: v1.0 Yukihiro Saito
//

#ifndef AUTOWARE__OBJECT_MERGER__ASSOCIATION__DATA_ASSOCIATION_HPP_
#define AUTOWARE__OBJECT_MERGER__ASSOCIATION__DATA_ASSOCIATION_HPP_

#define EIGEN_MPL2_ONLY

#include "autoware/object_merger/association/solver/gnn_solver.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "autoware_perception_msgs/msg/detected_objects.hpp"

#include <list>
#include <memory>
#include <unordered_map>
#include <vector>

namespace autoware::object_merger
{

class DataAssociation
{
private:
  Eigen::MatrixXi can_assign_matrix_;
  Eigen::MatrixXd max_dist_matrix_;
  Eigen::MatrixXd max_rad_matrix_;
  Eigen::MatrixXd min_iou_matrix_;
  const double score_threshold_;
  std::unique_ptr<autoware::object_merger::gnn_solver::GnnSolverInterface> gnn_solver_ptr_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  DataAssociation(
    std::vector<int> can_assign_vector, std::vector<double> max_dist_vector,
    std::vector<double> max_rad_vector, std::vector<double> min_iou_vector);
  void assign(
    const Eigen::MatrixXd & src, std::unordered_map<int, int> & direct_assignment,
    std::unordered_map<int, int> & reverse_assignment);
  Eigen::MatrixXd calcScoreMatrix(
    const autoware_perception_msgs::msg::DetectedObjects & objects0,
    const autoware_perception_msgs::msg::DetectedObjects & objects1);
  virtual ~DataAssociation() {}
};

}  // namespace autoware::object_merger

#endif  // AUTOWARE__OBJECT_MERGER__ASSOCIATION__DATA_ASSOCIATION_HPP_

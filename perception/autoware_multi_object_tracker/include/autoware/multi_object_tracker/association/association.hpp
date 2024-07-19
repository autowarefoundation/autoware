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

#ifndef AUTOWARE__MULTI_OBJECT_TRACKER__ASSOCIATION__ASSOCIATION_HPP_
#define AUTOWARE__MULTI_OBJECT_TRACKER__ASSOCIATION__ASSOCIATION_HPP_

#define EIGEN_MPL2_ONLY

#include "autoware/multi_object_tracker/association/solver/gnn_solver.hpp"
#include "autoware/multi_object_tracker/tracker/tracker.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "autoware_perception_msgs/msg/detected_objects.hpp"

#include <list>
#include <memory>
#include <unordered_map>
#include <vector>

namespace autoware::multi_object_tracker
{
class DataAssociation
{
private:
  Eigen::MatrixXi can_assign_matrix_;
  Eigen::MatrixXd max_dist_matrix_;
  Eigen::MatrixXd max_area_matrix_;
  Eigen::MatrixXd min_area_matrix_;
  Eigen::MatrixXd max_rad_matrix_;
  Eigen::MatrixXd min_iou_matrix_;
  const double score_threshold_;
  std::unique_ptr<gnn_solver::GnnSolverInterface> gnn_solver_ptr_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  DataAssociation(
    std::vector<int> can_assign_vector, std::vector<double> max_dist_vector,
    std::vector<double> max_area_vector, std::vector<double> min_area_vector,
    std::vector<double> max_rad_vector, std::vector<double> min_iou_vector);
  void assign(
    const Eigen::MatrixXd & src, std::unordered_map<int, int> & direct_assignment,
    std::unordered_map<int, int> & reverse_assignment);
  Eigen::MatrixXd calcScoreMatrix(
    const autoware_perception_msgs::msg::DetectedObjects & measurements,
    const std::list<std::shared_ptr<Tracker>> & trackers);
  virtual ~DataAssociation() {}
};

}  // namespace autoware::multi_object_tracker

#endif  // AUTOWARE__MULTI_OBJECT_TRACKER__ASSOCIATION__ASSOCIATION_HPP_

// Copyright 2021 Tier IV, Inc. All rights reserved.
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

#include "freespace_planning_algorithms/rrtstar_core.hpp"

#include <nlohmann/json.hpp>

#include <fstream>
#include <functional>
#include <memory>
#include <queue>
#include <stack>
#include <unordered_map>
#include <unordered_set>

// cspell: ignore rsspace
// In this case, RSSpace means "Reeds Shepp state Space"
namespace rrtstar_core
{
CSpace::CSpace(
  const Pose & lo, const Pose & hi, double r, std::function<bool(Pose)> is_obstacle_free)
: lo_(lo), hi_(hi), rsspace_(ReedsSheppStateSpace(r)), is_obstacle_free_(is_obstacle_free)
{
  std::random_device device;
  std::mt19937 gen(device());
  rand_gen_ = gen;
}

bool CSpace::isInside(const Pose & p) const
{
  if (p.x > hi_.x) return false;
  if (p.y > hi_.y) return false;
  if (p.x < lo_.x) return false;
  if (p.y < lo_.y) return false;
  return true;
}

Pose CSpace::uniformSampling()
{
  std::uniform_real_distribution<double> distr_x(lo_.x, hi_.x);
  std::uniform_real_distribution<double> distr_y(lo_.y, hi_.y);
  std::uniform_real_distribution<double> distr_yaw(0.0, 2 * M_PI);
  return Pose{distr_x(rand_gen_), distr_y(rand_gen_), distr_yaw(rand_gen_)};
}

Pose CSpace::ellipticInformedSampling(double c_best, const Pose & p_start, const Pose & p_goal)
{
  const double c_min = distanceLowerBound(p_start, p_goal);
  const double ellipse_r1 = c_best * 0.5;
  const double ellipse_r2 = sqrt(c_best * c_best - c_min * c_min) * 0.5;
  const double angle = atan2(p_goal.y - p_start.y, p_goal.x - p_start.x);
  const double x_center = (p_start.x + p_goal.x) * 0.5;
  const double y_center = (p_start.y + p_goal.y) * 0.5;

  std::uniform_real_distribution<double> distr_x(-ellipse_r1, ellipse_r1);
  std::uniform_real_distribution<double> distr_y(-ellipse_r2, ellipse_r2);
  std::uniform_real_distribution<double> distr_yaw(0, 2 * M_PI);
  while (true) {
    const double x_local = distr_x(rand_gen_);
    const double y_local = distr_y(rand_gen_);

    const bool is_inside_ellipse =
      (pow(x_local / ellipse_r1, 2) + pow(y_local / ellipse_r2, 2) < 1.0);
    if (!is_inside_ellipse)
      // Note that in the original paper, sample are generated directly from
      // the uniform distribution over the super-ellipsoid. However, because
      // our targetted-problem is 2-dim, it is fine to use just a rejection
      // sampling.
      continue;

    const double x_global = cos(angle) * x_local + -sin(angle) * y_local + x_center;
    const double y_global = sin(angle) * x_local + cos(angle) * y_local + y_center;
    const double yaw = distr_yaw(rand_gen_);
    return Pose{x_global, y_global, yaw};
  }
}

Pose CSpace::interpolate_child2parent(
  const Pose & pose_child, const Pose & pose_parent, double seg) const
{
  auto path = rsspace_.reedsShepp(pose_child, pose_parent);
  return interpolate(path, pose_child, seg);
}

Pose CSpace::interpolate(Path & path, const Pose & pose, double seg) const
{
  return rsspace_.interpolate(pose, path, seg / rsspace_.rho_);
}

void CSpace::sampleWayPoints_child2parent(
  const Pose & pose_child, const Pose & pose_parent, double step,
  std::vector<Pose> & pose_seq) const
{
  auto path = rsspace_.reedsShepp(pose_child, pose_parent);
  const double path_true_length = path.length() * rsspace_.rho_;
  for (double seg = 0; seg < path_true_length; seg += step) {
    const Pose pose_sampled = interpolate(path, pose_child, seg);
    pose_seq.push_back(pose_sampled);
  }
}

bool CSpace::isValidPath_child2parent(
  const Pose & pose_child, const Pose & pose_parent, double step) const
{
  auto path = rsspace_.reedsShepp(pose_child, pose_parent);
  const double path_true_length = path.length() * rsspace_.rho_;
  if (!isValidPose(pose_parent)) {
    return false;
  }
  for (double seg = 0; seg < path_true_length; seg += step) {
    Pose && pose_sampled = interpolate(path, pose_child, seg);
    if (!isValidPose(pose_sampled)) {
      return false;
    }
  }
  return true;
}

RRTStar::RRTStar(
  Pose x_start, Pose x_goal, double mu, double collision_check_resolution, bool is_informed,
  CSpace cspace)
: mu_(mu),
  collision_check_resolution_(collision_check_resolution),
  is_informed_(is_informed),
  cspace_(cspace)
{
  node_goal_ = std::make_shared<Node>(Node{x_goal, std::nullopt, 0.0});
  node_start_ = std::make_shared<Node>(Node{x_start, 0.0});
  nodes_.push_back(node_start_);
}

void RRTStar::extend()
{
  // Determine new node
  Pose x_rand;
  if (isSolutionFound() && is_informed_) {
    x_rand = cspace_.ellipticInformedSampling(
      *node_goal_->cost_from_start, node_start_->pose, node_goal_->pose);
  } else {
    x_rand = cspace_.uniformSampling();
  }

  const auto node_nearest = findNearestNode(x_rand);

  // NOTE: no child-parent relation here
  const Pose x_new = cspace_.interpolate_child2parent(node_nearest->pose, x_rand, mu_);

  if (!cspace_.isValidPath_child2parent(x_new, node_nearest->pose, collision_check_resolution_)) {
    return;
  }

  const auto & neighbor_nodes = findNeighborNodes(x_new);

  const auto & node_best_parent = getBestParentNode(x_new, node_nearest, neighbor_nodes);
  const auto & node_new = addNewNode(x_new, std::const_pointer_cast<Node>(node_best_parent));

  // Rewire
  const auto node_reconnect = getReconnectTargeNode(node_new, neighbor_nodes);
  if (node_reconnect) {
    reconnect(node_new, std::const_pointer_cast<Node>(node_reconnect));
  }

  // Check if reached
  bool is_reached =
    cspace_.isValidPath_child2parent(node_goal_->pose, node_new->pose, collision_check_resolution_);
  if (is_reached) {
    node_new->cost_to_goal = cspace_.distance(node_new->pose, node_goal_->pose);
    reached_nodes_.push_back(node_new);
  }

  if (isSolutionFound()) {
    // This cannot be inside if(is_reached){...} because we must update this anytime after rewiring
    // takes place
    double cost_min = inf;
    NodeSharedPtr node_best_parent;
    for (const auto & node : reached_nodes_) {
      const double cost = *(node->cost_from_start) + *(node->cost_to_goal);
      if (cost < cost_min) {
        cost_min = cost;
        node_best_parent = node;
      }
    }
    node_goal_->cost_from_start = cost_min;
    node_goal_->parent = node_best_parent;
    node_goal_->cost_to_parent = node_best_parent->cost_to_goal;
  }
}

void RRTStar::deleteNodeUsingBranchAndBound()
{
  // cspell: ignore Karaman
  // ref : III.B of Karaman et al. ICRA 2011
  if (!isSolutionFound()) {
    return;
  }

  std::unordered_map<NodeSharedPtr, int> node_index_map;
  for (size_t i = 0; i < nodes_.size(); ++i) {
    node_index_map[nodes_.at(i)] = i;
  }

  const auto optimal_cost_ubound = node_goal_->cost_from_start;
  std::unordered_set<size_t> delete_indices;

  const auto is_deleted = [&](const auto & node) -> bool {
    return delete_indices.find(node_index_map[node]) != delete_indices.end();
  };

  for (const auto & node : nodes_) {
    if (is_deleted(node)) {
      continue;
    }

    // This cost_to_goal (cost_to_go in the paper) is originally defined by Euclidean distance.
    // But we use cspace_.distance (reeds-sheep by default)
    const auto here_cost_to_goal_lbound = cspace_.distance(node->pose, node_goal_->pose);
    const auto here_optimal_cost_lbound = here_cost_to_goal_lbound + *node->cost_from_start;

    if (here_optimal_cost_lbound > optimal_cost_ubound) {
      delete_indices.insert(node_index_map[node]);
      const auto & node_parent = node->getParent();
      node_parent->deleteChild(node);

      // delete childs
      std::stack<NodeSharedPtr> node_stack;
      node_stack.push(node);
      while (!node_stack.empty()) {
        auto node_here = node_stack.top();

        node_stack.pop();
        delete_indices.insert(node_index_map[node_here]);

        for (auto & node_child : node_here->childs) {
          if (is_deleted(node_child)) {
            continue;
          }
          node_stack.push(node_child);
        }
      }
    }
  }

  // Because nodes_ are shrinking in erasing, delete indices must be sorted
  std::vector<size_t> delete_indices_vec(delete_indices.begin(), delete_indices.end());
  std::sort(delete_indices_vec.begin(), delete_indices_vec.end(), std::greater<size_t>());
  for (const size_t delete_idx : delete_indices_vec) {
    nodes_.erase(nodes_.begin() + delete_idx);
  }
}

std::vector<Pose> RRTStar::sampleSolutionWaypoints() const
{
  std::vector<Pose> poses;
  NodeSharedPtr node = node_goal_;
  while (!node->isRoot()) {
    const auto node_parent = node->getParent();
    cspace_.sampleWayPoints_child2parent(
      node->pose, node_parent->pose, collision_check_resolution_, poses);
    node = node_parent;
  }
  poses.push_back(node_start_->pose);
  std::reverse(poses.begin(), poses.end());
  return poses;
}

std::vector<NodeConstSharedPtr> RRTStar::getNodes() const
{
  std::vector<NodeConstSharedPtr> nodes;
  for (const auto & node : nodes_) {
    nodes.push_back(node);
  }
  return nodes;
}

void RRTStar::dumpState(std::string filename) const
{
  // Dump information of all nodes
  using json = nlohmann::json;

  std::unordered_map<NodeSharedPtr, int> node_index_map;
  for (size_t i = 0; i < nodes_.size(); ++i) {
    node_index_map[nodes_.at(i)] = i;
  }
  node_index_map[node_goal_] = node_index_map.size();

  auto serialize_node = [&](const NodeSharedPtr & node) {
    json j;
    j["pose"] = {node->pose.x, node->pose.y, node->pose.yaw};
    j["idx"] = node_index_map.at(node);

    const auto parent = node->getParent();
    if (!parent) {
      j["parent_idx"] = -1;
    } else {
      j["parent_idx"] = node_index_map.at(parent);

      // fill trajectory from parent to this node
      std::vector<Pose> poses;
      cspace_.sampleWayPoints_child2parent(
        node->pose, parent->pose, collision_check_resolution_, poses);
      for (const auto & pose : poses) {
        j["traj_piece"].push_back({pose.x, pose.y, pose.yaw});
      }
    }
    return j;
  };

  json j;
  j["radius"] = cspace_.getReedsSheppRadius();
  for (const auto & node : nodes_) {
    j["nodes"].push_back(serialize_node(node));
  }
  j["node_goal"] = serialize_node(node_goal_);
  std::ofstream file;
  file.open(filename);
  file << j;
  file.close();
}

NodeConstSharedPtr RRTStar::findNearestNode(const Pose & x_rand) const
{
  double dist_min = inf;
  NodeConstSharedPtr node_nearest;
  for (const auto & node : nodes_) {
    if (cspace_.distanceLowerBound(node->pose, x_rand) < dist_min) {
      const double dist_real = cspace_.distance(node->pose, x_rand);
      if (dist_real < dist_min) {
        dist_min = dist_real;
        node_nearest = node;
      }
    }
  }
  return node_nearest;
}

std::vector<NodeConstSharedPtr> RRTStar::findNeighborNodes(const Pose & x_new) const
{
  // In the original paper of rrtstar, radius is shrinking over time.
  // However, because we use reeds-shepp distance metric instead of Euclidean metric,
  // it is hard to design the shrinking radius update. Usage of informed sampling
  // makes the problem far more complex, as the sampling region is shrinking over
  // the time.
  // Due to above difficulty in design of radius update, radius is simply fixed here.
  // In practice, the fixed radius setting works well as long as mu_ value is
  // properly tuned. In car planning scenario, the order or planning world area
  // is similar, and turning radius is also similar through different problems.
  // So, tuning mu_ parameter is not so difficult.

  const double radius_neighbor = mu_;

  std::vector<NodeConstSharedPtr> nodes;
  for (auto & node : nodes_) {
    if (cspace_.distanceLowerBound(node->pose, x_new) > radius_neighbor) continue;
    const bool is_neighbor = (cspace_.distance(node->pose, x_new) < radius_neighbor);
    if (is_neighbor) {
      nodes.push_back(node);
    }
  }
  return nodes;
}

NodeSharedPtr RRTStar::addNewNode(const Pose & pose, NodeSharedPtr node_parent)
{
  const double cost_to_parent = cspace_.distance(pose, node_parent->pose);
  const double cost_from_start = *(node_parent->cost_from_start) + cost_to_parent;
  auto node_new =
    std::make_shared<Node>(Node{pose, cost_from_start, std::nullopt, cost_to_parent, node_parent});
  nodes_.push_back(node_new);
  node_parent->childs.push_back(node_new);
  return node_new;
}

NodeConstSharedPtr RRTStar::getReconnectTargeNode(
  const NodeConstSharedPtr node_new, const std::vector<NodeConstSharedPtr> & neighbor_nodes) const
{
  NodeConstSharedPtr node_reconnect = nullptr;

  for (const auto & node_neighbor : neighbor_nodes) {
    if (cspace_.isValidPath_child2parent(
          node_neighbor->pose, node_new->pose, collision_check_resolution_)) {
      const double cost_from_start_rewired =
        *node_new->cost_from_start + cspace_.distance(node_new->pose, node_neighbor->pose);
      if (cost_from_start_rewired < *node_neighbor->cost_from_start) {
        node_reconnect = node_neighbor;
      }
    }
  }

  return node_reconnect;
}

NodeConstSharedPtr RRTStar::getBestParentNode(
  const Pose & pose_new, const NodeConstSharedPtr & node_nearest,
  const std::vector<NodeConstSharedPtr> & neighbor_nodes) const
{
  NodeConstSharedPtr node_best = node_nearest;
  double cost_min =
    *(node_nearest->cost_from_start) + cspace_.distance(node_nearest->pose, pose_new);
  for (const auto & node : neighbor_nodes) {
    const double cost_start_to_new =
      *(node->cost_from_start) + cspace_.distance(node->pose, pose_new);
    if (cost_start_to_new < cost_min) {
      if (cspace_.isValidPath_child2parent(pose_new, node->pose, collision_check_resolution_)) {
        node_best = node;
        cost_min = cost_start_to_new;
      }
    }
  }
  return node_best;
}

void RRTStar::reconnect(const NodeSharedPtr & node_new, const NodeSharedPtr & node_reconnect)
{
  // connect node_new (parent) -> node_reconnect (child)

  // current state:
  // node_new -> #nil;
  // node_reconnect_parent -> node_reconnect -> #nil

  const auto node_reconnect_parent = node_reconnect->getParent();
  node_reconnect_parent->deleteChild(node_reconnect);
  node_reconnect->parent = std::weak_ptr<Node>();
  node_reconnect->cost_to_parent = std::nullopt;

  // Current state:
  // node_new_parent -> node_new -> #nil
  // node_reconnect_parent -> #nil
  // node_reconnect -> #nil
  const double cost_a2b = cspace_.distance(node_new->pose, node_reconnect->pose);
  node_new->childs.push_back(node_reconnect);
  node_reconnect->parent = node_new;
  node_reconnect->cost_to_parent = cost_a2b;
  node_reconnect->cost_from_start = *node_new->cost_from_start + cost_a2b;
  // Current state:
  // node_new_parent -> node_new -> node_reconnect -> #nil;
  // node_reconnect_parent -> #nil;

  // update cost of all descendents of node_reconnect
  std::queue<NodeSharedPtr> bf_queue;
  bf_queue.push(node_reconnect);
  while (!bf_queue.empty()) {
    const auto node = bf_queue.front();
    bf_queue.pop();
    for (const auto & child : node->childs) {
      child->cost_from_start = *node->cost_from_start + *child->cost_to_parent;
      bf_queue.push(child);
    }
  }
}

}  // namespace rrtstar_core

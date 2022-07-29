// Copyright 2018 Autoware Foundation. All rights reserved.
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

#include "shape_estimation/model/bounding_box.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <autoware_auto_perception_msgs/msg/shape.hpp>

#include <boost/math/tools/minima.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Quaternion.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#define EIGEN_MPL2_ONLY

#include <Eigen/Core>

constexpr float epsilon = 0.001;

BoundingBoxShapeModel::BoundingBoxShapeModel()
: ref_yaw_info_(boost::none), use_boost_bbox_optimizer_(false)
{
}

BoundingBoxShapeModel::BoundingBoxShapeModel(
  const boost::optional<ReferenceYawInfo> & ref_yaw_info, bool use_boost_bbox_optimizer)
: ref_yaw_info_(ref_yaw_info), use_boost_bbox_optimizer_(use_boost_bbox_optimizer)
{
}

bool BoundingBoxShapeModel::estimate(
  const pcl::PointCloud<pcl::PointXYZ> & cluster,
  autoware_auto_perception_msgs::msg::Shape & shape_output, geometry_msgs::msg::Pose & pose_output)
{
  float min_angle, max_angle;
  if (ref_yaw_info_) {
    min_angle = ref_yaw_info_.get().yaw - ref_yaw_info_.get().search_angle_range;
    max_angle = ref_yaw_info_.get().yaw + ref_yaw_info_.get().search_angle_range;
  } else {
    min_angle = 0.0;
    max_angle = M_PI * 0.5;
  }
  return fitLShape(cluster, min_angle, max_angle, shape_output, pose_output);
}

bool BoundingBoxShapeModel::fitLShape(
  const pcl::PointCloud<pcl::PointXYZ> & cluster, const float min_angle, const float max_angle,
  autoware_auto_perception_msgs::msg::Shape & shape_output, geometry_msgs::msg::Pose & pose_output)
{
  // calc min and max z for height
  float min_z = cluster.empty() ? 0.0 : cluster.at(0).z;
  float max_z = cluster.empty() ? 0.0 : cluster.at(0).z;
  for (const auto & point : cluster) {
    min_z = std::min(point.z, min_z);
    max_z = std::max(point.z, max_z);
  }

  /*
   * Paper : IV2017, Efficient L-Shape Fitting for Vehicle Detection Using Laser Scanners
   * Authors : Xio Zhang, Wenda Xu, Chiyu Dong and John M. Dolan
   */

  // Paper : Algo.2 Search-Based Rectangle Fitting
  double theta_star;
  if (use_boost_bbox_optimizer_) {
    theta_star = boostOptimize(cluster, min_angle, max_angle);
  } else {
    theta_star = optimize(cluster, min_angle, max_angle);
  }

  const float sin_theta_star = std::sin(theta_star);
  const float cos_theta_star = std::cos(theta_star);

  Eigen::Vector2f e_1_star;  // col.11, Algo.2
  Eigen::Vector2f e_2_star;
  e_1_star << cos_theta_star, sin_theta_star;
  e_2_star << -sin_theta_star, cos_theta_star;
  std::vector<float> C_1_star;  // col.11, Algo.2
  std::vector<float> C_2_star;  // col.11, Algo.2
  for (const auto & point : cluster) {
    C_1_star.push_back(point.x * e_1_star.x() + point.y * e_1_star.y());
    C_2_star.push_back(point.x * e_2_star.x() + point.y * e_2_star.y());
  }

  // col.12, Algo.2
  const float min_C_1_star = *std::min_element(C_1_star.begin(), C_1_star.end());
  const float max_C_1_star = *std::max_element(C_1_star.begin(), C_1_star.end());
  const float min_C_2_star = *std::min_element(C_2_star.begin(), C_2_star.end());
  const float max_C_2_star = *std::max_element(C_2_star.begin(), C_2_star.end());

  const float a_1 = cos_theta_star;
  const float b_1 = sin_theta_star;
  const float c_1 = min_C_1_star;
  const float a_2 = -1.0 * sin_theta_star;
  const float b_2 = cos_theta_star;
  const float c_2 = min_C_2_star;
  const float a_3 = cos_theta_star;
  const float b_3 = sin_theta_star;
  const float c_3 = max_C_1_star;
  const float a_4 = -1.0 * sin_theta_star;
  const float b_4 = cos_theta_star;
  const float c_4 = max_C_2_star;

  // calc center of bounding box
  float intersection_x_1 = (b_1 * c_2 - b_2 * c_1) / (a_2 * b_1 - a_1 * b_2);
  float intersection_y_1 = (a_1 * c_2 - a_2 * c_1) / (a_1 * b_2 - a_2 * b_1);
  float intersection_x_2 = (b_3 * c_4 - b_4 * c_3) / (a_4 * b_3 - a_3 * b_4);
  float intersection_y_2 = (a_3 * c_4 - a_4 * c_3) / (a_3 * b_4 - a_4 * b_3);

  // calc dimension of bounding box
  Eigen::Vector2f e_x;
  Eigen::Vector2f e_y;
  e_x << a_1 / (std::sqrt(a_1 * a_1 + b_1 * b_1)), b_1 / (std::sqrt(a_1 * a_1 + b_1 * b_1));
  e_y << a_2 / (std::sqrt(a_2 * a_2 + b_2 * b_2)), b_2 / (std::sqrt(a_2 * a_2 + b_2 * b_2));
  Eigen::Vector2f diagonal_vec;
  diagonal_vec << intersection_x_1 - intersection_x_2, intersection_y_1 - intersection_y_2;

  // calc yaw
  tf2::Quaternion quat;
  quat.setEuler(/* roll */ 0, /* pitch */ 0, /* yaw */ std::atan2(e_1_star.y(), e_1_star.x()));

  // output
  shape_output.type = autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX;
  shape_output.dimensions.x = std::fabs(e_x.dot(diagonal_vec));
  shape_output.dimensions.y = std::fabs(e_y.dot(diagonal_vec));
  shape_output.dimensions.z = std::max((max_z - min_z), epsilon);
  pose_output.position.x = (intersection_x_1 + intersection_x_2) * 0.5;
  pose_output.position.y = (intersection_y_1 + intersection_y_2) * 0.5;
  pose_output.position.z = min_z + shape_output.dimensions.z * 0.5;
  pose_output.orientation = tf2::toMsg(quat);
  // check wrong output
  shape_output.dimensions.x = std::max(static_cast<float>(shape_output.dimensions.x), epsilon);
  shape_output.dimensions.y = std::max(static_cast<float>(shape_output.dimensions.y), epsilon);

  return true;
}

float BoundingBoxShapeModel::calcClosenessCriterion(
  const std::vector<float> & C_1, const std::vector<float> & C_2)
{
  // Paper : Algo.4 Closeness Criterion
  const float min_c_1 = *std::min_element(C_1.begin(), C_1.end());  // col.2, Algo.4
  const float max_c_1 = *std::max_element(C_1.begin(), C_1.end());  // col.2, Algo.4
  const float min_c_2 = *std::min_element(C_2.begin(), C_2.end());  // col.3, Algo.4
  const float max_c_2 = *std::max_element(C_2.begin(), C_2.end());  // col.3, Algo.4

  std::vector<float> D_1;  // col.4, Algo.4
  for (const auto & c_1_element : C_1) {
    const float v = std::min(max_c_1 - c_1_element, c_1_element - min_c_1);
    D_1.push_back(v * v);
  }

  std::vector<float> D_2;  // col.5, Algo.4
  for (const auto & c_2_element : C_2) {
    const float v = std::min(max_c_2 - c_2_element, c_2_element - min_c_2);
    D_2.push_back(v * v);
  }
  constexpr float d_min = 0.1 * 0.1;
  constexpr float d_max = 0.4 * 0.4;
  float beta = 0;  // col.6, Algo.4
  for (size_t i = 0; i < D_1.size(); ++i) {
    if (d_max < std::min(D_1.at(i), D_2.at(i))) {
      continue;
    }
    const float d = std::max(std::min(D_1.at(i), D_2.at(i)), d_min);
    beta += 1.0 / d;
  }
  return beta;
}

float BoundingBoxShapeModel::optimize(
  const pcl::PointCloud<pcl::PointXYZ> & cluster, const float min_angle, const float max_angle)
{
  std::vector<std::pair<float /*theta*/, float /*q*/>> Q;
  constexpr float angle_resolution = M_PI / 180.0;
  for (float theta = min_angle; theta <= max_angle + epsilon; theta += angle_resolution) {
    Eigen::Vector2f e_1;
    e_1 << std::cos(theta), std::sin(theta);  // col.3, Algo.2
    Eigen::Vector2f e_2;
    e_2 << -std::sin(theta), std::cos(theta);  // col.4, Algo.2
    std::vector<float> C_1;                    // col.5, Algo.2
    std::vector<float> C_2;                    // col.6, Algo.2
    for (const auto & point : cluster) {
      C_1.push_back(point.x * e_1.x() + point.y * e_1.y());
      C_2.push_back(point.x * e_2.x() + point.y * e_2.y());
    }
    float q = calcClosenessCriterion(C_1, C_2);  // col.7, Algo.2
    Q.push_back(std::make_pair(theta, q));       // col.8, Algo.2
  }

  float theta_star{0.0};  // col.10, Algo.2
  float max_q = 0.0;
  for (size_t i = 0; i < Q.size(); ++i) {
    if (max_q < Q.at(i).second || i == 0) {
      max_q = Q.at(i).second;
      theta_star = Q.at(i).first;
    }
  }

  return theta_star;
}

float BoundingBoxShapeModel::boostOptimize(
  const pcl::PointCloud<pcl::PointXYZ> & cluster, const float min_angle, const float max_angle)
{
  auto closeness_func = [&](float theta) {
    Eigen::Vector2f e_1;
    e_1 << std::cos(theta), std::sin(theta);  // col.3, Algo.2
    Eigen::Vector2f e_2;
    e_2 << -std::sin(theta), std::cos(theta);  // col.4, Algo.2
    std::vector<float> C_1;                    // col.5, Algo.2
    std::vector<float> C_2;                    // col.6, Algo.2
    for (const auto & point : cluster) {
      C_1.push_back(point.x * e_1.x() + point.y * e_1.y());
      C_2.push_back(point.x * e_2.x() + point.y * e_2.y());
    }
    float q = calcClosenessCriterion(C_1, C_2);
    return -q;
  };

  int bits = 6;
  boost::uintmax_t max_iter = 20;
  std::pair<float, float> min = boost::math::tools::brent_find_minima(
    closeness_func, min_angle, max_angle + epsilon, bits, max_iter);
  float theta_star = min.first;
  return theta_star;
}

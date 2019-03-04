/*
 * Copyright 2018 Autoware Foundation. All rights reserved.
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
 *
 * v1.0 Yukihiro Saito
 */

#include "bounding_box.hpp"
#include <cmath>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define EIGEN_MPL2_ONLY

#include <Eigen/Core>

bool BoundingBoxModel::estimate(const pcl::PointCloud<pcl::PointXYZ>& cluster, autoware_msgs::DetectedObject& output)
{
  // calc centroid point for cylinder height(z)
  pcl::PointXYZ centroid;
  centroid.x = 0;
  centroid.y = 0;
  centroid.z = 0;
  for (const auto& pcl_point : cluster)
  {
    centroid.x += pcl_point.x;
    centroid.y += pcl_point.y;
    centroid.z += pcl_point.z;
  }
  centroid.x = centroid.x / (double)cluster.size();
  centroid.y = centroid.y / (double)cluster.size();
  centroid.z = centroid.z / (double)cluster.size();

  // calc min and max z for cylinder length
  double min_z = 0;
  double max_z = 0;
  for (size_t i = 0; i < cluster.size(); ++i)
  {
    if (cluster.at(i).z < min_z || i == 0)
      min_z = cluster.at(i).z;
    if (max_z < cluster.at(i).z || i == 0)
      max_z = cluster.at(i).z;
  }

  // calc circumscribed circle on x-y plane
  cv::Mat_<float> cv_points((int)cluster.size(), 2);
  for (size_t i = 0; i < cluster.size(); ++i)
  {
    cv_points(i, 0) = cluster.at(i).x;  // x
    cv_points(i, 1) = cluster.at(i).y;  // y
  }

  /*
   * Paper : IV2017, Efficient L-Shape Fitting for Vehicle Detection Using Laser Scanners
   * Authors : Xio Zhang, Wenda Xu, Chiyu Dong and John M. Dolan
   */

  // Paper : Algo.2 Search-Based Rectangle Fitting
  std::vector<std::pair<double /*theta*/, double /*q*/>> Q;
  const double max_angle = M_PI / 2.0;
  const double angle_reso = M_PI / 180.0;
  for (double theta = 0; theta < max_angle; theta += angle_reso)
  {
    Eigen::Vector2d e_1;
    e_1 << std::cos(theta), std::sin(theta);  // col.3, Algo.2
    Eigen::Vector2d e_2;
    e_2 << -std::sin(theta), std::cos(theta);  // col.4, Algo.2
    std::vector<double> C_1;                   // col.5, Algo.2
    std::vector<double> C_2;                   // col.6, Algo.2
    for (const auto& point : cluster)
    {
      C_1.push_back(point.x * e_1.x() + point.y * e_1.y());
      C_2.push_back(point.x * e_2.x() + point.y * e_2.y());
    }
    double q = calcClosenessCriterion(C_1, C_2);  // col.7, Algo.2
    Q.push_back(std::make_pair(theta, q));        // col.8, Algo.2
  }

  double theta_star;  // col.10, Algo.2
  double max_q;
  for (size_t i = 0; i < Q.size(); ++i)
  {
    if (max_q < Q.at(i).second || i == 0)
    {
      max_q = Q.at(i).second;
      theta_star = Q.at(i).first;
    }
  }

  Eigen::Vector2d e_1_star;  // col.11, Algo.2
  Eigen::Vector2d e_2_star;
  e_1_star << std::cos(theta_star), std::sin(theta_star);
  e_2_star << -std::sin(theta_star), std::cos(theta_star);
  std::vector<double> C_1_star;  // col.11, Algo.2
  std::vector<double> C_2_star;  // col.11, Algo.2
  for (const auto& point : cluster)
  {
    C_1_star.push_back(point.x * e_1_star.x() + point.y * e_1_star.y());
    C_2_star.push_back(point.x * e_2_star.x() + point.y * e_2_star.y());
  }

  // col.12, Algo.2
  const double min_C_1_star = *std::min_element(C_1_star.begin(), C_1_star.end());
  const double max_C_1_star = *std::max_element(C_1_star.begin(), C_1_star.end());
  const double min_C_2_star = *std::min_element(C_2_star.begin(), C_2_star.end());
  const double max_C_2_star = *std::max_element(C_2_star.begin(), C_2_star.end());

  const double a_1 = std::cos(theta_star);
  const double b_1 = std::sin(theta_star);
  const double c_1 = min_C_1_star;
  const double a_2 = -1.0 * std::sin(theta_star);
  const double b_2 = std::cos(theta_star);
  const double c_2 = min_C_2_star;
  const double a_3 = std::cos(theta_star);
  const double b_3 = std::sin(theta_star);
  const double c_3 = max_C_1_star;
  const double a_4 = -1.0 * std::sin(theta_star);
  const double b_4 = std::cos(theta_star);
  const double c_4 = max_C_2_star;

  // calc center of bounding box
  double intersection_x_1 = (b_1 * c_2 - b_2 * c_1) / (a_2 * b_1 - a_1 * b_2);
  double intersection_y_1 = (a_1 * c_2 - a_2 * c_1) / (a_1 * b_2 - a_2 * b_1);
  double intersection_x_2 = (b_3 * c_4 - b_4 * c_3) / (a_4 * b_3 - a_3 * b_4);
  double intersection_y_2 = (a_3 * c_4 - a_4 * c_3) / (a_3 * b_4 - a_4 * b_3);

  // calc dimention of bounding box
  Eigen::Vector2d e_x;
  Eigen::Vector2d e_y;
  e_x << a_1 / (std::sqrt(a_1 * a_1 + b_1 * b_1)), b_1 / (std::sqrt(a_1 * a_1 + b_1 * b_1));
  e_y << a_2 / (std::sqrt(a_2 * a_2 + b_2 * b_2)), b_2 / (std::sqrt(a_2 * a_2 + b_2 * b_2));
  Eigen::Vector2d diagonal_vec;
  diagonal_vec << intersection_x_1 - intersection_x_2, intersection_y_1 - intersection_y_2;

  // calc yaw
  tf2::Quaternion quat;
  quat.setEuler(/* roll */ 0, /* pitch */ 0, /* yaw */ std::atan2(e_1_star.y(), e_1_star.x()));

  output.pose.position.x = (intersection_x_1 + intersection_x_2) / 2.0;
  output.pose.position.y = (intersection_y_1 + intersection_y_2) / 2.0;
  output.pose.position.z = centroid.z;
  output.pose.orientation = tf2::toMsg(quat);
  constexpr double ep = 0.001;
  output.dimensions.x = std::fabs(e_x.dot(diagonal_vec));
  output.dimensions.y = std::fabs(e_y.dot(diagonal_vec));
  output.dimensions.z = std::max((max_z - min_z), ep);
  output.pose_reliable = true;

  // check wrong output
  if (output.dimensions.x < ep && output.dimensions.y < ep)
    return false;
  output.dimensions.x = std::max(output.dimensions.x, ep);
  output.dimensions.y = std::max(output.dimensions.y, ep);
  return true;
}

// double BoundingBoxModel::calcClosenessCriterion(const std::vector<double> &C_1, const std::vector<double> &C_2)
// {
//     // Paper : Algo.4 Closeness Criterion
//     const double min_c_1 = *std::min_element(C_1.begin(), C_1.end()); // col.2, Algo.4
//     const double max_c_1 = *std::max_element(C_1.begin(), C_1.end()); // col.2, Algo.4
//     const double min_c_2 = *std::min_element(C_2.begin(), C_2.end()); // col.3, Algo.4
//     const double max_c_2 = *std::max_element(C_2.begin(), C_2.end()); // col.3, Algo.4

//     double max_beta = 0; // col.6, Algo.4
//     {
//         std::vector<double> D_1; // col.4, Algo.4
//         for (const auto &c_1_element : C_1)
//         {
//             const double v = max_c_1 - c_1_element;
//             D_1.push_back(v * v);
//         }

//         std::vector<double> D_2; // col.5, Algo.4
//         for (const auto &c_2_element : C_2)
//         {
//             const double v = max_c_2 - c_2_element;
//             D_2.push_back(v * v);
//         }

//         const double d_min = 0.1 * 0.1;
//         const double d_max = 0.5 * 0.5;
//         double beta = 0; // col.6, Algo.4
//         for (size_t i = 0; i < D_1.size(); ++i)
//         {
//             const double d = std::min(std::max(std::min(D_1.at(i), D_2.at(i)), d_min), d_max);
//             beta += 1.0 / d;
//         }
//         if (max_beta < beta)
//             max_beta = beta;
//     }
//     {
//         std::vector<double> D_1; // col.4, Algo.4
//         for (const auto &c_1_element : C_1)
//         {
//             const double v = max_c_1 - c_1_element;
//             D_1.push_back(v * v);
//         }

//         std::vector<double> D_2; // col.5, Algo.4
//         for (const auto &c_2_element : C_2)
//         {
//             const double v = min_c_2 - c_2_element;
//             D_2.push_back(v * v);
//         }

//         const double d_min = 0.1;
//         const double d_max = 0.5;
//         double beta = 0; // col.6, Algo.4
//         for (size_t i = 0; i < D_1.size(); ++i)
//         {
//             const double d = std::min(std::max(std::min(D_1.at(i), D_2.at(i)), d_min), d_max);
//             beta += 1.0 / d;
//         }
//         if (max_beta < beta)
//             max_beta = beta;
//     }
//     {
//         std::vector<double> D_1; // col.4, Algo.4
//         for (const auto &c_1_element : C_1)
//         {
//             const double v = min_c_1 - c_1_element;
//             D_1.push_back(v * v);
//         }

//         std::vector<double> D_2; // col.5, Algo.4
//         for (const auto &c_2_element : C_2)
//         {
//             const double v = max_c_2 - c_2_element;
//             D_2.push_back(v * v);
//         }

//         const double d_min = 0.1;
//         const double d_max = 0.5;
//         double beta = 0; // col.6, Algo.4
//         for (size_t i = 0; i < D_1.size(); ++i)
//         {
//             const double d = std::min(std::max(std::min(D_1.at(i), D_2.at(i)), d_min), d_max);
//             beta += 1.0 / d;
//         }
//         if (max_beta < beta)
//             max_beta = beta;
//     }
//     {
//         std::vector<double> D_1; // col.4, Algo.4
//         for (const auto &c_1_element : C_1)
//         {
//             const double v = min_c_1 - c_1_element;
//             D_1.push_back(v * v);
//         }

//         std::vector<double> D_2; // col.5, Algo.4
//         for (const auto &c_2_element : C_2)
//         {
//             const double v = min_c_2 - c_2_element;
//             D_2.push_back(v * v);
//         }

//         const double d_min = 0.01;
//         const double d_max = 0.25;
//         double beta = 0; // col.6, Algo.4
//         for (size_t i = 0; i < D_1.size(); ++i)
//         {
//             const double d = std::min(std::max(std::min(D_1.at(i), D_2.at(i)), d_min), d_max);
//             beta += 1.0 / d;
//         }
//         if (max_beta < beta)
//             max_beta = beta;
//     }

//     return max_beta;
// }

double BoundingBoxModel::calcClosenessCriterion(const std::vector<double>& C_1, const std::vector<double>& C_2)
{
  // Paper : Algo.4 Closeness Criterion
  const double min_c_1 = *std::min_element(C_1.begin(), C_1.end());  // col.2, Algo.4
  const double max_c_1 = *std::max_element(C_1.begin(), C_1.end());  // col.2, Algo.4
  const double min_c_2 = *std::min_element(C_2.begin(), C_2.end());  // col.3, Algo.4
  const double max_c_2 = *std::max_element(C_2.begin(), C_2.end());  // col.3, Algo.4

  std::vector<double> D_1;  // col.4, Algo.4
  for (const auto& c_1_element : C_1)
  {
    const double v = std::min(max_c_1 - c_1_element, c_1_element - min_c_1);
    D_1.push_back(std::fabs(v));
  }

  std::vector<double> D_2;  // col.5, Algo.4
  for (const auto& c_2_element : C_2)
  {
    const double v = std::min(max_c_2 - c_2_element, c_2_element - min_c_2);
    D_2.push_back(v * v);
  }

  const double d_min = 0.05;
  const double d_max = 0.50;
  double beta = 0;  // col.6, Algo.4
  for (size_t i = 0; i < D_1.size(); ++i)
  {
    const double d = std::min(std::max(std::min(D_1.at(i), D_2.at(i)), d_min), d_max);
    beta += 1.0 / d;
  }
  return beta;
}

// double BoundingBoxModel::calcClosenessCriterion(const std::vector<double> &C_1, const std::vector<double> &C_2)
// {
//     // Paper : Algo.4 Closeness Criterion
//     const double min_c_1 = *std::min_element(C_1.begin(), C_1.end()); // col.2, Algo.4
//     const double max_c_1 = *std::max_element(C_1.begin(), C_1.end()); // col.2, Algo.4
//     const double min_c_2 = *std::min_element(C_2.begin(), C_2.end()); // col.3, Algo.4
//     const double max_c_2 = *std::max_element(C_2.begin(), C_2.end()); // col.3, Algo.4

//     std::vector<double> D_1; // col.4, Algo.4
//     for (const auto &c_1_element : C_1)
//     {
//         const double v = std::min(max_c_1 - c_1_element, c_1_element - min_c_1);
//         D_1.push_back(std::fabs(v));
//     }

//     std::vector<double> D_2; // col.5, Algo.4
//     for (const auto &c_2_element : C_2)
//     {
//         const double v = std::min(max_c_2 - c_2_element, c_2_element - min_c_2);
//         D_2.push_back(std::fabs(v));
//     }

//     const double d_min = 0.1;
//     double beta = 0; // col.6, Algo.4
//     for (size_t i = 0; i < D_1.size(); ++i)
//     {
//         const double d = std::max(std::min(D_1.at(i), D_2.at(i)), d_min);
//         beta += 1.0 / d;
//     }
//     return beta;
// }

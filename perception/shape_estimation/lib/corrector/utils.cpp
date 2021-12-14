// Copyright 2021 TierIV. All rights reserved.
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

#include "shape_estimation/corrector/utils.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <vector>

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace utils
{
bool correctVehicleBoundingBox(
  const CorrectionParameters & param, autoware_auto_perception_msgs::msg::Shape & shape_output,
  geometry_msgs::msg::Pose & pose_output)
{
  // TODO(Yukihiro Saito): refactor following code

  Eigen::Translation<double, 2> trans =
    Eigen::Translation<double, 2>(pose_output.position.x, pose_output.position.y);
  Eigen::Rotation2Dd rotate(tf2::getYaw(pose_output.orientation));
  Eigen::Affine2d affine_mat;
  affine_mat = trans * rotate.toRotationMatrix();

  /*
   *         ^ x
   *         |
   *        (0)
   * y       |
   * <--(1)--|--(3)--
   *         |
   *        (2)
   *         |
   */
  std::vector<Eigen::Vector2d> v_point;
  v_point.push_back(Eigen::Vector2d(shape_output.dimensions.x / 2.0, 0.0));
  v_point.push_back(Eigen::Vector2d(0.0, shape_output.dimensions.y / 2.0));
  v_point.push_back(Eigen::Vector2d(-shape_output.dimensions.x / 2.0, 0.0));
  v_point.push_back(Eigen::Vector2d(0.0, -shape_output.dimensions.y / 2.0));

  // most distant index from base link
  size_t first_most_distant_index = 0;
  {
    double distance = 0.0;
    for (size_t i = 0; i < v_point.size(); ++i) {
      if (distance < (affine_mat * v_point.at(i)).norm()) {
        distance = (affine_mat * v_point.at(i)).norm();
        first_most_distant_index = i;
      }
    }
  }
  // second distant index from base link
  size_t second_most_distant_index = 0;
  {
    double distance = 0.0;
    for (size_t i = 0; i < v_point.size(); ++i) {
      if (distance < (affine_mat * v_point.at(i)).norm() && i != first_most_distant_index) {
        distance = (affine_mat * v_point.at(i)).norm();
        second_most_distant_index = i;
      }
    }
  }
  // third distant index from base link
  size_t third_most_distant_index = 0;
  {
    double distance = 0.0;
    for (size_t i = 0; i < v_point.size(); ++i) {
      if (
        (distance < (affine_mat * v_point.at(i)).norm()) && i != first_most_distant_index &&
        i != second_most_distant_index) {
        distance = (affine_mat * v_point.at(i)).norm();
        third_most_distant_index = i;
      }
    }
  }

  // rule based correction
  Eigen::Vector2d correction_vector = Eigen::Vector2d::Zero();

  // 1,3 pair or 0,2 pair is most far index
  if (
    static_cast<int>(std::abs(
      static_cast<int>(first_most_distant_index) - static_cast<int>(second_most_distant_index))) %
      2 ==
    0) {
    if (
      param.min_width < (v_point.at(first_most_distant_index) * 2.0).norm() &&
      (v_point.at(first_most_distant_index) * 2.0).norm() < param.max_width) {
      if ((v_point.at(third_most_distant_index) * 2.0).norm() < param.max_length) {
        correction_vector = v_point.at(third_most_distant_index);
        if (correction_vector.x() == 0.0) {
          correction_vector.y() =
            std::max(std::abs(correction_vector.y()), param.avg_length / 2.0) *
              (correction_vector.y() < 0.0 ? -1.0 : 1.0) -
            correction_vector.y();
        } else if (correction_vector.y() == 0.0) {
          correction_vector.x() =
            std::max(std::abs(correction_vector.x()), param.avg_length / 2.0) *
              (correction_vector.x() < 0.0 ? -1.0 : 1.0) -
            correction_vector.x();
        }
      } else {
        return false;
      }
    } else if (  // NOLINT
      param.min_length < (v_point.at(first_most_distant_index) * 2.0).norm() &&
      (v_point.at(first_most_distant_index) * 2.0).norm() < param.max_length) {
      if ((v_point.at(third_most_distant_index) * 2.0).norm() < param.max_width) {
        correction_vector = v_point.at(third_most_distant_index);
        if (correction_vector.x() == 0.0) {
          correction_vector.y() = std::max(std::abs(correction_vector.y()), param.avg_width / 2.0) *
                                    (correction_vector.y() < 0.0 ? -1.0 : 1.0) -
                                  correction_vector.y();
        } else if (correction_vector.y() == 0.0) {
          correction_vector.x() = std::max(std::abs(correction_vector.x()), param.avg_width / 2.0) *
                                    (correction_vector.x() < 0.0 ? -1.0 : 1.0) -
                                  correction_vector.x();
        }
      } else {
        return false;
      }
    } else {
      return false;
    }
  }
  // fit width
  else if (  // NOLINT
    (param.min_width < (v_point.at(first_most_distant_index) * 2.0).norm() &&
     (v_point.at(first_most_distant_index) * 2.0).norm() < param.max_width) &&
    (param.min_width < (v_point.at(second_most_distant_index) * 2.0).norm() &&
     (v_point.at(second_most_distant_index) * 2.0).norm() <
       param.max_width))  // both of edge is within width threshold
  {
    correction_vector = v_point.at(first_most_distant_index);
    if (correction_vector.x() == 0.0) {
      correction_vector.y() = std::max(std::abs(correction_vector.y()), param.avg_length / 2.0) *
                                (correction_vector.y() < 0.0 ? -1.0 : 1.0) -
                              correction_vector.y();
    } else if (correction_vector.y() == 0.0) {
      correction_vector.x() = std::max(std::abs(correction_vector.x()), param.avg_length / 2.0) *
                                (correction_vector.x() < 0.0 ? -1.0 : 1.0) -
                              correction_vector.x();
    }
  } else if (  // NOLINT
    param.min_width < (v_point.at(first_most_distant_index) * 2.0).norm() &&
    (v_point.at(first_most_distant_index) * 2.0).norm() < param.max_width) {
    correction_vector = v_point.at(second_most_distant_index);
    if (correction_vector.x() == 0.0) {
      correction_vector.y() = std::max(std::abs(correction_vector.y()), param.avg_length / 2.0) *
                                (correction_vector.y() < 0.0 ? -1.0 : 1.0) -
                              correction_vector.y();
    } else if (correction_vector.y() == 0.0) {
      correction_vector.x() = std::max(std::abs(correction_vector.x()), param.avg_length / 2.0) *
                                (correction_vector.x() < 0.0 ? -1.0 : 1.0) -
                              correction_vector.x();
    }
  } else if (  // NOLINT
    param.min_width < (v_point.at(second_most_distant_index) * 2.0).norm() &&
    (v_point.at(second_most_distant_index) * 2.0).norm() < param.max_width) {
    correction_vector = v_point.at(first_most_distant_index);

    if (correction_vector.x() == 0.0) {
      correction_vector.y() = std::max(std::abs(correction_vector.y()), param.avg_length / 2.0) *
                                (correction_vector.y() < 0.0 ? -1.0 : 1.0) -
                              correction_vector.y();
    } else if (correction_vector.y() == 0.0) {
      correction_vector.x() = std::max(std::abs(correction_vector.x()), param.avg_length / 2.0) *
                                (correction_vector.x() < 0.0 ? -1.0 : 1.0) -
                              correction_vector.x();
    }
  }
  // fit length
  else if (  // NOLINT
    (param.min_length < (v_point.at(first_most_distant_index) * 2.0).norm() &&
     (v_point.at(first_most_distant_index) * 2.0).norm() < param.max_length) &&
    (v_point.at(second_most_distant_index) * 2.0).norm() < param.max_width) {
    correction_vector = v_point.at(second_most_distant_index);

    if (correction_vector.x() == 0.0) {
      correction_vector.y() = std::max(std::abs(correction_vector.y()), param.avg_width / 2.0) *
                                (correction_vector.y() < 0.0 ? -1.0 : 1.0) -
                              correction_vector.y();
    } else if (correction_vector.y() == 0.0) {
      correction_vector.x() = std::max(std::abs(correction_vector.x()), param.avg_width / 2.0) *
                                (correction_vector.x() < 0.0 ? -1.0 : 1.0) -
                              correction_vector.x();
    }
  } else if (  // NOLINT
    (param.min_length < (v_point.at(second_most_distant_index) * 2.0).norm() &&
     (v_point.at(second_most_distant_index) * 2.0).norm() < param.max_length) &&
    (v_point.at(first_most_distant_index) * 2.0).norm() < param.max_width) {
    correction_vector = v_point.at(first_most_distant_index);

    if (correction_vector.x() == 0.0) {
      correction_vector.y() = std::max(std::abs(correction_vector.y()), param.avg_width / 2.0) *
                                (correction_vector.y() < 0.0 ? -1.0 : 1.0) -
                              correction_vector.y();
    } else if (correction_vector.y() == 0.0) {
      correction_vector.x() = std::max(std::abs(correction_vector.x()), param.avg_width / 2.0) *
                                (correction_vector.x() < 0.0 ? -1.0 : 1.0) -
                              correction_vector.x();
    }
  } else {
    return false;
  }

  shape_output.dimensions.x += std::abs(correction_vector.x()) * 2.0;
  shape_output.dimensions.y += std::abs(correction_vector.y()) * 2.0;
  pose_output.position.x += (rotate.toRotationMatrix() * correction_vector).x();
  pose_output.position.y += (rotate.toRotationMatrix() * correction_vector).y();

  // correct to set long length is x, short length is y
  if (shape_output.dimensions.x < shape_output.dimensions.y) {
    geometry_msgs::msg::Vector3 rpy = tier4_autoware_utils::getRPY(pose_output.orientation);
    rpy.z = rpy.z + M_PI_2;
    pose_output.orientation = tier4_autoware_utils::createQuaternionFromRPY(rpy.x, rpy.y, rpy.z);
    double temp = shape_output.dimensions.x;
    shape_output.dimensions.x = shape_output.dimensions.y;
    shape_output.dimensions.y = temp;
  }

  return true;
}

bool correctVehicleBoundingBoxWithReferenceYaw(
  const CorrectionParameters & param, autoware_auto_perception_msgs::msg::Shape & shape_output,
  geometry_msgs::msg::Pose & pose_output)
{
  /*
    c1 is nearest point and other points are arranged like below
    c is center of bounding box
    width
    4---2
    |   |
    | c |length
    |   |
    3---1
   */

  Eigen::Vector3d c1, c2, c3, c4;

  Eigen::Affine3d base2obj_transform;
  tf2::fromMsg(pose_output, base2obj_transform);

  std::vector<Eigen::Vector3d> v_point;
  v_point.push_back(
    base2obj_transform *
    Eigen::Vector3d(shape_output.dimensions.x * 0.5, shape_output.dimensions.y * 0.5, 0.0));
  v_point.push_back(
    base2obj_transform *
    Eigen::Vector3d(-shape_output.dimensions.x * 0.5, shape_output.dimensions.y * 0.5, 0.0));
  v_point.push_back(
    base2obj_transform *
    Eigen::Vector3d(shape_output.dimensions.x * 0.5, -shape_output.dimensions.y * 0.5, 0.0));
  v_point.push_back(
    base2obj_transform *
    Eigen::Vector3d(-shape_output.dimensions.x * 0.5, -shape_output.dimensions.y * 0.5, 0.0));

  double distance = std::pow(24, 24);
  size_t nearest_idx = 0;
  for (size_t i = 0; i < v_point.size(); i++) {
    if (v_point.at(i).norm() < distance) {
      distance = v_point.at(i).norm();
      nearest_idx = i;
    }
  }

  c1 = v_point.at(nearest_idx);

  Eigen::Vector3d c = Eigen::Vector3d::Zero();
  Eigen::Vector3d local_c1 = base2obj_transform.inverse() * c1;
  Eigen::Vector3d radiation_vec = c - local_c1;

  double ex = radiation_vec.x();
  double ey = radiation_vec.y();
  Eigen::Vector3d e1 = (Eigen::Vector3d(0, -ey, 0) - local_c1).normalized();
  Eigen::Vector3d e2 = (Eigen::Vector3d(-ex, 0, 0) - local_c1).normalized();
  double length = 0;
  if (
    param.min_length < shape_output.dimensions.x && shape_output.dimensions.x < param.max_length) {
    length = shape_output.dimensions.x;
  } else {
    length = param.avg_length;
  }
  double width = 0;
  if (param.min_width < shape_output.dimensions.y && shape_output.dimensions.y < param.max_width) {
    width = shape_output.dimensions.y;
  } else {
    width = param.avg_width;
  }

  c2 = c1 + base2obj_transform.rotation() * (e1 * length);
  c3 = c1 + base2obj_transform.rotation() * (e2 * width);
  c4 = c1 + (c2 - c1) + (c3 - c1);

  shape_output.dimensions.x = (c2 - c1).norm();
  shape_output.dimensions.y = (c3 - c1).norm();
  Eigen::Vector3d new_centroid = c1 + ((c4 - c1) * 0.5);
  pose_output.position.x = new_centroid.x();
  pose_output.position.y = new_centroid.y();
  pose_output.position.z = new_centroid.z();

  return true;
}

}  // namespace utils

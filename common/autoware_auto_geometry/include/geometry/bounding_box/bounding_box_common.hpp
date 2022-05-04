// Copyright 2017-2019 the Autoware Foundation
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
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

/// \file
/// \brief Common functionality for bounding box computation algorithms

#ifndef GEOMETRY__BOUNDING_BOX__BOUNDING_BOX_COMMON_HPP_
#define GEOMETRY__BOUNDING_BOX__BOUNDING_BOX_COMMON_HPP_

#include <geometry/common_2d.hpp>
#include <geometry/visibility_control.hpp>

#include <autoware_auto_perception_msgs/msg/bounding_box.hpp>
#include <autoware_auto_perception_msgs/msg/detected_object.hpp>
#include <autoware_auto_perception_msgs/msg/shape.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <array>
#include <limits>
#include <vector>

namespace autoware
{
namespace common
{
namespace geometry
{
/// \brief Functions and types for generating enclosing bounding boxes around a set of points
namespace bounding_box
{
using BoundingBox = autoware_auto_perception_msgs::msg::BoundingBox;

/// \brief Computes height of bounding box given a full list of points
/// \param[in] begin The start of the list of points
/// \param[in] end One past the end of the list of points
/// \param[out] box A box for which the z component of centroid, corners, and size gets filled
/// \tparam IT An iterator type, must dereference into a point type with float member z, or
///            appropriate point adapter defined
template <typename IT>
void compute_height(const IT begin, const IT end, BoundingBox & box)
{
  float32_t max_z = -std::numeric_limits<float32_t>::max();
  float32_t min_z = std::numeric_limits<float32_t>::max();
  for (auto it = begin; it != end; ++it) {
    const float32_t z = point_adapter::z_(*it);
    if (z <= min_z) {
      min_z = z;
    }
    if (z >= max_z) {
      max_z = z;
    }
  }
  box.centroid.z = (max_z + min_z) * 0.5F;
  for (auto & corner : box.corners) {
    corner.z = box.centroid.z;
  }
  box.size.z = (max_z - min_z) * 0.5F;
}

/// \brief Computes height of bounding box given a full list of points
/// \param[in] begin Iterator pointing to the start of the range of points
/// \param[in] end Iterator pointing the the end of the range of points
/// \param[out] shape A shape in which vertices z values and height field will be set
/// \tparam IT An iterator type, must dereference into a point type with float member z, or
///            appropriate point adapter defined
template <typename IT>
void compute_height(const IT begin, const IT end, autoware_auto_perception_msgs::msg::Shape & shape)
{
  float32_t max_z = -std::numeric_limits<float32_t>::max();
  float32_t min_z = std::numeric_limits<float32_t>::max();
  for (auto it = begin; it != end; ++it) {
    const float32_t z = point_adapter::z_(*it);
    if (z <= min_z) {
      min_z = z;
    }
    if (z >= max_z) {
      max_z = z;
    }
  }
  for (auto & corner : shape.footprint.points) {
    corner.z = min_z;
  }
  shape.dimensions.z = max_z - min_z;
}

namespace details
{
/// Templated alias for getting a type without CV or reference qualification
template <typename T>
using base_type = typename std::remove_cv<typename std::remove_reference<T>::type>::type;

/// Templated alias for an array of 4 objects of type PointT
template <typename PointT>
using Point4 = std::array<PointT, 4U>;

/// \brief Helper struct for compile time introspection of array size from
/// stackoverflow.com/questions/16866033/getting-the-number-of-elements-in-stdarray-at-compile-time
template <typename>
struct array_size;
template <typename T, std::size_t N>
struct array_size<std::array<T, N>>
{
  static std::size_t const size = N;
};
static_assert(
  array_size<base_type<decltype(BoundingBox::corners)>>::size == 4U,
  "Bounding box does not have the right number of corners");

/// \brief Compute length, width, area of bounding box
/// \param[in] corners Corners of the current bounding box
/// \param[out] ret A point struct used to hold size of box defined by corners
void GEOMETRY_PUBLIC
size_2d(const decltype(BoundingBox::corners) & corners, geometry_msgs::msg::Point32 & ret);
/// \brief Computes centroid and orientation of a box given corners
/// \param[in] corners Array of final corners of bounding box
/// \param[out] box Message to have corners, orientation, and centroid updated
void GEOMETRY_PUBLIC
finalize_box(const decltype(BoundingBox::corners) & corners, BoundingBox & box);

/// \brief given support points and two orthogonal directions, compute corners of bounding box
/// \tparam PointT Type of a point, must have float members x and y`
/// \tparam IT An iterator dereferencable into PointT
/// \param[out] corners Gets filled with corner points of bounding box
/// \param[in] supports Iterators referring to support points of current bounding box
///                     e.g. bounding box is touching these points
/// \param[in] directions Directions of each edge of the bounding box from each support point
template <typename IT, typename PointT>
void compute_corners(
  decltype(BoundingBox::corners) & corners, const Point4<IT> & supports,
  const Point4<PointT> & directions)
{
  for (uint32_t idx = 0U; idx < corners.size(); ++idx) {
    const uint32_t jdx = (idx != 3U) ? (idx + 1U) : 0U;
    const auto pt =
      intersection_2d(*supports[idx], directions[idx], *supports[jdx], directions[jdx]);
    // TODO(c.ho) handle error
    point_adapter::xr_(corners[idx]) = point_adapter::x_(pt);
    point_adapter::yr_(corners[idx]) = point_adapter::y_(pt);
  }
}
// TODO(c.ho) type trait enum base

/// \brief Copy vertices of the given box into a Shape type
/// \param box Box to be converted
/// \return Shape type filled with box vertices
autoware_auto_perception_msgs::msg::Shape GEOMETRY_PUBLIC make_shape(const BoundingBox & box);

/// \brief Copy centroid and orientation info of the box into Pose type
/// \param box BoundingBox to be converted
/// \return Pose type filled with centroid and orientation from box
geometry_msgs::msg::Pose GEOMETRY_PUBLIC make_pose(const BoundingBox & box);

/// \brief Fill DetectedObject type with contents from a BoundingBox type
/// \param box BoundingBox to be converted
/// \return Filled DetectedObject type
autoware_auto_perception_msgs::msg::DetectedObject GEOMETRY_PUBLIC
make_detected_object(const BoundingBox & box);

/// \brief Transform corners from object-local coordinates using the given centroid and orientation
/// \param shape_msg Msg containing corners in object-local coordinates
/// \param centroid Centroid of the polygon whose corners are defined in shape_msg
/// \param orientation Orientation of the polygon
/// \return corners transformed such that their centroid and orientation correspond to the
///         given inputs
std::vector<geometry_msgs::msg::Point32> GEOMETRY_PUBLIC get_transformed_corners(
  const autoware_auto_perception_msgs::msg::Shape & shape_msg,
  const geometry_msgs::msg::Point & centroid, const geometry_msgs::msg::Quaternion & orientation);

}  // namespace details
}  // namespace bounding_box
}  // namespace geometry
}  // namespace common
}  // namespace autoware

#endif  // GEOMETRY__BOUNDING_BOX__BOUNDING_BOX_COMMON_HPP_

// Copyright 2020, The Autoware Foundation.
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
/// \file
/// \brief This file includes common transform functionality for autoware_auto_msgs

#ifndef AUTOWARE_AUTO_TF2__TF2_AUTOWARE_AUTO_MSGS_HPP_
#define AUTOWARE_AUTO_TF2__TF2_AUTOWARE_AUTO_MSGS_HPP_

#include <common/types.hpp>
#include <kdl/frames.hpp>

#include <autoware_auto_geometry_msgs/msg/quaternion32.hpp>
#include <autoware_auto_perception_msgs/msg/bounding_box.hpp>
#include <autoware_auto_perception_msgs/msg/bounding_box_array.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2/convert.h>
#include <tf2/time.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <string>

using autoware::common::types::float32_t;
using autoware::common::types::float64_t;
using BoundingBoxArray = autoware_auto_perception_msgs::msg::BoundingBoxArray;
using BoundingBox = autoware_auto_perception_msgs::msg::BoundingBox;

namespace tf2
{

/*************/
/** Point32 **/
/*************/

/** \brief Apply a geometry_msgs TransformStamped to a geometry_msgs Point32 type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The point to transform, as a Point32 message.
 * \param t_out The transformed point, as a Point32 message.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template <>
inline void doTransform(
  const geometry_msgs::msg::Point32 & t_in, geometry_msgs::msg::Point32 & t_out,
  const geometry_msgs::msg::TransformStamped & transform)
{
  const KDL::Vector v_out = gmTransformToKDL(transform) * KDL::Vector(t_in.x, t_in.y, t_in.z);
  t_out.x = static_cast<float>(v_out[0]);
  t_out.y = static_cast<float>(v_out[1]);
  t_out.z = static_cast<float>(v_out[2]);
}

/*************/
/** Polygon **/
/*************/

/** \brief Apply a geometry_msgs TransformStamped to a geometry_msgs Polygon type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The polygon to transform.
 * \param t_out The transformed polygon.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template <>
inline void doTransform(
  const geometry_msgs::msg::Polygon & t_in, geometry_msgs::msg::Polygon & t_out,
  const geometry_msgs::msg::TransformStamped & transform)
{
  // Don't call the Point32 doTransform to avoid doing this conversion every time
  const auto kdl_frame = gmTransformToKDL(transform);
  // We don't use std::back_inserter to allow aliasing between t_in and t_out
  t_out.points.resize(t_in.points.size());
  for (size_t i = 0; i < t_in.points.size(); ++i) {
    const KDL::Vector v_out =
      kdl_frame * KDL::Vector(t_in.points[i].x, t_in.points[i].y, t_in.points[i].z);
    t_out.points[i].x = static_cast<float>(v_out[0]);
    t_out.points[i].y = static_cast<float>(v_out[1]);
    t_out.points[i].z = static_cast<float>(v_out[2]);
  }
}

/******************/
/** Quaternion32 **/
/******************/

/** \brief Apply a geometry_msgs TransformStamped to an autoware_auto_msgs Quaternion32 type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The Quaternion32 message to transform.
 * \param t_out The transformed Quaternion32 message.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template <>
inline void doTransform(
  const autoware_auto_geometry_msgs::msg::Quaternion32 & t_in,
  autoware_auto_geometry_msgs::msg::Quaternion32 & t_out,
  const geometry_msgs::msg::TransformStamped & transform)
{
  KDL::Rotation r_in = KDL::Rotation::Quaternion(t_in.x, t_in.y, t_in.z, t_in.w);
  KDL::Rotation out = gmTransformToKDL(transform).M * r_in;

  double qx, qy, qz, qw;
  out.GetQuaternion(qx, qy, qz, qw);
  t_out.x = static_cast<float32_t>(qx);
  t_out.y = static_cast<float32_t>(qy);
  t_out.z = static_cast<float32_t>(qz);
  t_out.w = static_cast<float32_t>(qw);
}

/******************/
/** BoundingBox **/
/******************/

/** \brief Apply a geometry_msgs TransformStamped to an autoware_auto_msgs BoundingBox type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The BoundingBox message to transform.
 * \param t_out The transformed BoundingBox message.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template <>
inline void doTransform(
  const BoundingBox & t_in, BoundingBox & t_out,
  const geometry_msgs::msg::TransformStamped & transform)
{
  t_out = t_in;
  doTransform(t_in.orientation, t_out.orientation, transform);
  doTransform(t_in.centroid, t_out.centroid, transform);
  doTransform(t_in.corners[0], t_out.corners[0], transform);
  doTransform(t_in.corners[1], t_out.corners[1], transform);
  doTransform(t_in.corners[2], t_out.corners[2], transform);
  doTransform(t_in.corners[3], t_out.corners[3], transform);
  // TODO(jitrc): add conversion for other fields of BoundingBox, such as heading, variance, size
}

/**********************/
/** BoundingBoxArray **/
/**********************/

/** \brief Extract a timestamp from the header of a BoundingBoxArray message.
 * This function is a specialization of the getTimestamp template defined in tf2/convert.h.
 * \param t A timestamped BoundingBoxArray message to extract the timestamp from.
 * \return The timestamp of the message.
 */
template <>
inline tf2::TimePoint getTimestamp(const BoundingBoxArray & t)
{
  return tf2_ros::fromMsg(t.header.stamp);
}

/** \brief Extract a frame ID from the header of a BoundingBoxArray message.
 * This function is a specialization of the getFrameId template defined in tf2/convert.h.
 * \param t A timestamped BoundingBoxArray message to extract the frame ID from.
 * \return A string containing the frame ID of the message.
 */
template <>
inline std::string getFrameId(const BoundingBoxArray & t)
{
  return t.header.frame_id;
}

/** \brief Apply a geometry_msgs TransformStamped to an autoware_auto_msgs BoundingBoxArray type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The BoundingBoxArray to transform, as a timestamped BoundingBoxArray message.
 * \param t_out The transformed BoundingBoxArray, as a timestamped BoundingBoxArray message.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template <>
inline void doTransform(
  const BoundingBoxArray & t_in, BoundingBoxArray & t_out,
  const geometry_msgs::msg::TransformStamped & transform)
{
  t_out = t_in;
  for (auto idx = 0U; idx < t_in.boxes.size(); idx++) {
    doTransform(t_out.boxes[idx], t_out.boxes[idx], transform);
  }
  t_out.header.stamp = transform.header.stamp;
  t_out.header.frame_id = transform.header.frame_id;
}

}  // namespace tf2

#endif  // AUTOWARE_AUTO_TF2__TF2_AUTOWARE_AUTO_MSGS_HPP_

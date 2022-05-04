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
/// \brief This file implements 2D pca on a linked list of points to estimate an oriented
///        bounding box

#ifndef GEOMETRY__BOUNDING_BOX__EIGENBOX_2D_HPP_
#define GEOMETRY__BOUNDING_BOX__EIGENBOX_2D_HPP_

#include <geometry/bounding_box/bounding_box_common.hpp>

#include <limits>
#include <utility>

namespace autoware
{
namespace common
{
namespace geometry
{
namespace bounding_box
{
namespace details
{

/// \brief Simplified 2d covariance matrix
struct Covariance2d
{
  /// \brief Variance in the x direction
  float32_t xx;
  /// \brief Variance in the y direction
  float32_t yy;
  /// \brief x-y covariance
  float32_t xy;
  /// \brief Number of points
  std::size_t num_points;
};  // struct Covariance2d

/// \brief Compute 2d covariance matrix of a list of points using Welford's online algorithm
/// \param[in] begin An iterator pointing to the first point in a point list
/// \param[in] end An iterator pointing to one past the last point in the point list
/// \tparam IT An iterator type dereferencable into a point with float members x and y
/// \return A 2d covariance matrix for all points inthe list
template <typename IT>
Covariance2d covariance_2d(const IT begin, const IT end)
{
  Covariance2d ret{0.0F, 0.0F, 0.0F, 0U};
  float32_t ux = 0.0F;
  float32_t uy = 0.0F;
  float32_t num_points = 0.0F;
  using point_adapter::x_;
  using point_adapter::y_;
  for (auto it = begin; it != end; ++it) {
    ++ret.num_points;
    num_points = static_cast<float32_t>(ret.num_points);
    const auto & pt = *it;
    // update mean x
    const float32_t dx = x_(pt) - ux;
    ux = ux + (dx / num_points);
    // update cov
    const float32_t dy = y_(pt) - uy;
    ret.xy += (x_(pt) - ux) * (dy);
    // update mean y
    uy = uy + (dy / num_points);
    // update M2
    ret.xx += dx * (x_(pt) - ux);
    ret.yy += dy * (y_(pt) - uy);
  }
  // finalize sample (co-)variance
  if (ret.num_points > 1U) {
    num_points = num_points - 1.0F;
  }
  ret.xx /= num_points;
  ret.yy /= num_points;
  ret.xy /= num_points;

  return ret;
}

/// \brief Compute eigenvectors and eigenvalues
/// \param[in] cov 2d Covariance matrix
/// \param[out] eigvec1 First eigenvector
/// \param[out] eigvec2 Second eigenvector
/// \tparam PointT Point type that has at least float members x and y
/// \return A pairt of eigenvalues: The first is the larger eigenvalue
/// \throw std::runtime error if you would get degenerate covariance
template <typename PointT>
std::pair<float32_t, float32_t> eig_2d(const Covariance2d & cov, PointT & eigvec1, PointT & eigvec2)
{
  const float32_t tr_2 = (cov.xx + cov.yy) * 0.5F;
  const float32_t det = (cov.xx * cov.yy) - (cov.xy * cov.xy);
  // Add a small fudge to alleviate floating point errors
  float32_t disc = ((tr_2 * tr_2) - det) + std::numeric_limits<float32_t>::epsilon();
  if (disc < 0.0F) {
    throw std::runtime_error(
      "pca_2d: negative discriminant! Should never happen for well formed "
      "covariance matrix");
  }
  disc = sqrtf(disc);
  // compute eigenvalues
  const auto ret = std::make_pair(tr_2 + disc, tr_2 - disc);
  // compute eigenvectors
  using point_adapter::xr_;
  using point_adapter::yr_;
  // We compare squared value against floating epsilon to make sure that eigen vectors
  // are persistent against further calculations.
  // (e.g. taking cross product of two eigen vectors)
  if (fabsf(cov.xy * cov.xy) > std::numeric_limits<float32_t>::epsilon()) {
    xr_(eigvec1) = cov.xy;
    yr_(eigvec1) = ret.first - cov.xx;
    xr_(eigvec2) = cov.xy;
    yr_(eigvec2) = ret.second - cov.xx;
  } else {
    if (cov.xx > cov.yy) {
      xr_(eigvec1) = 1.0F;
      yr_(eigvec1) = 0.0F;
      xr_(eigvec2) = 0.0F;
      yr_(eigvec2) = 1.0F;
    } else {
      xr_(eigvec1) = 0.0F;
      yr_(eigvec1) = 1.0F;
      xr_(eigvec2) = 1.0F;
      yr_(eigvec2) = 0.0F;
    }
  }
  return ret;
}

/// \brief Given eigenvectors, compute support (furthest) point in each direction
/// \tparam IT An iterator type dereferencable into a point with float members x and y
/// \tparam PointT type of a point with float members x and y
/// \param[in] begin An iterator pointing to the first point in a point list
/// \param[in] end An iterator pointing to one past the last point in the point list
/// \param[in] eig1 First principal component of cluster
/// \param[in] eig2 Second principal component of cluster
/// \param[out] support Array to get filled with extreme points in each of the principal
///                     components
/// \return whether or not eig2 is ccw wrt eig1
template <typename IT, typename PointT>
bool8_t compute_supports(
  const IT begin, const IT end, const PointT & eig1, const PointT & eig2, Point4<IT> & support)
{
  const bool8_t ret = cross_2d(eig1, eig2) >= 0.0F;
  std::array<float32_t, 4U> metrics{
    {-std::numeric_limits<float32_t>::max(), -std::numeric_limits<float32_t>::max(),
     std::numeric_limits<float32_t>::max(), std::numeric_limits<float32_t>::max()}};
  for (auto it = begin; it != end; ++it) {
    const PointT & pt = *it;
    float32_t val = dot_2d(ret ? eig1 : eig2, pt);
    if (val > metrics[0U]) {
      metrics[0U] = val;
      support[0U] = it;
    }
    if (val < metrics[2U]) {
      metrics[2U] = val;
      support[2U] = it;
    }
    val = dot_2d(ret ? eig2 : eig1, pt);
    if (val > metrics[1U]) {
      metrics[1U] = val;
      support[1U] = it;
    }
    if (val < metrics[3U]) {
      metrics[3U] = val;
      support[3U] = it;
    }
  }
  return ret;
}

/// \brief Compute bounding box given a pair of basis directions
/// \tparam IT An iterator type dereferencable into a point with float members x and y
/// \tparam PointT Point type of the lists, must have float members x and y
/// \param[in] ax1 First basis direction, assumed to be normal to ax2
/// \param[in] ax2 Second basis direction, assumed to be normal to ax1, assumed to be ccw wrt ax1
/// \param[in] supports Array of iterators referring to points that are most extreme in each basis
///                     direction. Should be result of compute_supports function
/// \return A bounding box based on the basis axes and the support points
template <typename IT, typename PointT>
BoundingBox compute_bounding_box(
  const PointT & ax1, const PointT & ax2, const Point4<IT> & supports)
{
  decltype(BoundingBox::corners) corners;
  const Point4<PointT> directions{{ax1, ax2, minus_2d(ax1), minus_2d(ax2)}};
  compute_corners(corners, supports, directions);

  // build box
  BoundingBox bbox;
  finalize_box(corners, bbox);
  size_2d(corners, bbox.size);
  return bbox;
}
}  // namespace details

/// \brief Compute oriented bounding box using PCA. This uses all points in a list, and does not
///        modify the list. The resulting bounding box is not necessarily minimum in any way
/// \param[in] begin An iterator pointing to the first point in a point list
/// \param[in] end An iterator pointing to one past the last point in the point list
/// \tparam IT An iterator type dereferencable into a point with float members x and y
/// \return An oriented bounding box in x-y. This bounding box has no height information
template <typename IT>
BoundingBox eigenbox_2d(const IT begin, const IT end)
{
  // compute covariance
  const details::Covariance2d cov = details::covariance_2d(begin, end);

  // compute eigenvectors
  using PointT = details::base_type<decltype(*begin)>;
  PointT eig1;
  PointT eig2;
  const auto eigv = details::eig_2d(cov, eig1, eig2);

  // find extreme points
  details::Point4<IT> supports;
  const bool8_t is_ccw = details::compute_supports(begin, end, eig1, eig2, supports);
  // build box
  if (is_ccw) {
    std::swap(eig1, eig2);
  }
  BoundingBox bbox = details::compute_bounding_box(eig1, eig2, supports);
  bbox.value = eigv.first;

  return bbox;
}
}  // namespace bounding_box
}  // namespace geometry
}  // namespace common
}  // namespace autoware

#endif  // GEOMETRY__BOUNDING_BOX__EIGENBOX_2D_HPP_

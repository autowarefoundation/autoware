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

#ifndef GEOMETRY__BOUNDING_BOX__LFIT_HPP_
#define GEOMETRY__BOUNDING_BOX__LFIT_HPP_

#include <geometry/bounding_box/eigenbox_2d.hpp>

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

/// \brief A representation of the M matrix for the L-fit algorithm
struct LFitWs
{
  /// \brief Number of points in the first partition
  std::size_t p;
  /// \brief Number of points in the second partition
  std::size_t q;
  // assume matrix of form: [a b; c d]
  /// \brief Sum of x values in first partition
  float32_t m12a;
  /// \brief Sum of y values in first partition
  float32_t m12b;
  /// \brief Sum of y values in second partition
  float32_t m12c;
  /// \brief Negative sum of x values in second partition
  float32_t m12d;
  // m22 is a symmetric matrix
  /// \brief Sum_p x_2 + sum_q y_2
  float32_t m22a;
  /// \brief Sum_p x*y - sum_q x*y
  float32_t m22b;
  /// \brief Sum_p y_2 + sum_x y_2
  float32_t m22d;
};  // struct LFitWs

/// \brief Initialize M matrix for L-fit algorithm
/// \param[in] begin An iterator pointing to the first point in a point list
/// \param[in] end An iterator one past the last point in the point list
/// \param[in] size The number of points in the point list
/// \param[out] ws A representation of the M matrix to get initialized
/// \tparam IT The iterator type, should dereference to a point type with float members x and y
template <typename IT>
void init_lfit_ws(const IT begin, const IT end, const std::size_t size, LFitWs & ws)
{
  ws.p = 1UL;
  ws.q = size - 1UL;
  // init P terms (first partition)
  using point_adapter::x_;
  using point_adapter::y_;
  const auto & pt = *begin;
  const float32_t px = x_(pt);
  const float32_t py = y_(pt);
  // assume matrix of form: [a b; c d]
  ws.m12a = px;
  ws.m12b = py;
  ws.m12c = 0.0F;
  ws.m12d = 0.0F;
  // m22 is a symmetric matrix
  ws.m22a = px * px;
  ws.m22b = px * py;
  ws.m22d = py * py;
  auto it = begin;
  ++it;
  for (; it != end; ++it) {
    const auto & qt = *it;
    const float32_t qx = x_(qt);
    const float32_t qy = y_(qt);
    ws.m12c += qy;
    ws.m12d -= qx;
    ws.m22a += qy * qy;
    ws.m22b -= qx * qy;
    ws.m22d += qx * qx;
  }
}

/// \brief Solves the L fit problem for a given M matrix
/// \tparam PointT The point type of the cluster being L-fitted
/// \param[in] ws A representation of the M Matrix
/// \param[out] dir The best fit direction for this partition/M matrix
/// \return The L2 residual for this fit (the score, lower is better)
template <typename PointT>
float32_t solve_lfit(const LFitWs & ws, PointT & dir)
{
  const float32_t pi = 1.0F / static_cast<float32_t>(ws.p);
  const float32_t qi = 1.0F / static_cast<float32_t>(ws.q);
  const Covariance2d M{// matrix of form [x, z; z y]
                       ws.m22a - (((ws.m12a * ws.m12a) * pi) + ((ws.m12c * ws.m12c) * qi)),
                       ws.m22d - (((ws.m12b * ws.m12b) * pi) + ((ws.m12d * ws.m12d) * qi)),
                       ws.m22b - (((ws.m12a * ws.m12b) * pi) + ((ws.m12c * ws.m12d) * qi)), 0UL};
  PointT eig1;
  const auto eigv = eig_2d(M, eig1, dir);
  return eigv.second;
}

/// \brief Increments L fit M matrix with information in the point
/// \tparam PointT The point type
/// \param[in] pt The point to increment the M matrix with
/// \param[inout] ws A representation of the M matrix
template <typename PointT>
void increment_lfit_ws(const PointT & pt, LFitWs & ws)
{
  const float32_t px = point_adapter::x_(pt);
  const float32_t py = point_adapter::y_(pt);
  ws.m12a += px;
  ws.m12b += py;
  ws.m12c -= py;
  ws.m12d += px;
  ws.m22b += 2.0F * px * py;
  const float32_t px2y2 = (px - py) * (px + py);
  ws.m22a += px2y2;
  ws.m22d -= px2y2;
}

/// \tparam IT An iterator type that should dereference into a point type with float members x and y
template <typename PointT>
class LFitCompare
{
public:
  /// \brief Constructor, initializes normal direction
  /// \param[in] hint A 2d vector with the direction that will be used to order the
  ///                 point list
  explicit LFitCompare(const PointT & hint)
  : m_nx(point_adapter::x_(hint)), m_ny(point_adapter::y_(hint))
  {
  }

  /// \brief Comparator operation, returns true if the projection of a the tangent line
  ///        is smaller than the projection of b
  /// \param[in] p The first point for comparison
  /// \param[in] q The second point for comparison
  /// \return True if a has a smaller projection than b on the tangent line
  bool8_t operator()(const PointT & p, const PointT & q) const
  {
    using point_adapter::x_;
    using point_adapter::y_;
    return ((x_(p) * m_nx) + (y_(p) * m_ny)) < ((x_(q) * m_nx) + (y_(q) * m_ny));
  }

private:
  const float32_t m_nx;
  const float32_t m_ny;
};  // class LFitCompare

/// \brief The main implementation of L-fitting a bounding box to a list of points.
/// Assumes sufficiently valid, large enough, and appropriately ordered point list
/// \param[in] begin An iterator pointing to the first point in a point list
/// \param[in] end An iterator pointing to one past the last point in the point list
/// \param[in] size The number of points in the point list
/// \return A bounding box that minimizes the LFit residual
/// \tparam IT An iterator type dereferencable into a point with float members x and y
template <typename IT>
BoundingBox lfit_bounding_box_2d_impl(const IT begin, const IT end, const std::size_t size)
{
  // initialize M
  LFitWs ws{};
  init_lfit_ws(begin, end, size, ws);
  // solve initial problem
  details::base_type<decltype(*begin)> best_normal;
  float32_t min_eig = solve_lfit(ws, best_normal);
  // solve subsequent problems
  auto it = begin;
  ++it;
  for (; it != end; ++it) {
    // update M
    ws.p += 1U;
    ws.q -= 1U;
    if (ws.q == 0U) {  // checks for q = 0 case
      break;
    }
    increment_lfit_ws(*it, ws);
    // solve incremented problem
    decltype(best_normal) dir;
    const float32_t score = solve_lfit(ws, dir);
    // update optima
    if (score < min_eig) {
      min_eig = score;
      best_normal = dir;
    }
  }
  // can recover best corner point, but don't care, need to cover all points
  const auto inorm = 1.0F / norm_2d(best_normal);
  if (!std::isnormal(inorm)) {
    throw std::runtime_error{"LFit: Abnormal norm"};
  }
  best_normal = times_2d(best_normal, inorm);
  auto best_tangent = get_normal(best_normal);
  // find extreme points
  Point4<IT> supports;
  const bool8_t is_ccw = details::compute_supports(begin, end, best_normal, best_tangent, supports);
  if (is_ccw) {
    std::swap(best_normal, best_tangent);
  }
  BoundingBox bbox = details::compute_bounding_box(best_normal, best_tangent, supports);
  bbox.value = min_eig;

  return bbox;
}
}  // namespace details

/// \brief Compute bounding box which best fits an L-shaped cluster. Uses the method proposed
///        in "Efficient L-shape fitting of laser scanner data for vehicle pose estimation"
/// \return An oriented bounding box in x-y. This bounding box has no height information
/// \param[in] begin An iterator pointing to the first point in a point list
/// \param[in] end An iterator pointing to one past the last point in the point list
/// \param[in] size The number of points in the point list
/// \param[in] hint An iterator pointing to the point whose normal will be used to sort the list
/// \return A pair where the first element is an iterator pointing to the nearest point, and the
///         second element is the size of the list
/// \tparam IT An iterator type dereferencable into a point with float members x and y
/// \throw std::domain_error If the number of points is too few
template <typename IT, typename PointT>
BoundingBox lfit_bounding_box_2d(
  const IT begin, const IT end, const PointT hint, const std::size_t size)
{
  if (size <= 1U) {
    throw std::domain_error("LFit requires >= 2 points!");
  }
  // NOTE: assumes points are in base_link/sensor frame!
  // sort along tangent wrt sensor origin
  // lint -e522 NOLINT Not a pure function: data structure iterators are pointing to is modified
  std::partial_sort(begin, end, end, details::LFitCompare<PointT>{hint});

  return details::lfit_bounding_box_2d_impl(begin, end, size);
}

/// \brief Compute bounding box which best fits an L-shaped cluster. Uses the method proposed
///        in "Efficient L-shape fitting of laser scanner data for vehicle pose estimation".
/// This implementation sorts the list using std::sort
/// \return An oriented bounding box in x-y. This bounding box has no height information
/// \param[in] begin An iterator pointing to the first point in a point list
/// \param[in] end An iterator pointing to one past the last point in the point list
/// \tparam IT An iterator type dereferencable into a point with float members x and y
template <typename IT>
BoundingBox lfit_bounding_box_2d(const IT begin, const IT end)
{
  // use the principal component as the sorting direction
  const auto cov = details::covariance_2d(begin, end);
  using PointT = details::base_type<decltype(*begin)>;
  PointT eig1;
  PointT eig2;
  (void)details::eig_2d(cov, eig1, eig2);
  (void)eig2;
  return lfit_bounding_box_2d(begin, end, eig1, cov.num_points);
}
}  // namespace bounding_box
}  // namespace geometry
}  // namespace common
}  // namespace autoware

#endif  // GEOMETRY__BOUNDING_BOX__LFIT_HPP_

// Copyright 2019 the Autoware Foundation
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
/// \brief This file implements a spatial hash for efficient fixed-radius near neighbor queries in
///        2D

#ifndef GEOMETRY__SPATIAL_HASH_CONFIG_HPP_
#define GEOMETRY__SPATIAL_HASH_CONFIG_HPP_

#include "helper_functions/crtp.hpp"

#include <common/types.hpp>
#include <geometry/common_2d.hpp>
#include <geometry/visibility_control.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;
using autoware::common::types::float64_t;

namespace autoware
{
namespace common
{
namespace geometry
{
/// \brief All objects related to the spatial hash data structure for efficient near neighbor lookup
namespace spatial_hash
{
/// \brief Index type for identifying bins of the hash/lattice
using Index = std::size_t;
/// \brief Spatial hash functionality not intended to be used by an external user
namespace details
{
/// \brief Internal struct for packing three indices together
///
/// The use of this struct publically is a violation of our coding standards, but I claim it's
/// fine because (a) it's details, (b) it is literally three unrelated members packaged together.
/// This type is needed for conceptual convenience so I don't have massive function parameter
/// lists
struct GEOMETRY_PUBLIC Index3
{
  Index x;
  Index y;
  Index z;
};  // struct Index3

using BinRange = std::pair<Index3, Index3>;
}  // namespace details

/// \brief The base class for the configuration object for the SpatialHash class
/// \tparam Derived The type of the derived class to support static polymorphism/CRTP
template <typename Derived>
class GEOMETRY_PUBLIC Config : public autoware::common::helper_functions::crtp<Derived>
{
public:
  /// \brief Constructor for spatial hash
  /// \param[in] min_x The minimum x value for the spatial hash
  /// \param[in] max_x The maximum x value for the spatial hash
  /// \param[in] min_y The minimum y value for the spatial hash
  /// \param[in] max_y The maximum y value for the spatial hash
  /// \param[in] min_z The minimum z value for the spatial hash
  /// \param[in] max_z The maximum z value for the spatial hash
  /// \param[in] radius The look up radius
  /// \param[in] capacity The maximum number of points the spatial hash can store
  Config(
    const float32_t min_x, const float32_t max_x, const float32_t min_y, const float32_t max_y,
    const float32_t min_z, const float32_t max_z, const float32_t radius, const Index capacity)
  : m_min_x{min_x},
    m_min_y{min_y},
    m_min_z{min_z},
    m_max_x{max_x},
    m_max_y{max_y},
    m_max_z{max_z},
    m_side_length{radius},
    m_side_length2{radius * radius},
    m_side_length_inv{1.0F / radius},
    m_capacity{capacity},
    m_max_x_idx{check_basis_direction(min_x, max_x)},
    m_max_y_idx{check_basis_direction(min_y, max_y)},
    m_max_z_idx{check_basis_direction(min_z, max_z)},
    m_y_stride{m_max_x_idx + 1U},
    m_z_stride{m_y_stride * (m_max_y_idx + 1U)}
  {
    if (radius <= 0.0F) {
      throw std::domain_error("Error constructing SpatialHash: must have positive side length");
    }

    if ((m_max_y_idx + m_y_stride) > std::numeric_limits<Index>::max() / 2U) {
      // TODO(c.ho) properly check for multiplication overflow
      throw std::domain_error("SpatialHash::Config: voxel index may overflow!");
    }
    // small fudging to prevent weird boundary effects
    // (e.g (x=xmax, y) rolls index over to (x=0, y+1)
    constexpr auto FEPS = std::numeric_limits<float32_t>::epsilon();
    // lint -e{1938} read only access is fine NOLINT
    m_max_x -= FEPS;
    m_max_y -= FEPS;
    m_max_z -= FEPS;
    if ((m_z_stride + m_max_z_idx) > std::numeric_limits<Index>::max() / 2U) {
      // TODO(c.ho) properly check for multiplication overflow
      throw std::domain_error("SpatialHash::Config: voxel index may overflow!");
    }
    // I don't know if this is even possible given other checks
    if (std::numeric_limits<Index>::max() == m_max_z_idx) {
      throw std::domain_error("SpatialHash::Config: max z index exceeds reasonable value");
    }
  }

  /// \brief Given a reference index triple, compute the first and last bin
  /// \param[in] ref The reference index triple
  /// \param[in] radius The allowable radius for any point in the reference bin to any point in the
  ///                   range
  /// \return A pair where the first element is an index triple of the first bin, and the second
  ///         element is an index triple for the last bin
  details::BinRange bin_range(const details::Index3 & ref, const float radius) const
  {
    // Compute distance in units of voxels
    const Index iradius = static_cast<Index>(std::ceil(radius / m_side_length));
    // Dumb ternary because potentially unsigned Index type
    const Index xmin = (ref.x > iradius) ? (ref.x - iradius) : 0U;
    const Index ymin = (ref.y > iradius) ? (ref.y - iradius) : 0U;
    const Index zmin = (ref.z > iradius) ? (ref.z - iradius) : 0U;
    // In 2D mode, m_max_z should be 0, same with ref.z
    const Index xmax = std::min(ref.x + iradius, m_max_x_idx);
    const Index ymax = std::min(ref.y + iradius, m_max_y_idx);
    const Index zmax = std::min(ref.z + iradius, m_max_z_idx);
    // return bottom-left portion of cube and top-right portion of cube
    return {{xmin, ymin, zmin}, {xmax, ymax, zmax}};
  }

  /// \brief Get next index within a given range
  /// \return True if idx is valid and still in range
  /// \param[in] range The max and min bin indices
  /// \param[inout] idx The index to be incremented, updated even if a negative result occurs
  bool8_t next_bin(const details::BinRange & range, details::Index3 & idx) const
  {
    // TODO(c.ho) is there any way to make this neater without triple nested if?
    bool8_t ret = true;
    ++idx.x;
    if (idx.x > range.second.x) {
      idx.x = range.first.x;
      ++idx.y;
      if (idx.y > range.second.y) {
        idx.y = range.first.y;
        ++idx.z;
        if (idx.z > range.second.z) {
          ret = false;
        }
      }
    }
    return ret;
  }
  /// \brief Get the maximum capacity of the spatial hash
  /// \return The capacity
  Index get_capacity() const { return m_capacity; }

  /// \brief Getter for the side length, equivalently the lookup radius
  float32_t radius2() const { return m_side_length2; }

  ////////////////////////////////////////////////////////////////////////////////////////////
  // "Polymorphic" API
  /// \brief Compute the single index given a point
  /// \param[in] x The x component of the point
  /// \param[in] y The y component of the point
  /// \param[in] z The z component of the point
  /// \return The combined index of the bin for a given point
  Index bin(const float32_t x, const float32_t y, const float32_t z) const
  {
    return this->impl().bin_(x, y, z);
  }
  /// \brief Compute whether the query bin and reference bin could possibly contain a pair of points
  ///        such that their distance is within a certain threshold
  /// \param[in] ref The index triple of the bin containing the reference point
  /// \param[in] query The index triple of the bin being queried
  /// \param[in] ref_distance2 The squared threshold distance
  /// \return True if query and ref could possibly hold points within reference distance to one
  ///         another
  bool is_candidate_bin(
    const details::Index3 & ref, const details::Index3 & query, const float ref_distance2) const
  {
    return this->impl().valid(ref, query, ref_distance2);
  }
  /// \brief Compute the decomposed index given a point
  /// \param[in] x The x component of the point
  /// \param[in] y The y component of the point
  /// \param[in] z The z component of the point
  /// \return The decomposed index triple of the bin for the given point
  details::Index3 index3(const float32_t x, const float32_t y, const float32_t z) const
  {
    return this->impl().index3_(x, y, z);
  }
  /// \brief Compute the composed single index given a decomposed index
  /// \param[in] idx A decomposed index triple for a bin
  /// \return The composed bin index
  Index index(const details::Index3 & idx) const { return this->impl().index_(idx); }
  /// \brief Compute the squared distance between the two points
  /// \tparam PointT A point type with float members x, y and z, or point adapters defined
  /// \param[in] x The x component of the first point
  /// \param[in] y The y component of the first point
  /// \param[in] z The z component of the first point
  /// \param[in] pt The other point being compared
  /// \return The squared distance between the points (2d or 3d)
  template <typename PointT>
  float32_t distance_squared(
    const float32_t x, const float32_t y, const float32_t z, const PointT & pt) const
  {
    return this->impl().distance_squared_(x, y, z, pt);
  }

protected:
  /// \brief Computes the index in the x basis direction
  /// \param[in] x The x value of a point
  /// \return The x offset of the index
  Index x_index(const float32_t x) const
  {
    return static_cast<Index>(
      std::floor((std::min(std::max(x, m_min_x), m_max_x) - m_min_x) * m_side_length_inv));
  }
  /// \brief Computes the index in the y basis direction
  /// \param[in] y The y value of a point
  /// \return The x offset of the index
  Index y_index(const float32_t y) const
  {
    return static_cast<Index>(
      std::floor((std::min(std::max(y, m_min_y), m_max_y) - m_min_y) * m_side_length_inv));
  }
  /// \brief Computes the index in the z basis direction
  /// \param[in] z The z value of a point
  /// \return The x offset of the index
  Index z_index(const float32_t z) const
  {
    return static_cast<Index>(
      std::floor((std::min(std::max(z, m_min_z), m_max_z) - m_min_z) * m_side_length_inv));
  }
  /// \brief Compose the provided index offsets
  Index bin_impl(const Index xdx, const Index ydx) const { return xdx + (ydx * m_y_stride); }
  /// \brief Compose the provided index offsets
  Index bin_impl(const Index xdx, const Index ydx, const Index zdx) const
  {
    return bin_impl(xdx, ydx) + (zdx * m_z_stride);
  }

  /// \brief The index offset of a point given it's x and y values
  /// \param[in] x The x value of a point
  /// \param[in] y the y value of a point
  /// \return The index of the point in the 2D case, or the offset within a z-slice of the hash
  Index bin_impl(const float32_t x, const float32_t y) const
  {
    return bin_impl(x_index(x), y_index(y));
  }
  /// \brief The index of a point given it's x, y and z values
  /// \param[in] x The x value of a point
  /// \param[in] y the y value of a point
  /// \param[in] z the z value of a point
  /// \return The index of the bin for the specified point
  Index bin_impl(const float32_t x, const float32_t y, const float32_t z) const
  {
    return bin_impl(x, y) + (z_index(z) * m_z_stride);
  }
  /// \brief The distance between two indices as a float, where adjacent indices have zero
  ///        distance (e.g. dist(0, 1) = 0)
  float32_t idx_distance(const Index ref_idx, const Index query_idx) const
  {
    /// Not using fabs because Index is (possibly) unsigned
    const Index idist = (ref_idx >= query_idx) ? (ref_idx - query_idx) : (query_idx - ref_idx);
    float32_t dist = static_cast<float32_t>(idist) - 1.0F;
    return std::max(dist, 0.0F);
  }

  /// \brief Get side length squared
  float side_length2() const { return m_side_length2; }

private:
  /// \brief Sanity check a range in a basis direction
  /// \return The number of voxels in the given basis direction
  Index check_basis_direction(const float32_t min, const float32_t max) const
  {
    if (min >= max) {
      throw std::domain_error("SpatialHash::Config: must have min < max");
    }
    // This family of checks is to ensure that you don't get weird casting effects due to huge
    // floating point values
    const float64_t dmax = static_cast<float64_t>(max);
    const float64_t dmin = static_cast<float64_t>(min);
    const float64_t width = (dmax - dmin) * static_cast<float64_t>(m_side_length_inv);
    constexpr float64_t fltmax = static_cast<float64_t>(std::numeric_limits<float32_t>::max());
    if (fltmax <= width) {
      throw std::domain_error("SpatialHash::Config: voxel size approaching floating point limit");
    }
    return static_cast<Index>(width);
  }
  float32_t m_min_x;
  float32_t m_min_y;
  float32_t m_min_z;
  float32_t m_max_x;
  float32_t m_max_y;
  float32_t m_max_z;
  float32_t m_side_length;
  float32_t m_side_length2;
  float32_t m_side_length_inv;
  Index m_capacity;
  Index m_max_x_idx;
  Index m_max_y_idx;
  Index m_max_z_idx;
  Index m_y_stride;
  Index m_z_stride;
};  // class Config

/// \brief Configuration class for a 2d spatial hash
class GEOMETRY_PUBLIC Config2d : public Config<Config2d>
{
public:
  /// \brief Config constructor for 2D spatial hash
  /// \param[in] min_x The minimum x value for the spatial hash
  /// \param[in] max_x The maximum x value for the spatial hash
  /// \param[in] min_y The minimum y value for the spatial hash
  /// \param[in] max_y The maximum y value for the spatial hash
  /// \param[in] radius The lookup distance
  /// \param[in] capacity The maximum number of points the spatial hash can store
  Config2d(
    const float32_t min_x, const float32_t max_x, const float32_t min_y, const float32_t max_y,
    const float32_t radius, const Index capacity);
  /// \brief The index of a point given it's x, y and z values, 2d implementation
  /// \param[in] x The x value of a point
  /// \param[in] y the y value of a point
  /// \param[in] z the z value of a point
  /// \return The index of the bin for the specified point
  Index bin_(const float32_t x, const float32_t y, const float32_t z) const;
  /// \brief Determine if a bin could possibly hold a point within a distance to any point in a
  ///        reference bin for the 2D case
  /// \param[in] ref The decomposed index triple of the reference bin
  /// \param[in] query The decomposed index triple of the bin being queried
  /// \param[in] ref_distance2 The squared threshold distance
  /// \return True if the reference bin and query bin could possibly hold a point within the
  ///         reference distance
  bool valid(
    const details::Index3 & ref, const details::Index3 & query, const float ref_distance2) const;
  /// \brief Compute the decomposed index given a point, 2d implementation
  /// \param[in] x The x component of the point
  /// \param[in] y The y component of the point
  /// \param[in] z The z component of the point
  /// \return The decomposed index triple of the bin for the given point
  details::Index3 index3_(const float32_t x, const float32_t y, const float32_t z) const;
  /// \brief Compute the composed single index given a decomposed index, 2d implementation
  /// \param[in] idx A decomposed index triple for a bin
  /// \return The composed bin index
  Index index_(const details::Index3 & idx) const;
  /// \brief Compute the squared distance between the two points, 2d implementation
  /// \tparam PointT A point type with float members x, y and z, or point adapters defined
  /// \param[in] x The x component of the first point
  /// \param[in] y The y component of the first point
  /// \param[in] z The z component of the first point
  /// \param[in] pt The other point being compared
  /// \return The squared distance between the points (2d)
  template <typename PointT>
  float32_t distance_squared_(
    const float32_t x, const float32_t y, const float32_t z, const PointT & pt) const
  {
    (void)z;
    const float32_t dx = x - point_adapter::x_(pt);
    const float32_t dy = y - point_adapter::y_(pt);
    return (dx * dx) + (dy * dy);
  }
};  // class Config2d

/// \brief Configuration class for a 3d spatial hash
class GEOMETRY_PUBLIC Config3d : public Config<Config3d>
{
public:
  /// \brief Config constructor for a 3d spatial hash
  /// \param[in] min_x The minimum x value for the spatial hash
  /// \param[in] max_x The maximum x value for the spatial hash
  /// \param[in] min_y The minimum y value for the spatial hash
  /// \param[in] max_y The maximum y value for the spatial hash
  /// \param[in] min_z The minimum z value for the spatial hash
  /// \param[in] max_z The maximum z value for the spatial hash
  /// \param[in] radius The lookup distance
  /// \param[in] capacity The maximum number of points the spatial hash can store
  Config3d(
    const float32_t min_x, const float32_t max_x, const float32_t min_y, const float32_t max_y,
    const float32_t min_z, const float32_t max_z, const float32_t radius, const Index capacity);
  /// \brief The index of a point given it's x, y and z values, 3d implementation
  /// \param[in] x The x value of a point
  /// \param[in] y the y value of a point
  /// \param[in] z the z value of a point
  /// \return The index of the bin for the specified point
  Index bin_(const float32_t x, const float32_t y, const float32_t z) const;
  /// \brief Determine if a bin could possibly hold a point within a distance to any point in a
  ///        reference bin for the 3D case
  /// \param[in] ref The decomposed index triple of the reference bin
  /// \param[in] query The decomposed index triple of the bin being queried
  /// \param[in] ref_distance2 The squared threshold distance
  /// \return True if the reference bin and query bin could possibly hold a point within the
  ///         reference distance
  bool valid(
    const details::Index3 & ref, const details::Index3 & query, const float ref_distance2) const;
  /// \brief Compute the decomposed index given a point, 3d implementation
  /// \param[in] x The x component of the point
  /// \param[in] y The y component of the point
  /// \param[in] z The z component of the point
  /// \return The decomposed index triple of the bin for the given point
  details::Index3 index3_(const float32_t x, const float32_t y, const float32_t z) const;
  /// \brief Compute the composed single index given a decomposed index, 3d implementation
  /// \param[in] idx A decomposed index triple for a bin
  /// \return The composed bin index
  Index index_(const details::Index3 & idx) const;
  /// \brief Compute the squared distance between the two points, 3d implementation
  /// \tparam PointT A point type with float members x, y and z, or point adapters defined
  /// \param[in] x The x component of the first point
  /// \param[in] y The y component of the first point
  /// \param[in] z The z component of the first point
  /// \param[in] pt The other point being compared
  /// \return The squared distance between the points (3d)
  template <typename PointT>
  float32_t distance_squared_(
    const float32_t x, const float32_t y, const float32_t z, const PointT & pt) const
  {
    const float32_t dx = x - point_adapter::x_(pt);
    const float32_t dy = y - point_adapter::y_(pt);
    const float32_t dz = z - point_adapter::z_(pt);
    return (dx * dx) + (dy * dy) + (dz * dz);
  }
};  // class Config3d
}  // namespace spatial_hash
}  // namespace geometry
}  // namespace common
}  // namespace autoware

#endif  // GEOMETRY__SPATIAL_HASH_CONFIG_HPP_

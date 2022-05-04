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

#ifndef GEOMETRY__SPATIAL_HASH_HPP_
#define GEOMETRY__SPATIAL_HASH_HPP_

#include <common/types.hpp>
#include <geometry/spatial_hash_config.hpp>
#include <geometry/visibility_control.hpp>

#include <unordered_map>
#include <utility>
#include <vector>

using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;

namespace autoware
{
namespace common
{
namespace geometry
{
/// \brief All objects related to the spatial hash data structure for efficient near neighbor lookup
namespace spatial_hash
{

/// \brief An implementation of the spatial hash or integer lattice data structure for efficient
///        (O(1)) near neighbor queries.
/// \tparam PointT The point type stored in this data structure. Must have float members x, y, and z
///
/// This implementation can support both 2D and 3D queries
/// (though only one type per data structure), and can support queries of varying radius. This data
/// structure cannot do near neighbor lookups for euclidean distance in arbitrary dimensions.
template <typename PointT, typename ConfigT>
class GEOMETRY_PUBLIC SpatialHashBase
{
  using Index3 = details::Index3;
  // lint -e{9131} NOLINT There's no other way to make this work in a static assert
  static_assert(
    std::is_same<ConfigT, Config2d>::value || std::is_same<ConfigT, Config3d>::value,
    "SpatialHash only works with Config2d or Config3d");

public:
  using Hash = std::unordered_multimap<Index, PointT>;
  using IT = typename Hash::const_iterator;
  /// \brief Wrapper around an iterator and a distance (from some query point)
  class Output
  {
  public:
    /// \brief Constructor
    /// \param[in] iterator An iterator pointing to some point
    /// \param[in] distance The euclidean distance (2d or 3d) to a reference point
    Output(const IT iterator, const float32_t distance) : m_iterator(iterator), m_distance(distance)
    {
    }
    /// \brief Get stored point
    /// \return A const reference to the stored point
    const PointT & get_point() const { return m_iterator->second; }
    /// \brief Get underlying iterator
    /// \return A copy of the underlying iterator
    IT get_iterator() const { return m_iterator; }
    /// \brief Convert to underlying point
    /// \return A reference to the underlying point
    operator const PointT &() const { return get_point(); }
    /// \brief Convert to underlying iterator
    /// \return A copy of the iterator
    operator IT() const { return get_iterator(); }
    /// \brief Get distance to reference point
    /// \return The distance
    float32_t get_distance() const { return m_distance; }

  private:
    IT m_iterator;
    float32_t m_distance;
  };  // class Output
  using OutputVector = typename std::vector<Output>;

  /// \brief Constructor
  /// \param[in] cfg The configuration object for this class
  explicit SpatialHashBase(const ConfigT & cfg)
  : m_config{cfg},
    m_hash(),
    m_neighbors{},  // TODO(c.ho) reserve, but there's no default constructor for output
    m_bins_hit{},   // zero initialization (and below)
    m_neighbors_found{}
  {
  }

  /// \brief Inserts point
  /// \param[in] pt The Point to insert
  /// \return Iterator pointing to the inserted point
  /// \throw std::length_error If the data structure is at capacity
  IT insert(const PointT & pt)
  {
    if (size() >= capacity()) {
      throw std::length_error{"SpatialHash: Cannot insert past capacity"};
    }
    return insert_impl(pt);
  }

  /// \brief Inserts a range of points
  /// \param[in] begin The start of the range of points to insert
  /// \param[in] end The end of the range of points to insert
  /// \tparam IteratorT The iterator type
  /// \throw std::length_error If the range of points to insert exceeds the data structure's
  ///                          capacity
  template <typename IteratorT>
  void insert(IteratorT begin, IteratorT end)
  {
    // This check is here for strong exception safety
    if ((size() + std::distance(begin, end)) > capacity()) {
      throw std::length_error{"SpatialHash: Cannot multi-insert past capacity"};
    }
    for (IteratorT it = begin; it != end; ++it) {
      (void)insert_impl(*it);
    }
  }

  /// \brief Removes the specified element from the data structure
  /// \param[in] point An iterator pointing to a point to be removed
  /// \return An iterator pointing to the element after the erased element
  /// \throw std::domain_error If pt is invalid or does not belong to this data structure
  ///
  /// \note There is no reliable way to check if an iterator is invalid. The checks here are
  /// based on a heuristic and is not guaranteed to find all invalid iterators. This method
  /// should be used with care and only on valid iterators
  IT erase(const IT point)
  {
    if (end() == m_hash.find(point->first)) {
      throw std::domain_error{"SpatialHash: Attempting to erase invalid iterator"};
    }
    return m_hash.erase(point);
  }

  /// \brief Reset the state of the data structure
  void clear() { m_hash.clear(); }
  /// \brief Get current number of element stored in this data structure
  /// \return Number of stored elements
  Index size() const { return m_hash.size(); }
  /// \brief Get the maximum capacity of the data structure
  /// \return The capacity of the data structure
  Index capacity() const { return m_config.get_capacity(); }
  /// \brief Whether the hash is empty
  /// \return True if data structure is empty
  bool8_t empty() const { return m_hash.empty(); }
  /// \brief Get iterator to beginning of data structure
  /// \return Iterator
  IT begin() const { return m_hash.begin(); }
  /// \brief Get iterator to end of data structure
  /// \return Iterator
  IT end() const { return m_hash.end(); }
  /// \brief Get iterator to beginning of data structure
  /// \return Iterator
  IT cbegin() const { return begin(); }
  /// \brief Get iterator to end of data structure
  /// \return Iterator
  IT cend() const { return end(); }

  /// \brief Get the number of bins touched during the lifetime of this object, for debugging and
  ///        size tuning
  /// \return The total number of bins touched during near() queries
  Index bins_hit() const { return m_bins_hit; }

  /// \brief Get number of near neighbors found during the lifetime of this object, for debugging
  ///        and size tuning
  /// \return The total number of neighbors found during near() queries
  Index neighbors_found() const { return m_neighbors_found; }

protected:
  /// \brief Finds all points within a fixed radius of a reference point
  /// \param[in] x The x component of the reference point
  /// \param[in] y The y component of the reference point
  /// \param[in] z The z component of the reference point, respected only if the spatial hash is not
  ///              2D.
  /// \param[in] radius The radius within which to find all near points
  /// \return A const reference to a vector containing iterators pointing to
  ///         all points within the radius, and the actual distance to the reference point
  const OutputVector & near_impl(
    const float32_t x, const float32_t y, const float32_t z, const float32_t radius)
  {
    // reset output
    m_neighbors.clear();
    // Compute bin, bin range
    const Index3 ref_idx = m_config.index3(x, y, z);
    const float32_t radius2 = radius * radius;
    const details::BinRange idx_range = m_config.bin_range(ref_idx, radius);
    Index3 idx = idx_range.first;
    // For bins in radius
    do {  // guaranteed to have at least the bin ref_idx is in
      // update book-keeping
      ++m_bins_hit;
      // Iterating in a square/cube pattern is easier than constructing sphere pattern
      if (m_config.is_candidate_bin(ref_idx, idx, radius2)) {
        // For point in bin
        const Index jdx = m_config.index(idx);
        const auto range = m_hash.equal_range(jdx);
        for (auto it = range.first; it != range.second; ++it) {
          const auto & pt = it->second;
          const float32_t dist2 = m_config.distance_squared(x, y, z, pt);
          if (dist2 <= radius2) {
            // Only compute true distance if necessary
            m_neighbors.emplace_back(it, sqrtf(dist2));
          }
        }
      }
    } while (m_config.next_bin(idx_range, idx));
    // update book-keeping
    m_neighbors_found += m_neighbors.size();
    return m_neighbors;
  }

private:
  /// \brief Internal insert method with no error checking
  /// \param[in] pt The Point to insert
  GEOMETRY_LOCAL IT insert_impl(const PointT & pt)
  {
    // Compute bin
    const Index idx =
      m_config.bin(point_adapter::x_(pt), point_adapter::y_(pt), point_adapter::z_(pt));
    // Insert into bin
    return m_hash.insert(std::make_pair(idx, pt));
  }

  const ConfigT m_config;
  Hash m_hash;
  OutputVector m_neighbors;
  Index m_bins_hit;
  Index m_neighbors_found;
};  // class SpatialHashBase

/// \brief The class to be used for specializing on
/// apex_app::common::geometry::spatial_hash::SpatialHashBase to provide different function
/// signatures on 2D and 3D configurations
/// \tparam PointT The point type stored in this data structure. Must have float members x, y and z
template <typename PointT, typename ConfigT>
class GEOMETRY_PUBLIC SpatialHash;

/// \brief Explicit specialization of SpatialHash for 2D configuration
/// \tparam PointT The point type stored in this data structure.
template <typename PointT>
class GEOMETRY_PUBLIC SpatialHash<PointT, Config2d> : public SpatialHashBase<PointT, Config2d>
{
public:
  using OutputVector = typename SpatialHashBase<PointT, Config2d>::OutputVector;

  explicit SpatialHash(const Config2d & cfg) : SpatialHashBase<PointT, Config2d>(cfg) {}

  /// \brief Finds all points within a fixed radius of a reference point
  /// \param[in] x The x component of the reference point
  /// \param[in] y The y component of the reference point
  /// \param[in] radius The radius within which to find all near points
  /// \return A const reference to a vector containing iterators pointing to
  ///         all points within the radius, and the actual distance to the reference point
  const OutputVector & near(const float32_t x, const float32_t y, const float32_t radius)
  {
    return this->near_impl(x, y, 0.0F, radius);
  }

  /// \brief Finds all points within a fixed radius of a reference point
  /// \param[in] pt The reference point. Only the x and y members are respected.
  /// \param[in] radius The radius within which to find all near points
  /// \return A const reference to a vector containing iterators pointing to
  ///         all points within the radius, and the actual distance to the reference point
  const OutputVector & near(const PointT & pt, const float32_t radius)
  {
    return near(point_adapter::x_(pt), point_adapter::y_(pt), radius);
  }
};

/// \brief Explicit specialization of SpatialHash for 3D configuration
/// \tparam PointT The point type stored in this data structure. Must have float members x, y and z
template <typename PointT>
class GEOMETRY_PUBLIC SpatialHash<PointT, Config3d> : public SpatialHashBase<PointT, Config3d>
{
public:
  using OutputVector = typename SpatialHashBase<PointT, Config3d>::OutputVector;

  explicit SpatialHash(const Config3d & cfg) : SpatialHashBase<PointT, Config3d>(cfg) {}

  /// \brief Finds all points within a fixed radius of a reference point
  /// \param[in] x The x component of the reference point
  /// \param[in] y The y component of the reference point
  /// \param[in] z The z component of the reference point, respected only if the spatial hash is not
  ///              2D.
  /// \param[in] radius The radius within which to find all near points
  /// \return A const reference to a vector containing iterators pointing to
  ///         all points within the radius, and the actual distance to the reference point
  const OutputVector & near(
    const float32_t x, const float32_t y, const float32_t z, const float32_t radius)
  {
    return this->near_impl(x, y, z, radius);
  }

  /// \brief Finds all points within a fixed radius of a reference point
  /// \param[in] pt The reference point.
  /// \param[in] radius The radius within which to find all near points
  /// \return A const reference to a vector containing iterators pointing to
  ///         all points within the radius, and the actual distance to the reference point
  const OutputVector & near(const PointT & pt, const float32_t radius)
  {
    return near(point_adapter::x_(pt), point_adapter::y_(pt), point_adapter::z_(pt), radius);
  }
};

template <typename T>
using SpatialHash2d = SpatialHash<T, Config2d>;
template <typename T>
using SpatialHash3d = SpatialHash<T, Config3d>;
}  // namespace spatial_hash
}  // namespace geometry
}  // namespace common
}  // namespace autoware

#endif  // GEOMETRY__SPATIAL_HASH_HPP_

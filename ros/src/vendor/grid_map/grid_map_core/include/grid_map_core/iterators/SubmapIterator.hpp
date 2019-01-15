/*
 * SubmapIterator.hpp
 *
 *  Created on: Sep 12, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include "grid_map_core/GridMap.hpp"
#include "grid_map_core/SubmapGeometry.hpp"
#include "grid_map_core/BufferRegion.hpp"

#include <Eigen/Core>

namespace grid_map {

/*!
 * Iterator class to iterate through a rectangular part of the map (submap).
 * Before using this iterator, make sure that the requested submap is
 * actually contained in the grid map.
 */
class SubmapIterator
{
public:

  /*!
   * Constructor.
   * @param submap the submap geometry to iterate over.
   */
  SubmapIterator(const grid_map::SubmapGeometry& submap);

  /*!
   * Constructor.
   * @param submap the buffer region of a grid map to iterate over.
   */
  SubmapIterator(const grid_map::GridMap& gridMap, const grid_map::BufferRegion& bufferRegion);

  /*!
   * Constructor.
   * @param gridMap the grid map to iterate on.
   * @param submapStartIndex the start index of the submap, typically top-left index.
   * @param submapSize the size of the submap to iterate on.
   */
  SubmapIterator(const grid_map::GridMap& gridMap, const Index& submapStartIndex,
                 const Size& submapSize);

  /*!
   * Copy constructor.
   * @param other the object to copy.
   */
  SubmapIterator(const SubmapIterator* other);

  /*!
   * Assignment operator.
   * @param iterator the iterator to copy data from.
   * @return a reference to *this.
   */
  SubmapIterator& operator =(const SubmapIterator& other);

  /*!
   * Compare to another iterator.
   * @return whether the current iterator points to a different address than the other one.
   */
  bool operator !=(const SubmapIterator& other) const;

  /*!
   * Dereference the iterator with const.
   * @return the value to which the iterator is pointing.
   */
  const Index& operator *() const;

  /*!
   * Get the current index in the submap.
   * @return the current index in the submap.
   */
  const Index& getSubmapIndex() const;

  /*!
   * Increase the iterator to the next element.
   * @return a reference to the updated iterator.
   */
  SubmapIterator& operator ++();

  /*!
   * Indicates if iterator is past end.
   * @return true if iterator is out of scope, false if end has not been reached.
   */
  bool isPastEnd() const;

  /*!
   * Returns the size of the submap covered by the iterator.
   * @return the size of the submap covered by the iterator.
   */
  const Size& getSubmapSize() const;

private:

  //! Size of the buffer.
  Size size_;

  //! Start index of the circular buffer.
  Index startIndex_;

  //! Current index.
  Index index_;

  //! Submap buffer size.
  Size submapSize_;

  //! Top left index of the submap.
  Index submapStartIndex_;

  //! Current index in the submap.
  Index submapIndex_;

  //! Is iterator out of scope.
  bool isPastEnd_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} /* namespace */

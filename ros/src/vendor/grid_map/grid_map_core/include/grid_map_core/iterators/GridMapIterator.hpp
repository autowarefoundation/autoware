/*
 * GridMapIterator.hpp
 *
 *  Created on: Sep 22, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include "grid_map_core/GridMap.hpp"

// Eigen
#include <Eigen/Core>

namespace grid_map {

/*!
 * Iterator class to iterate trough the entire grid map.
 */
class GridMapIterator
{
public:

  /*!
   * Constructor.
   * @param gridMap the grid map to iterate on.
   */
  GridMapIterator(const grid_map::GridMap &gridMap);

  /*!
   * Copy constructor.
   * @param other the object to copy.
   */
  GridMapIterator(const GridMapIterator* other);

  /*!
   * Assignment operator.
   * @param iterator the iterator to copy data from.
   * @return a reference to *this.
   */
  GridMapIterator& operator =(const GridMapIterator& other);

  /*!
   * Compare to another iterator.
   * @return whether the current iterator points to a different address than the other one.
   */
  bool operator !=(const GridMapIterator& other) const;

  /*!
   * Dereference the iterator to return the regular index (2-dim.) of the cell
   * to which the iterator is pointing at.
   * @return the regular index (2-dim.) of the cell on which the iterator is pointing.
   */
  const Index operator *() const;

  /*!
   * Returns the the linear (1-dim.) index of the cell the iterator is pointing at.
   * Note: Use this access for improved efficiency when working with large maps.
   * Example: See `runGridMapIteratorVersion3()` of `grid_map_demos/src/iterator_benchmark.cpp`.
   * @return the 1d linear index.
   */
  const size_t& getLinearIndex() const;

  /*!
   * Retrieve the index as unwrapped index, i.e., as the corresponding index of a
   * grid map with no circular buffer offset.
   */
  const Index getUnwrappedIndex() const;

  /*!
   * Increase the iterator to the next element.
   * @return a reference to the updated iterator.
   */
  virtual GridMapIterator& operator ++();

  /*!
   * Return the end iterator
   * @return the end iterator (useful when performing normal iterator processing with ++).
   */
  GridMapIterator end() const;

  /*!
   * Indicates if iterator is past end.
   * @return true if iterator is out of scope, false if end has not been reached.
   */
  bool isPastEnd() const;

protected:

  //! Size of the buffer.
  Size size_;

  //! Start index of the circular buffer.
  Index startIndex_;

  //! Linear size of the data.
  size_t linearSize_;

  //! Linear index.
  size_t linearIndex_;

  //! Is iterator out of scope.
  bool isPastEnd_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} /* namespace */

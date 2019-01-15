/*
 * CircleIterator.hpp
 *
 *  Created on: Nov 13, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include "grid_map_core/GridMap.hpp"
#include "grid_map_core/iterators/SubmapIterator.hpp"

#include <Eigen/Core>

#include <memory>

namespace grid_map {

/*!
 * Iterator class to iterate through a circular area of the map.
 */
class CircleIterator
{
public:

  /*!
   * Constructor.
   * @param gridMap the grid map to iterate on.
   * @param center the position of the circle center.
   * @param radius the radius of the circle.
   */
  CircleIterator(const GridMap& gridMap, const Position& center, const double radius);

  /*!
   * Assignment operator.
   * @param iterator the iterator to copy data from.
   * @return a reference to *this.
   */
  CircleIterator& operator =(const CircleIterator& other);

  /*!
   * Compare to another iterator.
   * @return whether the current iterator points to a different address than the other one.
   */
  bool operator !=(const CircleIterator& other) const;

  /*!
   * Dereference the iterator with const.
   * @return the value to which the iterator is pointing.
   */
  const Index& operator *() const;

  /*!
   * Increase the iterator to the next element.
   * @return a reference to the updated iterator.
   */
  CircleIterator& operator ++();

  /*!
   * Indicates if iterator is past end.
   * @return true if iterator is out of scope, false if end has not been reached.
   */
  bool isPastEnd() const;

private:

  /*!
   * Check if current index is inside the circle.
   * @return true if inside, false otherwise.
   */
  bool isInside() const;

  /*!
   * Finds the submap that fully contains the circle and returns the parameters.
   * @param[in] center the position of the circle center.
   * @param[in] radius the radius of the circle.
   * @param[out] startIndex the start index of the submap.
   * @param[out] bufferSize the buffer size of the submap.
   */
  void findSubmapParameters(const Position& center, const double radius,
                            Index& startIndex, Size& bufferSize) const;

  //! Position of the circle center;
  Position center_;

  //! Radius of the circle.
  double radius_;

  //! Square of the radius (for efficiency).
  double radiusSquare_;

  //! Grid submap iterator. // TODO Think of using unique_ptr instead.
  std::shared_ptr<SubmapIterator> internalIterator_;

  //! Map information needed to get position from iterator.
  Length mapLength_;
  Position mapPosition_;
  double resolution_;
  Size bufferSize_;
  Index bufferStartIndex_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} /* namespace */

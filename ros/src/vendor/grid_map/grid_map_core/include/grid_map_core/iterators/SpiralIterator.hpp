/*
 * SpiralIterator.hpp
 *
 *  Created on: Jul 7, 2015
 *      Author: Martin Wermelinger
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include "grid_map_core/GridMap.hpp"

#include <Eigen/Core>
#include <memory>
#include <vector>

namespace grid_map {

/*!
 * Iterator class to iterate through a circular area of the map with a spiral.
 */
class SpiralIterator
{
public:

  /*!
   * Constructor.
   * @param gridMap the grid map to iterate on.
   * @param center the position of the circle center.
   * @param radius the radius of the circle.
   */
  SpiralIterator(const grid_map::GridMap& gridMap, const Eigen::Vector2d& center, const double radius);

  /*!
   * Assignment operator.
   * @param iterator the iterator to copy data from.
   * @return a reference to *this.
   */
  SpiralIterator& operator =(const SpiralIterator& other);

  /*!
   * Compare to another iterator.
   * @return whether the current iterator points to a different address than the other one.
   */
  bool operator !=(const SpiralIterator& other) const;

  /*!
   * Dereference the iterator with const.
   * @return the value to which the iterator is pointing.
   */
  const Eigen::Array2i& operator *() const;

  /*!
   * Increase the iterator to the next element.
   * @return a reference to the updated iterator.
   */
  SpiralIterator& operator ++();

  /*!
   * Indicates if iterator is past end.
   * @return true if iterator is out of scope, false if end has not been reached.
   */
  bool isPastEnd() const;

  /*!
   * Gets the radius of current ring that is iterated through.
   * @return radius of the current ring that is used for iteration.
   */
  double getCurrentRadius() const;

private:

  /*!
   * Check if index is inside the circle.
   * @return true if inside, false otherwise.
   */
  bool isInside(const Index index) const;

  /*!
   * Uses the current distance to get the points of a ring
   * around the center.
   */
  void generateRing();

  int signum(const int val) {
      return (0 < val) - (val < 0);
  }

  //! Position of the circle center;
  Position center_;
  Index indexCenter_;


  //! Radius of the circle.
  double radius_;

  //! Square of the radius for efficiency.
  double radiusSquare_;

  //! Number of rings into the circle is divided.
  unsigned int nRings_;
  unsigned int distance_;
  std::vector<Index> pointsRing_;

  //! Map information needed to get position from iterator.
  Length mapLength_;
  Position mapPosition_;
  double resolution_;
  Size bufferSize_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} /* namespace */

/*
 * LineIterator.hpp
 *
 *  Created on: Nov 13, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include "grid_map_core/GridMap.hpp"
#include "grid_map_core/iterators/SubmapIterator.hpp"

#include <Eigen/Core>

namespace grid_map {

/*!
 * Iterator class to iterate over a line in the map.
 * Based on Bresenham Line Drawing algorithm.
 */
class LineIterator
{
public:

  /*!
   * Constructor.
   * @param gridMap the grid map to iterate on.
   * @param start the starting point of the line.
   * @param end the ending point of the line.
   */
  LineIterator(const grid_map::GridMap& gridMap, const Position& start, const Position& end);

  /*!
   * Constructor.
   * @param gridMap the grid map to iterate on.
   * @param start the starting index of the line.
   * @param end the ending index of the line.
   */
  LineIterator(const grid_map::GridMap& gridMap, const Index& start, const Index& end);

  /*!
   * Assignment operator.
   * @param iterator the iterator to copy data from.
   * @return a reference to *this.
   */
  LineIterator& operator =(const LineIterator& other);

  /*!
   * Compare to another iterator.
   * @return whether the current iterator points to a different address than the other one.
   */
  bool operator !=(const LineIterator& other) const;

  /*!
   * Dereference the iterator with const.
   * @return the value to which the iterator is pointing.
   */
  const Index& operator *() const;

  /*!
   * Increase the iterator to the next element.
   * @return a reference to the updated iterator.
   */
  LineIterator& operator ++();

  /*!
   * Indicates if iterator is past end.
   * @return true if iterator is out of scope, false if end has not been reached.
   */
  bool isPastEnd() const;

private:


  /*!
   * Construct function.
   * @param gridMap the grid map to iterate on.
   * @param start the starting index of the line.
   * @param end the ending index of the line.
   * @return true if successful, false otherwise.
   */
  bool initialize(const grid_map::GridMap& gridMap, const Index& start, const Index& end);

  /*!
   * Computes the parameters requires for the line drawing algorithm.
   */
  void initializeIterationParameters();

  /*!
   * Finds the index of a position on a line within the limits of the map.
   * @param[in] gridMap the grid map that defines the map boundaries.
   * @param[in] start the position that will be limited to the map range.
   * @param[in] end the ending position of the line.
   * @param[out] index the index of the moved start position.
   * @return true if successful, false otherwise.
   */
  bool getIndexLimitedToMapRange(const grid_map::GridMap& gridMap, const Position& start,
                                 const Position& end, Index& index);

  //! Current index.
  Index index_;

  //! Starting index of the line.
  Index start_;

  //! Ending index of the line.
  Index end_;

  //! Current cell number.
  unsigned int iCell_;

  //! Number of cells in the line.
  unsigned int nCells_;

  //! Helper variables for Bresenham Line Drawing algorithm.
  Size increment1_, increment2_;
  int denominator_, numerator_, numeratorAdd_;

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

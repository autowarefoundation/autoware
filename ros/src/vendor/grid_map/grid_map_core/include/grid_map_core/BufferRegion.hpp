/*
 * BufferRegion.hpp
 *
 *  Created on: Aug 19, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include "grid_map_core/TypeDefs.hpp"

namespace grid_map {

/*!
 * This class holds information about a rectangular region
 * of cells of the circular buffer.
 */
class BufferRegion
{
 public:

  /*!
   * The definition of the buffer region positions.
   */
  enum class Quadrant
  {
    Undefined,
    TopLeft,
    TopRight,
    BottomLeft,
    BottomRight
  };

  constexpr static unsigned int nQuadrants = 4;

  BufferRegion();
  BufferRegion(const Index& startIndex, const Size& size, const BufferRegion::Quadrant& quadrant);
  virtual ~BufferRegion();

  const Index& getStartIndex() const;
  void setStartIndex(const Index& startIndex);
  const Size& getSize() const;
  void setSize(const Size& size);
  BufferRegion::Quadrant getQuadrant() const;
  void setQuadrant(BufferRegion::Quadrant type);

 private:

  //! Start index (typically top-left) of the buffer region.
  Index staretIndex_;

  //! Size of the buffer region.
  Size size_;

  //! Quadrant type of the buffer region.
  Quadrant quadrant_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} /* namespace grid_map */

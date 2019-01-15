/*
 * SubmapGeometry.cpp
 *
 *  Created on: Aug 18, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <grid_map_core/GridMapMath.hpp>
#include <grid_map_core/SubmapGeometry.hpp>

namespace grid_map {

SubmapGeometry::SubmapGeometry(const GridMap& gridMap, const Position& position,
                               const Length& length, bool& isSuccess)
    : gridMap_(gridMap)
{
  isSuccess = getSubmapInformation(startIndex_, size_, position_, length_,
                                   requestedIndexInSubmap_, position, length, gridMap_.getLength(),
                                   gridMap_.getPosition(), gridMap_.getResolution(),
                                   gridMap_.getSize(), gridMap_.getStartIndex());
}

SubmapGeometry::~SubmapGeometry()
{
}

const GridMap& SubmapGeometry::getGridMap() const
{
  return gridMap_;
}

const Length& SubmapGeometry::getLength() const
{
  return length_;
}

const Position& SubmapGeometry::getPosition() const
{
  return position_;
}

const Index& SubmapGeometry::getRequestedIndexInSubmap() const
{
  return requestedIndexInSubmap_;
}

const Size& SubmapGeometry::getSize() const
{
  return size_;
}

double SubmapGeometry::getResolution() const
{
  return gridMap_.getResolution();
}

const Index& SubmapGeometry::getStartIndex() const
{
  return startIndex_;
}

} /* namespace grid_map */

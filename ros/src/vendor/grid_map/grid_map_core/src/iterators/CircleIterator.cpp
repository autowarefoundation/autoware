/*
 * Circleterator.hpp
 *
 *  Created on: Nov 13, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map_core/iterators/CircleIterator.hpp"
#include "grid_map_core/GridMapMath.hpp"

using namespace std;

namespace grid_map {

CircleIterator::CircleIterator(const GridMap& gridMap, const Position& center, const double radius)
    : center_(center),
      radius_(radius)
{
  radiusSquare_ = pow(radius_, 2);
  mapLength_ = gridMap.getLength();
  mapPosition_ = gridMap.getPosition();
  resolution_ = gridMap.getResolution();
  bufferSize_ = gridMap.getSize();
  bufferStartIndex_ = gridMap.getStartIndex();
  Index submapStartIndex;
  Index submapBufferSize;
  findSubmapParameters(center, radius, submapStartIndex, submapBufferSize);
  internalIterator_ = std::shared_ptr<SubmapIterator>(new SubmapIterator(gridMap, submapStartIndex, submapBufferSize));
  if(!isInside()) ++(*this);
}

CircleIterator& CircleIterator::operator =(const CircleIterator& other)
{
  center_ = other.center_;
  radius_ = other.radius_;
  radiusSquare_ = other.radiusSquare_;
  internalIterator_ = other.internalIterator_;
  mapLength_ = other.mapLength_;
  mapPosition_ = other.mapPosition_;
  resolution_ = other.resolution_;
  bufferSize_ = other.bufferSize_;
  bufferStartIndex_ = other.bufferStartIndex_;
  return *this;
}

bool CircleIterator::operator !=(const CircleIterator& other) const
{
  return (internalIterator_ != other.internalIterator_);
}

const Index& CircleIterator::operator *() const
{
  return *(*internalIterator_);
}

CircleIterator& CircleIterator::operator ++()
{
  ++(*internalIterator_);
  if (internalIterator_->isPastEnd()) return *this;

  for ( ; !internalIterator_->isPastEnd(); ++(*internalIterator_)) {
    if (isInside()) break;
  }

  return *this;
}

bool CircleIterator::isPastEnd() const
{
  return internalIterator_->isPastEnd();
}

bool CircleIterator::isInside() const
{
  Position position;
  getPositionFromIndex(position, *(*internalIterator_), mapLength_, mapPosition_, resolution_, bufferSize_, bufferStartIndex_);
  double squareNorm = (position - center_).array().square().sum();
  return (squareNorm <= radiusSquare_);
}

void CircleIterator::findSubmapParameters(const Position& center, const double radius,
                                          Index& startIndex, Size& bufferSize) const
{
  Position topLeft = center.array() + radius;
  Position bottomRight = center.array() - radius;
  boundPositionToRange(topLeft, mapLength_, mapPosition_);
  boundPositionToRange(bottomRight, mapLength_, mapPosition_);
  getIndexFromPosition(startIndex, topLeft, mapLength_, mapPosition_, resolution_, bufferSize_, bufferStartIndex_);
  Index endIndex;
  getIndexFromPosition(endIndex, bottomRight, mapLength_, mapPosition_, resolution_, bufferSize_, bufferStartIndex_);
  bufferSize = getSubmapSizeFromCornerIndeces(startIndex, endIndex, bufferSize_, bufferStartIndex_);
}

} /* namespace grid_map */


/*
 * SubmapIterator.hpp
 *
 *  Created on: Sep 22, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map_core/iterators/SubmapIterator.hpp"
#include "grid_map_core/GridMapMath.hpp"

using namespace std;

namespace grid_map {

SubmapIterator::SubmapIterator(const grid_map::SubmapGeometry& submap)
    : SubmapIterator(submap.getGridMap(), submap.getStartIndex(), submap.getSize())
{
}

SubmapIterator::SubmapIterator(const grid_map::GridMap& gridMap,
                               const grid_map::BufferRegion& bufferRegion)
    : SubmapIterator(gridMap, bufferRegion.getStartIndex(), bufferRegion.getSize())
{
}


SubmapIterator::SubmapIterator(const grid_map::GridMap& gridMap, const Index& submapStartIndex,
                               const Size& submapSize)
{
  size_ = gridMap.getSize();
  startIndex_ = gridMap.getStartIndex();
  index_ = submapStartIndex;
  submapSize_ = submapSize;
  submapStartIndex_ = submapStartIndex;
  submapIndex_.setZero();
  isPastEnd_ = false;
}

SubmapIterator::SubmapIterator(const SubmapIterator* other)
{
  size_ = other->size_;
  startIndex_ = other->startIndex_;
  submapSize_ = other->submapSize_;
  submapStartIndex_ = other->submapStartIndex_;
  index_ = other->index_;
  submapIndex_ = other->submapIndex_;
  isPastEnd_ = other->isPastEnd_;
}

SubmapIterator& SubmapIterator::operator =(const SubmapIterator& other)
{
  size_ = other.size_;
  startIndex_ = other.startIndex_;
  submapSize_ = other.submapSize_;
  submapStartIndex_ = other.submapStartIndex_;
  index_ = other.index_;
  submapIndex_ = other.submapIndex_;
  isPastEnd_ = other.isPastEnd_;
  return *this;
}

bool SubmapIterator::operator !=(const SubmapIterator& other) const
{
  return (index_ != other.index_).any();
}

const Index& SubmapIterator::operator *() const
{
  return index_;
}

const Index& SubmapIterator::getSubmapIndex() const
{
  return submapIndex_;
}

SubmapIterator& SubmapIterator::operator ++()
{
  isPastEnd_ = !incrementIndexForSubmap(submapIndex_, index_, submapStartIndex_,
                                        submapSize_, size_, startIndex_);
  return *this;
}

bool SubmapIterator::isPastEnd() const
{
  return isPastEnd_;
}

const Size& SubmapIterator::getSubmapSize() const
{
  return submapSize_;
}

} /* namespace grid_map */


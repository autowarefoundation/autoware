/*
 * LineIterator.hpp
 *
 *  Created on: Nov 13, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map_core/iterators/LineIterator.hpp"
#include "grid_map_core/GridMapMath.hpp"

using namespace std;

namespace grid_map {

LineIterator::LineIterator(const grid_map::GridMap& gridMap, const Position& start,
                           const Position& end)
{
  Index startIndex, endIndex;
  if (getIndexLimitedToMapRange(gridMap, start, end, startIndex)
      && getIndexLimitedToMapRange(gridMap, end, start, endIndex))
    initialize(gridMap, startIndex, endIndex);
}

LineIterator::LineIterator(const grid_map::GridMap& gridMap, const Index& start, const Index& end)
{
  initialize(gridMap, start, end);
}

LineIterator& LineIterator::operator =(const LineIterator& other)
{
  index_ = other.index_;
  start_ = other.start_;
  end_ = other.end_;
  iCell_ = other.iCell_;
  nCells_ = other.nCells_;
  increment1_ = other.increment1_;
  increment2_ = other.increment2_;
  denominator_ = other.denominator_;
  numerator_ = other.numerator_;
  numeratorAdd_ = other.numeratorAdd_;
  mapLength_ = other.mapLength_;
  mapPosition_ = other.mapPosition_;
  resolution_ = other.resolution_;
  bufferSize_ = other.bufferSize_;
  bufferStartIndex_ = other.bufferStartIndex_;
  return *this;
}

bool LineIterator::operator !=(const LineIterator& other) const
{
  return (index_ != other.index_).any();
}

const Index& LineIterator::operator *() const
{
  return index_;
}

LineIterator& LineIterator::operator ++()
{
  numerator_ += numeratorAdd_;  // Increase the numerator by the top of the fraction.
  if (numerator_ >= denominator_) {
    numerator_ -= denominator_;
    const Index unwrappedIndex = getIndexFromBufferIndex(index_, bufferSize_, bufferStartIndex_) + increment1_;
    index_ = getBufferIndexFromIndex(unwrappedIndex, bufferSize_, bufferStartIndex_);
  }
  const Index unwrappedIndex = getIndexFromBufferIndex(index_, bufferSize_, bufferStartIndex_) + increment2_;
  index_ = getBufferIndexFromIndex(unwrappedIndex, bufferSize_, bufferStartIndex_);
  ++iCell_;
  return *this;
}

bool LineIterator::isPastEnd() const
{
  return iCell_ >= nCells_;
}

bool LineIterator::initialize(const grid_map::GridMap& gridMap, const Index& start, const Index& end)
{
    start_ = start;
    end_ = end;
    mapLength_ = gridMap.getLength();
    mapPosition_ = gridMap.getPosition();
    resolution_ = gridMap.getResolution();
    bufferSize_ = gridMap.getSize();
    bufferStartIndex_ = gridMap.getStartIndex();
    initializeIterationParameters();
    return true;
}

bool LineIterator::getIndexLimitedToMapRange(const grid_map::GridMap& gridMap,
                                             const Position& start, const Position& end,
                                             Index& index)
{
  Position newStart = start;
  Vector direction = (end - start).normalized();
  while (!gridMap.getIndex(newStart, index)) {
    newStart += (gridMap.getResolution() - std::numeric_limits<double>::epsilon()) * direction;
    if ((end - newStart).norm() < gridMap.getResolution() - std::numeric_limits<double>::epsilon())
      return false;
  }
  return true;
}

void LineIterator::initializeIterationParameters()
{
  iCell_ = 0;
  index_ = start_;

  const Index unwrappedStart = getIndexFromBufferIndex(start_, bufferSize_, bufferStartIndex_);
  const Index unwrappedEnd = getIndexFromBufferIndex(end_, bufferSize_, bufferStartIndex_);
  const Size delta = (unwrappedEnd - unwrappedStart).abs();

  if (unwrappedEnd.x() >= unwrappedStart.x()) {
    // x-values increasing.
    increment1_.x() = 1;
    increment2_.x() = 1;
  } else {
    // x-values decreasing.
    increment1_.x() = -1;
    increment2_.x() = -1;
  }

  if (unwrappedEnd.y() >= unwrappedStart.y()) {
    // y-values increasing.
    increment1_.y() = 1;
    increment2_.y() = 1;
  } else {
    // y-values decreasing.
    increment1_.y() = -1;
    increment2_.y() = -1;
  }

  if (delta.x() >= delta.y()) {
    // There is at least one x-value for every y-value.
    increment1_.x() = 0; // Do not change the x when numerator >= denominator.
    increment2_.y() = 0; // Do not change the y for every iteration.
    denominator_ = delta.x();
    numerator_ = delta.x() / 2;
    numeratorAdd_ = delta.y();
    nCells_ = delta.x() + 1; // There are more x-values than y-values.
  } else {
    // There is at least one y-value for every x-value
    increment2_.x() = 0; // Do not change the x for every iteration.
    increment1_.y() = 0; // Do not change the y when numerator >= denominator.
    denominator_ = delta.y();
    numerator_ = delta.y() / 2;
    numeratorAdd_ = delta.x();
    nCells_ = delta.y() + 1; // There are more y-values than x-values.
  }
}

} /* namespace grid_map */

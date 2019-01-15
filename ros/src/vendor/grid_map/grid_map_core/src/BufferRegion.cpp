/*
 * BufferRegion.cpp
 *
 *  Created on: Aug 19, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */
#include <grid_map_core/BufferRegion.hpp>

namespace grid_map {

BufferRegion::BufferRegion() :
    staretIndex_(Index::Zero()),
    size_(Size::Zero()),
    quadrant_(BufferRegion::Quadrant::Undefined)
{
}

BufferRegion::BufferRegion(const Index& index, const Size& size, const BufferRegion::Quadrant& quadrant) :
    staretIndex_(index),
    size_(size),
    quadrant_(quadrant)
{
}

BufferRegion::~BufferRegion()
{
}

const Index& BufferRegion::getStartIndex() const
{
  return staretIndex_;
}

void BufferRegion::setStartIndex(const Index& staretIndex)
{
  staretIndex_ = staretIndex;
}

const Size& BufferRegion::getSize() const
{
  return size_;
}

void BufferRegion::setSize(const Size& size)
{
  size_ = size;
}

BufferRegion::Quadrant BufferRegion::getQuadrant() const
{
  return quadrant_;
}

void BufferRegion::setQuadrant(BufferRegion::Quadrant type)
{
  quadrant_ = type;
}

} /* namespace grid_map */



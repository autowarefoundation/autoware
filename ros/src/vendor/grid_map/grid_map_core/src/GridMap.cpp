/*
 * GridMap.cpp
 *
 *  Created on: Jul 14, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map_core/GridMap.hpp"
#include "grid_map_core/GridMapMath.hpp"
#include "grid_map_core/SubmapGeometry.hpp"
#include "grid_map_core/iterators/GridMapIterator.hpp"

#include <Eigen/Dense>

#include <iostream>
#include <cassert>
#include <math.h>
#include <algorithm>
#include <stdexcept>

using namespace std;
using namespace grid_map;

namespace grid_map {

GridMap::GridMap(const std::vector<std::string>& layers)
{
  position_.setZero();
  length_.setZero();
  resolution_ = 0.0;
  size_.setZero();
  startIndex_.setZero();
  timestamp_ = 0;
  layers_ = layers;

  for (auto& layer : layers_) {
    data_.insert(std::pair<std::string, Matrix>(layer, Matrix()));
  }
}

GridMap::GridMap() :
    GridMap(std::vector<std::string>())
{
}

GridMap::~GridMap()
{
}

void GridMap::setGeometry(const Length& length, const double resolution,
                          const Position& position)
{
  assert(length(0) > 0.0);
  assert(length(1) > 0.0);
  assert(resolution > 0.0);

  Size size;
  size(0) = static_cast<int>(round(length(0) / resolution)); // There is no round() function in Eigen.
  size(1) = static_cast<int>(round(length(1) / resolution));
  resize(size);
  clearAll();

  resolution_ = resolution;
  length_ = (size_.cast<double>() * resolution_).matrix();
  position_ = position;
  startIndex_.setZero();

  return;
}

void GridMap::setGeometry(const SubmapGeometry& geometry)
{
  setGeometry(geometry.getLength(), geometry.getResolution(), geometry.getPosition());
}

void GridMap::setBasicLayers(const std::vector<std::string>& basicLayers)
{
  basicLayers_ = basicLayers;
}

const std::vector<std::string>& GridMap::getBasicLayers() const
{
  return basicLayers_;
}

bool GridMap::hasBasicLayers() const
{
  return basicLayers_.size() > 0;
}

bool GridMap::hasSameLayers(const GridMap& other) const
{
  for (const auto& layer : layers_) {
    if (!other.exists(layer)) return false;
  }
  return true;
}

void GridMap::add(const std::string& layer, const double value)
{
  add(layer, Matrix::Constant(size_(0), size_(1), value));
}

void GridMap::add(const std::string& layer, const Matrix& data)
{
  assert(size_(0) == data.rows());
  assert(size_(1) == data.cols());

  if (exists(layer)) {
    // Type exists already, overwrite its data.
    data_.at(layer) = data;
  } else {
    // Type does not exist yet, add type and data.
    data_.insert(std::pair<std::string, Matrix>(layer, data));
    layers_.push_back(layer);
  }
}

bool GridMap::exists(const std::string& layer) const
{
  return !(data_.find(layer) == data_.end());
}

const Matrix& GridMap::get(const std::string& layer) const
{
  try {
    return data_.at(layer);
  } catch (const std::out_of_range& exception) {
    throw std::out_of_range("GridMap::get(...) : No map layer '" + layer + "' available.");
  }
}

Matrix& GridMap::get(const std::string& layer)
{
  try {
    return data_.at(layer);
  } catch (const std::out_of_range& exception) {
    throw std::out_of_range("GridMap::get(...) : No map layer of type '" + layer + "' available.");
  }
}

const Matrix& GridMap::operator [](const std::string& layer) const
{
  return get(layer);
}

Matrix& GridMap::operator [](const std::string& layer)
{
  return get(layer);
}

bool GridMap::erase(const std::string& layer)
{
  const auto dataIterator = data_.find(layer);
  if (dataIterator == data_.end()) return false;
  data_.erase(dataIterator);

  const auto layerIterator = std::find(layers_.begin(), layers_.end(), layer);
  if (layerIterator == layers_.end()) return false;
  layers_.erase(layerIterator);

  const auto basicLayerIterator = std::find(basicLayers_.begin(), basicLayers_.end(), layer);
  if (basicLayerIterator != basicLayers_.end()) basicLayers_.erase(basicLayerIterator);

  return true;
}

const std::vector<std::string>& GridMap::getLayers() const
{
  return layers_;
}

float& GridMap::atPosition(const std::string& layer, const Position& position)
{
  Index index;
  if (getIndex(position, index)) {
    return at(layer, index);
  }
  throw std::out_of_range("GridMap::atPosition(...) : Position is out of range.");
}

float GridMap::atPosition(const std::string& layer, const Position& position, InterpolationMethods interpolationMethod) const
{
  switch (interpolationMethod) {
      case InterpolationMethods::INTER_LINEAR:
      {
        float value;
        if (atPositionLinearInterpolated(layer, position, value))
          return value;
        else
            interpolationMethod = InterpolationMethods::INTER_NEAREST;
      }
      case  InterpolationMethods::INTER_NEAREST:
      {
        Index index;
        if (getIndex(position, index)) {
        return at(layer, index);
        }
        else
        throw std::out_of_range("GridMap::atPosition(...) : Position is out of range.");
        break;
      }
      default:
        throw std::runtime_error("GridMap::atPosition(...) : Specified interpolation method not implemented.");
  }
}

float& GridMap::at(const std::string& layer, const Index& index)
{
  try {
    return data_.at(layer)(index(0), index(1));
  } catch (const std::out_of_range& exception) {
    throw std::out_of_range("GridMap::at(...) : No map layer '" + layer + "' available.");
  }
}

float GridMap::at(const std::string& layer, const Index& index) const
{
  try {
    return data_.at(layer)(index(0), index(1));
  } catch (const std::out_of_range& exception) {
    throw std::out_of_range("GridMap::at(...) : No map layer '" + layer + "' available.");
  }
}

bool GridMap::getIndex(const Position& position, Index& index) const
{
  return getIndexFromPosition(index, position, length_, position_, resolution_, size_, startIndex_);
}

bool GridMap::getPosition(const Index& index, Position& position) const
{
  return getPositionFromIndex(position, index, length_, position_, resolution_, size_, startIndex_);
}

bool GridMap::isInside(const Position& position) const
{
  return checkIfPositionWithinMap(position, length_, position_);
}

bool GridMap::isValid(const Index& index) const
{
  return isValid(index, basicLayers_);
}

bool GridMap::isValid(const Index& index, const std::string& layer) const
{
  if (!isfinite(at(layer, index))) return false;
  return true;
}

bool GridMap::isValid(const Index& index, const std::vector<std::string>& layers) const
{
  if (layers.empty()) return false;
  for (auto& layer : layers) {
    if (!isfinite(at(layer, index))) return false;
  }
  return true;
}

bool GridMap::getPosition3(const std::string& layer, const Index& index,
                           Position3& position) const
{
  if (!isValid(index, layer)) return false;
  Position position2d;
  getPosition(index, position2d);
  position.head(2) = position2d;
  position.z() = at(layer, index);
  return true;
}

bool GridMap::getVector(const std::string& layerPrefix, const Index& index,
                        Eigen::Vector3d& vector) const
{
  std::vector<std::string> layers;
  layers.push_back(layerPrefix + "x");
  layers.push_back(layerPrefix + "y");
  layers.push_back(layerPrefix + "z");
  if (!isValid(index, layers)) return false;
  for (size_t i = 0; i < 3; ++i) {
    vector(i) = at(layers[i], index);
  }
  return true;
}

GridMap GridMap::getSubmap(const Position& position, const Length& length,
                           bool& isSuccess) const
{
  Index index;
  return getSubmap(position, length, index, isSuccess);
}

GridMap GridMap::getSubmap(const Position& position, const Length& length,
                           Index& indexInSubmap, bool& isSuccess) const
{
  // Submap the generate.
  GridMap submap(layers_);
  submap.setBasicLayers(basicLayers_);
  submap.setTimestamp(timestamp_);
  submap.setFrameId(frameId_);

  // Get submap geometric information.
  SubmapGeometry submapInformation(*this, position, length, isSuccess);
  if (isSuccess == false) return GridMap(layers_);
  submap.setGeometry(submapInformation);
  submap.startIndex_.setZero(); // Because of the way we copy the data below.

  // Copy data.
  std::vector<BufferRegion> bufferRegions;

  if (!getBufferRegionsForSubmap(bufferRegions, submapInformation.getStartIndex(),
                                 submap.getSize(), size_, startIndex_)) {
    cout << "Cannot access submap of this size." << endl;
    isSuccess = false;
    return GridMap(layers_);
  }

  for (const auto& data : data_) {
    for (const auto& bufferRegion : bufferRegions) {
      Index index = bufferRegion.getStartIndex();
      Size size = bufferRegion.getSize();

      if (bufferRegion.getQuadrant() == BufferRegion::Quadrant::TopLeft) {
        submap.data_[data.first].topLeftCorner(size(0), size(1)) = data.second.block(index(0), index(1), size(0), size(1));
      } else if (bufferRegion.getQuadrant() == BufferRegion::Quadrant::TopRight) {
        submap.data_[data.first].topRightCorner(size(0), size(1)) = data.second.block(index(0), index(1), size(0), size(1));
      } else if (bufferRegion.getQuadrant() == BufferRegion::Quadrant::BottomLeft) {
        submap.data_[data.first].bottomLeftCorner(size(0), size(1)) = data.second.block(index(0), index(1), size(0), size(1));
      } else if (bufferRegion.getQuadrant() == BufferRegion::Quadrant::BottomRight) {
        submap.data_[data.first].bottomRightCorner(size(0), size(1)) = data.second.block(index(0), index(1), size(0), size(1));
      }

    }
  }

  isSuccess = true;
  return submap;
}

void GridMap::setPosition(const Position& position)
{
  position_ = position;
}

bool GridMap::move(const Position& position, std::vector<BufferRegion>& newRegions)
{
  Index indexShift;
  Position positionShift = position - position_;
  getIndexShiftFromPositionShift(indexShift, positionShift, resolution_);
  Position alignedPositionShift;
  getPositionShiftFromIndexShift(alignedPositionShift, indexShift, resolution_);

  // Delete fields that fall out of map (and become empty cells).
  for (int i = 0; i < indexShift.size(); i++) {
    if (indexShift(i) != 0) {
      if (abs(indexShift(i)) >= getSize()(i)) {
        // Entire map is dropped.
        clearAll();
        newRegions.push_back(BufferRegion(Index(0, 0), getSize(), BufferRegion::Quadrant::Undefined));
      } else {
        // Drop cells out of map.
        int sign = (indexShift(i) > 0 ? 1 : -1);
        int startIndex = startIndex_(i) - (sign < 0 ? 1 : 0);
        int endIndex = startIndex - sign + indexShift(i);
        int nCells = abs(indexShift(i));
        int index = (sign > 0 ? startIndex : endIndex);
        wrapIndexToRange(index, getSize()(i));

        if (index + nCells <= getSize()(i)) {
          // One region to drop.
          if (i == 0) {
            clearRows(index, nCells);
            newRegions.push_back(BufferRegion(Index(index, 0), Size(nCells, getSize()(1)), BufferRegion::Quadrant::Undefined));
          } else if (i == 1) {
            clearCols(index, nCells);
            newRegions.push_back(BufferRegion(Index(0, index), Size(getSize()(0), nCells), BufferRegion::Quadrant::Undefined));
          }
        } else {
          // Two regions to drop.
          int firstIndex = index;
          int firstNCells = getSize()(i) - firstIndex;
          if (i == 0) {
            clearRows(firstIndex, firstNCells);
            newRegions.push_back(BufferRegion(Index(firstIndex, 0), Size(firstNCells, getSize()(1)), BufferRegion::Quadrant::Undefined));
          } else if (i == 1) {
            clearCols(firstIndex, firstNCells);
            newRegions.push_back(BufferRegion(Index(0, firstIndex), Size(getSize()(0), firstNCells), BufferRegion::Quadrant::Undefined));
          }

          int secondIndex = 0;
          int secondNCells = nCells - firstNCells;
          if (i == 0) {
            clearRows(secondIndex, secondNCells);
            newRegions.push_back(BufferRegion(Index(secondIndex, 0), Size(secondNCells, getSize()(1)), BufferRegion::Quadrant::Undefined));
          } else if (i == 1) {
            clearCols(secondIndex, secondNCells);
            newRegions.push_back(BufferRegion(Index(0, secondIndex), Size(getSize()(0), secondNCells), BufferRegion::Quadrant::Undefined));
          }
        }
      }
    }
  }

  // Update information.
  startIndex_ += indexShift;
  wrapIndexToRange(startIndex_, getSize());
  position_ += alignedPositionShift;

  // Check if map has been moved at all.
  return (indexShift.any() != 0);
}

bool GridMap::move(const Position& position)
{
  std::vector<BufferRegion> newRegions;
  return move(position, newRegions);
}

bool GridMap::addDataFrom(const GridMap& other, bool extendMap, bool overwriteData,
                          bool copyAllLayers, std::vector<std::string> layers)
{
  // Set the layers to copy.
  if (copyAllLayers) layers = other.getLayers();

  // Resize map.
  if (extendMap) extendToInclude(other);

  // Check if all layers to copy exist and add missing layers.
  for (const auto& layer : layers) {
    if (std::find(layers_.begin(), layers_.end(), layer) == layers_.end()) {
      add(layer);
    }
  }
  // Copy data.
  for (GridMapIterator iterator(*this); !iterator.isPastEnd(); ++iterator) {
    if (isValid(*iterator) && !overwriteData) continue;
    Position position;
    getPosition(*iterator, position);
    Index index;
    if (!other.isInside(position)) continue;
    other.getIndex(position, index);
    for (const auto& layer : layers) {
      if (!other.isValid(index, layer)) continue;
      at(layer, *iterator) = other.at(layer, index);
    }
  }

  return true;
}

bool GridMap::extendToInclude(const GridMap& other)
{
  // Get dimension of maps.
  Position topLeftCorner(position_.x() + length_.x() / 2.0, position_.y() + length_.y() / 2.0);
  Position bottomRightCorner(position_.x() - length_.x() / 2.0, position_.y() - length_.y() / 2.0);
  Position topLeftCornerOther(other.getPosition().x() + other.getLength().x() / 2.0, other.getPosition().y() + other.getLength().y() / 2.0);
  Position bottomRightCornerOther(other.getPosition().x() - other.getLength().x() / 2.0, other.getPosition().y() - other.getLength().y() / 2.0);
  // Check if map needs to be resized.
  bool resizeMap = false;
  Position extendedMapPosition = position_;
  Length extendedMapLength = length_;
  if (topLeftCornerOther.x() > topLeftCorner.x()) {
    extendedMapPosition.x() += (topLeftCornerOther.x() - topLeftCorner.x()) / 2.0;
    extendedMapLength.x() += topLeftCornerOther.x() - topLeftCorner.x();
    resizeMap = true;
  }
  if (topLeftCornerOther.y() > topLeftCorner.y()) {
    extendedMapPosition.y() += (topLeftCornerOther.y() - topLeftCorner.y()) / 2.0;
    extendedMapLength.y() += topLeftCornerOther.y() - topLeftCorner.y();
    resizeMap = true;
  }
  if (bottomRightCornerOther.x() < bottomRightCorner.x()) {
    extendedMapPosition.x() -= (bottomRightCorner.x() - bottomRightCornerOther.x()) / 2.0;
    extendedMapLength.x() += bottomRightCorner.x() - bottomRightCornerOther.x();
    resizeMap = true;
  }
  if (bottomRightCornerOther.y() < bottomRightCorner.y()) {
    extendedMapPosition.y() -= (bottomRightCorner.y() - bottomRightCornerOther.y()) / 2.0;
    extendedMapLength.y() += bottomRightCorner.y() - bottomRightCornerOther.y();
    resizeMap = true;
  }
  // Resize map and copy data to new map.
  if (resizeMap) {
    GridMap mapCopy = *this;
    setGeometry(extendedMapLength, resolution_, extendedMapPosition);
    // Align new map with old one.
    Vector shift = position_ - mapCopy.getPosition();
    shift.x() = std::fmod(shift.x(), resolution_);
    shift.y() = std::fmod(shift.y(), resolution_);
    if (std::abs(shift.x()) < resolution_ / 2.0) {
      position_.x() -= shift.x();
    } else {
      position_.x() += resolution_ - shift.x();
    }
    if (size_.x() % 2 != mapCopy.getSize().x() % 2) {
      position_.x() += -std::copysign(resolution_ / 2.0, shift.x());
    }
    if (std::abs(shift.y()) < resolution_ / 2.0) {
      position_.y() -= shift.y();
    } else {
      position_.y() += resolution_ - shift.y();
    }
    if (size_.y() % 2 != mapCopy.getSize().y() % 2) {
      position_.y() += -std::copysign(resolution_ / 2.0, shift.y());
    }
    // Copy data.
    for (GridMapIterator iterator(*this); !iterator.isPastEnd(); ++iterator) {
      if (isValid(*iterator)) continue;
      Position position;
      getPosition(*iterator, position);
      Index index;
      if (!mapCopy.isInside(position)) continue;
      mapCopy.getIndex(position, index);
      for (const auto& layer : layers_) {
        at(layer, *iterator) = mapCopy.at(layer, index);
      }
    }
  }
  return true;
}

void GridMap::setTimestamp(const Time timestamp)
{
  timestamp_ = timestamp;
}

Time GridMap::getTimestamp() const
{
  return timestamp_;
}

void GridMap::resetTimestamp()
{
  timestamp_ = 0.0;
}

void GridMap::setFrameId(const std::string& frameId)
{
  frameId_ = frameId;
}

const std::string& GridMap::getFrameId() const
{
  return frameId_;
}

const Length& GridMap::getLength() const
{
  return length_;
}

const Position& GridMap::getPosition() const
{
  return position_;
}

double GridMap::getResolution() const
{
  return resolution_;
}

const Size& GridMap::getSize() const
{
  return size_;
}

void GridMap::setStartIndex(const Index& startIndex) {
  startIndex_ = startIndex;
}

const Index& GridMap::getStartIndex() const
{
  return startIndex_;
}

bool GridMap::isDefaultStartIndex() const
{
  return (startIndex_ == 0).all();
}

void GridMap::convertToDefaultStartIndex()
{
  if (isDefaultStartIndex()) return;

  std::vector<BufferRegion> bufferRegions;
  if (!getBufferRegionsForSubmap(bufferRegions, startIndex_, size_, size_, startIndex_)) {
    throw std::out_of_range("Cannot access submap of this size.");
  }

  for (auto& data : data_) {
    auto tempData(data.second);
    for (const auto& bufferRegion : bufferRegions) {
      Index index = bufferRegion.getStartIndex();
      Size size = bufferRegion.getSize();

      if (bufferRegion.getQuadrant() == BufferRegion::Quadrant::TopLeft) {
        tempData.topLeftCorner(size(0), size(1)) = data.second.block(index(0), index(1), size(0), size(1));
      } else if (bufferRegion.getQuadrant() == BufferRegion::Quadrant::TopRight) {
        tempData.topRightCorner(size(0), size(1)) = data.second.block(index(0), index(1), size(0), size(1));
      } else if (bufferRegion.getQuadrant() == BufferRegion::Quadrant::BottomLeft) {
        tempData.bottomLeftCorner(size(0), size(1)) = data.second.block(index(0), index(1), size(0), size(1));
      } else if (bufferRegion.getQuadrant() == BufferRegion::Quadrant::BottomRight) {
        tempData.bottomRightCorner(size(0), size(1)) = data.second.block(index(0), index(1), size(0), size(1));
      }
    }
    data.second = tempData;
  }

  startIndex_.setZero();
}

void GridMap::clear(const std::string& layer)
{
  try {
    data_.at(layer).setConstant(NAN);
  } catch (const std::out_of_range& exception) {
    throw std::out_of_range("GridMap::clear(...) : No map layer '" + layer + "' available.");
  }
}

void GridMap::clearBasic()
{
  for (auto& layer : basicLayers_) {
    clear(layer);
  }
}

void GridMap::clearAll()
{
  for (auto& data : data_) {
    data.second.setConstant(NAN);
  }
}

void GridMap::clearRows(unsigned int index, unsigned int nRows)
{
  std::vector<std::string> layersToClear;
  if (basicLayers_.size() > 0) layersToClear = basicLayers_;
  else layersToClear = layers_;
  for (auto& layer : layersToClear) {
    data_.at(layer).block(index, 0, nRows, getSize()(1)).setConstant(NAN);
  }
}

void GridMap::clearCols(unsigned int index, unsigned int nCols)
{
  std::vector<std::string> layersToClear;
  if (basicLayers_.size() > 0) layersToClear = basicLayers_;
  else layersToClear = layers_;
  for (auto& layer : layersToClear) {
    data_.at(layer).block(0, index, getSize()(0), nCols).setConstant(NAN);
  }
}

bool GridMap::atPositionLinearInterpolated(const std::string& layer, const Position& position,
                                           float& value) const
{
  Position point;
  Index indices[4];
  bool idxTempDir;
  size_t idxShift[4];
  
  getIndex(position, indices[0]);
  getPosition(indices[0], point);
  
  if (position.x() >= point.x()) {
    indices[1] = indices[0] + Index(-1, 0); // Second point is above first point.
    idxTempDir = true;
  } else {
    indices[1] = indices[0] + Index(+1, 0);
    idxTempDir = false;
  }
  if (position.y() >= point.y()) {
      indices[2] = indices[0] + Index(0, -1); // Third point is right of first point.
      if(idxTempDir){ idxShift[0]=0; idxShift[1]=1; idxShift[2]=2; idxShift[3]=3; }
      else          { idxShift[0]=1; idxShift[1]=0; idxShift[2]=3; idxShift[3]=2; }
      
      
  } else { 
      indices[2] = indices[0] + Index(0, +1); 
      if(idxTempDir){ idxShift[0]=2; idxShift[1]=3; idxShift[2]=0; idxShift[3]=1; }
      else          { idxShift[0]=3; idxShift[1]=2; idxShift[2]=1; idxShift[3]=0; }
  }
  indices[3].x() = indices[1].x();
  indices[3].y() = indices[2].y();
  
  const Size& mapSize = getSize();
  const size_t bufferSize = mapSize(0) * mapSize(1);
  const size_t startIndexLin = getLinearIndexFromIndex(startIndex_, mapSize);
  const size_t endIndexLin = startIndexLin + bufferSize;
  const auto& layerMat = operator[](layer);
  float         f[4];

  for (size_t i = 0; i < 4; ++i) {
    const size_t indexLin = getLinearIndexFromIndex(indices[idxShift[i]], mapSize);
    if ((indexLin < startIndexLin) || (indexLin > endIndexLin)) return false;
    f[i] = layerMat(indexLin);
  }

  getPosition(indices[idxShift[0]], point);
  const Position positionRed     = ( position - point ) / resolution_;
  const Position positionRedFlip = Position(1.,1.) - positionRed;
  
  value = f[0] * positionRedFlip.x() * positionRedFlip.y() + 
          f[1] *     positionRed.x() * positionRedFlip.y() +
          f[2] * positionRedFlip.x() *     positionRed.y() +
          f[3] *     positionRed.x() *     positionRed.y();
  return true;
}

void GridMap::resize(const Index& size)
{
  size_ = size;
  for (auto& data : data_) {
    data.second.resize(size_(0), size_(1));
  }
}

} /* namespace */


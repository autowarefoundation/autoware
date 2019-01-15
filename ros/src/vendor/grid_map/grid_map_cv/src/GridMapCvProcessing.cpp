/*
 * GridMapCvProcessing.cpp
 *
 *  Created on: Apr 15, 2016
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map_cv/GridMapCvProcessing.hpp"
#include "grid_map_cv/GridMapCvConverter.hpp"

namespace grid_map {

GridMapCvProcessing::GridMapCvProcessing()
{
}

GridMapCvProcessing::~GridMapCvProcessing()
{
}

bool GridMapCvProcessing::changeResolution(const GridMap& gridMapSource,
                             GridMap& gridMapResult,
                             const double resolution,
                             const int interpolationAlgorithm)
{
  GridMap gridMapSourceCopy(gridMapSource);
  gridMapSourceCopy.convertToDefaultStartIndex();
  const double sizeFactor = gridMapSourceCopy.getResolution() / resolution;
  bool firstLayer = true;
  for (const auto& layer : gridMapSourceCopy.getLayers()) {
    cv::Mat imageSource, imageResult;
    const float minValue = gridMapSourceCopy.get(layer).minCoeffOfFinites();
    const float maxValue = gridMapSourceCopy.get(layer).maxCoeffOfFinites();
    const bool hasNaN = gridMapSourceCopy.get(layer).hasNaN();
    bool result;
    if (hasNaN) {
      result = GridMapCvConverter::toImage<unsigned short, 4>(gridMapSourceCopy, layer, CV_16UC4, minValue, maxValue, imageSource);
    } else {
      result = GridMapCvConverter::toImage<unsigned short, 1>(gridMapSourceCopy, layer, CV_16UC1, minValue, maxValue, imageSource);
    }
    if (!result) return false;
    cv::resize(imageSource, imageResult, cv::Size(0.0, 0.0), sizeFactor, sizeFactor, interpolationAlgorithm);
    if (firstLayer) {
      if (!GridMapCvConverter::initializeFromImage(imageResult, resolution, gridMapResult, gridMapSourceCopy.getPosition()))
        return false;
      firstLayer = false;
    }
    if (hasNaN) {
      result = GridMapCvConverter::addLayerFromImage<unsigned short, 4>(imageResult, layer, gridMapResult, minValue, maxValue);
    } else {
      result = GridMapCvConverter::addLayerFromImage<unsigned short, 1>(imageResult, layer, gridMapResult, minValue, maxValue);
    }
    if (!result) return false;
  }
  gridMapResult.setFrameId(gridMapSourceCopy.getFrameId());
  gridMapResult.setTimestamp(gridMapSourceCopy.getTimestamp());
  gridMapResult.setBasicLayers(gridMapSourceCopy.getBasicLayers());
  return true;
}

} /* namespace */

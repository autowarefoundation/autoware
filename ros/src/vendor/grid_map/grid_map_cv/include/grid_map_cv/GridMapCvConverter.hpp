/*
 * GridMapCvConverter.hpp
 *
 *  Created on: Apr 14, 2016
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include <grid_map_core/grid_map_core.hpp>

// OpenCV
#include <cv_bridge/cv_bridge.h>

// STD
#include <iostream>

namespace grid_map {

/*!
 * Conversions between grid maps and OpenCV images.
 */
class GridMapCvConverter
{
 public:
  /*!
   * Initializes the geometry of a grid map from an image. This changes
   * the geometry of the map and deletes all contents of the layers!
   * @param[in] image the image.
   * @param[in] resolution the desired resolution of the grid map [m/cell].
   * @param[out] gridMap the grid map to be initialized.
   * @param[in](optional) position the position of the grid map.
   * @return true if successful, false otherwise.
   */
  static bool initializeFromImage(const cv::Mat& image, const double resolution,
                                  grid_map::GridMap& gridMap, const grid_map::Position& position)
  {
    const double lengthX = resolution * image.rows;
    const double lengthY = resolution * image.cols;
    Length length(lengthX, lengthY);
    gridMap.setGeometry(length, resolution, position);
    return true;
  }

  /*!
   * Adds a layer with data from image.
   * @param[in] image the image to be added. If it is a color image
   * (bgr or bgra encoding), it will be transformed in a grayscale image.
   * @param[in] layer the layer that is filled with the image data.
   * @param[out] gridMap the grid map to be populated.
   * @param[in](optional) lowerValue value of the layer corresponding to black image pixels.
   * @param[in](optional) upperValue value of the layer corresponding to white image pixels.
   * @param[in](optional) alphaThreshold the threshold ([0.0, 1.0]) for the alpha value at which
   * cells in the grid map are marked as empty.
   * @return true if successful, false otherwise.
   */
  template<typename Type_, int NChannels_>
  static bool addLayerFromImage(const cv::Mat& image, const std::string& layer,
                                grid_map::GridMap& gridMap, const float lowerValue = 0.0,
                                const float upperValue = 1.0, const double alphaThreshold = 0.5)
  {
    if (gridMap.getSize()(0) != image.rows || gridMap.getSize()(1) != image.cols) {
      std::cerr << "Image size does not correspond to grid map size!" << std::endl;
      return false;
    }

    bool isColor = false;
    if (image.channels() >= 3) isColor = true;
    bool hasAlpha = false;
    if (image.channels() >= 4) hasAlpha = true;

    cv::Mat imageMono;
    if (isColor && !hasAlpha) {
      cv::cvtColor(image, imageMono, CV_BGR2GRAY);
    } else if (isColor && hasAlpha) {
      cv::cvtColor(image, imageMono, CV_BGRA2GRAY);
    } else if (!isColor && !hasAlpha){
      imageMono = image;
    } else {
      std::cerr << "Something went wrong when adding grid map layer form image!" << std::endl;
      return false;
    }

    const float mapValueDifference = upperValue - lowerValue;

    float maxImageValue;
    if (std::is_same<Type_, float>::value || std::is_same<Type_, double>::value) {
      maxImageValue = 1.0;
    } else if (std::is_same<Type_, unsigned short>::value || std::is_same<Type_, unsigned char>::value) {
      maxImageValue = (float)std::numeric_limits<Type_>::max();
    } else {
      std::cerr << "This image type is not supported." << std::endl;
      return false;
    }

    const Type_ alphaTreshold = (Type_)(alphaThreshold * maxImageValue);

    gridMap.add(layer);
    grid_map::Matrix& data = gridMap[layer];

    for (GridMapIterator iterator(gridMap); !iterator.isPastEnd(); ++iterator) {
      const Index index(*iterator);

      // Check for alpha layer.
      if (hasAlpha) {
        const Type_ alpha = image.at<cv::Vec<Type_, NChannels_>>(index(0), index(1))[NChannels_ - 1];
        if (alpha < alphaTreshold) continue;
      }

      // Compute value.
      const Type_ imageValue = imageMono.at<Type_>(index(0), index(1));
      const float mapValue = lowerValue + mapValueDifference * ((float) imageValue / maxImageValue);
      data(index(0), index(1)) = mapValue;
    }

    return true;
  };

  /*!
   * Adds a color layer with data from an image.
   * @param[in] image the image to be added (BGR format).
   * @param[in] layer the layer that is filled with the image.
   * @param[out] gridMap the grid map to be populated.
   * @return true if successful, false otherwise.
   */
  template<typename Type_, int NChannels_>
  static bool addColorLayerFromImage(const cv::Mat& image, const std::string& layer,
                                     grid_map::GridMap& gridMap)
  {
    if (gridMap.getSize()(0) != image.rows || gridMap.getSize()(1) != image.cols) {
      std::cerr << "Image size does not correspond to grid map size!" << std::endl;
      return false;
    }

    bool hasAlpha = false;
    if (image.channels() >= 4) hasAlpha = true;

    cv::Mat imageRGB;
    if (hasAlpha) {
      cv::cvtColor(image, imageRGB, CV_BGRA2RGB);
    } else {
      imageRGB = image;
    }

    gridMap.add(layer);

    for (GridMapIterator iterator(gridMap); !iterator.isPastEnd(); ++iterator) {
      const auto& cvColor = imageRGB.at<cv::Vec<Type_, 3>>((*iterator)(0), (*iterator)(1));
      Eigen::Vector3i colorVector;
      colorVector(0) = cvColor[0];
      colorVector(1) = cvColor[1];
      colorVector(2) = cvColor[2];
      colorVectorToValue(colorVector, gridMap.at(layer, *iterator));
    }

    return true;
  }

  /*!
   * Creates a cv mat from a grid map layer.
   * This conversion sets the corresponding black and white pixel value to the
   * min. and max. data of the layer data.
   * @param[in] grid map to be added.
   * @param[in] layer the layer that is converted to the image.
   * @param[in] encoding the desired encoding of the image.
   * @param[in] lowerValue the value of the layer corresponding to black image pixels.
   * @param[in] upperValue the value of the layer corresponding to white image pixels.
   * @param[out] image the image to be populated.
   * @return true if successful, false otherwise.
   */
  template<typename Type_, int NChannels_>
  static bool toImage(const grid_map::GridMap& gridMap, const std::string& layer,
                      const int encoding, cv::Mat& image)
  {
    const float minValue = gridMap.get(layer).minCoeffOfFinites();
    const float maxValue = gridMap.get(layer).maxCoeffOfFinites();
    return toImage<Type_, NChannels_>(gridMap, layer, encoding, minValue, maxValue, image);
  };

  /*!
   * Creates a cv mat from a grid map layer.
   * @param[in] grid map to be added.
   * @param[in] layer the layer that is converted to the image.
   * @param[in] encoding the desired encoding of the image.
   * @param[in] lowerValue the value of the layer corresponding to black image pixels.
   * @param[in] upperValue the value of the layer corresponding to white image pixels.
   * @param[out] image the image to be populated.
   * @return true if successful, false otherwise.
   */
  template<typename Type_, int NChannels_>
  static bool toImage(const grid_map::GridMap& gridMap, const std::string& layer,
                      const int encoding, const float lowerValue, const float upperValue,
                      cv::Mat& image)
  {
    // Initialize image.
    if (gridMap.getSize()(0) > 0 && gridMap.getSize()(1) > 0) {
      image = cv::Mat::zeros(gridMap.getSize()(0), gridMap.getSize()(1), encoding);
    } else {
      std::cerr << "Invalid grid map?" << std::endl;
      return false;
    }

    // Get max image value.
    Type_ imageMax;
    if (std::is_same<Type_, float>::value || std::is_same<Type_, double>::value) {
      imageMax = 1.0;
    } else if (std::is_same<Type_, unsigned short>::value || std::is_same<Type_, unsigned char>::value) {
      imageMax = (Type_)std::numeric_limits<Type_>::max();
    } else {
      std::cerr << "This image type is not supported." << std::endl;
      return false;
    }

    // Clamp outliers.
    grid_map::GridMap map = gridMap;
    map.get(layer) = map.get(layer).unaryExpr(grid_map::Clamp<float>(lowerValue, upperValue));
    const grid_map::Matrix& data = map[layer];

    // Convert to image.
    bool isColor = false;
    if (image.channels() >= 3) isColor = true;
    bool hasAlpha = false;
    if (image.channels() >= 4) hasAlpha = true;

    for (GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
      const Index index(*iterator);
      if (std::isfinite(data(index(0), index(1)))) {
        const float& value = data(index(0), index(1));
        const Type_ imageValue = (Type_) (((value - lowerValue) / (upperValue - lowerValue)) * (float) imageMax);
        const Index imageIndex(iterator.getUnwrappedIndex());
        unsigned int channel = 0;
        image.at<cv::Vec<Type_, NChannels_>>(imageIndex(0), imageIndex(1))[channel] = imageValue;

        if (isColor) {
          image.at<cv::Vec<Type_, NChannels_>>(imageIndex(0), imageIndex(1))[++channel] = imageValue;
          image.at<cv::Vec<Type_, NChannels_>>(imageIndex(0), imageIndex(1))[++channel] = imageValue;
        }
        if (hasAlpha) {
          image.at<cv::Vec<Type_, NChannels_>>(imageIndex(0), imageIndex(1))[++channel] = imageMax;
        }
      }
    }

    return true;
  };

};

} /* namespace */

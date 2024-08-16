// Copyright 2024 The Autoware Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#pragma once
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc_c.h>

#include <functional>
#include <string>
#include <vector>

namespace autoware::traffic_light::visualization
{
/**
 * @brief A struct of parameters to load shape image.
 */
struct ShapeImgParam
{
  std::string filename;  //!< Filename of shape image.
  bool h_flip;           //!< Whether to flip horizontally
  bool v_flip;           //!< Whether to flip vertically
};

/**
 * @brief Draw traffic light shapes on the camera view image.
 * @param image Camera view image.
 * @param params Shape parameters to load shape image.
 * @param size Shape image size to resize.
 * @param position Top-left position of a ROI.
 * @param color Rectangle color.
 * @param probability Classification probability.
 */
void drawShape(
  cv::Mat & image, const std::vector<ShapeImgParam> & params, int size, const cv::Point & position,
  const cv::Scalar & color, float probability);

/**
 * @brief Load shape images and concatenate them.
 * @param params Parameters for each shape image.
 * @param size Image size to resize.
 * @param scale_factor Scale factor to resize.
 * @return If no parameter is specified returns empty Mat, otherwise returns horizontally
 * concatenated image.
 */
cv::Mat loadShapeImage(
  const std::vector<ShapeImgParam> & params, int size, double scale_factor = 0.3);

/**
 * @brief Load parameter of circle.
 *
 * @return Parameter of circle.
 */
inline ShapeImgParam circleImgParam()
{
  return {"circle.png", false, false};
}

/**
 * @brief Load parameter of left-arrow.
 *
 * @return Parameter of left-arrow.
 */
inline ShapeImgParam leftArrowImgParam()
{
  return {"left_arrow.png", false, false};
}

/**
 * @brief Load parameter of right-arrow.
 *
 * @return Parameter of right-arrow, the image is flipped left-arrow horizontally.
 */
inline ShapeImgParam rightArrowImgParam()
{
  return {"left_arrow.png", true, false};
}

/**
 * @brief Load parameter of straight-arrow.
 *
 * @return Parameter of straight-arrow.
 */
inline ShapeImgParam straightArrowImgParam()
{
  return {"straight_arrow.png", false, false};
}

/**
 * @brief Load parameter of down-arrow.
 *
 * @return Parameter of down-arrow, the image is flipped straight-arrow vertically.
 */
inline ShapeImgParam downArrowImgParam()
{
  return {"straight_arrow.png", false, true};
}

/**
 * @brief Load parameter of straight-left-arrow.
 *
 * @return Parameter of straight-left-arrow, the image is flipped down-left-arrow vertically.
 */
inline ShapeImgParam straightLeftArrowImgParam()
{
  return {"down_left_arrow.png", false, true};
}

/**
 * @brief Load parameter of straight-right-arrow.
 *
 * @return Parameter of straight-right-arrow, the image is flipped down-left-arrow both horizontally
 * and vertically.
 */
inline ShapeImgParam straightRightArrowImgParam()
{
  return {"down_left_arrow.png", true, true};
}

/**
 * @brief Load parameter of down-left-arrow.
 *
 * @return Parameter of down-left-arrow.
 */
inline ShapeImgParam downLeftArrowImgParam()
{
  return {"down_left_arrow.png", false, false};
}

/**
 * @brief Load parameter of down-right-arrow.
 *
 * @return Parameter of down-right-arrow, the image is flipped straight-arrow horizontally.
 */
inline ShapeImgParam downRightArrowImgParam()
{
  return {"down_left_arrow.png", true, false};
}

/**
 * @brief Load parameter of cross-arrow.
 *
 * @return Parameter of cross-arrow.
 */
inline ShapeImgParam crossImgParam()
{
  return {"cross.png", false, false};
}

/**
 * @brief Load parameter of unknown shape.
 *
 * @return Parameter of unkown shape.
 */
inline ShapeImgParam unknownImgParam()
{
  return {"unknown.png", false, false};
}

/**
 * @brief Draw traffic light shapes on the camera view image.
 * @param image Camera view image.
 * @param shapes Shape names.
 * @param size Shape image size to resize.
 * @param position Top-left position of a ROI.
 * @param color Color of traffic light.
 * @param probability Classification probability.
 */
void drawTrafficLightShape(
  cv::Mat & image, const std::vector<std::string> & shapes, int size, const cv::Point & position,
  const cv::Scalar & color, float probability);

}  // namespace autoware::traffic_light::visualization

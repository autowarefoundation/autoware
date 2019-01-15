/*
 * GridMapVisualizationHelpers.cpp
 *
 *  Created on: Jun 24, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map_visualization/GridMapVisualizationHelpers.hpp"

#include <grid_map_ros/GridMapMsgHelpers.hpp>
#include <grid_map_core/GridMapMath.hpp>

using namespace Eigen;

namespace grid_map_visualization {

void getColorMessageFromColorVector(std_msgs::ColorRGBA& colorMessage, const Eigen::Vector3f& colorVector, bool resetTransparency)
{
  colorMessage.r = colorVector(0);
  colorMessage.g = colorVector(1);
  colorMessage.b = colorVector(2);
  if (resetTransparency) colorMessage.a = 1.0;
}

void getColorVectorFromColorMessage(Eigen::Vector3f& colorVector, const std_msgs::ColorRGBA& colorMessage)
{
  colorVector << colorMessage.r, colorMessage.g, colorMessage.b;
}

void setColorFromColorValue(std_msgs::ColorRGBA& color, const unsigned long& colorValue, bool resetTransparency)
{
  Vector3f colorVector;
  grid_map::colorValueToVector(colorValue, colorVector);
  getColorMessageFromColorVector(color, colorVector, resetTransparency);
}

void setColorChannelFromValue(float& colorChannel, const double value, const double lowerValueBound,
                              const double upperValueBound, const bool invert, const double colorChannelLowerValue,
                              const double colorChannelUpperValue)
{
  float tempColorChannelLowerValue = colorChannelLowerValue;
  float tempColorChannelUpperValue = colorChannelUpperValue;

  if (invert)
  {
    tempColorChannelLowerValue = colorChannelUpperValue;
    tempColorChannelUpperValue = colorChannelLowerValue;
  }

  colorChannel = static_cast<float>(computeLinearMapping(value, lowerValueBound, upperValueBound, tempColorChannelLowerValue, tempColorChannelUpperValue));
}

void interpolateBetweenColors(std_msgs::ColorRGBA& color, const std_msgs::ColorRGBA& colorForLowerValue,
                              const std_msgs::ColorRGBA& colorForUpperValue, const double value,
                              const double lowerValueBound, const double upperValueBound)
{
  setColorChannelFromValue(color.r, value, lowerValueBound, upperValueBound, false, colorForLowerValue.r, colorForUpperValue.r);
  setColorChannelFromValue(color.g, value, lowerValueBound, upperValueBound, false, colorForLowerValue.g, colorForUpperValue.g);
  setColorChannelFromValue(color.b, value, lowerValueBound, upperValueBound, false, colorForLowerValue.b, colorForUpperValue.b);
}

void setSaturationFromValue(std_msgs::ColorRGBA& color, const double value, const double lowerValueBound,
                            const double upperValueBound, const double maxSaturation, const double minSaturation)
{
  // Based on "changeSaturation" function by Darel Rex Finley.
  const Eigen::Array3f HspFactors(.299, .587, .114); // see http://alienryderflex.com/hsp.html
  float saturationChange = static_cast<float>(computeLinearMapping(value, value, upperValueBound, maxSaturation, minSaturation));
  Vector3f colorVector;
  getColorVectorFromColorMessage(colorVector, color);
  float perceivedBrightness = sqrt((colorVector.array().square() * HspFactors).sum());
  colorVector = perceivedBrightness + saturationChange * (colorVector.array() - perceivedBrightness);
  colorVector = (colorVector.array().min(Array3f::Ones())).matrix();
  getColorMessageFromColorVector(color, colorVector, false);
}

void setColorFromValue(std_msgs::ColorRGBA& color, const double value, const double lowerValueBound, const double upperValueBound)
{
  Vector3f hsl; // Hue: [0, 2 Pi], Saturation and Lightness: [0, 1]
  Vector3f rgb;

  hsl[0] = static_cast<float>(computeLinearMapping(value, lowerValueBound, upperValueBound, 0.0, 2.0 * M_PI));
  hsl[1] = 1.0;
  hsl[2] = 1.0;

  float offset = 2.0 / 3.0 * M_PI;
  Array3f rgbOffset(0, -offset, offset);
  rgb = ((rgbOffset + hsl[0]).cos() + 0.5).min(Array3f::Ones()).max(Array3f::Zero()) * hsl[2];
  float white = Vector3f(0.3, 0.59, 0.11).transpose() * rgb;
  float saturation = 1.0 - hsl[1];
  rgb = rgb + ((-rgb.array() + white) * saturation).matrix();

  getColorMessageFromColorVector(color, rgb, false);
}

double computeLinearMapping(
    const double& sourceValue, const double& sourceLowerValue, const double& sourceUpperValue,
    const double& mapLowerValue, const double& mapUpperValue)
{
  double m = (mapLowerValue - mapUpperValue) / (sourceLowerValue - sourceUpperValue);
  double b = mapUpperValue - m * sourceUpperValue;
  double mapValue = m * sourceValue + b;
  if (mapLowerValue < mapUpperValue)
  {
    mapValue = std::max(mapValue, mapLowerValue);
    mapValue = std::min(mapValue, mapUpperValue);
  }
  else
  {
    mapValue = std::min(mapValue, mapLowerValue);
    mapValue = std::max(mapValue, mapUpperValue);
  }
  return mapValue;
}

} /* namespace */

/*
 * GridMapVisualizationHelpers.hpp
 *
 *  Created on: Jun 24, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// ROS
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>

// Eigen
#include <Eigen/Core>

namespace grid_map_visualization {

/*!
 * Create a color message from a color vector.
 * @param[out] colorMessage the color message.
 * @param[in] colorVector the color vector
 * @param[in] resetTransparency if transparency should be reset (to fully visible) or left at current value.
 */
void getColorMessageFromColorVector(std_msgs::ColorRGBA& colorMessage, const Eigen::Vector3f& colorVector, bool resetTransparency = true);

/*!
 * Create a color vector from a color message.
 * @param[out] colorVector the color vector.
 * @param[in] colorMessage the color message.
 */
void getColorVectorFromColorMessage(Eigen::Vector3f& colorVector, const std_msgs::ColorRGBA& colorMessage);

/*!
 * Sets a color message from a color value.
 * @param[out] color the color message.
 * @param[in] colorValue the color value.
 * @param[in] resetTransparency if transparency should be reset (to fully visible) or left at current value.
 */
void setColorFromColorValue(std_msgs::ColorRGBA& color, const unsigned long& colorValue, bool resetTransparency = true);

/*!
 * Set the color channel from a scalar value.
 * @param[out] colorChannel the color channel to be set.
 * @param[in] value the scalar value.
 * @param[in] lowerValueBound the lower boundary of the value.
 * @param[in] upperValueBound the upper boundary of the value.
 * @param[in] invert if interpolation should be inverted.
 * @param[in] colorChannelLowerValue the lower value for the color channel.
 * @param[in] colorChannelUpperValue the upper value for the color channel.
 */
void setColorChannelFromValue(float& colorChannel, const double value, const double lowerValueBound,
                              const double upperValueBound, const bool invert = false, const double colorChannelLowerValue = 0.0,
                              const double colorChannelUpperValue = 1.0);

/*!
 * Sets the color to the interpolation between two colors based on a scalar value.
 * @param[out] color the color the be set.
 * @param[in] colorForLowerValue the color for the lower value boundary.
 * @param[in] colorForUpperValue the color for the upper value boundary.
 * @param[in] value the scalar value.
 * @param[in] lowerValueBound the lower boundary of the value.
 * @param[in] upperValueBound the upper boundary of the value.
 */
void interpolateBetweenColors(std_msgs::ColorRGBA& color, const std_msgs::ColorRGBA& colorForLowerValue,
                              const std_msgs::ColorRGBA& colorForUpperValue, const double value,
                              const double lowerValueBound, const double upperValueBound);

/*!
 * Sets the saturation of the color from a scalar value.
 * @param[out] color the color the be set.
 * @param[in] value the scalar value.
 * @param[in] lowerValueBound the lower boundary of the value.
 * @param[in] upperValueBound the upper boundary of the value.
 * @param[in] maxSaturation the maximum saturation.
 * @param[in] minSaturation the minimum saturation.
 */
void setSaturationFromValue(std_msgs::ColorRGBA& color, const double value, const double lowerValueBound,
                            const double upperValueBound, const double maxSaturation, const double minSaturation);

/*!
 * Set the color from the rainbow color spectrum based on scalar value.
 * @param[out] color the color the be set.
 * @param[in] value the scalar value.
 * @param[in] lowerValueBound the lower boundary of the value.
 * @param[in] upperValueBound the upper boundary of the value.
 */
void setColorFromValue(std_msgs::ColorRGBA& color, const double value, const double lowerValueBound,
                       const double upperValueBound);

/*!
 * Computes a linear mapping for a query from the source and to the map.
 * @param sourceValue the query from the source.
 * @param sourceLowerValue the lower source boundary.
 * @param sourceUpperValue the upper source boundary.
 * @param mapLowerValue the lower map boundary.
 * @param mapUpperValue the upper map boundary.
 * @return the query mapped to the map.
 */
double computeLinearMapping(
    const double& sourceValue, const double& sourceLowerValue, const double& sourceUpperValue,
    const double& mapLowerValue, const double& mapUpperValue);

} /* namespace */

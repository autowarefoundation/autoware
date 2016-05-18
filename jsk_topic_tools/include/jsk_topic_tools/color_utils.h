// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/


#ifndef JSK_TOPIC_TOOLS_COLOR_UTILS_H_
#define JSK_TOPIC_TOOLS_COLOR_UTILS_H_

#include <std_msgs/ColorRGBA.h>

/**
 * @file color_utils.h
 * @brief Provide color utility functions
 */

namespace jsk_topic_tools
{
  /** @brief choose a color out of 20 good selected colors
   *
   * The color format is same as d3.js (https://github.com/mbostock/d3/wiki/Ordinal-Scales#category20)
   * @param[in] i index value
   * @return return std_msgs::ColorRGBA and each value is in range between 0 and 1.
   */
  std_msgs::ColorRGBA colorCategory20(int i);

  /** @brief returns heat-map color (http://en.wikipedia.org/wiki/Heat_map)
   *
   * @param[in] v Value to colorize. This should be in [0.0:1.0]
   * @return return std_msgs::ColorRGBA and each value is in range between 0 and 1.
   */
  std_msgs::ColorRGBA heatColor(double v);
}

#endif

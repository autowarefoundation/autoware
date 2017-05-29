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

#include "jsk_topic_tools/color_utils.h"

namespace jsk_topic_tools
{
  std_msgs::ColorRGBA colorCategory20(int i)
  {
    std_msgs::ColorRGBA c;
    c.a = 1.0;
    switch (i % 20) {
    case 0:
    {
      c.r = 0.121569;
      c.g = 0.466667;
      c.b = 0.705882;
    }
    break;
    case 1:
    {
      c.r = 0.682353;
      c.g = 0.780392;
      c.b = 0.909804;
    }
    break;
    case 2:
    {
      c.r = 1.000000;
      c.g = 0.498039;
      c.b = 0.054902;
    }
    break;
    case 3:
    {
      c.r = 1.000000;
      c.g = 0.733333;
      c.b = 0.470588;
    }
    break;
    case 4:
    {
      c.r = 0.172549;
      c.g = 0.627451;
      c.b = 0.172549;
    }
    break;
    case 5:
    {
      c.r = 0.596078;
      c.g = 0.874510;
      c.b = 0.541176;
    }
    break;
    case 6:
    {
      c.r = 0.839216;
      c.g = 0.152941;
      c.b = 0.156863;
    }
    break;
    case 7:
    {
      c.r = 1.000000;
      c.g = 0.596078;
      c.b = 0.588235;
    }
    break;
    case 8:
    {
      c.r = 0.580392;
      c.g = 0.403922;
      c.b = 0.741176;
    }
    break;
    case 9:
    {
      c.r = 0.772549;
      c.g = 0.690196;
      c.b = 0.835294;
    }
    break;
    case 10:
    {
      c.r = 0.549020;
      c.g = 0.337255;
      c.b = 0.294118;
    }
    break;
    case 11:
    {
      c.r = 0.768627;
      c.g = 0.611765;
      c.b = 0.580392;
    }
    break;
    case 12:
    {
      c.r = 0.890196;
      c.g = 0.466667;
      c.b = 0.760784;
    }
    break;
    case 13:
    {
      c.r = 0.968627;
      c.g = 0.713725;
      c.b = 0.823529;
    }
    break;
    case 14:
    {
      c.r = 0.498039;
      c.g = 0.498039;
      c.b = 0.498039;
    }
    break;
    case 15:
    {
      c.r = 0.780392;
      c.g = 0.780392;
      c.b = 0.780392;
    }
    break;
    case 16:
    {
      c.r = 0.737255;
      c.g = 0.741176;
      c.b = 0.133333;
    }
    break;
    case 17:
    {
      c.r = 0.858824;
      c.g = 0.858824;
      c.b = 0.552941;
    }
    break;
    case 18:
    {
      c.r = 0.090196;
      c.g = 0.745098;
      c.b = 0.811765;
    }
    break;
    case 19:
    {
      c.r = 0.619608;
      c.g = 0.854902;
      c.b = 0.898039;
    }
    break;
    }
    return c;
  }

  std_msgs::ColorRGBA heatColor(double v)
  {
    double ratio = 2 * v;
    int b = std::max(0.0, 255 * (1.0 - ratio));
    int r = std::max(0.0, 255 * (ratio - 1.0));
    int g = 255 - b - r;
    std_msgs::ColorRGBA color;
    color.a = 1.0;
    color.r = r / 255.0;
    color.g = g / 255.0;
    color.b = b / 255.0;
    return color;
  }

}

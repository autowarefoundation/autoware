// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, JSK Lab
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

#include "jsk_recognition_utils/rgb_colors.h"
#include <opencv2/core/core.hpp>


namespace jsk_recognition_utils
{

  cv::Vec3d getRGBColor(const int color)
  {
    cv::Vec3d c;
    switch (color % 146) {
      case ALICEBLUE: c = cv::Vec3d(0.941, 0.973, 1); break;
      case ANTIQUEWHITE: c = cv::Vec3d(0.98, 0.922, 0.843); break;
      case AQUA: c = cv::Vec3d(0, 1, 1); break;
      case AQUAMARINE: c = cv::Vec3d(0.498, 1, 0.831); break;
      case AZURE: c = cv::Vec3d(0.941, 1, 1); break;
      case BEIGE: c = cv::Vec3d(0.961, 0.961, 0.863); break;
      case BISQUE: c = cv::Vec3d(1, 0.894, 0.769); break;
      case BLACK: c = cv::Vec3d(0, 0, 0); break;
      case BLANCHEDALMOND: c = cv::Vec3d(1, 0.922, 0.804); break;
      case BLUE: c = cv::Vec3d(0, 0, 1); break;
      case BLUEVIOLET: c = cv::Vec3d(0.541, 0.169, 0.886); break;
      case BROWN: c = cv::Vec3d(0.647, 0.165, 0.165); break;
      case BURLYWOOD: c = cv::Vec3d(0.871, 0.722, 0.529); break;
      case CADETBLUE: c = cv::Vec3d(0.373, 0.62, 0.627); break;
      case CHARTREUSE: c = cv::Vec3d(0.498, 1, 0); break;
      case CHOCOLATE: c = cv::Vec3d(0.824, 0.412, 0.118); break;
      case CORAL: c = cv::Vec3d(1, 0.498, 0.314); break;
      case CORNFLOWERBLUE: c = cv::Vec3d(0.392, 0.584, 0.929); break;
      case CORNSILK: c = cv::Vec3d(1, 0.973, 0.863); break;
      case CRIMSON: c = cv::Vec3d(0.863, 0.0784, 0.235); break;
      case CYAN: c = cv::Vec3d(0, 1, 1); break;
      case DARKBLUE: c = cv::Vec3d(0, 0, 0.545); break;
      case DARKCYAN: c = cv::Vec3d(0, 0.545, 0.545); break;
      case DARKGOLDENROD: c = cv::Vec3d(0.722, 0.525, 0.0431); break;
      case DARKGRAY: c = cv::Vec3d(0.663, 0.663, 0.663); break;
      case DARKGREEN: c = cv::Vec3d(0, 0.392, 0); break;
      case DARKGREY: c = cv::Vec3d(0.663, 0.663, 0.663); break;
      case DARKKHAKI: c = cv::Vec3d(0.741, 0.718, 0.42); break;
      case DARKMAGENTA: c = cv::Vec3d(0.545, 0, 0.545); break;
      case DARKOLIVEGREEN: c = cv::Vec3d(0.333, 0.42, 0.184); break;
      case DARKORANGE: c = cv::Vec3d(1, 0.549, 0); break;
      case DARKORCHID: c = cv::Vec3d(0.6, 0.196, 0.8); break;
      case DARKRED: c = cv::Vec3d(0.545, 0, 0); break;
      case DARKSALMON: c = cv::Vec3d(0.914, 0.588, 0.478); break;
      case DARKSEAGREEN: c = cv::Vec3d(0.561, 0.737, 0.561); break;
      case DARKSLATEBLUE: c = cv::Vec3d(0.282, 0.239, 0.545); break;
      case DARKSLATEGRAY: c = cv::Vec3d(0.184, 0.31, 0.31); break;
      case DARKSLATEGREY: c = cv::Vec3d(0.184, 0.31, 0.31); break;
      case DARKTURQUOISE: c = cv::Vec3d(0, 0.808, 0.82); break;
      case DARKVIOLET: c = cv::Vec3d(0.58, 0, 0.827); break;
      case DEEPPINK: c = cv::Vec3d(1, 0.0784, 0.576); break;
      case DEEPSKYBLUE: c = cv::Vec3d(0, 0.749, 1); break;
      case DIMGRAY: c = cv::Vec3d(0.412, 0.412, 0.412); break;
      case DIMGREY: c = cv::Vec3d(0.412, 0.412, 0.412); break;
      case DODGERBLUE: c = cv::Vec3d(0.118, 0.565, 1); break;
      case FIREBRICK: c = cv::Vec3d(0.698, 0.133, 0.133); break;
      case FLORALWHITE: c = cv::Vec3d(1, 0.98, 0.941); break;
      case FORESTGREEN: c = cv::Vec3d(0.133, 0.545, 0.133); break;
      case FUCHSIA: c = cv::Vec3d(1, 0, 1); break;
      case GAINSBORO: c = cv::Vec3d(0.863, 0.863, 0.863); break;
      case GHOSTWHITE: c = cv::Vec3d(0.973, 0.973, 1); break;
      case GOLD: c = cv::Vec3d(1, 0.843, 0); break;
      case GOLDENROD: c = cv::Vec3d(0.855, 0.647, 0.125); break;
      case GRAY: c = cv::Vec3d(0.502, 0.502, 0.502); break;
      case GREEN: c = cv::Vec3d(0, 0.502, 0); break;
      case GREENYELLOW: c = cv::Vec3d(0.678, 1, 0.184); break;
      case GREY: c = cv::Vec3d(0.502, 0.502, 0.502); break;
      case HONEYDEW: c = cv::Vec3d(0.941, 1, 0.941); break;
      case HOTPINK: c = cv::Vec3d(1, 0.412, 0.706); break;
      case INDIANRED: c = cv::Vec3d(0.804, 0.361, 0.361); break;
      case INDIGO: c = cv::Vec3d(0.294, 0, 0.51); break;
      case IVORY: c = cv::Vec3d(1, 1, 0.941); break;
      case KHAKI: c = cv::Vec3d(0.941, 0.902, 0.549); break;
      case LAVENDER: c = cv::Vec3d(0.902, 0.902, 0.98); break;
      case LAVENDERBLUSH: c = cv::Vec3d(1, 0.941, 0.961); break;
      case LAWNGREEN: c = cv::Vec3d(0.486, 0.988, 0); break;
      case LEMONCHIFFON: c = cv::Vec3d(1, 0.98, 0.804); break;
      case LIGHTBLUE: c = cv::Vec3d(0.678, 0.847, 0.902); break;
      case LIGHTCORAL: c = cv::Vec3d(0.941, 0.502, 0.502); break;
      case LIGHTCYAN: c = cv::Vec3d(0.878, 1, 1); break;
      case LIGHTGOLDENRODYELLOW: c = cv::Vec3d(0.98, 0.98, 0.824); break;
      case LIGHTGRAY: c = cv::Vec3d(0.827, 0.827, 0.827); break;
      case LIGHTGREEN: c = cv::Vec3d(0.565, 0.933, 0.565); break;
      case LIGHTGREY: c = cv::Vec3d(0.827, 0.827, 0.827); break;
      case LIGHTPINK: c = cv::Vec3d(1, 0.714, 0.757); break;
      case LIGHTSALMON: c = cv::Vec3d(1, 0.627, 0.478); break;
      case LIGHTSEAGREEN: c = cv::Vec3d(0.125, 0.698, 0.667); break;
      case LIGHTSKYBLUE: c = cv::Vec3d(0.529, 0.808, 0.98); break;
      case LIGHTSLATEGRAY: c = cv::Vec3d(0.467, 0.533, 0.6); break;
      case LIGHTSLATEGREY: c = cv::Vec3d(0.467, 0.533, 0.6); break;
      case LIGHTSTEELBLUE: c = cv::Vec3d(0.69, 0.769, 0.871); break;
      case LIGHTYELLOW: c = cv::Vec3d(1, 1, 0.878); break;
      case LIME: c = cv::Vec3d(0, 1, 0); break;
      case LIMEGREEN: c = cv::Vec3d(0.196, 0.804, 0.196); break;
      case LINEN: c = cv::Vec3d(0.98, 0.941, 0.902); break;
      case MAGENTA: c = cv::Vec3d(1, 0, 1); break;
      case MAROON: c = cv::Vec3d(0.502, 0, 0); break;
      case MEDIUMAQUAMARINE: c = cv::Vec3d(0.4, 0.804, 0.667); break;
      case MEDIUMBLUE: c = cv::Vec3d(0, 0, 0.804); break;
      case MEDIUMORCHID: c = cv::Vec3d(0.729, 0.333, 0.827); break;
      case MEDIUMPURPLE: c = cv::Vec3d(0.576, 0.439, 0.859); break;
      case MEDIUMSEAGREEN: c = cv::Vec3d(0.235, 0.702, 0.443); break;
      case MEDIUMSLATEBLUE: c = cv::Vec3d(0.482, 0.408, 0.933); break;
      case MEDIUMSPRINGGREEN: c = cv::Vec3d(0, 0.98, 0.604); break;
      case MEDIUMTURQUOISE: c = cv::Vec3d(0.282, 0.82, 0.8); break;
      case MEDIUMVIOLETRED: c = cv::Vec3d(0.78, 0.0824, 0.522); break;
      case MIDNIGHTBLUE: c = cv::Vec3d(0.098, 0.098, 0.439); break;
      case MINTCREAM: c = cv::Vec3d(0.961, 1, 0.98); break;
      case MISTYROSE: c = cv::Vec3d(1, 0.894, 0.882); break;
      case MOCCASIN: c = cv::Vec3d(1, 0.894, 0.71); break;
      case NAVAJOWHITE: c = cv::Vec3d(1, 0.871, 0.678); break;
      case NAVY: c = cv::Vec3d(0, 0, 0.502); break;
      case OLDLACE: c = cv::Vec3d(0.992, 0.961, 0.902); break;
      case OLIVE: c = cv::Vec3d(0.502, 0.502, 0); break;
      case OLIVEDRAB: c = cv::Vec3d(0.42, 0.557, 0.137); break;
      case ORANGE: c = cv::Vec3d(1, 0.647, 0); break;
      case ORANGERED: c = cv::Vec3d(1, 0.271, 0); break;
      case ORCHID: c = cv::Vec3d(0.855, 0.439, 0.839); break;
      case PALEGOLDENROD: c = cv::Vec3d(0.933, 0.91, 0.667); break;
      case PALEGREEN: c = cv::Vec3d(0.596, 0.984, 0.596); break;
      case PALEVIOLETRED: c = cv::Vec3d(0.686, 0.933, 0.933); break;
      case PAPAYAWHIP: c = cv::Vec3d(1, 0.937, 0.835); break;
      case PEACHPUFF: c = cv::Vec3d(1, 0.855, 0.725); break;
      case PERU: c = cv::Vec3d(0.804, 0.522, 0.247); break;
      case PINK: c = cv::Vec3d(1, 0.753, 0.796); break;
      case PLUM: c = cv::Vec3d(0.867, 0.627, 0.867); break;
      case POWDERBLUE: c = cv::Vec3d(0.69, 0.878, 0.902); break;
      case PURPLE: c = cv::Vec3d(0.502, 0, 0.502); break;
      case RED: c = cv::Vec3d(1, 0, 0); break;
      case ROSYBROWN: c = cv::Vec3d(0.737, 0.561, 0.561); break;
      case ROYALBLUE: c = cv::Vec3d(0.255, 0.412, 0.882); break;
      case SADDLEBROWN: c = cv::Vec3d(0.545, 0.271, 0.0745); break;
      case SALMON: c = cv::Vec3d(0.98, 0.502, 0.447); break;
      case SANDYBROWN: c = cv::Vec3d(0.98, 0.643, 0.376); break;
      case SEAGREEN: c = cv::Vec3d(0.18, 0.545, 0.341); break;
      case SEASHELL: c = cv::Vec3d(1, 0.961, 0.933); break;
      case SIENNA: c = cv::Vec3d(0.627, 0.322, 0.176); break;
      case SILVER: c = cv::Vec3d(0.753, 0.753, 0.753); break;
      case SKYBLUE: c = cv::Vec3d(0.529, 0.808, 0.922); break;
      case SLATEBLUE: c = cv::Vec3d(0.416, 0.353, 0.804); break;
      case SLATEGRAY: c = cv::Vec3d(0.439, 0.502, 0.565); break;
      case SLATEGREY: c = cv::Vec3d(0.439, 0.502, 0.565); break;
      case SNOW: c = cv::Vec3d(1, 0.98, 0.98); break;
      case SPRINGGREEN: c = cv::Vec3d(0, 1, 0.498); break;
      case STEELBLUE: c = cv::Vec3d(0.275, 0.51, 0.706); break;
      case TAN: c = cv::Vec3d(0.824, 0.706, 0.549); break;
      case TEAL: c = cv::Vec3d(0, 0.502, 0.502); break;
      case THISTLE: c = cv::Vec3d(0.847, 0.749, 0.847); break;
      case TOMATO: c = cv::Vec3d(1, 0.388, 0.278); break;
      case TURQUOISE: c = cv::Vec3d(0.251, 0.878, 0.816); break;
      case VIOLET: c = cv::Vec3d(0.933, 0.51, 0.933); break;
      case WHEAT: c = cv::Vec3d(0.961, 0.871, 0.702); break;
      case WHITE: c = cv::Vec3d(1, 1, 1); break;
      case WHITESMOKE: c = cv::Vec3d(0.961, 0.961, 0.961); break;
      case YELLOW: c = cv::Vec3d(1, 1, 0); break;
      case YELLOWGREEN: c = cv::Vec3d(0.604, 0.804, 0.196); break;
    }  // switch
    return c;
  }

}

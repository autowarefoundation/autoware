// Copyright 2023 TIER IV, Inc.
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

#ifndef YABLOC_COMMON__COLOR_HPP_
#define YABLOC_COMMON__COLOR_HPP_

#include <opencv4/opencv2/core.hpp>

#include <std_msgs/msg/color_rgba.hpp>

namespace yabloc::common
{

struct Color
{
  Color(float r, float g, float b, float a = 1.0f) : r(r), g(g), b(b), a(a) {}

  explicit Color(const std_msgs::msg::ColorRGBA & rgba) : r(rgba.r), g(rgba.g), b(rgba.b), a(rgba.a)
  {
  }

  explicit Color(const cv::Scalar & rgb, float a = 1.0f)
  : r(rgb[2] / 255.f), g(rgb[1] / 255.f), b(rgb[0] / 255.f), a(a)
  {
  }

  operator uint32_t() const
  {
    union uchar4_uint32 {
      uint8_t rgba[4];
      uint32_t u32;
    };
    uchar4_uint32 tmp;
    tmp.rgba[0] = static_cast<uint8_t>(r * 255);
    tmp.rgba[1] = static_cast<uint8_t>(g * 255);
    tmp.rgba[2] = static_cast<uint8_t>(b * 255);
    tmp.rgba[3] = static_cast<uint8_t>(a * 255);
    return tmp.u32;
  }
  operator const cv::Scalar() const { return cv::Scalar(255 * b, 255 * g, 255 * r); }
  operator const std_msgs::msg::ColorRGBA() const
  {
    std_msgs::msg::ColorRGBA rgba;
    rgba.a = a;
    rgba.r = r;
    rgba.g = g;
    rgba.b = b;
    return rgba;
  }

  float r, g, b, a;
};

namespace color_scale
{
Color hsv_to_rgb(float h, float s, float v);
Color rainbow(float value);
Color blue_red(float value);
}  // namespace color_scale

}  // namespace yabloc::common

#endif  // YABLOC_COMMON__COLOR_HPP_

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

#include "yabloc_common/color.hpp"

namespace yabloc::common::color_scale
{
Color rainbow(float value)
{
  // clang-format off
  float r = 1.0f;
  float g = 1.0f;
  float b = 1.0f;
  value = std::clamp(value, 0.0f, 1.0f);
  if (value < 0.25f) {
    r = 0; g = 4 * (value);
  } else if (value < 0.5f) {
    r = 0; b = 1 + 4 * (0.25f - value);
  } else if (value < 0.75f) {
    r = 4 * (value - 0.5f); b = 0;
  } else {
    g = 1 + 4 * (0.75f - value); b = 0;
  }
  // clang-format on
  return {r, g, b};
}

Color hsv_to_rgb(float h_, float s_, float v_)
{
  const float h = std::clamp(h_, 0.f, 360.f);
  const float max = v_;
  const float min = max * (1.0f - s_);

  if (h < 60) return {max, h / 60 * (max - min) + min, min};
  if (h < 120) return {(120 - h) / 60 * (max - min) + min, max, min};
  if (h < 180) return {min, max, (h - 120) / 60 * (max - min) + min};
  if (h < 240) return {min, (240 - h) / 60 * (max - min) + min, max};
  if (h < 300) return {(h - 240) / 60 * (max - min) + min, min, max};
  return {max, min, (360 - h) / 60 * (max - min) + min};
}

Color blue_red(float value)
{
  value = std::clamp(value, 0.0f, 1.0f);
  float h = (value < 0.5f) ? 0.f : 240.f;
  float s = std::abs(value - 0.5f) / 0.5f;
  float v = 1.0f;
  return hsv_to_rgb(h, s, v);
  // return {h, s, v};
}

}  // namespace yabloc::common::color_scale

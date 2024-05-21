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

#ifndef MATERIAL_COLORS_HPP_
#define MATERIAL_COLORS_HPP_
#include <string>

namespace autoware
{
namespace state_rviz_plugin
{
namespace colors
{
struct MaterialColors
{
  std::string primary = "#8BD0F0";
  std::string surface_tint = "#8BD0F0";
  std::string on_primary = "#003546";
  std::string primary_container = "#004D64";
  std::string on_primary_container = "#BEE9FF";
  std::string secondary = "#B4CAD6";
  std::string on_secondary = "#1F333C";
  std::string secondary_container = "#354A54";
  std::string on_secondary_container = "#D0E6F2";
  std::string tertiary = "#C6C2EA";
  std::string on_tertiary = "#2F2D4D";
  std::string tertiary_container = "#454364";
  std::string on_tertiary_container = "#E3DFFF";
  std::string error = "#FFB4AB";
  std::string on_error = "#690005";
  std::string error_container = "#93000A";
  std::string on_error_container = "#FFDAD6";
  std::string background = "#0F1417";
  std::string on_background = "#DFE3E7";
  std::string surface = "#0F1417";
  std::string on_surface = "#DFE3E7";
  std::string surface_variant = "#40484C";
  std::string on_surface_variant = "#C0C8CD";
  std::string outline = "#8A9297";
  std::string outline_variant = "#40484C";
  std::string shadow = "#000000";
  std::string scrim = "#000000";
  std::string inverse_surface = "#DFE3E7";
  std::string inverse_on_surface = "#2C3134";
  std::string inverse_primary = "#126682";
  std::string primary_fixed = "#BEE9FF";
  std::string on_primary_fixed = "#001F2A";
  std::string primary_fixed_dim = "#8BD0F0";
  std::string on_primary_fixed_variant = "#004D64";
  std::string secondary_fixed = "#D0E6F2";
  std::string on_secondary_fixed = "#081E27";
  std::string secondary_fixed_dim = "#B4CAD6";
  std::string on_secondary_fixed_variant = "#354A54";
  std::string tertiary_fixed = "#E3DFFF";
  std::string on_tertiary_fixed = "#1A1836";
  std::string tertiary_fixed_dim = "#C6C2EA";
  std::string on_tertiary_fixed_variant = "#454364";
  std::string surface_dim = "#0F1417";
  std::string surface_bright = "#353A3D";
  std::string surface_container_lowest = "#0A0F11";
  std::string surface_container_low = "#171C1F";
  std::string surface_container = "#1B2023";
  std::string surface_container_high = "#262B2E";
  std::string surface_container_highest = "#303538";
  std::string disabled_elevated_button_bg = "#292D30";
  std::string success = "#8DF08B";
  std::string warning = "#EEF08B";
  std::string info = "#8BD0F0";
  std::string danger = "#F08B8B";
};

inline MaterialColors default_colors;
}  // namespace colors
}  // namespace state_rviz_plugin
}  // namespace autoware

#endif  // MATERIAL_COLORS_HPP_

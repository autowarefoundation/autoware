// Copyright 2021 Tier IV, Inc.
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
/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * Copyright (c) 2018, Bosch Software Innovations GmbH.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef DRIVABLE_AREA__DISPLAY_HPP_
#define DRIVABLE_AREA__DISPLAY_HPP_

#include <memory>
#include <string>
#include <vector>

#ifndef Q_MOC_RUN

#include <OgreMaterial.h>
#include <OgreSharedPtr.h>
#include <OgreTexture.h>
#include <OgreVector3.h>

#endif  // Q_MOC_RUN

#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"
#include "rviz_common/message_filter_display.hpp"
#include "rviz_default_plugins/displays/map/swatch.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

#include "autoware_auto_planning_msgs/msg/path.hpp"
#include "map_msgs/msg/occupancy_grid_update.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace Ogre
{
class ManualObject;
}  // namespace Ogre

namespace rviz_common::properties
{
class EnumProperty;
class FloatProperty;
class IntProperty;
class Property;
class QuaternionProperty;
class VectorProperty;

}  // namespace rviz_common::properties

namespace rviz_plugins
{
class AlphaSetter;
using rviz_default_plugins::displays::Swatch;

/**
 * \class AutowareDrivableAreaDisplay
 * \brief Displays a map along the XY plane.
 */
class RVIZ_DEFAULT_PLUGINS_PUBLIC AutowareDrivableAreaDisplay
: public rviz_common::MessageFilterDisplay<autoware_auto_planning_msgs::msg::Path>
{
  Q_OBJECT

public:
  // TODO(botteroa-si): Constructor for testing, remove once ros_nodes can be mocked and call
  // initialize() instead
  explicit AutowareDrivableAreaDisplay(rviz_common::DisplayContext * context);
  AutowareDrivableAreaDisplay();
  ~AutowareDrivableAreaDisplay() override;

  void onInitialize() override;
  void fixedFrameChanged() override;
  void reset() override;

  [[nodiscard]] float getResolution() const { return resolution_; }
  [[nodiscard]] size_t getWidth() const { return width_; }
  [[nodiscard]] size_t getHeight() const { return height_; }

  /** @brief Copy msg into current_map_ and call showMap(). */
  void processMessage(autoware_auto_planning_msgs::msg::Path::ConstSharedPtr msg) override;

public Q_SLOTS:  // NOLINT
  void showMap();

Q_SIGNALS:
  /** @brief Emitted when a new map is received*/
  void mapUpdated();

protected Q_SLOTS:
  void updateAlpha();
  void updateDrawUnder() const;
  void updatePalette();
  /** @brief Show current_map_ in the scene. */
  void transformMap();
  void updateMapUpdateTopic();

protected:  // NOLINT for Qt
  void updateTopic() override;
  void update(float wall_dt, float ros_dt) override;

  void subscribe() override;
  void unsubscribe() override;

  void onEnable() override;

  /** @brief Copy update's data into current_map_ and call showMap(). */
  void incomingUpdate(map_msgs::msg::OccupancyGridUpdate::ConstSharedPtr update);

  [[nodiscard]] bool updateDataOutOfBounds(
    map_msgs::msg::OccupancyGridUpdate::ConstSharedPtr update) const;
  void updateMapDataInMemory(map_msgs::msg::OccupancyGridUpdate::ConstSharedPtr update);

  void clear();

  void subscribeToUpdateTopic();
  void unsubscribeToUpdateTopic();

  void showValidMap();
  void resetSwatchesIfNecessary(size_t width, size_t height, float resolution);
  void createSwatches();
  static void doubleSwatchNumber(
    size_t & swatch_width, size_t & swatch_height, int & number_swatches);
  void tryCreateSwatches(
    size_t width, size_t height, float resolution, size_t swatch_width, size_t swatch_height,
    int number_swatches);
  static size_t getEffectiveDimension(
    size_t map_dimension, size_t swatch_dimension, size_t position);
  void updateSwatches() const;

  std::vector<std::shared_ptr<Swatch>> swatches_;
  std::vector<Ogre::TexturePtr> palette_textures_;
  std::vector<bool> color_scheme_transparency_;
  bool loaded_;

  float resolution_;
  size_t width_;
  size_t height_;
  std::string frame_;
  nav_msgs::msg::OccupancyGrid current_map_;

  rclcpp::Subscription<map_msgs::msg::OccupancyGridUpdate>::SharedPtr update_subscription_;
  rclcpp::QoS update_profile_;

  rviz_common::properties::RosTopicProperty * update_topic_property_;
  rviz_common::properties::QosProfileProperty * update_profile_property_;
  rviz_common::properties::FloatProperty * resolution_property_;
  rviz_common::properties::IntProperty * width_property_;
  rviz_common::properties::IntProperty * height_property_;
  rviz_common::properties::VectorProperty * position_property_;
  rviz_common::properties::QuaternionProperty * orientation_property_;
  rviz_common::properties::FloatProperty * alpha_property_;
  rviz_common::properties::Property * draw_under_property_;
  rviz_common::properties::EnumProperty * color_scheme_property_;
  rviz_common::properties::BoolProperty * transform_timestamp_property_;

  uint32_t update_messages_received_;
};

}  // namespace rviz_plugins

#endif  // DRIVABLE_AREA__DISPLAY_HPP_

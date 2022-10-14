// Copyright 2022 Tier IV, Inc.
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

// Copyright (c) 2014, JSK Lab
// All rights reserved.
//
// Software License Agreement (BSD License)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of {copyright_holder} nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.S SOFTWARE, EVEN IF ADVISED OF THE
//  POSSIBILITY OF SUCH DAMAGE.

#ifndef TIER4_DEBUG_RVIZ_PLUGIN__FLOAT32_MULTI_ARRAY_STAMPED_PIE_CHART_HPP_
#define TIER4_DEBUG_RVIZ_PLUGIN__FLOAT32_MULTI_ARRAY_STAMPED_PIE_CHART_HPP_

#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/parse_color.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/validate_floats.hpp>
#include <tier4_debug_rviz_plugin/jsk_overlay_utils.hpp>

#include <tier4_debug_msgs/msg/float32_multi_array_stamped.hpp>

#include <mutex>

namespace rviz_plugins
{
class Float32MultiArrayStampedPieChartDisplay : public rviz_common::Display
{
  Q_OBJECT
public:
  Float32MultiArrayStampedPieChartDisplay();
  virtual ~Float32MultiArrayStampedPieChartDisplay();

  // methods for OverlayPickerTool
  virtual bool isInRegion(int x, int y);
  virtual void movePosition(int x, int y);
  virtual void setPosition(int x, int y);
  virtual int getX() { return left_; }
  virtual int getY() { return top_; }

protected:
  virtual void subscribe();
  virtual void unsubscribe();
  virtual void onEnable();
  virtual void onDisable();
  virtual void onInitialize();
  virtual void processMessage(
    const tier4_debug_msgs::msg::Float32MultiArrayStamped::ConstSharedPtr msg);
  virtual void drawPlot(double val);
  virtual void update(float wall_dt, float ros_dt);
  // properties
  rviz_common::properties::StringProperty * update_topic_property_;
  rviz_common::properties::IntProperty * data_index_property_;
  rviz_common::properties::IntProperty * size_property_;
  rviz_common::properties::IntProperty * left_property_;
  rviz_common::properties::IntProperty * top_property_;
  rviz_common::properties::ColorProperty * fg_color_property_;
  rviz_common::properties::ColorProperty * bg_color_property_;
  rviz_common::properties::ColorProperty * text_color_property_;
  rviz_common::properties::FloatProperty * fg_alpha_property_;
  rviz_common::properties::FloatProperty * fg_alpha2_property_;
  rviz_common::properties::FloatProperty * bg_alpha_property_;
  rviz_common::properties::FloatProperty * text_alpha_property_;
  rviz_common::properties::IntProperty * text_size_property_;
  rviz_common::properties::FloatProperty * max_value_property_;
  rviz_common::properties::FloatProperty * min_value_property_;
  rviz_common::properties::BoolProperty * show_caption_property_;
  rviz_common::properties::BoolProperty * auto_color_change_property_;
  rviz_common::properties::ColorProperty * max_color_property_;
  rviz_common::properties::ColorProperty * med_color_property_;
  rviz_common::properties::FloatProperty * max_color_threshold_property_;
  rviz_common::properties::FloatProperty * med_color_threshold_property_;
  rviz_common::properties::BoolProperty * clockwise_rotate_property_;

  rclcpp::Subscription<tier4_debug_msgs::msg::Float32MultiArrayStamped>::SharedPtr sub_;
  int left_;
  int top_;
  uint16_t texture_size_;
  QColor fg_color_;
  QColor bg_color_;
  QColor max_color_;
  QColor med_color_;
  int text_size_;
  bool show_caption_;
  bool auto_color_change_;
  int caption_offset_;
  double fg_alpha_;
  double fg_alpha2_;
  double bg_alpha_;
  double max_value_;
  double min_value_;
  double max_color_threshold_;
  double med_color_threshold_;
  bool update_required_;
  bool first_time_;
  float data_;
  int data_index_{0};
  jsk_rviz_plugins::OverlayObject::Ptr overlay_;
  bool clockwise_rotate_;

  std::mutex mutex_;

protected Q_SLOTS:
  void updateTopic();
  void updateDataIndex();
  void updateSize();
  void updateTop();
  void updateLeft();
  void updateBGColor();
  void updateTextSize();
  void updateFGColor();
  void updateFGAlpha();
  void updateFGAlpha2();
  void updateBGAlpha();
  void updateMinValue();
  void updateMaxValue();
  void updateShowCaption();
  void updateAutoColorChange();
  void updateMaxColor();
  void updateMedColor();
  void updateMaxColorThreshold();
  void updateMedColorThreshold();
  void updateClockwiseRotate();

private:
};

}  // namespace rviz_plugins

#endif  // TIER4_DEBUG_RVIZ_PLUGIN__FLOAT32_MULTI_ARRAY_STAMPED_PIE_CHART_HPP_

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

#include <QPainter>
#include <rviz_common/display_context.hpp>
#include <rviz_common/uniform_string_stream.hpp>
#include <tier4_debug_rviz_plugin/float32_multi_array_stamped_pie_chart.hpp>

namespace rviz_plugins
{

Float32MultiArrayStampedPieChartDisplay::Float32MultiArrayStampedPieChartDisplay()
: rviz_common::Display(), update_required_(false), first_time_(true), data_(0.0)
{
  update_topic_property_ = new rviz_common::properties::StringProperty(
    "Topic", "", "tier4_debug_msgs::msg::Float32MultiArrayStamped topic to subscribe to.", this,
    SLOT(updateTopic()), this);
  data_index_property_ = new rviz_common::properties::IntProperty(
    "data index", 0, "data index in message to visualize", this, SLOT(updateDataIndex()));
  size_property_ = new rviz_common::properties::IntProperty(
    "size", 128, "size of the plotter window", this, SLOT(updateSize()));
  left_property_ = new rviz_common::properties::IntProperty(
    "left", 128, "left of the plotter window", this, SLOT(updateLeft()));
  top_property_ = new rviz_common::properties::IntProperty(
    "top", 128, "top of the plotter window", this, SLOT(updateTop()));
  fg_color_property_ = new rviz_common::properties::ColorProperty(
    "foreground color", QColor(25, 255, 240), "color to draw line", this, SLOT(updateFGColor()));
  fg_alpha_property_ = new rviz_common::properties::FloatProperty(
    "foreground alpha", 0.7, "alpha blending value for foreground", this, SLOT(updateFGAlpha()));
  fg_alpha2_property_ = new rviz_common::properties::FloatProperty(
    "foreground alpha 2", 0.4, "alpha blending value for foreground for indicator", this,
    SLOT(updateFGAlpha2()));
  bg_color_property_ = new rviz_common::properties::ColorProperty(
    "background color", QColor(0, 0, 0), "background color", this, SLOT(updateBGColor()));
  bg_alpha_property_ = new rviz_common::properties::FloatProperty(
    "background alpha", 0.0, "alpha blending value for background", this, SLOT(updateBGAlpha()));
  text_size_property_ = new rviz_common::properties::IntProperty(
    "text size", 14, "text size", this, SLOT(updateTextSize()));
  show_caption_property_ = new rviz_common::properties::BoolProperty(
    "show caption", false, "show caption", this, SLOT(updateShowCaption()));
  max_value_property_ = new rviz_common::properties::FloatProperty(
    "max value", 1.0, "max value of pie chart", this, SLOT(updateMaxValue()));
  min_value_property_ = new rviz_common::properties::FloatProperty(
    "min value", 0.0, "min value of pie chart", this, SLOT(updateMinValue()));
  auto_color_change_property_ = new rviz_common::properties::BoolProperty(
    "auto color change", false, "change the color automatically", this,
    SLOT(updateAutoColorChange()));
  max_color_property_ = new rviz_common::properties::ColorProperty(
    "max color", QColor(255, 0, 0), "only used if auto color change is set to True.", this,
    SLOT(updateMaxColor()));

  med_color_property_ = new rviz_common::properties::ColorProperty(
    "med color", QColor(255, 0, 0), "only used if auto color change is set to True.", this,
    SLOT(updateMedColor()));

  max_color_threshold_property_ = new rviz_common::properties::FloatProperty(
    "max color change threshold", 0, "change the max color at threshold", this,
    SLOT(updateMaxColorThreshold()));

  med_color_threshold_property_ = new rviz_common::properties::FloatProperty(
    "med color change threshold", 0, "change the med color at threshold ", this,
    SLOT(updateMedColorThreshold()));

  clockwise_rotate_property_ = new rviz_common::properties::BoolProperty(
    "clockwise rotate direction", false, "change the rotate direction", this,
    SLOT(updateClockwiseRotate()));
}

Float32MultiArrayStampedPieChartDisplay::~Float32MultiArrayStampedPieChartDisplay()
{
  if (overlay_->isVisible()) {
    overlay_->hide();
  }
  delete update_topic_property_;
  delete fg_color_property_;
  delete bg_color_property_;
  delete fg_alpha_property_;
  delete fg_alpha2_property_;
  delete bg_alpha_property_;
  delete top_property_;
  delete left_property_;
  delete size_property_;
  delete min_value_property_;
  delete max_value_property_;
  delete max_color_property_;
  delete med_color_property_;
  delete text_size_property_;
  delete show_caption_property_;
}

void Float32MultiArrayStampedPieChartDisplay::onInitialize()
{
  static int count = 0;
  rviz_common::UniformStringStream ss;
  ss << "Float32MultiArrayStampedPieChart" << count++;
  auto logger = context_->getRosNodeAbstraction().lock()->get_raw_node()->get_logger();
  overlay_.reset(new jsk_rviz_plugins::OverlayObject(scene_manager_, logger, ss.str()));
  onEnable();
  updateSize();
  updateLeft();
  updateTop();
  updateFGColor();
  updateBGColor();
  updateFGAlpha();
  updateFGAlpha2();
  updateBGAlpha();
  updateMinValue();
  updateMaxValue();
  updateTextSize();
  updateShowCaption();
  updateAutoColorChange();
  updateMaxColor();
  updateMedColor();
  updateMaxColorThreshold();
  updateMedColorThreshold();
  updateClockwiseRotate();
  overlay_->updateTextureSize(texture_size_, texture_size_ + caption_offset_);
  overlay_->hide();
}

void Float32MultiArrayStampedPieChartDisplay::update(
  [[maybe_unused]] float wall_dt, [[maybe_unused]] float ros_dt)
{
  if (update_required_) {
    update_required_ = false;
    overlay_->updateTextureSize(texture_size_, texture_size_ + caption_offset_);
    overlay_->setPosition(left_, top_);
    overlay_->setDimensions(overlay_->getTextureWidth(), overlay_->getTextureHeight());
    drawPlot(data_);
  }
}

void Float32MultiArrayStampedPieChartDisplay::processMessage(
  const tier4_debug_msgs::msg::Float32MultiArrayStamped::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!overlay_->isVisible()) {
    return;
  }
  if (data_index_ < 0 || static_cast<int>(msg->data.size()) <= data_index_) {
    return;
  }

  if (data_ != msg->data.at(data_index_) || first_time_) {
    first_time_ = false;
    data_ = msg->data.at(data_index_);
    update_required_ = true;
  }
}

void Float32MultiArrayStampedPieChartDisplay::drawPlot(double val)
{
  QColor fg_color(fg_color_);

  if (auto_color_change_) {
    double r = std::min(1.0, fabs((val - min_value_) / (max_value_ - min_value_)));
    if (r > 0.6) {
      double r2 = (r - 0.6) / 0.4;
      fg_color.setRed((max_color_.red() - fg_color_.red()) * r2 + fg_color_.red());
      fg_color.setGreen((max_color_.green() - fg_color_.green()) * r2 + fg_color_.green());
      fg_color.setBlue((max_color_.blue() - fg_color_.blue()) * r2 + fg_color_.blue());
    }
    if (max_color_threshold_ != 0) {
      if (r > max_color_threshold_) {
        fg_color.setRed(max_color_.red());
        fg_color.setGreen(max_color_.green());
        fg_color.setBlue(max_color_.blue());
      }
    }
    if (med_color_threshold_ != 0) {
      if (max_color_threshold_ > r && r > med_color_threshold_) {
        fg_color.setRed(med_color_.red());
        fg_color.setGreen(med_color_.green());
        fg_color.setBlue(med_color_.blue());
      }
    }
  }

  QColor fg_color2(fg_color);
  QColor bg_color(bg_color_);
  fg_color.setAlpha(fg_alpha_);
  fg_color2.setAlpha(fg_alpha2_);
  bg_color.setAlpha(bg_alpha_);
  int width = overlay_->getTextureWidth();
  int height = overlay_->getTextureHeight();
  {
    jsk_rviz_plugins::ScopedPixelBuffer buffer = overlay_->getBuffer();
    QImage Hud = buffer.getQImage(*overlay_, bg_color);
    QPainter painter(&Hud);
    painter.setRenderHint(QPainter::Antialiasing, true);

    const int outer_line_width = 5;
    const int value_line_width = 10;
    const int value_indicator_line_width = 2;
    const int value_padding = 5;

    const int value_offset = outer_line_width + value_padding + value_line_width / 2;

    painter.setPen(QPen(fg_color, outer_line_width, Qt::SolidLine));

    painter.drawEllipse(
      outer_line_width / 2, outer_line_width / 2, width - outer_line_width,
      height - outer_line_width - caption_offset_);

    painter.setPen(QPen(fg_color2, value_indicator_line_width, Qt::SolidLine));
    painter.drawEllipse(
      value_offset, value_offset, width - value_offset * 2,
      height - value_offset * 2 - caption_offset_);

    const double ratio = (val - min_value_) / (max_value_ - min_value_);
    const double rotate_direction = clockwise_rotate_ ? -1.0 : 1.0;
    const double ratio_angle = ratio * 360.0 * rotate_direction;
    const double start_angle_offset = -90;
    painter.setPen(QPen(fg_color, value_line_width, Qt::SolidLine));
    painter.drawArc(
      QRectF(
        value_offset, value_offset, width - value_offset * 2,
        height - value_offset * 2 - caption_offset_),
      start_angle_offset * 16, ratio_angle * 16);
    QFont font = painter.font();
    font.setPointSize(text_size_);
    font.setBold(true);
    painter.setFont(font);
    painter.setPen(QPen(fg_color, value_line_width, Qt::SolidLine));
    std::ostringstream s;
    s << std::fixed << std::setprecision(2) << val;
    painter.drawText(
      0, 0, width, height - caption_offset_, Qt::AlignCenter | Qt::AlignVCenter, s.str().c_str());

    // caption
    if (show_caption_) {
      painter.drawText(
        0, height - caption_offset_, width, caption_offset_, Qt::AlignCenter | Qt::AlignVCenter,
        getName());
    }

    // done
    painter.end();
    // Unlock the pixel buffer
  }
}

void Float32MultiArrayStampedPieChartDisplay::subscribe()
{
  std::string topic_name = update_topic_property_->getStdString();

  // NOTE: Remove all spaces since topic name filled with only spaces will crash
  topic_name.erase(std::remove(topic_name.begin(), topic_name.end(), ' '), topic_name.end());

  if (topic_name.length() > 0 && topic_name != "/") {
    rclcpp::Node::SharedPtr raw_node = context_->getRosNodeAbstraction().lock()->get_raw_node();
    sub_ = raw_node->create_subscription<tier4_debug_msgs::msg::Float32MultiArrayStamped>(
      topic_name, 1,
      std::bind(
        &Float32MultiArrayStampedPieChartDisplay::processMessage, this, std::placeholders::_1));
  }
}

void Float32MultiArrayStampedPieChartDisplay::unsubscribe() { sub_.reset(); }

void Float32MultiArrayStampedPieChartDisplay::onEnable()
{
  subscribe();
  overlay_->show();
  first_time_ = true;
}

void Float32MultiArrayStampedPieChartDisplay::onDisable()
{
  unsubscribe();
  overlay_->hide();
}

void Float32MultiArrayStampedPieChartDisplay::updateSize()
{
  std::lock_guard<std::mutex> lock(mutex_);

  texture_size_ = size_property_->getInt();
  update_required_ = true;
}

void Float32MultiArrayStampedPieChartDisplay::updateTop()
{
  top_ = top_property_->getInt();
  update_required_ = true;
}

void Float32MultiArrayStampedPieChartDisplay::updateLeft()
{
  left_ = left_property_->getInt();
  update_required_ = true;
}

void Float32MultiArrayStampedPieChartDisplay::updateBGColor()
{
  bg_color_ = bg_color_property_->getColor();
  update_required_ = true;
}

void Float32MultiArrayStampedPieChartDisplay::updateFGColor()
{
  fg_color_ = fg_color_property_->getColor();
  update_required_ = true;
}

void Float32MultiArrayStampedPieChartDisplay::updateFGAlpha()
{
  fg_alpha_ = fg_alpha_property_->getFloat() * 255.0;
  update_required_ = true;
}

void Float32MultiArrayStampedPieChartDisplay::updateFGAlpha2()
{
  fg_alpha2_ = fg_alpha2_property_->getFloat() * 255.0;
  update_required_ = true;
}

void Float32MultiArrayStampedPieChartDisplay::updateBGAlpha()
{
  bg_alpha_ = bg_alpha_property_->getFloat() * 255.0;
  update_required_ = true;
}

void Float32MultiArrayStampedPieChartDisplay::updateMinValue()
{
  min_value_ = min_value_property_->getFloat();
  update_required_ = true;
}

void Float32MultiArrayStampedPieChartDisplay::updateMaxValue()
{
  max_value_ = max_value_property_->getFloat();
  update_required_ = true;
}

void Float32MultiArrayStampedPieChartDisplay::updateTextSize()
{
  std::lock_guard<std::mutex> lock(mutex_);

  text_size_ = text_size_property_->getInt();
  QFont font;
  font.setPointSize(text_size_);
  caption_offset_ = QFontMetrics(font).height();
  update_required_ = true;
}

void Float32MultiArrayStampedPieChartDisplay::updateShowCaption()
{
  show_caption_ = show_caption_property_->getBool();
  update_required_ = true;
}

void Float32MultiArrayStampedPieChartDisplay::updateTopic()
{
  unsubscribe();
  subscribe();
}

void Float32MultiArrayStampedPieChartDisplay::updateDataIndex()
{
  data_index_ = data_index_property_->getInt();
  update_required_ = true;
}

void Float32MultiArrayStampedPieChartDisplay::updateAutoColorChange()
{
  auto_color_change_ = auto_color_change_property_->getBool();
  if (auto_color_change_) {
    max_color_property_->show();
    med_color_property_->show();
    max_color_threshold_property_->show();
    med_color_threshold_property_->show();
  } else {
    max_color_property_->hide();
    med_color_property_->hide();
    max_color_threshold_property_->hide();
    med_color_threshold_property_->hide();
  }
  update_required_ = true;
}

void Float32MultiArrayStampedPieChartDisplay::updateMaxColor()
{
  max_color_ = max_color_property_->getColor();
  update_required_ = true;
}

void Float32MultiArrayStampedPieChartDisplay::updateMedColor()
{
  med_color_ = med_color_property_->getColor();
  update_required_ = true;
}

void Float32MultiArrayStampedPieChartDisplay::updateMaxColorThreshold()
{
  max_color_threshold_ = max_color_threshold_property_->getFloat();
  update_required_ = true;
}

void Float32MultiArrayStampedPieChartDisplay::updateMedColorThreshold()
{
  med_color_threshold_ = med_color_threshold_property_->getFloat();
  update_required_ = true;
}

void Float32MultiArrayStampedPieChartDisplay::updateClockwiseRotate()
{
  clockwise_rotate_ = clockwise_rotate_property_->getBool();
  update_required_ = true;
}

bool Float32MultiArrayStampedPieChartDisplay::isInRegion(int x, int y)
{
  return (top_ < y && top_ + texture_size_ > y && left_ < x && left_ + texture_size_ > x);
}

void Float32MultiArrayStampedPieChartDisplay::movePosition(int x, int y)
{
  top_ = y;
  left_ = x;
}

void Float32MultiArrayStampedPieChartDisplay::setPosition(int x, int y)
{
  top_property_->setValue(y);
  left_property_->setValue(x);
}
}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::Float32MultiArrayStampedPieChartDisplay, rviz_common::Display)

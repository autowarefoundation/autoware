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
#include "include/custom_slider.hpp"

CustomSlider::CustomSlider(Qt::Orientation orientation, QWidget * parent)
: QSlider(orientation, parent)
{
  setMinimumHeight(40);  // Ensure there's enough space for the custom track
}

// cppcheck-suppress unusedFunction
void CustomSlider::paintEvent(QPaintEvent *)
{
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  painter.setPen(Qt::NoPen);

  // Initialize style option
  QStyleOptionSlider opt;
  initStyleOption(&opt);

  QRect grooveRect =
    style()->subControlRect(QStyle::CC_Slider, &opt, QStyle::SC_SliderGroove, this);
  int centerY = grooveRect.center().y();
  QRect handleRect =
    style()->subControlRect(QStyle::CC_Slider, &opt, QStyle::SC_SliderHandle, this);

  int value = this->value();
  int minValue = this->minimum();
  int maxValue = this->maximum();

  int trackThickness = 14;
  int gap = 8;

  QRect beforeRect(
    grooveRect.x(), centerY - trackThickness / 2, handleRect.center().x() - grooveRect.x() - gap,
    trackThickness);

  QRect afterRect(
    handleRect.center().x() + gap, centerY - trackThickness / 2,
    grooveRect.right() - handleRect.center().x() - gap, trackThickness);

  QColor inactiveTrackColor(
    autoware::state_rviz_plugin::colors::default_colors.primary_container.c_str());
  QColor activeTrackColor(autoware::state_rviz_plugin::colors::default_colors.primary.c_str());
  QColor handleColor(autoware::state_rviz_plugin::colors::default_colors.primary.c_str());

  // only draw the active track if the value is more than the gap from the minimum
  if (value > minValue + gap / 2) {
    QPainterPath beforePath;
    beforePath.moveTo(beforeRect.left(), centerY + trackThickness / 2);  // Start from bottom-left
    beforePath.quadTo(
      beforeRect.left(), centerY - trackThickness / 2, beforeRect.left() + trackThickness * 0.5,
      centerY - trackThickness / 2);
    beforePath.lineTo(beforeRect.right() - trackThickness * 0.1, centerY - trackThickness / 2);
    beforePath.quadTo(
      beforeRect.right(), centerY - trackThickness / 2, beforeRect.right(), centerY);
    beforePath.quadTo(
      beforeRect.right(), centerY + trackThickness / 2, beforeRect.right() - trackThickness * 0.1,
      centerY + trackThickness / 2);
    beforePath.lineTo(beforeRect.left() + trackThickness * 0.5, centerY + trackThickness / 2);
    beforePath.quadTo(beforeRect.left(), centerY + trackThickness / 2, beforeRect.left(), centerY);
    painter.fillPath(beforePath, activeTrackColor);
  }

  if (value < maxValue - gap / 2) {
    QPainterPath afterPath;
    afterPath.moveTo(afterRect.left(), centerY + trackThickness / 2);
    afterPath.quadTo(
      afterRect.left(), centerY - trackThickness / 2, afterRect.left() + trackThickness * 0.1,
      centerY - trackThickness / 2);
    afterPath.lineTo(afterRect.right() - trackThickness * 0.5, centerY - trackThickness / 2);
    afterPath.quadTo(afterRect.right(), centerY - trackThickness / 2, afterRect.right(), centerY);
    afterPath.quadTo(
      afterRect.right(), centerY + trackThickness / 2, afterRect.right() - trackThickness * 0.5,
      centerY + trackThickness / 2);
    afterPath.lineTo(afterRect.left() + trackThickness * 0.1, centerY + trackThickness / 2);
    afterPath.quadTo(afterRect.left(), centerY + trackThickness / 2, afterRect.left(), centerY);
    painter.fillPath(afterPath, inactiveTrackColor);
  }

  painter.setBrush(handleColor);
  int handleLineHeight = 30;
  int handleLineWidth = 4;
  int handleLineRadius = 2;
  QRect handleLineRect(
    handleRect.center().x() - handleLineWidth / 2, centerY - handleLineHeight / 2, handleLineWidth,
    handleLineHeight);
  QPainterPath handlePath;
  handlePath.addRoundedRect(handleLineRect, handleLineRadius, handleLineRadius);
  painter.fillPath(handlePath, handleColor);
}

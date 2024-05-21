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
#include "include/custom_toggle_switch.hpp"

CustomToggleSwitch::CustomToggleSwitch(QWidget * parent) : QCheckBox(parent)
{
  setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
  setCursor(Qt::PointingHandCursor);

  connect(this, &QCheckBox::stateChanged, this, [this]() {
    if (!blockSignalsGuard) {
      update();  // Force repaint
    }
  });
}

QSize CustomToggleSwitch::sizeHint() const
{
  return QSize(50, 30);  // Preferred size of the toggle switch
}

void CustomToggleSwitch::paintEvent(QPaintEvent *)
{
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);

  int margin = 2;
  int circleRadius = height() / 2 - margin * 2;
  QRect r = rect().adjusted(margin, margin, -margin, -margin);
  bool isChecked = this->isChecked();

  QColor uncheckedIndicatorColor =
    QColor(autoware::state_rviz_plugin::colors::default_colors.outline.c_str());
  QColor checkedIndicatorColor =
    QColor(autoware::state_rviz_plugin::colors::default_colors.on_primary.c_str());
  QColor indicatorColor = isChecked ? checkedIndicatorColor : uncheckedIndicatorColor;

  QColor uncheckedBgColor =
    QColor(autoware::state_rviz_plugin::colors::default_colors.surface_container_highest.c_str());
  QColor checkedBgColor =
    QColor(autoware::state_rviz_plugin::colors::default_colors.primary.c_str());

  QColor bgColor = isChecked ? checkedBgColor : uncheckedBgColor;

  QRect borderR = r.adjusted(-margin, -margin, margin, margin);
  p.setBrush(bgColor);
  p.setPen(Qt::NoPen);
  p.drawRoundedRect(borderR, circleRadius + 4, circleRadius + 4);

  p.setBrush(bgColor);
  p.setPen(Qt::NoPen);
  p.drawRoundedRect(r, circleRadius + 4, circleRadius + 4);

  int minX = r.left() + margin * 2;
  int maxX = r.right() - circleRadius * 2 - margin;
  int circleX = isChecked ? maxX : minX;
  QRect circleRect(circleX, r.top() + margin, circleRadius * 2, circleRadius * 2);
  p.setBrush(indicatorColor);
  p.drawEllipse(circleRect);
}

void CustomToggleSwitch::mouseReleaseEvent(QMouseEvent * event)
{
  if (event->button() == Qt::LeftButton) {
    setCheckedState(!isChecked());
  }
  QCheckBox::mouseReleaseEvent(event);
}

void CustomToggleSwitch::setCheckedState(bool state)
{
  blockSignalsGuard = true;
  setChecked(state);
  blockSignalsGuard = false;
  update();  // Force repaint
}

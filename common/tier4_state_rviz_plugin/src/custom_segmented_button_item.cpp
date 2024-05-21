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
#include "include/custom_segmented_button_item.hpp"

CustomSegmentedButtonItem::CustomSegmentedButtonItem(const QString & text, QWidget * parent)
: QPushButton(text, parent),
  bgColor(
    QColor(autoware::state_rviz_plugin::colors::default_colors.surface_container_low.c_str())),
  checkedBgColor(
    QColor(autoware::state_rviz_plugin::colors::default_colors.secondary_container.c_str())),
  inactiveTextColor(QColor(autoware::state_rviz_plugin::colors::default_colors.on_surface.c_str())),
  activeTextColor(
    QColor(autoware::state_rviz_plugin::colors::default_colors.on_secondary_container.c_str())),
  isHovered(false),
  isActivated(false),
  isDisabled(false)

{
  setCheckable(true);
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  setCursor(Qt::PointingHandCursor);
}

void CustomSegmentedButtonItem::setColors(
  const QColor & bg, const QColor & checkedBg, const QColor & activeText,
  const QColor & inactiveText)
{
  bgColor = bg;
  checkedBgColor = checkedBg;
  activeTextColor = activeText;
  inactiveTextColor = inactiveText;
  update();
}

// void CustomSegmentedButtonItem::updateSize()
// {
//   QFontMetrics fm(font());
//   int width = fm.horizontalAdvance(text()) + 40;  // Add padding
//   int height = fm.height() + 20;                  // Add padding
//   setFixedSize(width, height);
// }

void CustomSegmentedButtonItem::setHovered(bool hovered)
{
  isHovered = hovered;
  updateCheckableState();
}

void CustomSegmentedButtonItem::setCheckableButton(bool checkable)
{
  setCheckable(checkable);
  updateCheckableState();
}

void CustomSegmentedButtonItem::updateCheckableState()
{
  setCursor(
    isDisabled ? Qt::ForbiddenCursor
               : (isCheckable() ? Qt::PointingHandCursor : Qt::ForbiddenCursor));
  update();
}

void CustomSegmentedButtonItem::setDisabledButton(bool disabled)
{
  isDisabled = disabled;
  updateCheckableState();
}

void CustomSegmentedButtonItem::setActivated(bool activated)
{
  isActivated = activated;
  update();
}

void CustomSegmentedButtonItem::paintEvent(QPaintEvent *)
{
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);

  // Adjust opacity for disabled state
  if (isDisabled) {
    painter.setOpacity(0.3);
  } else {
    painter.setOpacity(1.0);
  }

  // Determine the button's color based on its state
  QColor buttonColor;
  if (isDisabled) {
    buttonColor = disabledBgColor;
  } else if (isHovered && !isChecked() && isCheckable()) {
    buttonColor = hoverColor;
  } else if (isActivated) {
    buttonColor = checkedBgColor;
  } else {
    buttonColor = isChecked() ? checkedBgColor : bgColor;
  }

  // Determine if this is the first or last button
  bool isFirstButton = false;
  bool isLastButton = false;

  QHBoxLayout * parentLayout = qobject_cast<QHBoxLayout *>(parentWidget()->layout());
  if (parentLayout) {
    int index = parentLayout->indexOf(this);
    isFirstButton = (index == 0);
    isLastButton = (index == parentLayout->count() - 1);
  }

  // Draw button background

  QRect buttonRect = rect().adjusted(0, 1, 0, -1);

  if (isFirstButton) {
    buttonRect.setLeft(buttonRect.left() + 1);
  }
  if (isLastButton) {
    buttonRect.setRight(buttonRect.right() - 1);
  }

  QPainterPath path;
  double radius = (height() - 2) / 2;

  path.setFillRule(Qt::WindingFill);
  if (isFirstButton) {
    path.addRoundedRect(buttonRect, radius, radius);
    path.addRect(QRect(
      (buttonRect.left() + buttonRect.width() - radius),
      buttonRect.top() + buttonRect.height() - radius, radius,
      radius));  // Bottom Right
    path.addRect(QRect(
      (buttonRect.left() + buttonRect.width() - radius), buttonRect.top(), radius,
      radius));  // Top Right
  } else if (isLastButton) {
    path.addRoundedRect(buttonRect, radius, radius);
    path.addRect(QRect(
      (buttonRect.left()), buttonRect.top() + buttonRect.height() - radius, radius,
      radius));  // Bottom left
    path.addRect(QRect((buttonRect.left()), buttonRect.top(), radius,
                       radius));  // Top Left
  } else {
    path.addRect(buttonRect);
  }
  painter.fillPath(path, buttonColor);

  // Draw button border
  QPen pen(QColor(autoware::state_rviz_plugin::colors::default_colors.outline.c_str()), 1);
  pen.setJoinStyle(Qt::RoundJoin);
  pen.setCapStyle(Qt::RoundCap);
  painter.setPen(pen);
  painter.drawPath(path.simplified());

  // Draw button text
  QColor textColor = (isChecked() ? activeTextColor : inactiveTextColor);
  painter.setPen(textColor);
  painter.drawText(rect(), Qt::AlignCenter, text());
}

void CustomSegmentedButtonItem::enterEvent(QEvent * event)
{
  if (isCheckable()) {
    isHovered = true;
    update();
  }
  QPushButton::enterEvent(event);
}

void CustomSegmentedButtonItem::leaveEvent(QEvent * event)
{
  if (isCheckable()) {
    isHovered = false;
    update();
  }
  QPushButton::leaveEvent(event);
}

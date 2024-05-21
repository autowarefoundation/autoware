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
#ifndef CUSTOM_BUTTON_HPP_
#define CUSTOM_BUTTON_HPP_

#include "material_colors.hpp"

#include <QColor>
#include <QFontMetrics>
#include <QGraphicsDropShadowEffect>
#include <QPainter>
#include <QPainterPath>
#include <QPushButton>
#include <QStyleOption>

class CustomElevatedButton : public QPushButton
{
  Q_OBJECT

public:
  explicit CustomElevatedButton(
    const QString & text,
    const QColor & bgColor =
      QColor(autoware::state_rviz_plugin::colors::default_colors.surface_container_low.c_str()),
    const QColor & textColor =
      QColor(autoware::state_rviz_plugin::colors::default_colors.primary.c_str()),
    const QColor & hoverColor =
      QColor(autoware::state_rviz_plugin::colors::default_colors.surface_container_highest.c_str()),
    const QColor & disabledBgColor =
      QColor(autoware::state_rviz_plugin::colors::default_colors.surface_container_high.c_str()),
    const QColor & disabledTextColor =
      QColor(autoware::state_rviz_plugin::colors::default_colors.on_surface_variant.c_str()),
    QWidget * parent = nullptr);
  void updateStyle(
    const QString & text, const QColor & bgColor, const QColor & textColor,
    const QColor & hoverColor, const QColor & disabledBgColor, const QColor & disabledTextColor);

protected:
  void paintEvent(QPaintEvent * event) override;
  void enterEvent(QEvent * event) override;
  void leaveEvent(QEvent * event) override;
  QSize sizeHint() const override;
  QSize minimumSizeHint() const override;

private:
  QColor backgroundColor =
    QColor(autoware::state_rviz_plugin::colors::default_colors.surface_container_low.c_str());
  QColor textColor = QColor(autoware::state_rviz_plugin::colors::default_colors.primary.c_str());
  QColor hoverColor =
    QColor(autoware::state_rviz_plugin::colors::default_colors.surface_container_highest.c_str());
  QColor disabledBgColor =
    QColor(autoware::state_rviz_plugin::colors::default_colors.disabled_elevated_button_bg.c_str());
  QColor disabledTextColor =
    QColor(autoware::state_rviz_plugin::colors::default_colors.on_surface.c_str());
  bool isHovered = false;
};

#endif  // CUSTOM_BUTTON_HPP_

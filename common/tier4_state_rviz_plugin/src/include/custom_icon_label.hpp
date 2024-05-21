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
#ifndef CUSTOM_ICON_LABEL_HPP_
#define CUSTOM_ICON_LABEL_HPP_

#include "material_colors.hpp"

#include <QColor>
#include <QLabel>
#include <QMap>
#include <QPainter>
#include <QPainterPath>
#include <QPixmap>
#include <QStyleOption>
#include <QWidget>

enum IconState { Active, Pending, Danger, None, Crash };

class CustomIconLabel : public QLabel
{
  Q_OBJECT

public:
  explicit CustomIconLabel(
    const QColor & bgColor =
      QColor(autoware::state_rviz_plugin::colors::default_colors.surface_container_high.c_str()),
    QWidget * parent = nullptr);
  void updateStyle(IconState state, const QColor & bgColor);

protected:
  void paintEvent(QPaintEvent * event) override;
  QSize sizeHint() const override;
  QSize minimumSizeHint() const override;

private:
  void loadIcons();
  QPixmap icon;
  QColor backgroundColor =
    QColor(autoware::state_rviz_plugin::colors::default_colors.surface_container_high.c_str());
  QMap<IconState, QPixmap> iconMap;
};

#endif  // CUSTOM_ICON_LABEL_HPP_

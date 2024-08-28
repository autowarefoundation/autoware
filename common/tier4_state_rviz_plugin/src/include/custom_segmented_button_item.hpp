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
#ifndef CUSTOM_SEGMENTED_BUTTON_ITEM_HPP_
#define CUSTOM_SEGMENTED_BUTTON_ITEM_HPP_

#include "material_colors.hpp"

#include <QColor>
#include <QHBoxLayout>
#include <QPainter>
#include <QPainterPath>
#include <QPushButton>
#include <QStyleOption>

class CustomSegmentedButtonItem : public QPushButton
{
  Q_OBJECT

public:
  explicit CustomSegmentedButtonItem(const QString & text, QWidget * parent = nullptr);

  void setActivated(bool activated);
  void setCheckableButton(bool checkable);
  void setDisabledButton(bool disabled);
  void setHovered(bool hovered);

protected:
  void paintEvent(QPaintEvent * event) override;
  void enterEvent(QEvent * event) override;
  void leaveEvent(QEvent * event) override;

private:
  void updateCheckableState();

  QColor bgColor;
  QColor checkedBgColor;
  QColor hoverColor =
    QColor(autoware::state_rviz_plugin::colors::default_colors.surface_container_highest.c_str());
  QColor inactiveTextColor;
  QColor activeTextColor;
  QColor disabledBgColor =
    QColor(autoware::state_rviz_plugin::colors::default_colors.surface_dim.c_str());
  QColor disabledTextColor =
    QColor(autoware::state_rviz_plugin::colors::default_colors.on_surface_variant.c_str());
  bool isHovered = false;
  bool isActivated = false;
  bool isDisabled = false;
};

#endif  // CUSTOM_SEGMENTED_BUTTON_ITEM_HPP_

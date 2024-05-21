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
#ifndef CUSTOM_SEGMENTED_BUTTON_HPP_
#define CUSTOM_SEGMENTED_BUTTON_HPP_

#include "custom_segmented_button_item.hpp"
#include "material_colors.hpp"

#include <QButtonGroup>
#include <QColor>
#include <QHBoxLayout>
#include <QPainter>
#include <QPainterPath>
#include <QPushButton>
#include <QStyleOption>
#include <QWidget>

class CustomSegmentedButton : public QWidget
{
  Q_OBJECT

public:
  explicit CustomSegmentedButton(QWidget * parent = nullptr);

  CustomSegmentedButtonItem * addButton(const QString & text);
  QButtonGroup * getButtonGroup() const;

  QSize sizeHint() const override;
  QSize minimumSizeHint() const override;

Q_SIGNALS:
  void buttonClicked(int id);

protected:
  void paintEvent(QPaintEvent * event) override;

private:
  QButtonGroup * buttonGroup;
  QHBoxLayout * layout;
};

#endif  // CUSTOM_SEGMENTED_BUTTON_HPP_

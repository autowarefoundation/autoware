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
#ifndef CUSTOM_SLIDER_HPP_
#define CUSTOM_SLIDER_HPP_
#include "material_colors.hpp"

#include <QPaintEvent>
#include <QPainter>
#include <QPainterPath>
#include <QSlider>
#include <QStyleOptionSlider>

class CustomSlider : public QSlider
{
  Q_OBJECT

public:
  explicit CustomSlider(Qt::Orientation orientation, QWidget * parent = nullptr);

protected:
  void paintEvent(QPaintEvent * event) override;
};

#endif  // CUSTOM_SLIDER_HPP_

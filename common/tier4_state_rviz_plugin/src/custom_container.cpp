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
#include "include/custom_container.hpp"

#include "src/include/material_colors.hpp"

CustomContainer::CustomContainer(QWidget * parent) : QFrame(parent), cornerRadius(15)
{
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  setContentsMargins(0, 0, 0, 0);
  layout = new QGridLayout(this);
  layout->setMargin(0);                      // Margin between the border and child widgets
  layout->setSpacing(0);                     // Spacing between child widgets
  layout->setContentsMargins(10, 0, 10, 0);  // Margin between the border and the layout
  setLayout(layout);
}

// cppcheck-suppress unusedFunction
QGridLayout * CustomContainer::getLayout() const
{
  return layout;  // Provide access to the layout
}

QSize CustomContainer::sizeHint() const
{
  QSize size = layout->sizeHint();
  int width = size.width() + 20;    // Adding padding
  int height = size.height() + 20;  // Adding padding
  return QSize(width, height);
}

// cppcheck-suppress unusedFunction
QSize CustomContainer::minimumSizeHint() const
{
  return sizeHint();
}

void CustomContainer::paintEvent(QPaintEvent *)
{
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);

  // Draw background
  QPainterPath path;
  path.addRoundedRect(rect(), height() / 2, height() / 2);  // Use height for rounded corners
  painter.setPen(Qt::NoPen);
  painter.setBrush(QColor(
    autoware::state_rviz_plugin::colors::default_colors.surface.c_str()));  // Background color
  painter.drawPath(path);
}

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
#include "include/custom_label.hpp"

#include "src/include/material_colors.hpp"

#include <QColor>
#include <QGraphicsDropShadowEffect>
#include <QLinearGradient>
#include <QPainter>
#include <QPainterPath>
#include <QStyleOption>

CustomLabel::CustomLabel(const QString & text, QWidget * parent) : QLabel(text, parent)
{
  setFont(QFont("Roboto", 10, QFont::Bold));  // Set the font as needed
  setAlignment(Qt::AlignCenter);

  // Add shadow effect
  QGraphicsDropShadowEffect * shadowEffect = new QGraphicsDropShadowEffect(this);
  shadowEffect->setBlurRadius(15);  // Blur radius for a smoother shadow
  shadowEffect->setOffset(3, 3);    // Offset for the shadow
  shadowEffect->setColor(
    QColor(autoware::state_rviz_plugin::colors::default_colors.shadow.c_str()));  // Shadow color
  setGraphicsEffect(shadowEffect);
}

QSize CustomLabel::sizeHint() const
{
  QFontMetrics fm(font());
  int textWidth = fm.horizontalAdvance(text());
  int textHeight = fm.height();
  int width = textWidth + 40;  // Adding padding
  int height = textHeight;     // Adding padding
  return QSize(width, height);
}

QSize CustomLabel::minimumSizeHint() const
{
  return sizeHint();
}

void CustomLabel::paintEvent(QPaintEvent *)
{
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);

  int cornerRadius = height() / 2;  // Making the corner radius proportional to the height

  // Draw background
  QPainterPath path;
  path.addRoundedRect(rect().adjusted(1, 1, -1, -1), cornerRadius, cornerRadius);

  painter.setPen(Qt::NoPen);
  painter.setBrush(backgroundColor);

  painter.drawPath(path);

  // Set text color and draw text
  painter.setPen(textColor);
  painter.drawText(rect(), Qt::AlignCenter, text());
}

void CustomLabel::updateStyle(
  const QString & text, const QColor & bg_color, const QColor & text_color)
{
  setText(text);
  backgroundColor = bg_color;
  textColor = text_color;
  update();  // Force repaint
}

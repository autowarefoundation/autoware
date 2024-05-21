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
#include "include/custom_icon_label.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

CustomIconLabel::CustomIconLabel(const QColor & bgColor, QWidget * parent)
: QLabel(parent), backgroundColor(bgColor)
{
  loadIcons();
  setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
  setAlignment(Qt::AlignCenter);
  setFixedSize(35, 35);
}

void CustomIconLabel::loadIcons()
{
  std::string packagePath = ament_index_cpp::get_package_share_directory("tier4_state_rviz_plugin");

  iconMap[Active] = QPixmap(QString::fromStdString(packagePath + "/icons/assets/active.png"));
  iconMap[Pending] = QPixmap(QString::fromStdString(packagePath + "/icons/assets/pending.png"));
  iconMap[Danger] = QPixmap(QString::fromStdString(packagePath + "/icons/assets/danger.png"));
  iconMap[None] = QPixmap(QString::fromStdString(packagePath + "/icons/assets/none.png"));
  iconMap[Crash] = QPixmap(QString::fromStdString(packagePath + "/icons/assets/crash.png"));

  icon = iconMap[None];  // Default icon
}

void CustomIconLabel::updateStyle(IconState state, const QColor & bgColor)
{
  icon = iconMap[state];
  backgroundColor = bgColor;
  update();  // Force repaint
}

QSize CustomIconLabel::sizeHint() const
{
  int size = qMax(icon.width(), icon.height()) + 20;  // Adding padding
  return QSize(size, size);
}

QSize CustomIconLabel::minimumSizeHint() const
{
  return sizeHint();
}

void CustomIconLabel::paintEvent(QPaintEvent *)
{
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);

  int diameter = qMin(width(), height());
  int radius = diameter / 2;

  // Draw background circle
  QPainterPath path;
  path.addEllipse(width() / 2 - radius, height() / 2 - radius, diameter, diameter);
  painter.setPen(Qt::NoPen);
  painter.setBrush(backgroundColor);
  painter.drawPath(path);

  // Draw icon in the center
  if (!icon.isNull()) {
    QSize iconSize = icon.size().scaled(diameter * 0.6, diameter * 0.6, Qt::KeepAspectRatio);
    QPoint iconPos((width() - iconSize.width()) / 2, (height() - iconSize.height()) / 2);
    painter.drawPixmap(
      iconPos, icon.scaled(iconSize, Qt::KeepAspectRatio, Qt::SmoothTransformation));
  }
}

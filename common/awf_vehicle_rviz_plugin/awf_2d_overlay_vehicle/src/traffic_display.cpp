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

#include "traffic_display.hpp"

#include <QFontDatabase>
#include <QPainter>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rviz_rendering/render_system.hpp>

#include <OgreHardwarePixelBuffer.h>
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>
#include <OgreTexture.h>
#include <OgreTextureManager.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <memory>
#include <string>

namespace awf_2d_overlay_vehicle
{

TrafficDisplay::TrafficDisplay()
{
  // Load the traffic light image
  std::string package_path = ament_index_cpp::get_package_share_directory("awf_2d_overlay_vehicle");
  std::string image_path = package_path + "/assets/images/traffic.png";
  traffic_light_image_.load(image_path.c_str());
}

void TrafficDisplay::updateTrafficLightData(
  const autoware_perception_msgs::msg::TrafficSignalArray::ConstSharedPtr & msg)
{
  current_traffic_ = *msg;
}

void TrafficDisplay::drawTrafficLightIndicator(QPainter & painter, const QRectF & backgroundRect)
{
  // Enable Antialiasing for smoother drawing
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.setRenderHint(QPainter::SmoothPixmapTransform, true);

  // Define the area for the circle (background)
  QRectF circleRect = backgroundRect;
  circleRect.setWidth(backgroundRect.width() / 2 - 20);
  circleRect.setHeight(backgroundRect.height() - 20);
  circleRect.moveTopRight(QPointF(backgroundRect.right() - 10, backgroundRect.top() + 10));

  painter.setBrush(QBrush(gray));
  painter.drawEllipse(circleRect.center(), 30, 30);

  // Define the area for the traffic light image (should be smaller or positioned within the circle)
  QRectF imageRect =
    circleRect.adjusted(15, 15, -15, -15);  // Adjusting the rectangle to make the image smaller

  QImage scaled_traffic_image = traffic_light_image_.scaled(
    imageRect.size().toSize(), Qt::KeepAspectRatio, Qt::SmoothTransformation);

  if (current_traffic_.signals.size() > 0) {
    switch (current_traffic_.signals[0].elements[0].color) {
      case 1:
        painter.setBrush(QBrush(red));
        painter.drawEllipse(circleRect.center(), 30, 30);
        break;
      case 2:
        painter.setBrush(QBrush(yellow));
        painter.drawEllipse(circleRect.center(), 30, 30);
        break;
      case 3:
        painter.setBrush(QBrush(green));
        painter.drawEllipse(circleRect.center(), 30, 30);
        break;
      case 4:
        painter.setBrush(QBrush(gray));
        painter.drawEllipse(circleRect.center(), 30, 30);
        break;
      default:
        painter.setBrush(QBrush(gray));
        painter.drawEllipse(circleRect.center(), 30, 30);
        break;
    }
  }
  // make the image thicker
  painter.setPen(QPen(Qt::black, 2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));

  painter.drawImage(imageRect, scaled_traffic_image);
}

QImage TrafficDisplay::coloredImage(const QImage & source, const QColor & color)
{
  QImage result = source;
  QPainter p(&result);
  p.setCompositionMode(QPainter::CompositionMode_SourceAtop);
  p.fillRect(result.rect(), color);
  p.end();
  return result;
}

}  // namespace awf_2d_overlay_vehicle

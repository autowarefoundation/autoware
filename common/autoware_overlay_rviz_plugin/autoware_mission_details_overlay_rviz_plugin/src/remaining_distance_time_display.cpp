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

#include "remaining_distance_time_display.hpp"

#include <QColor>
#include <QFontDatabase>
#include <QPainter>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <qpoint.h>

#include <cmath>
#include <iomanip>
#include <memory>
#include <string>

namespace autoware::mission_details_overlay_rviz_plugin
{

RemainingDistanceTimeDisplay::RemainingDistanceTimeDisplay()
: remaining_distance_(0.0), remaining_time_(0.0)
{
  std::string package_path =
    ament_index_cpp::get_package_share_directory("autoware_mission_details_overlay_rviz_plugin");
  std::string font_path = package_path + "/assets/font/Quicksand/static/Quicksand-Regular.ttf";
  std::string font_path2 = package_path + "/assets/font/Quicksand/static/Quicksand-Bold.ttf";
  int fontId = QFontDatabase::addApplicationFont(
    font_path.c_str());  // returns -1 on failure (see docs for more info)
  int fontId2 = QFontDatabase::addApplicationFont(
    font_path2.c_str());  // returns -1 on failure (see docs for more info)
  if (fontId == -1 || fontId2 == -1) {
    std::cout << "Failed to load the Quicksand font.";
  }

  std::string dist_image = package_path + "/assets/path.png";
  std::string time_image = package_path + "/assets/av_timer.png";
  icon_dist_.load(dist_image.c_str());
  icon_time_.load(time_image.c_str());
  icon_dist_scaled_ = icon_dist_.scaled(28, 28, Qt::KeepAspectRatio, Qt::SmoothTransformation);
  icon_time_scaled_ = icon_time_.scaled(28, 28, Qt::KeepAspectRatio, Qt::SmoothTransformation);
}

void RemainingDistanceTimeDisplay::updateRemainingDistanceTimeData(
  const autoware_internal_msgs::msg::MissionRemainingDistanceTime::ConstSharedPtr & msg)
{
  remaining_distance_ = msg->remaining_distance;
  remaining_time_ = msg->remaining_time;
}

void RemainingDistanceTimeDisplay::drawRemainingDistanceTimeDisplay(
  QPainter & painter, const QRectF & backgroundRect, const QColor & text_color)
{
  const QFont font("Quicksand", 15, QFont::Bold);
  painter.setFont(font);

  // We'll display the distance and time in two rows

  auto calculate_inner_rect = [](const QRectF & outer_rect, double ratio_x, double ratio_y) {
    QSizeF size_inner(outer_rect.width() * ratio_x, outer_rect.height() * ratio_y);
    QPointF top_left_inner = QPointF(
      outer_rect.center().x() - size_inner.width() / 2,
      outer_rect.center().y() - size_inner.height() / 2);
    return QRectF(top_left_inner, size_inner);
  };

  QRectF rect_inner = calculate_inner_rect(backgroundRect, 0.7, 0.7);
  // Proportionally extend the rect to the right to account for visual icon distance to the left
  rect_inner.setWidth(rect_inner.width() * 1.08);

  // Calculate distance row rectangle
  const QSizeF distance_row_size(rect_inner.width(), rect_inner.height() / 2);
  const QPointF distance_row_top_left(rect_inner.topLeft());
  const QRectF distance_row_rect_outer(distance_row_top_left, distance_row_size);

  // Calculate time row rectangle
  const QSizeF time_row_size(rect_inner.width(), rect_inner.height() / 2);
  const QPointF time_row_top_left(
    rect_inner.topLeft().x(), rect_inner.topLeft().y() + rect_inner.height() / 2);
  const QRectF time_row_rect_outer(time_row_top_left, time_row_size);

  const auto rect_time = calculate_inner_rect(time_row_rect_outer, 1.0, 0.9);
  const auto rect_dist = calculate_inner_rect(distance_row_rect_outer, 1.0, 0.9);

  auto place_row = [&, this](
                     const QRectF & rect, const QImage & icon, const QString & str_value,
                     const QString & str_unit) {
    // order: icon, value, unit

    // place the icon
    QPointF icon_top_left(rect.topLeft().x(), rect.center().y() - icon.height() / 2.0);
    painter.drawImage(icon_top_left, icon);

    // place the unit text
    const float unit_width = 40.0;
    QRectF rect_text_unit(
      rect.topRight().x() - unit_width, rect.topRight().y(), unit_width, rect.height());

    painter.setPen(text_color);
    painter.drawText(rect_text_unit, Qt::AlignLeft | Qt::AlignVCenter, str_unit);

    // place the value text
    const double margin_x = 5.0;  // margin around the text

    const double used_width = icon.width() + rect_text_unit.width() + margin_x * 2.0;

    QRectF rect_text(
      rect.topLeft().x() + icon.width() + margin_x, rect.topLeft().y(), rect.width() - used_width,
      rect.height());

    painter.drawText(rect_text, Qt::AlignRight | Qt::AlignVCenter, str_value);
  };

  // remaining_time_ is in seconds
  if (remaining_time_ <= 60) {
    place_row(rect_time, icon_time_scaled_, QString::number(remaining_time_, 'f', 0), "sec");
  } else if (remaining_time_ <= 600) {
    place_row(rect_time, icon_time_scaled_, QString::number(remaining_time_ / 60.0, 'f', 1), "min");
  } else if (remaining_time_ <= 3600) {
    place_row(rect_time, icon_time_scaled_, QString::number(remaining_time_ / 60.0, 'f', 0), "min");
  } else if (remaining_time_ <= 36000) {
    place_row(
      rect_time, icon_time_scaled_, QString::number(remaining_time_ / 3600.0, 'f', 1), "hrs");
  } else {
    place_row(
      rect_time, icon_time_scaled_, QString::number(remaining_time_ / 3600.0, 'f', 0), "hrs");
  }

  // remaining_distance_ is in meters
  if (remaining_distance_ <= 10) {
    place_row(rect_dist, icon_dist_scaled_, QString::number(remaining_distance_, 'f', 1), "m");
  } else if (remaining_distance_ <= 1000) {
    place_row(rect_dist, icon_dist_scaled_, QString::number(remaining_distance_, 'f', 0), "m");
  } else if (remaining_distance_ <= 10000) {
    place_row(
      rect_dist, icon_dist_scaled_, QString::number(remaining_distance_ / 1000.0, 'f', 2), "km");
  } else if (remaining_distance_ <= 100000) {
    place_row(
      rect_dist, icon_dist_scaled_, QString::number(remaining_distance_ / 1000.0, 'f', 1), "km");
  } else {
    place_row(
      rect_dist, icon_dist_scaled_, QString::number(remaining_distance_ / 1000.0, 'f', 0), "km");
  }
}

}  // namespace autoware::mission_details_overlay_rviz_plugin

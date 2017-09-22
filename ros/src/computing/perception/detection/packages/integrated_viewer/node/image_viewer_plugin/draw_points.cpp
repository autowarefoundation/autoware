#include "draw_points.h"
#include <opencv2/opencv.hpp>

namespace integrated_viewer
{
  DrawPoints::DrawPoints(void) {
    // set color map
    cv::Mat gray_scale(256, 1, CV_8UC1);

    for (int i = 0; i < 256; i++) {
      gray_scale.at<uchar>(i) = i;
    }

    cv::applyColorMap(gray_scale, color_map_, cv::COLORMAP_JET);
  } //   DrawPoints::DrawPoints()


  void DrawPoints::Draw(const autoware_msgs::PointsImage::ConstPtr& points,
                        cv::Mat &image, int drawn_size) {
    if (points == NULL) {
      return;
    }

    int width = image.size().width;
    int height = image.size().height;

    // Calculate minimum and maximum value of distance in this points image
    float min_distance, max_distance;
    min_distance = max_distance = points->distance[0];

    for (int i = 1; i < width * height; i++) {
      float distance = points->distance[i];
      max_distance = (distance > max_distance) ? distance : max_distance;
      min_distance = (distance < min_distance) ? distance : min_distance;
    }
    float distance_range = max_distance - min_distance;

    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        int index = y * width + x;

        float distance = points->distance[index];
        if (distance == 0) {
          continue;
        }

        // Specify which color will be use for this point
        int color_id = distance_range ? ((distance - min_distance) * 255 / distance_range) : 128;

        // Divide color into each element
        cv::Vec3b color = color_map_.at<cv::Vec3b>(color_id);
        int red   = color[0];
        int green = color[1];
        int blue  = color[2];

        // Draw a point
        int minus_offset = 0;
        int plus_offset = static_cast<int>(drawn_size/2);
        if (drawn_size % 2 == 0) {
          minus_offset = static_cast<int>(drawn_size/2) - 1;
        } else {
          minus_offset = static_cast<int>(drawn_size/2);
        }
        cv::rectangle(image,
                      cv::Point(x - minus_offset, y - minus_offset),
                      cv::Point(x + plus_offset, y + plus_offset),
                      CV_RGB(red, green, blue),
                      CV_FILLED);
      }
    }

  } // DrawPoints::Draw()

} // end namespace integrated_viewer

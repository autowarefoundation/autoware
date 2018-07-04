#include "draw_rects.h"
#include <string>
#include <vector>


#include <opencv2/core/version.hpp>

#if (CV_MAJOR_VERSION == 3)
#include "gencolors.cpp"
#else
#include <opencv2/contrib/contrib.hpp>
#include <autoware_msgs/DetectedObjectArray.h>

#endif

namespace integrated_viewer
{
  const int        DrawRects::kRectangleThickness = 3;
  
  DrawRects::DrawRects(void) {
    // Generate color map to represent tracked object
#if (CV_MAJOR_VERSION == 3)
    generateColors(color_map_, 25);
#else
    cv::generateColors(color_map_, 25);
#endif

  } // DrawRects::DrawRects()


  void DrawRects::DrawImageObj(const autoware_msgs::DetectedObjectArray::ConstPtr& detected_objects,
                               cv::Mat &image) {
    if (detected_objects == NULL) {
        std::cout << "EMPTY objects" << std::endl;
      return;
    }
    
    // Draw rectangles for each object
    for (const auto& detected_object : detected_objects->objects) {
      // Make label shown on a rectangle
      std::ostringstream label;
      label << detected_object.label << ":" << std::setprecision(2) << detected_object.score;

      // Draw object information label
      DrawLabel(label.str(), cv::Point(detected_object.x, detected_object.y), image);

      // Draw rectangle
      cv::rectangle(image,
                    cv::Point(detected_object.x, detected_object.y),
                    cv::Point(detected_object.x + detected_object.width, detected_object.y + detected_object.height),
                    cv::Scalar(detected_object.color.r, detected_object.color.g, detected_object.color.b),
                    kRectangleThickness,
                    CV_AA,
                    0);
    }
  } // DrawRects::DrawImageObj()

  void DrawRects::DrawLabel(const std::string& label,
                            const cv::Point& rectangle_origin,
                            cv::Mat &image) {
    // label's property
    const int    font_face      = cv::FONT_HERSHEY_COMPLEX;
    const double font_scale     = 0.5;
    const int    font_thickness = 1;
    int          font_baseline  = 0;

    cv::Size label_size = cv::getTextSize(label,
                                          font_face,
                                          font_scale,
                                          font_thickness,
                                          &font_baseline);

    cv::Point label_origin = cv::Point(rectangle_origin.x - kRectangleThickness,
                                       rectangle_origin.y - font_baseline - kRectangleThickness);

    // Fill label's background by black
    cv::rectangle(image,
                  cv::Point(label_origin.x, label_origin.y + font_baseline),
                  cv::Point(label_origin.x + label_size.width, label_origin.y - label_size.height),
                  CV_RGB(0, 0, 0),
                  CV_FILLED);

    // Draw label text by white
    cv::putText(image,
                label,
                label_origin,
                font_face,
                font_scale,
                CV_RGB(255, 255, 255));
        
  } // DrawRects::DrawLabel()
} // end namespace integrated_viewer

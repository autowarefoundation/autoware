#include "draw_rects.h"
#include <string>
#include <vector>
#include <opencv2/contrib/contrib.hpp>

namespace integrated_viewer
{
  const int        DrawRects::kRectangleThickness = 3;
  const cv::Scalar DrawRects::kBlue               = CV_RGB(0, 0, 255);
  const cv::Scalar DrawRects::kGreen              = CV_RGB(0, 255, 0);
  
  DrawRects::DrawRects(void) {
    // Generate color map to represent tracked object
    cv::generateColors(color_map_, 25);

  } // DrawRects::DrawRects()


  void DrawRects::DrawImageObj(const cv_tracker::image_obj::ConstPtr& rect_data,
                               cv::Mat &image) {
    if (rect_data == NULL) {
      return;
    }

    cv::Scalar rectangle_color;
    if (rect_data->type == "car") {
      rectangle_color = kBlue;
    } else {
      rectangle_color = kGreen;
    }
    
    // Draw rectangles for each objects
    for (const auto& rectangle : rect_data->obj) {
      // Make label shown on a rectangle
      std::ostringstream label;
      label << rect_data->type << ":" << std::setprecision(2) << rectangle.score;

      // Draw object information label
      DrawLabel(label.str(), cv::Point(rectangle.x, rectangle.y), image);

      // Draw rectangle
      cv::rectangle(image,
                    cv::Point(rectangle.x, rectangle.y),
                    cv::Point(rectangle.x + rectangle.width, rectangle.y + rectangle.height),
                    rectangle_color,
                    kRectangleThickness,
                    CV_AA,
                    0);
    }
  } // DrawRects::DrawImageObj()


  void DrawRects::DrawImageObjRanged(const cv_tracker::image_obj_ranged::ConstPtr& rect_data,
                                     cv::Mat &image) {
    if (rect_data == NULL) {
      return;
    }

    cv::Scalar rectangle_color;
    if (rect_data->type == "car") {
      rectangle_color = kBlue;
    } else {
      rectangle_color = kGreen;
    }
    
    // Draw rectangles for each objects
    for (const auto& ojbect : rect_data->obj) {
      // Make label shown on a rectangle
      std::ostringstream label;
      label << rect_data->type << " : " << std::fixed << std::setprecision(2) << ojbect.range / 100 << " m";

      // Draw object information label
      DrawLabel(label.str(), cv::Point(ojbect.rect.x, ojbect.rect.y), image);

      // Draw rectangle
      cv::rectangle(image,
                    cv::Point(ojbect.rect.x, ojbect.rect.y),
                    cv::Point(ojbect.rect.x + ojbect.rect.width, ojbect.rect.y + ojbect.rect.height),
                    rectangle_color,
                    kRectangleThickness,
                    CV_AA,
                    0);
    }
  } // DrawRects::DrawImageObjRanged()


  void DrawRects::DrawImageObjTracked(const cv_tracker::image_obj_tracked::ConstPtr& rect_data,
                                      cv::Mat &image) {
    if (rect_data == NULL) {
      return;
    }

    for (const auto& object : rect_data->rect_ranged) {
      int index = &object - &(rect_data->rect_ranged[0]);
      int object_id = rect_data->obj_id[index];

      // Make label shown on a rectangle
      std::ostringstream label;
      label << rect_data->type << "_" << object_id << " : " << std::setprecision(2) << rect_data->lifespan[index];

      // Draw object information label
      DrawLabel(label.str(), cv::Point(object.rect.x, object.rect.y), image);
      
      // Draw rectangle
      cv::rectangle(image,
                    cv::Point(object.rect.x, object.rect.y),
                    cv::Point(object.rect.x + object.rect.width, object.rect.y + object.rect.height),
                    color_map_[object_id],
                    kRectangleThickness,
                    CV_AA,
                    0);
    }
  } // DrawRects::DrawImageObjTracked()


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

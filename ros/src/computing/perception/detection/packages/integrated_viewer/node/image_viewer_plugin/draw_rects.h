#ifndef DRAW_RECTS_H
#define DRAW_RECTS_H

#include <opencv/cv.h>
#include <cv_tracker/image_obj.h>
#include <cv_tracker/image_obj_ranged.h>
#include <cv_tracker/image_obj_tracked.h>

namespace integrated_viewer {
  // helper class to draw detection result rectangle
  class DrawRects{
  public:
    explicit DrawRects(void);
    void DrawImageObj(const cv_tracker::image_obj::ConstPtr& rect_data, cv::Mat& image);
    void DrawImageObjRanged(const cv_tracker::image_obj_ranged::ConstPtr& rect_data, cv::Mat& image);
    void DrawImageObjTracked(const cv_tracker::image_obj_tracked::ConstPtr& rect_data, cv::Mat& image);

  protected:
    static const int kRectangleThickness;
  
  private:
    void DrawLabel(const std::string& label, const cv::Point& rectangle_origin, cv::Mat& image);
    std::vector<cv::Scalar> color_map_;
    static const cv::Scalar kBlue;
    static const cv::Scalar kGreen;
  };
}
#endif // DRAW_RECTS_H

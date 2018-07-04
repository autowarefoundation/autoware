#ifndef DRAW_RECTS_H
#define DRAW_RECTS_H

#include <opencv/cv.h>
#include <autoware_msgs/DetectedObjectArray.h>

namespace integrated_viewer {
  // helper class to draw detection result rectangle
  class DrawRects{
  public:
    explicit DrawRects(void);
    void DrawImageObj(const autoware_msgs::DetectedObjectArray::ConstPtr& rect_data, cv::Mat& image);

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

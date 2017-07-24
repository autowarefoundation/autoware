#ifndef DRAW_POINTS_H
#define DRAW_POINTS_H

#include <opencv/cv.h>
#include "autoware_msgs/PointsImage.h"

namespace integrated_viewer {
  // helper class to draw points image
  class DrawPoints{
  public:
    explicit DrawPoints(void);
    void Draw(const autoware_msgs::PointsImage::ConstPtr& points, cv::Mat& image, int drawn_size);

  private:
    cv::Mat color_map_;

  };

} // end namespace integrated_viewer

#endif // DRAW_POINTS_H

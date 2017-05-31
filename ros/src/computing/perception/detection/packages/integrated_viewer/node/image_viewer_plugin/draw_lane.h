#ifndef DRAW_LANE_H
#define DRAW_LANE_H
#include <opencv/cv.h>
#include <lane_detector/ImageLaneObjects.h>

namespace integrated_viewer {
  // helper class to draw detected lane line
  class DrawLane{
  public:
    explicit DrawLane(void);
    void Draw(const lane_detector::ImageLaneObjects::ConstPtr& lane, cv::Mat &image);

  protected:
    static const int kLineThickness;

  private:
    static const cv::Scalar kRed;
  };
}
#endif // DRAW_LANE_H

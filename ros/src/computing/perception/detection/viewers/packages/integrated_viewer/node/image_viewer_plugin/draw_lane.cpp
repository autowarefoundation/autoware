#include "draw_lane.h"

#include <opencv/cv.hpp>
#include <opencv2/core/version.hpp>

#if (CV_MAJOR_VERSION != 3)
#include <opencv2/contrib/contrib.hpp>
#endif

namespace integrated_viewer
{
  const int DrawLane::kLineThickness = 3;
  const cv::Scalar DrawLane::kRed = CV_RGB(255, 0, 0);

  DrawLane::DrawLane(void) {
  }  // DrawLane::DrawLane()

  void DrawLane::Draw(const autoware_msgs::ImageLaneObjects::ConstPtr& lane,
                      cv::Mat &image) {
    if (lane == NULL) {
      return;
    }

    // Draw detected lane
    cv::Point left_lane_start(lane->lane_l_x1, lane->lane_l_y1);
    cv::Point left_lane_end(lane->lane_l_x2, lane->lane_l_y2);
    cv::Point right_lane_start(lane->lane_r_x1, lane->lane_r_y1);
    cv::Point right_lane_end(lane->lane_r_x2, lane->lane_r_y2);

    cv::line(image, left_lane_start, left_lane_end, kRed, kLineThickness);
    cv::line(image, right_lane_start, right_lane_end, kRed, kLineThickness);

  }  // void DrawLane::Draw()
}

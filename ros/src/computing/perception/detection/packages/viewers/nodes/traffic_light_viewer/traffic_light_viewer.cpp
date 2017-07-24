#include <ros/ros.h>
#include <autoware_msgs/traffic_light.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define WINDOW_NAME "traffic light detection result"
#define WINDOW_SIZE 500
#define RADIUS      200


static const int32_t TRAFFIC_LIGHT_RED     = 0;
static const int32_t TRAFFIC_LIGHT_GREEN   = 1;
static const int32_t TRAFFIC_LIGHT_UNKNOWN = 2;

static void signalState_cb(const autoware_msgs::traffic_light::ConstPtr& msg)
{
  const int   fontFace      = cv::FONT_HERSHEY_COMPLEX;
  const float fontScale     = 1.0f;
  const int   fontThickness = 1;
  int         baseLine      = 0;
  cv::Point   textOrg;
  std::string label;
  cv::Scalar  textColor;
  cv::Scalar  signalColor;

  switch (msg->traffic_light) {
  case TRAFFIC_LIGHT_RED:
    label       = "RED";
    textColor   = CV_RGB(255, 0, 0);
    signalColor = CV_RGB(255, 0, 0);
    break;
  case TRAFFIC_LIGHT_GREEN:
    label       = "GREEN";
    textColor   = CV_RGB(0, 255, 0);
    signalColor = CV_RGB(0, 255, 0);
    break;
  default:
    label       = "NO SIGNAL DETECTED";
    textColor   = CV_RGB(255, 255, 255);
    signalColor = CV_RGB(0, 0, 0);
    break;
  }

  cv::Mat result(WINDOW_SIZE, WINDOW_SIZE, CV_8UC3, cv::Scalar(0));

  cv::circle(result, cv::Point(WINDOW_SIZE/2, WINDOW_SIZE/2), RADIUS, signalColor, CV_FILLED);

  cv::Size textSize = cv::getTextSize(label,
                                      fontFace,
                                      fontScale,
                                      fontThickness,
                                      &baseLine);

  textOrg = cv::Point(0, textSize.height + baseLine);

  cv::putText(result,
              label,
              textOrg,
              fontFace,
              fontScale,
              textColor,
              fontThickness,
              CV_AA);

  if (cvGetWindowHandle(WINDOW_NAME) != NULL) // Guard not to write destroyed window by using close button on the window
    {
      cv::imshow(WINDOW_NAME, result);
      cv::waitKey(5);
    }


}

int main(int argc, char* argv[])
{
  cv::namedWindow(WINDOW_NAME, cv::WINDOW_NORMAL);
  cv::startWindowThread();

  ros::init(argc, argv, "traffic_light_viewer");

  ros::NodeHandle n;

  ros::Subscriber signalState_sub = n.subscribe("/light_color", 1, signalState_cb);

  ros::spin();

  cv::destroyWindow(WINDOW_NAME);

  return 0;
}

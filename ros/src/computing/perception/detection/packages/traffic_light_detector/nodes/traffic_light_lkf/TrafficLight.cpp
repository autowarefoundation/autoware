#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include "TrafficLight.h"
//#include "TLVideoWriter.h"
//#include "VideoSettingsLocal.h"
#include <float.h>
#include <math.h>
#include <sstream>
//#include "traffic_light_detector/ExtractedPosition.h"
#include "traffic_light_detector/Signals.h"

using namespace cv;
// Variables
static TrafficLightDetector detector;

static Mat frame;
static int TL_COUNT = 1;


static inline bool IsNearlyZero(double x)
{
  double abs_x = fabs(x);
  int scale = 100;
  return(abs_x < DBL_MIN*scale);
}

static void image_raw_cb(const sensor_msgs::Image& image_source)
{
  //  cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image_source, sensor_msgs::image_encodings::BGR8);
  cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image_source);
  frame = cv_image->image.clone();

}

static void extractedPos_cb(const traffic_light_detector::Signals::ConstPtr& extractedPos)
{
  setContexts(detector);

  if (frame.empty() || detector.contexts.size()==0)
    return;

  /* test output */
  imshow("subscribed", frame);
  Mat targetScope;
  frame.copyTo(targetScope);
  circle(targetScope, detector.contexts.at(0).redCenter, detector.contexts.at(0).lampRadius, CV_RGB(255, 0, 0), 2, 0);
  circle(targetScope, detector.contexts.at(0).yellowCenter, detector.contexts.at(0).lampRadius, CV_RGB(255, 255, 0), 2, 0);
  circle(targetScope, detector.contexts.at(0).greenCenter, detector.contexts.at(0).lampRadius, CV_RGB(0, 255, 0), 2, 0);
  imshow("target scope", targetScope);


  Mat grayScale;
  cvtColor(frame, grayScale, CV_RGB2GRAY);
  LightState lightState = detector.brightnessDetect(grayScale);

  /* reset contexts */
  detector.contexts.clear();

  /* test output */
  Mat result = Mat::zeros(500, 500, CV_8UC3);
  std::cout << "lightState: ";
  switch(lightState) {
  case GREEN:
    std::cout << "GREEN" << std::endl;
    circle(result, Point(250, 250), 200, CV_RGB(0, 255, 0), -1);
    putText(result, "GREEN", Point(0, 30), FONT_HERSHEY_TRIPLEX, 1.0, CV_RGB(255, 255, 255), 1, CV_AA);
    break;
  case YELLOW:
    std::cout << "YELLOW" << std::endl;
    circle(result, Point(250, 250), 200, CV_RGB(255, 255, 0), -1);
    putText(result, "YELLOW", Point(0, 30), FONT_HERSHEY_TRIPLEX, 1.0, CV_RGB(255, 255, 255), 1, CV_AA);
    break;
  case RED:
    std::cout << "RED" << std::endl;
    circle(result, Point(250, 250), 200, CV_RGB(255, 0, 0), -1);
    putText(result, "RED", Point(0, 30), FONT_HERSHEY_TRIPLEX, 1.0, CV_RGB(255, 255, 255), 1, CV_AA);
    break;
  case UNDEFINED:
    std::cout << "UNDEFINED" << std::endl;
    putText(result, "UNDEFINED", Point(0, 30), FONT_HERSHEY_TRIPLEX, 1.0, CV_RGB(255, 255, 255), 1, CV_AA);
    break;
  }

  imshow("detection result", result);
  waitKey(5);
}

int main(int argc, char* argv[]) {

  //	printf("***** Traffic lights app *****\n");

  setContexts(detector);

  ros::init(argc, argv, "traffic_light_lkf");

  ros::NodeHandle n;

  ros::Subscriber image_sub = n.subscribe("/image_raw", 1, image_raw_cb);
  ros::Subscriber position_sub = n.subscribe("/traffic_light_pixel_xy", 1, extractedPos_cb);

  ros::spin();

  return 0;
  //  namedWindow(MAIN_WINDOW_NAME);
  //setMouseCallback(MAIN_WINDOW_NAME, mouseCallback);

#if 0
  setContexts(detector);


  Mat previous, resultImage;
  int image_num = 1;

  while (1) {
    std::stringstream ss;
    ss << SRC_PATH << "/Data/TrafficLight/" << image_num++ << ".png";
    std::string image_path = ss.str();
    Mat frame = imread(image_path);
    if (frame.empty())
      {
        std::cout << "cannot open file:" << image_path << std::endl;
        std::cout << "detection finish." << std::endl;
        break;
      }
    frame.copyTo(resultImage);

    Mat vehicleResult;
    cvtColor(frame, frame, CV_RGB2GRAY);
    LightState lightState = detector.brightnessDetect(frame);
    vector<Rect> boundedRects;


    /* test output */
    std::cout << "lightState: ";
    switch(lightState) {
    case GREEN:
      std::cout << "GREEN" << std::endl;
      break;
    case YELLOW:
      std::cout << "YELLOW" << std::endl;
      break;
    case RED:
      std::cout << "RED" << std::endl;
      break;
    case UNDEFINED:
      std::cout << "UNDEFINED" << std::endl;
      break;
    }
    cv::rectangle(resultImage, contexts[0].topLeft, contexts[0].botRight, CV_RGB(255, 0, 0), 5, 8);
    cv::circle(resultImage, contexts[0].redCenter, contexts[0].lampRadius, CV_RGB(255, 0, 0), -1, 0);
    cv::circle(resultImage, contexts[0].yellowCenter, contexts[0].lampRadius, CV_RGB(255, 255, 0), -1, 0);
    cv::circle(resultImage, contexts[0].greenCenter, contexts[0].lampRadius, CV_RGB(0, 255, 0), -1, 0);


    imshow(MAIN_WINDOW_NAME, resultImage);

    char c = waitKey(0);
    if (c == 27) break;

    previous.release();
    previous.data = frame.data;
  }

  return 0;
#endif
}



void setContexts(TrafficLightDetector &detector) {

  for (int i = 0; i < TL_COUNT; i++) {
    //    contexts[i].lightState = UNDEFINED;
    //    detector.contexts.push_back(contexts[i]);
    Context tmp_context(Point(99, 44), // aRedCenter
                        Point(74, 44), // aYellowCenter
                        Point(49, 45), // aGreenCenter
                        10,            // aLampRadius
                        Point(0, 0),   // aTopLeft
                        Point(0, 0)    // aBotRight
                        );
    tmp_context.lightState = UNDEFINED;
    detector.contexts.push_back(tmp_context);

  }
}
#if 0
void drawTrafficLights(Mat &targetImg, LightState lightState) {
  switch (lightState) {
  case GREEN:
    circle(targetImg, GREEN_DRAW_CENTER, LIGHT_DRAW_RADIUS, MY_COLOR_GREEN, -1);
    break;
  case YELLOW:
    circle(targetImg, YELLOW_DRAW_CENTER, LIGHT_DRAW_RADIUS, MY_COLOR_YELLOW, -1);
    break;
  case RED:
    circle(targetImg, RED_DRAW_CENTER, LIGHT_DRAW_RADIUS, MY_COLOR_RED, -1);
    break;
  }
}

static Mat showMask, redMask, blueMask, greenMask;
void initMasks(char *pathToShowMask) {
  /* Initialize show mask */
  showMask = loadMask(pathToShowMask);
  Mat grayMask(showMask);
  cvtColor(showMask, showMask, CV_GRAY2RGB);
  showMask.copyTo(redMask);
  showMask.copyTo(blueMask);
  showMask.copyTo(greenMask);
  redMask.setTo(MY_COLOR_RED, grayMask);
  blueMask.setTo(MY_COLOR_BLUE, grayMask);
  greenMask.setTo(MY_COLOR_GREEN, grayMask);
}

void drawEnforcement(Mat &targetImg, bool isEnforced, LightState lightState) {

  addWeighted(targetImg, 1.0, showMask, -0.5, 0, targetImg);
  if (isEnforced) {
    if (lightState == GREEN)
      addWeighted(targetImg, 1.0, greenMask, 2.0, 0, targetImg);
    else
      addWeighted(targetImg, 1.0, redMask, 2.0, 0, targetImg);
  } else {
    addWeighted(targetImg, 1.0, blueMask, 2.0, 0, targetImg);
  }

}

void drawBoundedRects(Mat &targetImg, vector<Rect> boundedRects) {
  for (int i = 0; i< boundedRects.size(); i++) {
    if (boundedRects[i].width >= MIN_WIDTH && boundedRects[i].width <= MAX_WIDTH && boundedRects[i].height >= MIN_HEIGHT && boundedRects[i].height <= MAX_HEIGHT) {
      rectangle(targetImg, boundedRects[i].tl(), boundedRects[i].br(), MY_COLOR_PURPLE, 2, 8, 0);
    }
  }
}
#endif

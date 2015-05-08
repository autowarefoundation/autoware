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

using namespace cv;
// Variables
static TrafficLightDetector detector;

static Mat frame;
//static int TL_COUNT = 1;


static inline bool IsNearlyZero(double x)
{
  double abs_x = fabs(x);
  int scale = 100;
  return(abs_x < DBL_MIN*scale);
}

static void putResult_inText(Mat *image, const vector<Context> &contexts)
{
  std::string label;
  const int fontFace = FONT_HERSHEY_COMPLEX_SMALL;
  const float fontScale = 1.0f;
  const int fontThickness = 1;
  int baseline = 0;
  CvPoint textOrg;
  CvScalar textColor;

  for (unsigned int i=0; i<contexts.size(); i++)
    {
      Context ctx = contexts.at(i);
      if (ctx.lampRadius < MINIMAM_RADIUS)
        continue;

      switch(ctx.lightState) {
      case GREEN:
        label = "GREEN";
        textColor = CV_RGB(0, 255, 0);
        break;
      case YELLOW:
        label = "YELLOW";
        textColor = CV_RGB(255, 255, 0);
        break;
      case RED:
        label = "RED";
        textColor = CV_RGB(255, 0, 0);
        break;
      case UNDEFINED:
        label = "UNDEFINED";
        textColor = CV_RGB(0, 0, 0);
      }

      getTextSize(label,
                  fontFace,
                  fontScale,
                  fontThickness,
                  &baseline);

      textOrg = Point(ctx.topLeft.x, ctx.botRight.y + baseline);

      putText(*image,
              label,
              textOrg,
              fontFace,
              fontScale,
              textColor,
              fontThickness,
              CV_AA);
    }
}

static void image_raw_cb(const sensor_msgs::Image& image_source)
{
  cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image_source, sensor_msgs::image_encodings::BGR8);
  //  cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image_source);
  frame = cv_image->image.clone();

}

static void extractedPos_cb(const traffic_light_detector::Signals::ConstPtr& extractedPos)
{
  if (frame.empty())
    return;

  /* test output */
  // imshow("subscribed", frame);
  // waitKey(5);

  /* reset contexts */
  detector.contexts.clear();

  setContexts(detector, extractedPos);

  // if (detector.contexts.size()==0)
  //   return;

  /* test output */
  Mat targetScope;
  frame.copyTo(targetScope);
  for (unsigned int i=0; i<detector.contexts.size(); i++)
    {
      if (detector.contexts.at(i).lampRadius < MINIMAM_RADIUS)
        continue;

      circle(targetScope, detector.contexts.at(i).redCenter, detector.contexts.at(i).lampRadius, CV_RGB(255, 0, 0), 1, 0);
      circle(targetScope, detector.contexts.at(i).yellowCenter, detector.contexts.at(i).lampRadius, CV_RGB(255, 255, 0), 1, 0);
      circle(targetScope, detector.contexts.at(i).greenCenter, detector.contexts.at(i).lampRadius, CV_RGB(0, 255, 0), 1, 0);
    }


  Mat grayScale;
  cvtColor(frame, grayScale, CV_RGB2GRAY);
  detector.brightnessDetect(grayScale);

  /* test output */
  putResult_inText(&targetScope, detector.contexts);
  // for (unsigned int i=0; i<detector.contexts.size(); i++)
  //   {
  //     Context ctx = detector.contexts.at(i);
  //     if (ctx.lampRadius < MINIMAM_RADIUS)
  //       continue;

  //     switch(ctx.lightState) {
  //     case GREEN:
  //       putText(targetScope, "GREEN", Point(ctx.topLeft.x, ctx.botRight.y), FONT_HERSHEY_COMPLEX_SMALL, 1.0, CV_RGB(0,255,0), 2, CV_AA);
  //       break;
  //     case YELLOW:
  //       putText(targetScope, "YELLOW", Point(ctx.topLeft.x, ctx.botRight.y), FONT_HERSHEY_COMPLEX_SMALL, 1.0, CV_RGB(255,255,0), 2, CV_AA);
  //       break;
  //     case RED:
  //       putText(targetScope, "RED", Point(ctx.topLeft.x, ctx.botRight.y), FONT_HERSHEY_COMPLEX_SMALL, 1.0, CV_RGB(255,0,0), 2, CV_AA);
  //       break;
  //     case UNDEFINED:
  //       putText(targetScope, "UNDEFINED", Point(ctx.topLeft.x, ctx.botRight.y), FONT_HERSHEY_COMPLEX_SMALL, 1.0, CV_RGB(0,0,0), 2, CV_AA);
  //       break;
  //     }
  //   }

  imshow("target scope", targetScope);
  waitKey(5);
}

int main(int argc, char* argv[]) {

  //	printf("***** Traffic lights app *****\n");

  //setContexts(detector);

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



void setContexts(TrafficLightDetector &detector,
                 const traffic_light_detector::Signals::ConstPtr& extractedPos)
{

  /* copy parts of data to local variable */
  std::vector<traffic_light_detector::ExtractedPosition> signals;
  std::vector<traffic_light_detector::ExtractedPosition>::iterator sig_iterator;
  for (unsigned int i=0; i<extractedPos->Signals.size(); i++ )
    {
      traffic_light_detector::ExtractedPosition tmp;
      tmp.u      = extractedPos->Signals.at(i).u;
      tmp.v      = extractedPos->Signals.at(i).v;
      tmp.radius = extractedPos->Signals.at(i).radius;
      tmp.type   = extractedPos->Signals.at(i).type;
      tmp.linkId = extractedPos->Signals.at(i).linkId;
      signals.push_back(tmp);
    }

  std::vector<int> linkid_vector;
  for (sig_iterator=signals.begin(); sig_iterator<signals.end(); sig_iterator++) {
    linkid_vector.push_back(sig_iterator->linkId);
  }

  /* get array that has unique linkID values as its element */
  std::sort(linkid_vector.begin(), linkid_vector.end());
  std::vector<int>::iterator new_end = std::unique(linkid_vector.begin(), linkid_vector.end());
  linkid_vector.erase(new_end, linkid_vector.end());

  /* assemble fragmented signal lamp in a context */
  for (unsigned int ctx_idx=0; ctx_idx<linkid_vector.size(); ctx_idx++)
    {
      Context ctx;
      int min_radius  = 20;
      int most_left   = frame.cols;
      int most_top    = frame.rows;
      int most_right  = 0;
      int most_bottom = 0;

      for (sig_iterator=signals.begin(); sig_iterator<signals.end(); sig_iterator++)
        {
          int img_x = sig_iterator->u;
          int img_y = sig_iterator->v;
          int radius = sig_iterator->radius;
          if (sig_iterator->linkId == linkid_vector.at(ctx_idx) &&
              0 < img_x - radius - ROI_MARGINE && img_x + radius + ROI_MARGINE < frame.cols &&
              0 < img_y - radius - ROI_MARGINE && img_y + radius + ROI_MARGINE < frame.rows)
            {
              switch (sig_iterator->type) {
              case 1:           /* RED */
                ctx.redCenter    = Point( img_x, img_y );
                break;
              case 2:           /* GREEN */
                ctx.greenCenter  = Point( img_x, img_y );
                break;
              case 3:           /* YELLOW */
                ctx.yellowCenter = Point( img_x, img_y );
                break;
              }
              min_radius    = (min_radius > radius) ? radius : min_radius;
              most_left     = (most_left > img_x - radius - ROI_MARGINE) ? img_x - radius - ROI_MARGINE : most_left;
              most_top      = (most_top > img_y - radius - ROI_MARGINE) ? img_y - radius - ROI_MARGINE : most_top;
              most_right    = (most_right < img_x + radius + ROI_MARGINE) ? img_x + radius + ROI_MARGINE : most_right;
              most_bottom   = (most_bottom < img_y + radius + ROI_MARGINE) ? img_y + radius + ROI_MARGINE : most_bottom;
            }
        }

      ctx.lampRadius = (int)(min_radius / 1.5);
      ctx.topLeft    = Point(most_left, most_top);
      ctx.botRight   = Point(most_right, most_bottom);
      ctx.lightState = UNDEFINED;

      detector.contexts.push_back(ctx);
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

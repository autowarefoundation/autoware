#include "TrafficLight.h"
#include "RegionTLR.h"
#include "TrafficLightDetector.h"

#define BLACK CV_RGB(0, 0, 0)
#define WHITE CV_RGB(255, 255, 255)

struct regionCandidate {
  cv::Point  center;
  int    idx;
  double circleLevel;
  bool   isBlacked;
};

//#define SHOW_DEBUG_INFO
extern thresholdSet thSet;      // declared in traffic_light_lkf.cpp

/*
  check if val is in range from lower to uppper
  This function also consider circulation
*/
static inline  bool IsRange(const double lower, const double upper, const double val)
{
  if (lower <= upper) {
    if (lower <= val && val <= upper) {
      return true;
    }
  } else {
    if (val <= upper || lower <= val)
      return true;
  }

  return false;
} /* static inline  bool IsRange() */


static void colorExtraction(const cv::Mat& src, // input HSV image
                            cv::Mat*       dst, // specified color extracted binarized image
                            const double   hue_lower, const double hue_upper, // hue thresholds
                            const double   sat_lower, const double sat_upper, // satulation thresholds
                            const double   val_lower, const double val_upper) // value thresholds
{
  /* create imput image copy */
  cv::Mat input_img = src.clone();

  *dst = cv::Scalar::all(0);

  /*
    ref:
    http://qiita.com/crakaC/items/65fab9d0b0ac29e68ab6
   */

  /* create LookUp Table */
  cv::Mat lut(256, 1, CV_8UC3);
  for (int i=0; i<256; i++)
    {
	  lut.at<cv::Vec3b>(i)[0] = (IsRange(hue_lower, hue_upper, Actual_Hue(i))) ? 255 : 0;
	  lut.at<cv::Vec3b>(i)[1] = (IsRange(sat_lower, sat_upper, Actual_Sat(i))) ? 255 : 0;
	  lut.at<cv::Vec3b>(i)[2] = (IsRange(val_lower, val_upper, Actual_Val(i))) ? 255 : 0;
    }

  /* apply LUT to input image */
  cv::Mat extracted(input_img.rows, input_img.cols, CV_8UC3);
  LUT(input_img, lut, extracted);

  /* divide image into each channel */
  std::vector<cv::Mat> channels;
  split(extracted, channels);

  /* create mask */
  bitwise_and(channels[0], channels[1], *dst);
  bitwise_and(*dst, channels[2], *dst);


} /* static void colorExtraction() */


static bool checkExtinctionLight(const cv::Mat&  src_img,
                                 const cv::Point top_left,
                                 const cv::Point bot_right,
                                 const cv::Point bright_center)
{

  /* check whether new roi is included by source image */
  cv::Point roi_top_left;
  roi_top_left.x = (top_left.x < 0) ? 0 :
    (src_img.cols < top_left.x) ? src_img.cols : top_left.x;
  roi_top_left.y = (top_left.y < 0) ? 0 :
    (src_img.rows < top_left.y) ? src_img.rows : top_left.y;

  cv::Point roi_bot_right;
  roi_bot_right.x = (bot_right.x < 0) ? 0 :
    (src_img.cols < bot_right.x) ? src_img.cols : bot_right.x;
  roi_bot_right.y = (bot_right.y < 0) ? 0 :
    (src_img.rows < bot_right.y) ? src_img.rows : bot_right.y;

  cv::Mat roi = src_img(cv::Rect(roi_top_left, roi_bot_right));

  cv::Mat roi_HSV;
  cvtColor(roi, roi_HSV, CV_BGR2HSV);

  cv::Mat hsv_channel[3];
  split(roi_HSV, hsv_channel);

  int anchor = 3;
  cv::Mat kernel = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2*anchor + 1, 2*anchor + 1), cv::Point(anchor, anchor));

  cv::Mat topHat_dark;
  morphologyEx(hsv_channel[2], topHat_dark, cv::MORPH_TOPHAT, kernel, cv::Point(anchor, anchor), 5);

  /* sharpening */
  cv::Mat tmp;
  threshold(topHat_dark, tmp, 0.1*255, 255, cv::THRESH_BINARY_INV);
  tmp.copyTo(topHat_dark);

  /* filter by its shape and search dark region */
  std::vector< std::vector<cv::Point> > dark_contours;
  std::vector<cv::Vec4i> dark_hierarchy;
  findContours(topHat_dark,
               dark_contours,
               dark_hierarchy,
               CV_RETR_CCOMP,
               CV_CHAIN_APPROX_NONE);

  int contours_idx = 0;
  bool isThere_dark = false;

  /* check whether "turned off light" like region are in this roi */
  for (unsigned int i=0; i<dark_contours.size(); i++)
    {
      cv::Rect bound = boundingRect(dark_contours.at(contours_idx));
      double area = contourArea(dark_contours.at(contours_idx));
      double perimeter = arcLength(dark_contours.at(contours_idx), true);
      double circleLevel = (IsNearlyZero(perimeter)) ? 0.0f : (4.0f * CV_PI * area / pow(perimeter, 2));

      if (std::max(bound.width, bound.height) < 2*std::min(bound.width, bound.height) && // dimension ratio
          CIRCLE_LEVEL_THRESHOLD <= circleLevel)                                         // round enough

        {
          isThere_dark = true;
          // std::cerr << "there is dark region" << std::endl;
        }

      contours_idx = dark_hierarchy[contours_idx][0];
      if (contours_idx < 0)
        break;
    }

  return isThere_dark;

} /* static bool checkExtinctionLight() */


static cv::Mat signalDetect_inROI(const cv::Mat& roi,
                                  const cv::Mat&     src_img,
                                  const double       estimatedRadius,
                                  const cv::Point roi_topLeft,
                                  bool in_turn_signal //if true it will not try to mask by using "circularity""
                                  )
{
  /* reduce noise */
  cv::Mat noiseReduced(roi.rows, roi.cols, CV_8UC3);
  GaussianBlur(roi, noiseReduced, cv::Size(3, 3), 0, 0);

  /* extract color information */
  cv::Mat red_mask(roi.rows, roi.cols, CV_8UC1);
  colorExtraction(noiseReduced       ,
                  &red_mask          ,
                  thSet.Red.Hue.lower, thSet.Red.Hue.upper,
                  thSet.Red.Sat.lower, thSet.Red.Sat.upper,
                  thSet.Red.Val.lower, thSet.Red.Val.upper);

  cv::Mat yellow_mask(roi.rows, roi.cols, CV_8UC1);
  colorExtraction(noiseReduced          ,
                  &yellow_mask          ,
                  thSet.Yellow.Hue.lower, thSet.Yellow.Hue.upper,
                  thSet.Yellow.Sat.lower, thSet.Yellow.Sat.upper,
                  thSet.Yellow.Val.lower, thSet.Yellow.Val.upper);

  cv::Mat green_mask(roi.rows, roi.cols, CV_8UC1);
  colorExtraction(noiseReduced         ,
                  &green_mask          ,
                  thSet.Green.Hue.lower, thSet.Green.Hue.upper,
                  thSet.Green.Sat.lower, thSet.Green.Sat.upper,
                  thSet.Green.Val.lower, thSet.Green.Val.upper);

  /* combine all color mask and create binarized image */
  cv::Mat binarized = cv::Mat::zeros(roi.rows, roi.cols, CV_8UC1);
  bitwise_or(red_mask, yellow_mask, binarized);
  bitwise_or(binarized, green_mask, binarized);
  threshold(binarized, binarized, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);

  /* filter by its shape and index each bright region */
  std::vector< std::vector<cv::Point> > bright_contours;
  std::vector<cv::Vec4i> bright_hierarchy;
  findContours(binarized,
               bright_contours,
               bright_hierarchy,
               CV_RETR_CCOMP,
               CV_CHAIN_APPROX_NONE);


  cv::Mat bright_mask = cv::Mat::zeros(roi.rows, roi.cols, CV_8UC1);

  int contours_idx = 0;
  std::vector<regionCandidate> candidates;
  for (unsigned int i=0; i<bright_contours.size(); i++)
    {
      cv::Rect bound = boundingRect(bright_contours.at(contours_idx));
      cv::Scalar rangeColor = BLACK;
      struct regionCandidate cnd;
      double area = contourArea(bright_contours.at(contours_idx)); // unit : pixel
      double perimeter = arcLength(bright_contours.at(contours_idx), true);
      double circleLevel = (IsNearlyZero(perimeter)) ? 0.0f : (4.0f * CV_PI * area / pow(perimeter, 2));

      double area_lower_limit = (3*sqrt(3)) * pow(estimatedRadius / 3.0, 2) / 4; // the area of inscribed triangle of 1/3 circle
      double area_upper_limit = pow(estimatedRadius, 2) * M_PI;                  // the area of the circle

      if (std::max(bound.width, bound.height) < 2*std::min(bound.width, bound.height) && /* dimension ratio */
          CIRCLE_LEVEL_THRESHOLD <= circleLevel                                       &&
          area_lower_limit <= area && area <= area_upper_limit)
        {
          // std::cerr << "circleLevel: " << circleLevel << std::endl;
          rangeColor    = WHITE;
          cnd.center.x  = bound.x + bound.width/2;
          cnd.center.y  = bound.y + bound.height/2;
          cnd.idx       = contours_idx;
          cnd.circleLevel = (IsNearlyZero(perimeter)) ? 0.0f : (4.0 * CV_PI * area / pow(perimeter, 2));
          cnd.isBlacked = false;
          candidates.push_back(cnd);
        }

      drawContours(bright_mask,
                   bright_contours,
                   contours_idx,
                   rangeColor,
                   CV_FILLED,
                   8,
                   bright_hierarchy,
                   0);

      /* only contours on toplevel are considered */
      contours_idx = bright_hierarchy[contours_idx][0];
      if (contours_idx < 0)
        break;
    }

#ifdef SHOW_DEBUG_INFO
  imshow("bright_mask", bright_mask);
  cv::waitKey(10);
#endif

  unsigned int candidates_num = candidates.size();

  // std::cerr << "before checkExtrinctionLight. candidates: " << candidates_num << std::endl;

  /* decrease candidates by checking existence of turned off light in their neighborhood */
  if (!in_turn_signal && candidates_num > 1)    /* if there are multipule candidates */
    {
      for (unsigned int i=0; i<candidates.size(); i++)
        {
          /* check wheter this candidate seems to be green lamp */
          cv::Point check_roi_topLeft  = cv::Point(candidates.at(i).center.x - 2*estimatedRadius + roi_topLeft.x,
                                                   candidates.at(i).center.y - 2*estimatedRadius + roi_topLeft.y);
          cv::Point check_roi_botRight = cv::Point(candidates.at(i).center.x + 6*estimatedRadius + roi_topLeft.x,
                                                   candidates.at(i).center.y + 2*estimatedRadius + roi_topLeft.y);
          bool likeGreen = checkExtinctionLight(src_img, check_roi_topLeft, check_roi_botRight, candidates.at(i).center);

          /* check wheter this candidate seems to be yellow lamp */
          check_roi_topLeft  = cv::Point(candidates.at(i).center.x - 4*estimatedRadius + roi_topLeft.x,
                                     candidates.at(i).center.y - 2*estimatedRadius + roi_topLeft.y);
          check_roi_botRight = cv::Point(candidates.at(i).center.x + 4*estimatedRadius + roi_topLeft.x,
                                     candidates.at(i).center.y + 2*estimatedRadius + roi_topLeft.y);
          bool likeYellow = checkExtinctionLight(src_img, check_roi_topLeft, check_roi_botRight, candidates.at(i).center);

          /* check wheter this candidate seems to be red lamp */
          check_roi_topLeft  = cv::Point(candidates.at(i).center.x - 6*estimatedRadius + roi_topLeft.x,
                                     candidates.at(i).center.y - 2*estimatedRadius + roi_topLeft.y);
          check_roi_botRight = cv::Point(candidates.at(i).center.x + 2*estimatedRadius + roi_topLeft.x,
                                     candidates.at(i).center.y + 2*estimatedRadius + roi_topLeft.y);
          bool likeRed = checkExtinctionLight(src_img, check_roi_topLeft, check_roi_botRight, candidates.at(i).center);


          if (!likeGreen && !likeYellow && !likeRed) /* this region may not be traffic light */
            {
              candidates_num--;
              drawContours(bright_mask,
                           bright_contours,
                           candidates.at(i).idx,
                           BLACK,
                           CV_FILLED,
                           8,
                           bright_hierarchy,
                           0);
              candidates.at(i).isBlacked = true;
            }
        }
    }

  // std::cerr << "after checkExtrinctionLight. candidates: " << candidates_num << std::endl;

  /* choose one candidate by comparing degree of circularity */
  if (!in_turn_signal && candidates_num > 1)       /* if there are still multiple candidates */
    {
      double min_diff = DBL_MAX;
      unsigned int min_idx = 0;

      /* search the region that has nearest degree of circularity to 1 */
      for (unsigned int i=0; i<candidates.size(); i++)
        {
          if(candidates.at(i).isBlacked)
            continue;

          double diff = fabs(1 - candidates.at(i).circleLevel);
          if (min_diff > diff)
            {
              min_diff = diff;
              min_idx = i;
            }
        }

      /* fill region of non-candidate */
      for (unsigned int i=0; i<candidates.size(); i++)
        {

          if(candidates.at(i).isBlacked)
            continue;

          cv::Scalar regionColor = BLACK;
          candidates.at(i).isBlacked = true;
          if (i == min_idx)
            {
              regionColor = WHITE;
              candidates.at(i).isBlacked = false;
            }

          drawContours(bright_mask,
                       bright_contours,
                       candidates.at(i).idx,
                       regionColor,
                       CV_FILLED,
                       8,
                       bright_hierarchy,
                       0);
        }
    }

  return bright_mask;

} /* static void signalDetect_inROI() */


/* constructor for non initialize value */
TrafficLightDetector::TrafficLightDetector() {}


void TrafficLightDetector::brightnessDetect(const cv::Mat &input) {

  cv::Mat tmpImage;
  input.copyTo(tmpImage);

  /* contrast correction */
  cv::Mat tmp;
  cvtColor(tmpImage, tmp, CV_BGR2HSV);
  std::vector<cv::Mat> hsv_channel;
  split(tmp, hsv_channel);

  float correction_factor = 10.0;
  uchar lut[256];
  for (int i=0; i<256; i++) {
    lut[i] = 255.0 / (1 + exp(-correction_factor*(i-128)/255));
  }

  LUT(hsv_channel[2], cv::Mat(cv::Size(256, 1), CV_8U, lut), hsv_channel[2]);
  merge(hsv_channel, tmp);
  cvtColor(tmp, tmpImage, CV_HSV2BGR);

  for (int i = 0; i < static_cast<int>(contexts.size()); i++) {
    Context context = contexts.at(i);

    if (context.topLeft.x > context.botRight.x)
      continue;

    /* extract region of interest from input image */
    cv::Mat roi = tmpImage(cv::Rect(context.topLeft, context.botRight));

    /* convert color space (BGR -> HSV) */
    cv::Mat roi_HSV;
    cvtColor(roi, roi_HSV, CV_BGR2HSV);

    /* search the place where traffic signals seem to be */
    cv::Mat    signalMask    = signalDetect_inROI(roi_HSV, input.clone(),
                                                  context.lampRadius,
                                                  context.topLeft,
                                                  context.leftTurnSignal || context.rightTurnSignal);

    /* detect which color is dominant */
    cv::Mat extracted_HSV;
    roi.copyTo(extracted_HSV, signalMask);

#ifdef SHOW_DEBUG_INFO
    extracted_HSV.copyTo(roi);
    imshow("tmpImage", tmpImage);
    cv::waitKey(5);
#endif

    cvtColor(extracted_HSV, extracted_HSV, CV_BGR2HSV);

    int red_pixNum    = 0;
    int yellow_pixNum = 0;
    int green_pixNum  = 0;
    int valid_pixNum  = 0;
    for (int y=0; y<extracted_HSV.rows; y++)
      {
        for (int x=0; x<extracted_HSV.cols; x++)
          {
            /* extract H, V value from pixel */
            double hue = Actual_Hue(extracted_HSV.at<cv::Vec3b>(y, x)[0]);
            uchar  val = extracted_HSV.at<cv::Vec3b>(y, x)[2];

            if (val == 0) {
              continue;         // this is masked pixel
            }
            valid_pixNum++;

            /* search which color is actually bright */
            if (IsRange(thSet.Red.Hue.lower, thSet.Red.Hue.upper, hue)) {
              red_pixNum++;
            }

            if (IsRange(thSet.Yellow.Hue.lower, thSet.Yellow.Hue.upper, hue)) {
              yellow_pixNum++;
            }

            if (IsRange(thSet.Green.Hue.lower, thSet.Green.Hue.upper, hue)) {
              green_pixNum++;
            }
          }
    }

    // std::cout << "(green, yellow, red) / valid = (" << green_pixNum << ", " << yellow_pixNum << ", " << red_pixNum << ") / " << valid_pixNum <<std::endl;

    bool isRed_bright;
    bool isYellow_bright;
    bool isGreen_bright;

    if (valid_pixNum > 0) {
      isRed_bright    = ( ((double)red_pixNum / valid_pixNum)    > 0.5) ? true : false;
      isYellow_bright = ( ((double)yellow_pixNum / valid_pixNum) > 0.5) ? true : false;
      isGreen_bright  = ( ((double)green_pixNum / valid_pixNum)  > 0.5) ? true : false;
    } else {
      isRed_bright    = false;
      isYellow_bright = false;
      isGreen_bright  = false;
    }

    int currentLightsCode = getCurrentLightsCode(isRed_bright, isYellow_bright, isGreen_bright);
    contexts.at(i).lightState = determineState(contexts.at(i).lightState, currentLightsCode, &(contexts.at(i).stateJudgeCount));

    roi.setTo(cv::Scalar(0));
  }
}

double getBrightnessRatioInCircle(const cv::Mat &input, const cv::Point center, const int radius) {

  int whitePoints = 0;
  int blackPoints = 0;

  for (int i = center.x - radius; i < center.x + radius; i++) {
    for (int j = center.y - radius; j < center.y + radius; j++) {
      if ((i - center.x)*(i - center.x) + (j - center.y)*(j - center.y) < radius*radius) {
        (input.at<uchar>(j,i) == 0) ? blackPoints++ : whitePoints++;
      }
    }
  }
  //printf("Ratio: %f\n", ((double)whitePoints) / (whitePoints + blackPoints));
  //std::cout << "(" << center.x << ", " << center.y << ") "<< "  white:" << whitePoints << " black: " << blackPoints << std::endl;
  return ((double)whitePoints) / (whitePoints + blackPoints);
}

int getCurrentLightsCode(bool display_red, bool display_yellow, bool display_green) {
  return (int)display_red + 2 * ((int) display_yellow) + 4 * ((int) display_green);
}

LightState determineState(LightState previousState, int currentLightsCode, int* stateJudgeCount) {
  //printf("Previous state: %d, currentCode: %d, Switched state to %d\n", previousState, currentLightsCode, STATE_TRANSITION_MATRIX[previousState][currentLightsCode]);
  LightState current = STATE_TRANSITION_MATRIX[previousState][currentLightsCode];

  if (current == UNDEFINED) {
    /* if current state is UNDEFINED, return previous state tentatively */
    return previousState;
  }

  if(*stateJudgeCount > CHANGE_STATE_THRESHOLD)
    {
      *stateJudgeCount = 0;
      return current;
    }
  else
    {
      (*stateJudgeCount)++;
      return previousState;
    }
}



/*
 *  Attempt to recognize by color tracking in HSV. Detects good only green, but need to
 *  play also with S and V parameters range.
 */
void TrafficLightDetector::colorDetect(const cv::Mat &input, cv::Mat &output, const cv::Rect coords, int Hmin, int Hmax) {

  if (input.channels() != 3) {
    return;
  }

  cv::Mat hsv, thresholded;
  cvtColor(input, hsv, CV_RGB2HSV, 0);
  inRange(hsv, cv::Scalar(Hmin,0,0), cv::Scalar(Hmax,255,255), thresholded);

  cvtColor(thresholded, thresholded, CV_GRAY2RGB);
  thresholded.copyTo(output);

  rectangle(output, coords, MY_COLOR_RED);
}

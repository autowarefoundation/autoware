#include "TrafficLight.h"
#include "TrafficLightDetector.h"

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


static void colorExtraction(const Mat&   src,                               // input HSV image
                            Mat*         dst,                               // specified color extracted binarized image
                            const double hue_lower, const double hue_upper, // hue thresholds
                            const double sat_lower, const double sat_upper, // satulation thresholds
                            const double val_lower, const double val_upper) // value thresholds
{
  /* create imput image copy */
  Mat input_img = src.clone();

  *dst = Scalar::all(0);

  /*
    ref:
    http://qiita.com/crakaC/items/65fab9d0b0ac29e68ab6
   */

  /* create LookUp Table */
  Mat lut(256, 1, CV_8UC3);
  for (int i=0; i<256; i++)
    {
	  lut.at<Vec3b>(i)[0] = (IsRange(hue_lower, hue_upper, Actual_Hue(i))) ? 255 : 0;
	  lut.at<Vec3b>(i)[1] = (IsRange(sat_lower, sat_upper, Actual_Sat(i))) ? 255 : 0;
	  lut.at<Vec3b>(i)[2] = (IsRange(val_lower, val_upper, Actual_Val(i))) ? 255 : 0;
    }

  /* apply LUT to input image */
  Mat extracted(input_img.rows, input_img.cols, CV_8UC3);
  LUT(input_img, lut, extracted);

  /* divide image into each channel */
  std::vector<Mat> channels;
  split(extracted, channels);

  /* create mask */
  bitwise_and(channels[0], channels[1], *dst);
  bitwise_and(*dst, channels[2], *dst);


} /* static void colorExtraction() */


static Mat signalDetect_inROI(const Mat& roi, const double estimatedRadius)
{
  /* reduce noise */
  Mat noiseReduced(roi.rows, roi.cols, CV_8UC3);
  GaussianBlur(roi, noiseReduced, Size(7, 7), 0, 0);

  /* extract color information */
  Mat red_mask(roi.rows, roi.cols, CV_8UC1);
  colorExtraction(noiseReduced       ,
                  &red_mask          ,
                  thSet.Red.Hue.lower, thSet.Red.Hue.upper,
                  thSet.Red.Sat.lower, thSet.Red.Sat.upper,
                  thSet.Red.Val.lower, thSet.Red.Val.upper);

  Mat yellow_mask(roi.rows, roi.cols, CV_8UC1);
  colorExtraction(noiseReduced          ,
                  &yellow_mask          ,
                  thSet.Yellow.Hue.lower, thSet.Yellow.Hue.upper,
                  thSet.Yellow.Sat.lower, thSet.Yellow.Sat.upper,
                  thSet.Yellow.Val.lower, thSet.Yellow.Val.upper);

  Mat green_mask(roi.rows, roi.cols, CV_8UC1);
  colorExtraction(noiseReduced         ,
                  &green_mask          ,
                  thSet.Green.Hue.lower, thSet.Green.Hue.upper,
                  thSet.Green.Sat.lower, thSet.Green.Sat.upper,
                  thSet.Green.Val.lower, thSet.Green.Val.upper);


  Mat red(roi.rows, roi.cols, CV_8UC3, CV_RGB(255, 0, 0));
  Mat red_test;
  red.copyTo(red_test, red_mask);


  Mat yellow(roi.rows, roi.cols, CV_8UC3, CV_RGB(255, 255, 0));
  Mat yellow_test;
  yellow.copyTo(yellow_test, yellow_mask);


  Mat green(roi.rows, roi.cols, CV_8UC3, CV_RGB(0, 255, 0));
  Mat green_test;
  green.copyTo(green_test, green_mask);


  // imshow ("red", red_test);
  // imshow ("yellow", yellow_test);
  // imshow ("green", green_test);
  // waitKey(10);

  /* combine all color mask and create binarized image */
  Mat binarized = Mat::zeros(roi.rows, roi.cols, CV_8UC1);
  bitwise_or(red_mask, yellow_mask, binarized);
  bitwise_or(binarized, green_mask, binarized);
  threshold(binarized, binarized, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);

  /* find contours in binarized image */
  std::vector< std::vector<Point> > contours;
  std::vector<Vec4i> hierarchy;

  findContours(binarized,
               contours,
               hierarchy,
               CV_RETR_CCOMP,
               CV_CHAIN_APPROX_NONE);

  /* shape judgement */
  Mat contours_img = Mat::zeros(binarized.rows, binarized.cols, CV_8UC3);
  int contours_idx = 0;
  for (unsigned int i=0; i<contours.size(); i++)
    {
      /* if the area contours has less than 3 points, it may not be detected as traffic signal */
      if (contours.at(contours_idx).size() < 3) {
        continue;
      }

      double area = contourArea(contours.at(contours_idx));
      double perimeter = arcLength(contours.at(contours_idx), true);
      double circleLevel = (IsNearlyZero(perimeter)) ? 0.0f : (4.0 * CV_PI * area / (perimeter * perimeter));

      /* correct search area center point */
      if (CIRCLE_LEVEL_THRESHOLD <= circleLevel && CIRCLE_AREA_THRESHOLD <= area)
        {
          Rect bound = boundingRect(contours.at(contours_idx));
          Point correctedCenter(bound.x + bound.width/2, bound.y + bound.height/2);
          circle(contours_img, correctedCenter, estimatedRadius, CV_RGB(255, 255, 255), CV_FILLED);
        }


      /* Only contours on toplevel are considerd */
      contours_idx = hierarchy[contours_idx][0];
      if (contours_idx < 0)
        break;
    }

  // imshow("contours_img", contours_img);
  // waitKey(10);

  return contours_img;


} /* static void signalDetect_inROI() */


/* constructor for non initialize value */
TrafficLightDetector::TrafficLightDetector() {}


void TrafficLightDetector::brightnessDetect(const Mat &input) {

  Mat tmpImage;
  input.copyTo(tmpImage);


  for (int i = 0; i < static_cast<int>(contexts.size()); i++) {
    Context context = contexts.at(i);

    if (context.topLeft.x > context.botRight.x)
      continue;

    /* extract region of interest from input image */
    Mat roi = tmpImage(Rect(context.topLeft, context.botRight));

    /* convert color space (BGR -> HSV) */
    Mat roi_HSV;
    cvtColor(roi, roi_HSV, CV_BGR2HSV);

    /* search the place where traffic signals seem to be */
    Mat    signalMask    = signalDetect_inROI(roi_HSV, context.lampRadius);

    /* detect which color is dominant */
    Mat extracted_HSV;
    bitwise_and(roi, signalMask, roi);

    // imshow("tmpImage", tmpImage);
    // waitKey(5);

    cvtColor(roi, extracted_HSV, CV_BGR2HSV);

    int red_pixNum    = 0;
    int yellow_pixNum = 0;
    int green_pixNum  = 0;
    int valid_pixNum  = 0;
    for (int y=0; y<extracted_HSV.rows; y++)
      {
        for (int x=0; x<extracted_HSV.cols; x++)
          {
            /* extract H, V value from pixel */
            double hue = Actual_Hue(extracted_HSV.at<Vec3b>(y, x)[0]);
            uchar  val = extracted_HSV.at<Vec3b>(y, x)[2];

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
      isRed_bright    = ( ((double)red_pixNum / valid_pixNum)    > 0.45) ? true : false; // detect red a little largely
      isYellow_bright = ( ((double)yellow_pixNum / valid_pixNum) > 0.5) ? true : false;
      isGreen_bright  = ( ((double)green_pixNum / valid_pixNum)  > 0.5) ? true : false;
    } else {
      isRed_bright    = false;
      isYellow_bright = false;
      isGreen_bright  = false;
    }

    int currentLightsCode = getCurrentLightsCode(isRed_bright, isYellow_bright, isGreen_bright);
    contexts.at(i).lightState = determineState(contexts.at(i).lightState, currentLightsCode, &(contexts.at(i).stateJudgeCount));

    roi.setTo(Scalar(0));
  }
}

double getBrightnessRatioInCircle(const Mat &input, const Point center, const int radius) {

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
void TrafficLightDetector::colorDetect(const Mat &input, Mat &output, const Rect coords, int Hmin, int Hmax) {

  if (input.channels() != 3) {
    return;
  }

  Mat hsv, thresholded;
  cvtColor(input, hsv, CV_RGB2HSV, 0);
  inRange(hsv, Scalar(Hmin,0,0), Scalar(Hmax,255,255), thresholded);

  cvtColor(thresholded, thresholded, CV_GRAY2RGB);
  thresholded.copyTo(output);

  rectangle(output, coords, MY_COLOR_RED);
}

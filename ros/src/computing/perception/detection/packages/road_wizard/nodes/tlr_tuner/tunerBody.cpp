#include "tunerBody.h"
#include "road_wizard/TunedResult.h"

static constexpr int32_t ADVERTISE_QUEUE_SIZE = 10;
static constexpr bool    ADVERTISE_LATCH      = true;

/* definition of class static private variables */
cv::Point       TunerBody::Clicked_point;
int             TunerBody::Signal_color;
cv::Mat         TunerBody::src_img;
cv::Mat         TunerBody::mask;
std::string     TunerBody::windowName;
thresholds_set  TunerBody::Red_set;
thresholds_set  TunerBody::Yellow_set;
thresholds_set  TunerBody::Green_set;
thresholds_set* TunerBody::Selected_set;
bool            TunerBody::updateImage;


/*============== Utility function ==============*/
static void onMouse(int event, int x, int y, int, void*)
{
  if (event != cv::EVENT_LBUTTONDOWN) {
    return;
  }

  TunerBody::setClickedPoint(cv::Point(x, y));

  return;
} /* void onMouse() */


/*============== Utility function ==============*/
static void colorTrack(const cv::Mat& hsv_img,
                       const int hue,
                       const int sat,
                       const int val,
                       const int hw,
                       const int sw,
                       const int vw,
                       cv::Mat *dst)
{
  cv::Mat kernel = cv::Mat::ones(5, 5, CV_8U); /* kernel for dilation and erosion*/

  /* create mask image */
  cv::inRange(hsv_img, cv::Scalar(hue - hw, sat - sw, val - vw), cv::Scalar(hue + hw, sat + sw, val + vw), *dst);

  /* remove noise */
  dilate(*dst, *dst, kernel, cv::Point(-1, -1), 2);
  erode(*dst, *dst, kernel, cv::Point(-1, -1), 2);

} /* void colorTrack() */


/*============== Utility function ==============*/
static int index_max(std::vector<std::vector<cv::Point> > cnt)
{
  unsigned int max_elementNum = 0;
  int maxIdx = -1;

  for (unsigned int i=0; i<cnt.size(); i++)
    {
      unsigned int elementNum = cnt[i].size();
      if (elementNum > max_elementNum) {
        max_elementNum = elementNum;
        maxIdx = i;
      }
    }

  return maxIdx;

} /* int index_max() */


TunerBody::TunerBody()
{
  /* initialize private values */
  Clicked_point = cv::Point(-1, -1);
  Signal_color  = GREEN;
  H_slider_val  = 0;
  S_slider_val  = 0;
  V_slider_val  = 0;
  windowName    = "Traffic Lignt Detector Tuner";

  Red_set.hue.center = 0;
  Red_set.hue.range  = 0;
  Red_set.sat.center = 0;
  Red_set.sat.range  = 0;
  Red_set.val.center = 0;
  Red_set.val.range  = 0;
  Red_set.isUpdated  = false;
  Yellow_set.hue.center = 0;
  Yellow_set.hue.range  = 0;
  Yellow_set.sat.center = 0;
  Yellow_set.sat.range  = 0;
  Yellow_set.val.center = 0;
  Yellow_set.val.range  = 0;
  Yellow_set.isUpdated  = false;
  Green_set.hue.center = 0;
  Green_set.hue.range  = 0;
  Green_set.sat.center = 0;
  Green_set.sat.range  = 0;
  Green_set.val.center = 0;
  Green_set.val.range  = 0;
  Green_set.isUpdated  = false;

  Selected_set = &Green_set;

  updateImage = true;

  /* create track bars */
  cv::namedWindow(windowName);
  cv::createTrackbar("H", windowName, &H_slider_val, 127, NULL, NULL);
  cv::createTrackbar("S", windowName, &S_slider_val, 127, NULL, NULL);
  cv::createTrackbar("V", windowName, &V_slider_val, 127, NULL, NULL);

} /* TunerBody::TunerBody() */


TunerBody::~TunerBody()
{
  cv::destroyAllWindows();
} /* TunerBody::~TunerBody() */


void TunerBody::image_raw_callBack(const sensor_msgs::Image& image_msg)
{
  cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
  if (updateImage) {
    src_img       = cv_image->image.clone();
    updateImage   = false;
    Clicked_point = cv::Point(-1, -1); // if image is reloaded, reset clicked point to out of image
  }
} /* TunerBody::image_raw_callBack() */


void TunerBody::launch(void)
{
  ros::NodeHandle n;

  ros::Subscriber image_sub = n.subscribe("/image_raw", 1, image_raw_callBack);

  ros::Publisher tunedResult_pub = n.advertise <road_wizard::TunedResult> ("tuned_result", ADVERTISE_QUEUE_SIZE, ADVERTISE_LATCH);

  /* valiables to check status change */
  cv::Point prev_clicked = cv::Point(-1, -1);
  int       prev_hw      = 0;
  int       prev_sw      = 0;
  int       prev_vw      = 0;

  while (ros::ok())
    {
      ros::spinOnce();

      if (src_img.empty())
        continue;

      base = cv::Mat::zeros(src_img.rows, src_img.cols * 2, CV_8UC3);
      mask = cv::Mat::zeros(src_img.rows, src_img.cols, CV_8UC1);

      cv::Mat result = src_img.clone();

      /* get clicked coordinates on the image */
      cv::Point targetPoint = Clicked_point;


      /* get current slider position */
      int hw = H_slider_val;
      int sw = S_slider_val;
      int vw = V_slider_val;

      cv::setMouseCallback(windowName, onMouse);

      /* convert input image to HSV color image */
      cv::Mat hsv_img;
      cv::cvtColor(src_img, hsv_img, cv::COLOR_BGR2HSV);

      if (targetPoint != prev_clicked &&
          targetPoint != cv::Point(-1, -1))
        {
          std::cerr << "Selected_set updated" << std::endl;
          int hue = (int)hsv_img.at<cv::Vec3b>(targetPoint.y, targetPoint.x)[0];
          int sat = (int)hsv_img.at<cv::Vec3b>(targetPoint.y, targetPoint.x)[1];
          int val = (int)hsv_img.at<cv::Vec3b>(targetPoint.y, targetPoint.x)[2];

          /* save HSV values into correspond variables */
          Selected_set->hue.center = hue;
          Selected_set->sat.center = sat;
          Selected_set->val.center = val;
          Selected_set->isUpdated  = true;
        }

      Selected_set->hue.range  = hw;
      Selected_set->sat.range  = sw;
      Selected_set->val.range  = vw;

      if (prev_hw != hw || prev_sw != sw || prev_vw != vw) {
        Selected_set->isUpdated = true;
      }

      colorTrack(hsv_img,
                 Selected_set->hue.center,
                 Selected_set->sat.center,
                 Selected_set->val.center,
                 Selected_set->hue.range,
                 Selected_set->sat.range,
                 Selected_set->val.range,
                 &mask);

      std::vector< std::vector<cv::Point> > contours;
      std::vector<cv::Vec4i> hierarchy;

      cv::Mat mask_clone = mask.clone();
      cv::findContours(mask_clone,
                       contours,
                       hierarchy,
                       CV_RETR_TREE,
                       CV_CHAIN_APPROX_SIMPLE);

      int maxIndex = index_max(contours);

      std::vector<cv::Point> hull;
      if (maxIndex != -1)
        {
          convexHull(contours[maxIndex], hull); /*draw round detected area by line*/
          drawContours(result, std::vector< std::vector<cv::Point> >(1, hull), 0, CV_RGB(220, 30, 20), 3);
        }

      /* display result */
      cv::Mat roi_result = base(cv::Rect(0, 0, result.cols, result.rows));
      result.copyTo(roi_result);

      cv::Scalar mask_color;
      switch (Signal_color) {
      case GREEN:
        mask_color = CV_RGB(0, 255, 0);
        break;
      case YELLOW:
        mask_color = CV_RGB(255, 255, 0);
        break;
      case RED:
        mask_color = CV_RGB(255, 0, 0);
        break;
      }

      cv::Mat colorBack(mask.rows, mask.cols, CV_8UC3, mask_color);
      cv::Mat mask_colored;
      colorBack.copyTo(mask_colored, mask);

      cv::Mat roi_mask = base(cv::Rect(result.cols, 0, mask_colored.cols, mask_colored.rows));
      mask_colored.copyTo(roi_mask);

      cv::imshow(windowName, base);
      cv::waitKey(10);
      // if ( (cv::waitKey(10)) == '\x1b') { /* if 'ESC' key is typed, finish the program */
      //   break;
      // }

      /* save current status */
      prev_clicked = targetPoint;
      prev_hw = hw;
      prev_sw = sw;
      prev_vw = vw;

      /* publish tuned result */
      road_wizard::TunedResult res;
      res.Red.Hue.center = Red_set.hue.center;
      res.Red.Hue.range  = Red_set.hue.range;
      res.Red.Sat.center = Red_set.sat.center;
      res.Red.Sat.range  = Red_set.sat.range;
      res.Red.Val.center = Red_set.val.center;
      res.Red.Val.range  = Red_set.val.range;

      res.Yellow.Hue.center = Yellow_set.hue.center;
      res.Yellow.Hue.range  = Yellow_set.hue.range;
      res.Yellow.Sat.center = Yellow_set.sat.center;
      res.Yellow.Sat.range  = Yellow_set.sat.range;
      res.Yellow.Val.center = Yellow_set.val.center;
      res.Yellow.Val.range  = Yellow_set.val.range;

      res.Green.Hue.center = Green_set.hue.center;
      res.Green.Hue.range  = Green_set.hue.range;
      res.Green.Sat.center = Green_set.sat.center;
      res.Green.Sat.range  = Green_set.sat.range;
      res.Green.Val.center = Green_set.val.center;
      res.Green.Val.range  = Green_set.val.range;

      tunedResult_pub.publish(res);

    }

  cv::destroyAllWindows();

} /* void TunerBody::launch() */


void TunerBody::setColor(signal_state state)
{
  switch (state) {
  case GREEN:
    Selected_set = &Green_set;
    break;
  case YELLOW:
    Selected_set = &Yellow_set;
    break;
  case RED:
    Selected_set = &Red_set;
    break;
  }

  Signal_color = state;

  /* update trackbar position according to current status */
  cv::setTrackbarPos("H", windowName, Selected_set->hue.range);
  cv::setTrackbarPos("S", windowName, Selected_set->sat.range);
  cv::setTrackbarPos("V", windowName, Selected_set->val.range);

  /* update mask image according to current status */
  cv::Mat hsv_img;
  cv::cvtColor(src_img, hsv_img, cv::COLOR_BGR2HSV);
  colorTrack(hsv_img,
             Selected_set->hue.center,
             Selected_set->sat.center,
             Selected_set->val.center,
             Selected_set->hue.range,
             Selected_set->sat.range,
             Selected_set->val.range,
             &mask);

} /* void TunerBody::setColor() */


void TunerBody::setClickedPoint(cv::Point pt)
{
  Clicked_point = pt;
} /* void TunerBody::setClickedPoint() */



void TunerBody::saveResult(std::string fileName)
{

  if (Red_set.isUpdated == false) {
    /*
      ========== Red : Default values ==========
      340               < Hue < 50 (circled)
      DEFAULT_SAT_LOWER < Sat < DEFAULT_SAT_UPPER
      DEFAULT_VAL_LOWER < Val < DEFAULT_VAL_UPPER
    */
    Red_set.hue.center = ( (((360 + 50) - 340 ) / 2 ) + 340) % 360;
    Red_set.hue.range  = (((360 + 50) - 340 ) / 2 );
    Red_set.sat.center = ((DEFAULT_SAT_UPPER - DEFAULT_SAT_LOWER) / 2 ) + DEFAULT_SAT_LOWER;
    Red_set.sat.range  = (DEFAULT_SAT_UPPER - DEFAULT_SAT_LOWER) / 2;
    Red_set.val.center = ((DEFAULT_VAL_UPPER - DEFAULT_VAL_LOWER) / 2 ) + DEFAULT_VAL_LOWER;
    Red_set.val.range  = (DEFAULT_VAL_UPPER - DEFAULT_VAL_LOWER) / 2;
    std::cout << "Red is default setting" << std::endl;
  }

  if (Yellow_set.isUpdated == false) {
    /*
      ========== Yellow : Default values ==========
      50                < Hue < 70
      DEFAULT_SAT_LOWER < Sat < DEFAULT_SAT_UPPER
      DEFAULT_VAL_LOWER < Val < DEFAULT_VAL_UPPER
     */
    Yellow_set.hue.center = ( ((70 - 50 ) / 2 ) + 50) % 360;
    Yellow_set.hue.range  = ((70 - 50 ) / 2 );
    Yellow_set.sat.center = ((DEFAULT_SAT_UPPER - DEFAULT_SAT_LOWER) / 2 ) + DEFAULT_SAT_LOWER;
    Yellow_set.sat.range  = (DEFAULT_SAT_UPPER - DEFAULT_SAT_LOWER) / 2;
    Yellow_set.val.center = ((DEFAULT_VAL_UPPER - DEFAULT_VAL_LOWER) / 2 ) + DEFAULT_VAL_LOWER;
    Yellow_set.val.range  = (DEFAULT_VAL_UPPER - DEFAULT_VAL_LOWER) / 2;
    std::cout << "Yellow is default setting" << std::endl;
  }

  if (Green_set.isUpdated == false) {
    /*
      ========== Green : Default values ==========
      80                < Hue < 190
      DEFAULT_SAT_LOWER < Sat < DEFAULT_SAT_UPPER
      DEFAULT_VAL_LOWER < Val < DEFAULT_VAL_UPPER
     */
    Green_set.hue.center = ( ((190 - 80 ) / 2 ) + 80) % 360;
    Green_set.hue.range  = ((190 - 80 ) / 2 );
    Green_set.sat.center = ((DEFAULT_SAT_UPPER - DEFAULT_SAT_LOWER) / 2 ) + DEFAULT_SAT_LOWER;
    Green_set.sat.range  = (DEFAULT_SAT_UPPER - DEFAULT_SAT_LOWER) / 2;
    Green_set.val.center = ((DEFAULT_VAL_UPPER - DEFAULT_VAL_LOWER) / 2 ) + DEFAULT_VAL_LOWER;
    Green_set.val.range  = (DEFAULT_VAL_UPPER - DEFAULT_VAL_LOWER) / 2;
    std::cout << "Green is default setting" << std::endl;
  }

  /* open file strage */
  cv::FileStorage cvfs(fileName, CV_STORAGE_WRITE);

  /* write data to file */
  {
    cv::WriteStructContext st_red(cvfs, "RED", CV_NODE_MAP);
    {
      cv::WriteStructContext st_hue(cvfs, "Hue", CV_NODE_MAP);
      cv::write(cvfs, "center", Red_set.hue.center);
      cv::write(cvfs, "range", Red_set.hue.range);
    }

    {
      cv::WriteStructContext st_hue(cvfs, "Saturation", CV_NODE_MAP);
      cv::write(cvfs, "center", Red_set.sat.center);
      cv::write(cvfs, "range", Red_set.sat.range);
    }

    {
      cv::WriteStructContext st_hue(cvfs, "Value", CV_NODE_MAP);
      cv::write(cvfs, "center", Red_set.val.center);
      cv::write(cvfs, "range", Red_set.val.range);
    }
  }

  {
    cv::WriteStructContext st_yellow(cvfs, "YELLOW", CV_NODE_MAP);
    {
      cv::WriteStructContext st_hue(cvfs, "Hue", CV_NODE_MAP);
      cv::write(cvfs, "center", Yellow_set.hue.center);
      cv::write(cvfs, "range", Yellow_set.hue.range);
    }

    {
      cv::WriteStructContext st_hue(cvfs, "Saturation", CV_NODE_MAP);
      cv::write(cvfs, "center", Yellow_set.sat.center);
      cv::write(cvfs, "range", Yellow_set.sat.range);
    }

    {
      cv::WriteStructContext st_hue(cvfs, "Value", CV_NODE_MAP);
      cv::write(cvfs, "center", Yellow_set.val.center);
      cv::write(cvfs, "range", Yellow_set.val.range);
    }
  }

  {
    cv::WriteStructContext st_green(cvfs, "GREEN", CV_NODE_MAP);
    {
      cv::WriteStructContext st_hue(cvfs, "Hue", CV_NODE_MAP);
      cv::write(cvfs, "center", Green_set.hue.center);
      cv::write(cvfs, "range", Green_set.hue.range);
    }

    {
      cv::WriteStructContext st_hue(cvfs, "Saturation", CV_NODE_MAP);
      cv::write(cvfs, "center", Green_set.sat.center);
      cv::write(cvfs, "range", Green_set.sat.range);
    }

    {
      cv::WriteStructContext st_hue(cvfs, "Value", CV_NODE_MAP);
      cv::write(cvfs, "center", Green_set.val.center);
      cv::write(cvfs, "range", Green_set.val.range);
    }
  }

} /* void TunerBody::saveResult() */


void TunerBody::openSetting(std::string fileName)
{
  /* open file strage */
  cv::FileStorage cvfs(fileName, CV_STORAGE_READ);

  /* read data from file */
  cv::FileNode topNode(cvfs.fs, NULL);
  {
    cv::FileNode nd_red = topNode[std::string("RED")];
    {
      cv::FileNode nd_hue = nd_red["Hue"];
      Red_set.hue.center  = nd_hue["center"];
      Red_set.hue.range   = nd_hue["range"];
    }

    {
      cv::FileNode nd_sat = nd_red["Saturation"];
      Red_set.sat.center  = nd_sat["center"];
      Red_set.sat.range   = nd_sat["range"];
    }

    {
      cv::FileNode nd_val = nd_red["Value"];
      Red_set.val.center  = nd_val["center"];
      Red_set.val.range   = nd_val["range"];
    }

    Red_set.isUpdated = true;
  }

  {
    cv::FileNode nd_yellow = topNode[std::string("YELLOW")];
    {
      cv::FileNode nd_hue   = nd_yellow["Hue"];
      Yellow_set.hue.center = nd_hue["center"];
      Yellow_set.hue.range  = nd_hue["range"];
    }

    {
      cv::FileNode nd_sat   = nd_yellow["Saturation"];
      Yellow_set.sat.center = nd_sat["center"];
      Yellow_set.sat.range  = nd_sat["range"];
    }

    {
      cv::FileNode nd_val   = nd_yellow["Value"];
      Yellow_set.val.center = nd_val["center"];
      Yellow_set.val.range  = nd_val["range"];
    }

    Yellow_set.isUpdated = true;
  }

  {
    cv::FileNode nd_green = topNode[std::string("GREEN")];
    {
      cv::FileNode nd_hue  = nd_green["Hue"];
      Green_set.hue.center = nd_hue["center"];
      Green_set.hue.range  = nd_hue["range"];
    }

    {
      cv::FileNode nd_sat  = nd_green["Saturation"];
      Green_set.sat.center = nd_sat["center"];
      Green_set.sat.range  = nd_sat["range"];
    }

    {
      cv::FileNode nd_val  = nd_green["Value"];
      Green_set.val.center = nd_val["center"];
      Green_set.val.range  = nd_val["range"];
    }

    Green_set.isUpdated = true;
  }

  /* set trackbar position to current status */
  cv::setTrackbarPos("H", windowName, Selected_set->hue.range);
  cv::setTrackbarPos("S", windowName, Selected_set->sat.range);
  cv::setTrackbarPos("V", windowName, Selected_set->val.range);

  /* sat mask image to current status */
  cv::Mat hsv_img;
  cv::cvtColor(src_img, hsv_img, cv::COLOR_BGR2HSV);
  colorTrack(hsv_img,
             Selected_set->hue.center,
             Selected_set->sat.center,
             Selected_set->val.center,
             Selected_set->hue.range,
             Selected_set->sat.range,
             Selected_set->val.range,
             &mask);

} /* void TunerBody::openSetting() */

void TunerBody::setUpdateImage(void)
{
  updateImage = true;
} /* void TunerBody::setUpdateImage() */

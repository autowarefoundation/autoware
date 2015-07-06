#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include "TrafficLight.h"
#include <float.h>
#include <math.h>
#include <sstream>
#include <runtime_manager/traffic_light.h>
#include <std_msgs/String.h>
#include "road_wizard/TunedResult.h"

thresholdSet thSet;

static ros::Publisher signalState_pub;
static ros::Publisher signalStateString_pub;
static constexpr int32_t ADVERTISE_QUEUE_SIZE = 10;
static constexpr bool    ADVERTISE_LATCH      = true;

using namespace cv;
// Variables
static TrafficLightDetector detector;

static Mat frame;


static double cvtInt2Double_hue(int center, int range)
{
  /* convert value range from OpenCV to Definition */
  double converted = (center + range) * 2.0f;

  if (converted < 0) {
    converted = 0.0f;
  } else if (360 < converted) {
    converted = converted - 360.0f;
  }

  return converted;
}


static double cvtInt2Double_sat(int center, int range)
{
  /* convert value range from OpenCV to Definition */
  double converted = (center + range) / 255.0f;
  if (converted < 0) {
    converted = 0.0f;
  } else if (1.0f < converted) {
    converted = 1.0f;
  }

  return converted;
}


static double cvtInt2Double_val(int center, int range)
{
  /* convert value range from OpenCV to Definition */
  double converted = (center + range) / 255.0f;
  if (converted < 0) {
    converted = 0;
  } else if (1.0f < converted) {
    converted = 1.0f;
  }

  return converted;
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
//      if (ctx.lampRadius < MINIMAM_RADIUS)
//        continue;

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

static void extractedPos_cb(const road_wizard::Signals::ConstPtr& extractedPos)
{
  if (frame.empty())
    return;

  setContexts(detector, extractedPos);

  /* test output */
  Mat targetScope;
  frame.copyTo(targetScope);
//   for (unsigned int i=0; i<detector.contexts.size(); i++)
//     {
// //      if (detector.contexts.at(i).lampRadius < MINIMAM_RADIUS)
// //        continue;

//       circle(targetScope, detector.contexts.at(i).redCenter, detector.contexts.at(i).lampRadius, CV_RGB(255, 0, 0), 1, 0);
//       circle(targetScope, detector.contexts.at(i).yellowCenter, detector.contexts.at(i).lampRadius, CV_RGB(255, 255, 0), 1, 0);
//       circle(targetScope, detector.contexts.at(i).greenCenter, detector.contexts.at(i).lampRadius, CV_RGB(0, 255, 0), 1, 0);
//     }


  // Mat grayScale;
  // cvtColor(frame, grayScale, CV_RGB2GRAY);
  // detector.brightnessDetect(grayScale);
  detector.brightnessDetect(frame);

  /* test output */
  putResult_inText(&targetScope, detector.contexts);

  // imshow("detection result", targetScope);
  // waitKey(5);

  /* publish result */
  runtime_manager::traffic_light state_msg;
  std_msgs::String state_string_msg;
  const int32_t TRAFFIC_LIGHT_RED     = 0;
  const int32_t TRAFFIC_LIGHT_GREEN   = 1;
  const int32_t TRAFFIC_LIGHT_UNKNOWN = 2;
  static int32_t prev_state = TRAFFIC_LIGHT_UNKNOWN;
  state_msg.traffic_light = TRAFFIC_LIGHT_UNKNOWN;
  for (unsigned int i=0; i<detector.contexts.size(); i++) {
	  switch (detector.contexts.at(i).lightState) {
	  case GREEN:
		  state_msg.traffic_light = TRAFFIC_LIGHT_GREEN;
          state_string_msg.data = "green signal";
		  break;
	  case YELLOW:
	  case RED:
		  state_msg.traffic_light = TRAFFIC_LIGHT_RED;
          state_string_msg.data = "red signal";
		  break;
	  case UNDEFINED:
		  state_msg.traffic_light = TRAFFIC_LIGHT_UNKNOWN;
          state_string_msg.data = "";
		  break;
	  }
	  if (state_msg.traffic_light != TRAFFIC_LIGHT_UNKNOWN)
		  break;  // publish the first state in detector.contexts
  }

  //signalState_pub.publish(state_msg);
  if (state_msg.traffic_light != prev_state) {
    signalStateString_pub.publish(state_string_msg);
  } else {
    state_string_msg.data = "";
    signalStateString_pub.publish(state_string_msg);
  }

  prev_state = state_msg.traffic_light;
}


static void tunedResult_cb(const road_wizard::TunedResult& msg)
{
  thSet.Red.Hue.upper = cvtInt2Double_hue(msg.Red.Hue.center, msg.Red.Hue.range);
  thSet.Red.Hue.lower = cvtInt2Double_hue(msg.Red.Hue.center, -msg.Red.Hue.range);
  thSet.Red.Sat.upper = cvtInt2Double_sat(msg.Red.Sat.center, msg.Red.Sat.range);
  thSet.Red.Sat.lower = cvtInt2Double_sat(msg.Red.Sat.center, -msg.Red.Sat.range);
  thSet.Red.Val.upper = cvtInt2Double_val(msg.Red.Val.center, msg.Red.Val.range);
  thSet.Red.Val.lower = cvtInt2Double_val(msg.Red.Val.center, -msg.Red.Val.range);

  thSet.Yellow.Hue.upper = cvtInt2Double_hue(msg.Yellow.Hue.center, msg.Yellow.Hue.range);
  thSet.Yellow.Hue.lower = cvtInt2Double_hue(msg.Yellow.Hue.center, -msg.Yellow.Hue.range);
  thSet.Yellow.Sat.upper = cvtInt2Double_sat(msg.Yellow.Sat.center, msg.Yellow.Sat.range);
  thSet.Yellow.Sat.lower = cvtInt2Double_sat(msg.Yellow.Sat.center, -msg.Yellow.Sat.range);
  thSet.Yellow.Val.upper = cvtInt2Double_val(msg.Yellow.Val.center, msg.Yellow.Val.range);
  thSet.Yellow.Val.lower = cvtInt2Double_val(msg.Yellow.Val.center, -msg.Yellow.Val.range);

  thSet.Green.Hue.upper = cvtInt2Double_hue(msg.Green.Hue.center, msg.Green.Hue.range);
  thSet.Green.Hue.lower = cvtInt2Double_hue(msg.Green.Hue.center, -msg.Green.Hue.range);
  thSet.Green.Sat.upper = cvtInt2Double_sat(msg.Green.Sat.center, msg.Green.Sat.range);
  thSet.Green.Sat.lower = cvtInt2Double_sat(msg.Green.Sat.center, -msg.Green.Sat.range);
  thSet.Green.Val.upper = cvtInt2Double_val(msg.Green.Val.center, msg.Green.Val.range);
  thSet.Green.Val.lower = cvtInt2Double_val(msg.Green.Val.center, -msg.Green.Val.range);

}


int main(int argc, char* argv[]) {

  //	printf("***** Traffic lights app *****\n");

  thSet.Red.Hue.upper = (double)DAYTIME_RED_UPPER;
  thSet.Red.Hue.lower = (double)DAYTIME_RED_LOWER;
  thSet.Red.Sat.upper = 1.0f;
  thSet.Red.Sat.lower = DAYTIME_S_SIGNAL_THRESHOLD;
  thSet.Red.Val.upper = 1.0f;
  thSet.Red.Val.lower = DAYTIME_V_SIGNAL_THRESHOLD;

  thSet.Yellow.Hue.upper = (double)DAYTIME_YELLOW_UPPER;
  thSet.Yellow.Hue.lower = (double)DAYTIME_YELLOW_LOWER;
  thSet.Yellow.Sat.upper = 1.0f;
  thSet.Yellow.Sat.lower = DAYTIME_S_SIGNAL_THRESHOLD;
  thSet.Yellow.Val.upper = 1.0f;
  thSet.Yellow.Val.lower = DAYTIME_V_SIGNAL_THRESHOLD;

  thSet.Green.Hue.upper = (double)DAYTIME_GREEN_UPPER;
  thSet.Green.Hue.lower = (double)DAYTIME_GREEN_LOWER;
  thSet.Green.Sat.upper = 1.0f;
  thSet.Green.Sat.lower = DAYTIME_S_SIGNAL_THRESHOLD;
  thSet.Green.Val.upper = 1.0f;
  thSet.Green.Val.lower = DAYTIME_V_SIGNAL_THRESHOLD;


  ros::init(argc, argv, "region_tlr");

  ros::NodeHandle n;

  ros::Subscriber image_sub = n.subscribe("/image_raw", 1, image_raw_cb);
  ros::Subscriber position_sub = n.subscribe("/traffic_light_pixel_xy", 1, extractedPos_cb);
  ros::Subscriber tunedResult_sub = n.subscribe("/tuned_result", 1, tunedResult_cb);

  signalState_pub = n.advertise<runtime_manager::traffic_light>("/traffic_light", ADVERTISE_QUEUE_SIZE, ADVERTISE_LATCH);
  signalStateString_pub = n.advertise<std_msgs::String>("/sound_player", ADVERTISE_QUEUE_SIZE);

  ros::spin();

  return 0;
}

/*
  define magnitude relationship of context
 */
static bool compareContext(const Context left, const Context right)
{
  /* if lampRadius is bigger, context is smaller */
  return left.lampRadius >= right.lampRadius;
}

void setContexts(TrafficLightDetector &detector,
                 const road_wizard::Signals::ConstPtr& extractedPos)
{

  /* copy parts of data to local variable */
  std::vector<road_wizard::ExtractedPosition> signals;
  std::vector<road_wizard::ExtractedPosition>::iterator sig_iterator;
  for (unsigned int i=0; i<extractedPos->Signals.size(); i++ )
    {
      road_wizard::ExtractedPosition tmp;
      tmp.signalId = extractedPos->Signals.at(i).signalId;
      tmp.u        = extractedPos->Signals.at(i).u;
      tmp.v        = extractedPos->Signals.at(i).v;
      tmp.radius   = extractedPos->Signals.at(i).radius;
      tmp.type     = extractedPos->Signals.at(i).type;
      tmp.linkId   = extractedPos->Signals.at(i).linkId;
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

  std::vector<Context> updatedSignals;

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
              0 < img_x - radius - 1.5 * min_radius && img_x + radius + 1.5 * min_radius < frame.cols &&
              0 < img_y - radius - 1.5 * min_radius && img_y + radius + 1.5 * min_radius < frame.rows)
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
                ctx.signalID     = sig_iterator->signalId; // use yellow light signalID as this context's representative
                break;
              }
              min_radius    = (min_radius > radius) ? radius : min_radius;
              most_left     = (most_left > img_x - radius -   1.5 * min_radius)  ? img_x - radius - 1.5 * min_radius : most_left;
              most_top      = (most_top > img_y - radius -    1.5 * min_radius)  ? img_y - radius - 1.5 * min_radius : most_top;
              most_right    = (most_right < img_x + radius +  1.5 * min_radius)  ? img_x + radius + 1.5 * min_radius : most_right;
              most_bottom   = (most_bottom < img_y + radius + 1.5 * min_radius)  ? img_y + radius + 1.5 * min_radius : most_bottom;
            }
        }

      ctx.lampRadius = (int)(min_radius / 1.5);
      ctx.topLeft    = Point(most_left, most_top);
      ctx.botRight   = Point(most_right, most_bottom);
      ctx.lightState = UNDEFINED;
      ctx.stateJudgeCount = 0;

      /* search whether this signal has already belonged in detector.contexts */
      bool isInserted = false;
      std::vector<int> eraseCandidate;
      for (unsigned int i=0; i<detector.contexts.size(); i++) {
        if (ctx.signalID == detector.contexts.at(i).signalID)
          {
            /* update to new information except to lightState */
            updatedSignals.push_back(ctx);
            updatedSignals.back().lightState      = detector.contexts.at(i).lightState;
            updatedSignals.back().stateJudgeCount = detector.contexts.at(i).stateJudgeCount;
            isInserted = true;
            break;
          }

      }

      if (isInserted == false)
        updatedSignals.push_back(ctx); // this ctx is new in detector.contexts

    }

  /* reset detector.contexts */
  detector.contexts.clear();
  detector.contexts.resize(updatedSignals.size());
  std::sort(updatedSignals.begin(), updatedSignals.end(), compareContext); // sort by lampRadius
  for (unsigned int i=0; i<updatedSignals.size(); i++) {
    detector.contexts.at(i) = updatedSignals.at(i);
  }
}

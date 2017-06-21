#include <vector>
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
#include "road_wizard/Signals.h"
#include "road_wizard/TunedResult.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Bool.h>

thresholdSet thSet;

static ros::Publisher signalState_pub;
static ros::Publisher signalStateString_pub;
static ros::Publisher marker_pub;
static ros::Publisher superimpose_image_pub;
static constexpr int32_t ADVERTISE_QUEUE_SIZE = 10;
static constexpr bool    ADVERTISE_LATCH      = true;
static uint32_t          shape                = visualization_msgs::Marker::SPHERE;

// Variables
static TrafficLightDetector detector;

static cv::Mat frame;

static bool              show_superimpose_result = false;
static const std::string window_name             = "superimpose result";

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
} /* static double cvtInt2Double_hue() */


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
} /* static double cvtInt2Double_sat() */


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
} /* static double cvtInt2Double_val() */


static void putResult_inText(cv::Mat *image, const std::vector<Context> &contexts)
{
  std::string label;
  const int fontFace = cv::FONT_HERSHEY_COMPLEX_SMALL;
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

      cv::getTextSize(label,
		      fontFace,
		      fontScale,
		      fontThickness,
		      &baseline);

      textOrg = cv::Point(ctx.topLeft.x, ctx.botRight.y + baseline);

      putText(*image,
              label,
              textOrg,
              fontFace,
              fontScale,
              textColor,
              fontThickness,
              CV_AA);
    }
} /* static void putResult_inText() */


static void image_raw_cb(const sensor_msgs::Image& image_source)
{
  cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image_source, sensor_msgs::image_encodings::BGR8);
  //  cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image_source);
  frame = cv_image->image.clone();

  /* Draw superimpose result on image */
  cv::Mat targetScope = frame.clone();
  for (unsigned int i=0; i<detector.contexts.size(); i++)
    {
      /* draw superimposed position of traffic lights */
      circle(targetScope, detector.contexts.at(i).redCenter, detector.contexts.at(i).lampRadius, CV_RGB(255, 0, 0), 1, 0);
      circle(targetScope, detector.contexts.at(i).yellowCenter, detector.contexts.at(i).lampRadius, CV_RGB(255, 255, 0), 1, 0);
      circle(targetScope, detector.contexts.at(i).greenCenter, detector.contexts.at(i).lampRadius, CV_RGB(0, 255, 0), 1, 0);
    }

  /* draw detection results */
  putResult_inText(&targetScope, detector.contexts);


  /* Publish superimpose result image */
  cv_bridge::CvImage msg_converter;
  msg_converter.header = image_source.header;
  msg_converter.encoding = sensor_msgs::image_encodings::BGR8;
  msg_converter.image = targetScope;
  superimpose_image_pub.publish(msg_converter.toImageMsg());

  /* Display superimpose result image in separate window*/
  if (show_superimpose_result)
    {
      if (cvGetWindowHandle(window_name.c_str()) != NULL) // Guard not to write destroyed window by using close button on the window
        {
          imshow(window_name, targetScope);
          cv::waitKey(5);
        }
    }

} /* static void image_raw_cb() */


static void extractedPos_cb(const road_wizard::Signals::ConstPtr& extractedPos)
{
  if (frame.empty())
    return;

  /* Set subscribed signal position into detector */
  Context::SetContexts(detector.contexts, extractedPos, frame.rows, frame.cols);

  detector.brightnessDetect(frame);

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

  if (state_msg.traffic_light != prev_state) {
    signalState_pub.publish(state_msg);
    signalStateString_pub.publish(state_string_msg);
  } else {
    state_string_msg.data = "";
    signalStateString_pub.publish(state_string_msg);
  }

  std_msgs::ColorRGBA color_black;
  color_black.r = 0.0f;
  color_black.g = 0.0f;
  color_black.b = 0.0f;
  color_black.a = 1.0f;

  std_msgs::ColorRGBA color_red;
  color_red.r = 1.0f;
  color_red.g = 0.0f;
  color_red.b = 0.0f;
  color_red.a = 1.0f;

  std_msgs::ColorRGBA color_yellow;
  color_yellow.r = 1.0f;
  color_yellow.g = 1.0f;
  color_yellow.b = 0.0f;
  color_yellow.a = 1.0f;

  std_msgs::ColorRGBA color_green;
  color_green.r = 0.0f;
  color_green.g = 1.0f;
  color_green.b = 0.0f;
  color_green.a = 1.0f;

  /* publish all detected result as ROS Marker */
  for (unsigned int i=0; i<detector.contexts.size(); i++)
    {
      Context ctx = detector.contexts.at(i);
      visualization_msgs::MarkerArray signalSet;
      visualization_msgs::Marker mk_red, mk_yellow, mk_green;

      /* Set the frame ID */
      mk_red.header.frame_id    = "map";
      mk_yellow.header.frame_id = "map";
      mk_green.header.frame_id  = "map";

      /* Set the namespace and id for this marker */
      mk_red.ns    = "tlr_result_red";
      mk_yellow.ns = "tlr_result_yellow";
      mk_green.ns  = "tlr_result_green";
      mk_red.id    = ctx.signalID;
      mk_yellow.id = ctx.signalID;
      mk_green.id  = ctx.signalID;

      /* Set the marker type */
      mk_red.type    = shape;
      mk_yellow.type = shape;
      mk_green.type  = shape;

      /* Set the pose of the marker */
      mk_red.pose.position.x    = ctx.redCenter3d.x;
      mk_red.pose.position.y    = ctx.redCenter3d.y;
      mk_red.pose.position.z    = ctx.redCenter3d.z;
      mk_yellow.pose.position.x = ctx.yellowCenter3d.x;
      mk_yellow.pose.position.y = ctx.yellowCenter3d.y;
      mk_yellow.pose.position.z = ctx.yellowCenter3d.z;
      mk_green.pose.position.x  = ctx.greenCenter3d.x;
      mk_green.pose.position.y  = ctx.greenCenter3d.y;
      mk_green.pose.position.z  = ctx.greenCenter3d.z;

      mk_red.pose.orientation.x    = 0.0;
      mk_red.pose.orientation.y    = 0.0;
      mk_red.pose.orientation.y    = 0.0;
      mk_red.pose.orientation.w    = 0.0;
      mk_yellow.pose.orientation.x = 0.0;
      mk_yellow.pose.orientation.y = 0.0;
      mk_yellow.pose.orientation.y = 0.0;
      mk_yellow.pose.orientation.w = 0.0;
      mk_green.pose.orientation.x  = 0.0;
      mk_green.pose.orientation.y  = 0.0;
      mk_green.pose.orientation.y  = 0.0;
      mk_green.pose.orientation.w  = 0.0;

      /* Set the scale of the marker -- We assume lamp radius as 30cm */
      mk_red.scale.x    = (double)0.3;
      mk_red.scale.y    = (double)0.3;
      mk_red.scale.z    = (double)0.3;
      mk_yellow.scale.x = (double)0.3;
      mk_yellow.scale.y = (double)0.3;
      mk_yellow.scale.z = (double)0.3;
      mk_green.scale.x  = (double)0.3;
      mk_green.scale.y  = (double)0.3;
      mk_green.scale.z  = (double)0.3;

      /* Set the color */
      switch (ctx.lightState) {
      case GREEN:
        mk_red.color    = color_black;
        mk_yellow.color = color_black;
        mk_green.color  = color_green;
        break;
      case YELLOW:
        mk_red.color    = color_black;
        mk_yellow.color = color_yellow;
        mk_green.color  = color_black;
        break;
      case RED:
        mk_red.color    = color_red;
        mk_yellow.color = color_black;
        mk_green.color  = color_black;
        break;
      case UNDEFINED:
        mk_red.color    = color_black;
        mk_yellow.color = color_black;
        mk_green.color  = color_black;
        break;
      }

      mk_red.lifetime    = ros::Duration(0.1);
      mk_yellow.lifetime = ros::Duration(0.1);
      mk_green.lifetime  = ros::Duration(0.1);

      signalSet.markers.push_back(mk_red);
      signalSet.markers.push_back(mk_yellow);
      signalSet.markers.push_back(mk_green);

      marker_pub.publish(signalSet);
    }

  prev_state = state_msg.traffic_light;
} /* static void extractedPos_cb() */


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

} /* static void tunedResult_cb() */


static void superimpose_cb(const std_msgs::Bool::ConstPtr& config_msg)
{
  show_superimpose_result = config_msg->data;

  if (show_superimpose_result) {
    cv::namedWindow(window_name, cv::WINDOW_NORMAL);
    cv::startWindowThread();
  }

  if (!show_superimpose_result) {
    cv::destroyWindow(window_name);
    cv::waitKey(1);
  }

} /* static void superimpose_cb() */

int main(int argc, char* argv[]) {

  //	printf("***** Traffic lights app *****\n");
#ifdef SHOW_DEBUG_INFO
  cv::namedWindow("tmpImage", cv::WINDOW_NORMAL);
  cv::namedWindow("bright_mask", cv::WINDOW_NORMAL);
  cv::startWindowThread();
#endif

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
  ros::NodeHandle private_nh("~");
  std::string image_topic_name;
  private_nh.param<std::string>("image_raw_topic", image_topic_name, "/image_raw");

  ros::Subscriber image_sub       = n.subscribe(image_topic_name, 1, image_raw_cb);
  ros::Subscriber position_sub    = n.subscribe("/roi_signal", 1, extractedPos_cb);
  ros::Subscriber tunedResult_sub = n.subscribe("/tuned_result", 1, tunedResult_cb);
  ros::Subscriber superimpose_sub = n.subscribe("/config/superimpose", 1, superimpose_cb);

  signalState_pub       = n.advertise<runtime_manager::traffic_light>("/light_color", ADVERTISE_QUEUE_SIZE, ADVERTISE_LATCH);
  signalStateString_pub = n.advertise<std_msgs::String>("/sound_player", ADVERTISE_QUEUE_SIZE);
  marker_pub            = n.advertise<visualization_msgs::MarkerArray>("tlr_result", ADVERTISE_QUEUE_SIZE);
  superimpose_image_pub= n.advertise<sensor_msgs::Image>("tlr_superimpose_image", ADVERTISE_QUEUE_SIZE);

  ros::spin();

  return 0;
} /* int main() */



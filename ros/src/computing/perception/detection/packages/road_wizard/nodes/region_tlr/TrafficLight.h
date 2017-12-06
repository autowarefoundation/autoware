#ifndef TRAFFIC_LIGHT_H
#define TRAFFIC_LIGHT_H

/* External includes */
#include <cmath>

/* Internal includes */
#include "TrafficLightDetector.h"

/* Extra includes */
#include "autoware_msgs/Signals.h"

#define MAIN_WINDOW_NAME "Main"
#define SETTINGS_WINDOW_NAME "Settings"

#define TLR_GREEN_SIGNAL_STR "green signal"
#define TLR_RED_SIGNAL_STR "red signal"
#define TLR_UNKNOWN_SIGNAL_STR ""

/* Functions declarations */
void setContexts(TrafficLightDetector &detector, const autoware_msgs::Signals::ConstPtr& extractedPos);

#define MINIMAM_RADIUS 3
#define ROI_MARGINE 25

static inline bool IsNearlyZero(double x)
{
  double abs_x = fabs(x);
  int scale = 100;
  return(abs_x < DBL_MIN*scale);
}

struct valueSet {
    double upper;
    double lower;
};

struct hsvSet {
    valueSet Hue;
    valueSet Sat;
    valueSet Val;
};

struct thresholdSet {
    hsvSet Red;
    hsvSet Yellow;
    hsvSet Green;
};

//#define SHOW_DEBUG_INFO

#endif

#ifndef REGION_TLR_H
#define REGION_TLR_H

/* External includes */
#include <cmath>

/* Internal includes */
#include "TrafficLightDetector.h"

/* Functions declarations */
void setContexts(TrafficLightDetector &detector,
                 const autoware_msgs::Signals::ConstPtr &extractedPos);

#define MINIMAM_RADIUS 3
#define ROI_MARGINE 25

static inline bool IsNearlyZero(double x) {
  double abs_x = fabs(x);
  int scale = 100;
  return (abs_x < DBL_MIN * scale);
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

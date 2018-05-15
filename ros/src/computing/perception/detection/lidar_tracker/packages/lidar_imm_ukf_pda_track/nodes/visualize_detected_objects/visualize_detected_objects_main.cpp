
#include "visualize_detected_objects.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "visualize_detected_objects");
  VisualizeDetectedObjects app;
  ros::spin();

  return 0;
}

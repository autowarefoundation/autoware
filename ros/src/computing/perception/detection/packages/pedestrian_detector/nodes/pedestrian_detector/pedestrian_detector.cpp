#include "ros/ros.h"
#include "dpm/ImageObjects.h"

#define XSTR(x) #x
#define STR(x) XSTR(x)

extern int dpm_ocv_main(int argc, char* argv[]);
extern int dpm_ttic_main(int argc, char* argv[], const char *cubin);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pedestrian_detector");

  ros::NodeHandle n;

  if(n.hasParam("/pedestrian_detector/algorithm")){
    std::string algo("ocv");
    n.getParam("/pedestrian_detector/algorithm", algo);

    if(algo == "gpu"){
      std::string cubin(STR(DEFAULT_CUBIN));
      if (n.hasParam("/pedestrian_detector/cubin"))
        n.getParam("/pedestrian_detector/cubin", cubin);
      return dpm_ttic_main(argc, argv, cubin.c_str());
    }
  }
  return dpm_ocv_main(argc, argv);
}

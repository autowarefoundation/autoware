#include "ros/ros.h"
#include "image/ImageObjects.h"

int dpm_ocv_main(int argc, char* argv[]);
int dpm_ttic_main(int argc, char* argv[], const char *cubin);

#define XSTR(x) #x
#define STR(x) XSTR(x)

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_detector");

  ros::NodeHandle n;

  if(n.hasParam("/image_detector/algorithm")){
    std::string algo("ocv");
    n.getParam("/image_detector/algorithm", algo);
    if(algo == "gpu"){
      std::string cubin(STR(DEFAULT_CUBIN));
      n.getParam("/image_detector/cubin", cubin);
      return dpm_ttic_main(argc, argv, cubin.c_str());
    }
  }
  return dpm_ocv_main(argc, argv);
}

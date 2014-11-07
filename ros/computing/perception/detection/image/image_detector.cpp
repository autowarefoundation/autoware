#include "ros/ros.h"
#include "image/ImageObjects.h"

int dpm_ocv_main(int argc, char* argv[]);
int dpm_ttic_main(int argc, char* argv[], const char *cubin);

#define XSTR(x) #x
#define STR(x) XSTR(x)

#if defined(CAR_DETECTOR)
#define NODE_NAME "car_detector"
#define ALGORITHM_PARAM "/car_detector/algorithm"
#define CUBIN_PARAM "/car_detector/cubin"
#elif defined(PEDESTRIAN_DETECTOR)
#define NODE_NAME "pedestrian_detector"
#define ALGORITHM_PARAM "/pedestrian_detector/algorithm"
#define CUBIN_PARAM "/pedestrian_detector/cubin"
#else
#error Invalid detector type
#endif

int main(int argc, char **argv)
{
  ros::init(argc, argv, NODE_NAME);

  ros::NodeHandle n;

  if(n.hasParam(ALGORITHM_PARAM)){
    std::string algo("ocv");
    n.getParam(ALGORITHM_PARAM, algo);
    if(algo == "gpu"){
      std::string cubin(STR(DEFAULT_CUBIN));
      if (n.hasParam(CUBIN_PARAM))
        n.getParam(CUBIN_PARAM, cubin);
      return dpm_ttic_main(argc, argv, cubin.c_str());
    }
  }
  return dpm_ocv_main(argc, argv);
}

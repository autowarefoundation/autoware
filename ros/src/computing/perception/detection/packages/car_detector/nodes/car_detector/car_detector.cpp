#include <string>
#include "ros/ros.h"
#include "dpm/ImageObjects.h"

#define XSTR(x) #x
#define STR(x) XSTR(x)

extern int dpm_ocv_main(int argc, char* argv[],
			const std::string& detection_type);
extern int dpm_ttic_main(int argc, char* argv[], const char *cubin,
			 const std::string& detection_type);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "car_detector");

  ros::NodeHandle n;
  std::string detection_type("car");
  if(n.hasParam("/car_detector/algorithm")){
    std::string algo("ocv");
    n.getParam("/car_detector/algorithm", algo);

    if(algo == "gpu"){
      std::string cubin(STR(DEFAULT_CUBIN));
      if (n.hasParam("/car_detector/cubin"))
        n.getParam("/car_detector/cubin", cubin);
      return dpm_ttic_main(argc, argv, cubin.c_str(), detection_type);
    }
  }
  return dpm_ocv_main(argc, argv, detection_type);
}

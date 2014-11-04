#include "ros/ros.h"
#include "image/ImageObjects.h"

int dpm_ocv_main(int argc, char* argv[]);
int dpm_ttic_main(int argc, char* argv[]);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_detector");

  ros::NodeHandle n;

  if(n.hasParam("/image_detector/algorithm")){
    std::string algo("ocv");
    n.getParam("/image_detector/algorithm", algo);
    if(algo.compare("ttic") == 0){
      return dpm_ttic_main(argc, argv);
    }
  }
  return dpm_ocv_main(argc, argv);
}

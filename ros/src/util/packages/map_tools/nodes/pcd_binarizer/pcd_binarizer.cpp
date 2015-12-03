/*
PCD binarizing program.
Yuki Kitsukawa
*/

#include <iostream>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main (int argc, char** argv)
{
  int i;

  if(argc < 2){
    std::cout << "Usage: rosrun map_tools pcd_binarizer '***.pcd'" << std::endl;
    exit(-1);
  }

  for(i = 1; i < argc; i++){
    // Loading input_cloud.
    std::string input = argv[i];
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (input, *input_cloud) == -1){
      std::cout << "Couldn't find " << input << "." << std::endl;
      break;
    }
    std::cout << "Input: " << input << std::endl;

    int tmp = input.find_last_of("/");
    std::string prefix = "bin_";
    std::string output = input.insert(tmp+1, prefix);

    if(pcl::io::savePCDFileBinary(output, *input_cloud) == -1){
      std::cout << "Failed saving " << output << std::endl;
      return -1;
    }
    std::cout << "Output: " << output << std::endl << std::endl;
  }

  return 0;

}

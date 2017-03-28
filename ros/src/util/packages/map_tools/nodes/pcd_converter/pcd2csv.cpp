/*
Convert pcd to csv.
Yuki Kitsukawa
*/

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main (int argc, char** argv)
{
  int i;

  if(argc < 2){
    std::cout << "Usage: rosrun map_tools csv2pcd 'PointTyppe' '***.csv'" << std::endl;
    exit(-1);
  }

  std::string point_type = argv[1];
  std::cout << "PointType: " << point_type << std::endl;

  for(i = 2; i < argc; i++){

    std::string input = argv[i];
    std::string output = argv[i];
    std::string::size_type pos = output.find("pcd");
    output.replace(pos, 3, "csv");

    std::ofstream ofs(output, std::ios::app);

    // Read PCD.
    if(point_type == "PointXYZ")
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
      if(pcl::io::loadPCDFile<pcl::PointXYZ> (input, *input_cloud) == -1)
      {
        std::cout << "Couldn't read " << input << "." << std::endl;
      }

      for(size_t i = 0; i < input_cloud->points.size(); ++i){
        ofs << input_cloud->points[i].x << "," << input_cloud->points[i].y << "," << input_cloud->points[i].z << std::endl;
      }

      std::cout << "Input: " << input << " (" << input_cloud->points.size() << " points.)" << std::endl;
      std::cout << "Output: " << output << std::endl;
      std::cout << std::endl;

    }

    if(point_type == "PointXYZI")
    {
      pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZI>);
      if(pcl::io::loadPCDFile<pcl::PointXYZI> (input, *input_cloud) == -1)
      {
        std::cout << "Couldn't read " << input << "." << std::endl;
      }

      for(size_t i = 0; i < input_cloud->points.size(); ++i){
        ofs << input_cloud->points[i].x << "," << input_cloud->points[i].y << "," << input_cloud->points[i].z << "," << input_cloud->points[i].intensity << std::endl;
      }

      std::cout << "Input: " << input << " (" << input_cloud->points.size() << " points.)" << std::endl;
      std::cout << "Output: " << output << std::endl;
      std::cout << std::endl;

    }

    if(point_type == "PointXYZRGB")
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
      if(pcl::io::loadPCDFile<pcl::PointXYZRGB> (input, *input_cloud) == -1)
      {
        std::cout << "Couldn't read " << input << "." << std::endl;
      }

      for(size_t i = 0; i < input_cloud->points.size(); ++i){
        ofs << input_cloud->points[i].x << "," << input_cloud->points[i].y << "," << input_cloud->points[i].z << "," << input_cloud->points[i].rgb << std::endl;
      }

      std::cout << "Input: " << input << " (" << input_cloud->points.size() << " points.)" << std::endl;
      std::cout << "Output: " << output << std::endl;
      std::cout << std::endl;

    }

  }

  return 0;

}

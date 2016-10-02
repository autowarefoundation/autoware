/*
PCD downsampling program using VoxelGrid filter.
Yuki Kitsukawa
*/

#include <iostream>
#include <string>
#include <sstream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

int main (int argc, char** argv)
{
  int i;

  if(argc < 3){
    std::cout << "Usage: rosrun map_tools pcd_filter \"leaf_size\" \"***.pcd\" " << std::endl;
    return 1;
  }

  double leaf_size = std::stod(argv[1]);

  for(i = 2; i < argc; i++){
    // Loading input_cloud.
    std::string input = argv[i];
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (input, *input_cloud) == -1){
      std::cout << "Couldn't find " << input << "." << std::endl;
      break;
    }
    std::cout << "Input: " << input << " (" << input_cloud->size () << " points) " << std::endl;

    // Filtering input scan
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid_filter;
    voxel_grid_filter.setLeafSize (leaf_size, leaf_size, leaf_size);
    voxel_grid_filter.setInputCloud (input_cloud);
    voxel_grid_filter.filter (*filtered_cloud);
    //    std::cout << "Filtered cloud contains " << filtered_cloud->size () << " data points from " << input << "."<< std::endl;

    int tmp = input.find_last_of("/");
    std::string prefix = std::to_string(leaf_size);
    prefix += "_";
    std::string output = input.insert(tmp+1, prefix);

    pcl::io::savePCDFileBinary(output, *filtered_cloud);
    std::cout << "Output: " << output << " (" << filtered_cloud->size () << " points) " << std::endl;
    std::cout << "Voxel Leaf Size: " << leaf_size << std::endl << std::endl;
  }

  return 0;

}

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

template <class T>
pcl::PointXYZ getReferencePoint(const T input_cloud){
    pcl::PointXYZ ref_point;
    for(auto pt: input_cloud->points){
        //make sure the point is a valid point
        if(std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)){
            ref_point.x = pt.x;
            ref_point.y = pt.y;
            ref_point.z = pt.z;
            break;
        }
    }
    return ref_point;
}

template <class T>
void translatePointCloud(const T input_cloud, T& output_cloud,const pcl::PointXYZ origin){
    output_cloud->points.clear();
    for(auto in_pt: input_cloud->points){
        auto out_pt = in_pt;
        out_pt.x -= origin.x;
        out_pt.y -= origin.y;
        out_pt.z -= origin.z;
        output_cloud->push_back(out_pt);
    }
    output_cloud->height = input_cloud->height;
    output_cloud->width = input_cloud->width;
}


int main (int argc, char** argv)
{
  int i;

  if(argc < 3){
    std::cout << "Usage: rosrun map_tools pcd_filter \"point_type [PointXYZ|PointXYZI|PointXYZRGB]\" \"leaf_size\" \"***.pcd\" " << std::endl;
    return 1;
  }

  std::string point_type = argv[1];
  double leaf_size = std::stod(argv[2]);

  for(i = 3; i < argc; i++){
    // Loading input_cloud.
    std::string input = argv[i];

    if(point_type == "PointXYZ"){
      pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
      if (pcl::io::loadPCDFile<pcl::PointXYZ> (input, *input_cloud) == -1){
        std::cout << "Couldn't find " << input << "." << std::endl;
        break;
      }
      std::cout << "Input: " << input << " (" << input_cloud->size () << " points) " << std::endl;

      //translate pointcloud to local frame to avoid losing precision
      pcl::PointCloud<pcl::PointXYZ>::Ptr translated_input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointXYZ origin = getReferencePoint(input_cloud);
      translatePointCloud(input_cloud, translated_input_cloud, origin);

      // Filtering input scan
      pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
      voxel_grid_filter.setLeafSize (leaf_size, leaf_size, leaf_size);
      voxel_grid_filter.setInputCloud (translated_input_cloud);
      voxel_grid_filter.filter (*filtered_cloud);

      //translate pointcloud back to original frame
      pcl::PointCloud<pcl::PointXYZ>::Ptr translated_filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointXYZ inverse_origin;
      inverse_origin.x = -origin.x;
      inverse_origin.y = -origin.y;
      inverse_origin.z = -origin.z;
      translatePointCloud(filtered_cloud, translated_filtered_cloud, inverse_origin);

      int tmp = input.find_last_of("/");
      std::string prefix = std::to_string(leaf_size);
      prefix = prefix.substr(0, 4);
      prefix += "_";
      std::string output = input.insert(tmp+1, prefix);

      pcl::io::savePCDFileBinary(output, *translated_filtered_cloud);
      std::cout << "Output: " << output << " (" << translated_filtered_cloud->size () << " points) " << std::endl;
      std::cout << "Voxel Leaf Size: " << leaf_size << std::endl << std::endl;
    }

    else if(point_type == "PointXYZI"){
      pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZI>);
      if (pcl::io::loadPCDFile<pcl::PointXYZI> (input, *input_cloud) == -1){
        std::cout << "Couldn't find " << input << "." << std::endl;
        break;
      }
      std::cout << "Input: " << input << " (" << input_cloud->size () << " points) " << std::endl;

      //translate pointcloud to local frame to avoid losing precision
      pcl::PointCloud<pcl::PointXYZI>::Ptr translated_input_cloud (new pcl::PointCloud<pcl::PointXYZI>);
      pcl::PointXYZ origin = getReferencePoint(input_cloud);
      translatePointCloud(input_cloud, translated_input_cloud, origin);

      // Filtering input scan
      pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZI>);
      pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
      voxel_grid_filter.setLeafSize (leaf_size, leaf_size, leaf_size);
      voxel_grid_filter.setInputCloud (translated_input_cloud);
      voxel_grid_filter.filter (*filtered_cloud);

      //translate pointcloud back to original frame
      pcl::PointCloud<pcl::PointXYZI>::Ptr translated_filtered_cloud (new pcl::PointCloud<pcl::PointXYZI>);
      pcl::PointXYZ inverse_origin;
      inverse_origin.x = -origin.x;
      inverse_origin.y = -origin.y;
      inverse_origin.z = -origin.z;
      translatePointCloud(filtered_cloud, translated_filtered_cloud, inverse_origin);

      int tmp = input.find_last_of("/");
      std::string prefix = std::to_string(leaf_size);
      prefix = prefix.substr(0, 4);
      prefix += "_";
      std::string output = input.insert(tmp+1, prefix);

      pcl::io::savePCDFileBinary(output, *translated_filtered_cloud);
      std::cout << "Output: " << output << " (" << translated_filtered_cloud->size () << " points) " << std::endl;
      std::cout << "Voxel Leaf Size: " << leaf_size << std::endl << std::endl;
    }

    else if(point_type == "PointXYZRGB"){
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
      if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (input, *input_cloud) == -1){
        std::cout << "Couldn't find " << input << "." << std::endl;
        break;
      }
      std::cout << "Input: " << input << " (" << input_cloud->size () << " points) " << std::endl;

      //translate pointcloud to local frame to avoid losing precision
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr translated_input_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::PointXYZ origin = getReferencePoint(input_cloud);
      translatePointCloud(input_cloud, translated_input_cloud, origin);

      // Filtering input scan
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid_filter;
      voxel_grid_filter.setLeafSize (leaf_size, leaf_size, leaf_size);
      voxel_grid_filter.setInputCloud (translated_input_cloud);
      voxel_grid_filter.filter (*filtered_cloud);

      //translate pointcloud back to original frame
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr translated_filtered_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::PointXYZ inverse_origin;
      inverse_origin.x = -origin.x;
      inverse_origin.y = -origin.y;
      inverse_origin.z = -origin.z;
      translatePointCloud(filtered_cloud, translated_filtered_cloud, inverse_origin);

      int tmp = input.find_last_of("/");
      std::string prefix = std::to_string(leaf_size);
      prefix = prefix.substr(0, 4);
      prefix += "_";
      std::string output = input.insert(tmp+1, prefix);

      pcl::io::savePCDFileBinary(output, *translated_filtered_cloud);
      std::cout << "Output: " << output << " (" << translated_filtered_cloud->size () << " points) " << std::endl;
      std::cout << "Voxel Leaf Size: " << leaf_size << std::endl << std::endl;
    }
  }

  return 0;

}

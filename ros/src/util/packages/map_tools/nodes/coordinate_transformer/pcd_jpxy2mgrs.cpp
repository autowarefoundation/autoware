/*
 * The rights of this source code conform to
 * https://github.com/CPFL/Autoware/blob/master/LICENSE
 */

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>
#include <string>
#include "mgrs_converter.hpp"

struct pcd_xyz_grid
{
  std::string mgrs_code;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  unsigned int size;
};

struct pcd_xyzi_grid
{
  std::string mgrs_code;
  pcl::PointCloud<pcl::PointXYZI> cloud;
  unsigned int size;
};

struct pcd_xyzrgb_grid
{
  std::string mgrs_code;
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  unsigned int size;
};

void pcd_jpxy2mgrs(pcl::PointCloud<pcl::PointXYZ>::Ptr input, std::string path, const int plane_zone)
{
  map_tools::MgrsConverter converter;
  std::vector<pcd_xyz_grid> grids;

  for (auto &itr : *input)
  {
    std::string code;
    double easting, northing;
    double lat, lon, alt;
    converter.jpxy2latlon(static_cast<double>(itr.y), static_cast<double>(itr.x), static_cast<double>(itr.z),
                          plane_zone, lat, lon, alt);
    std::tie(code, easting, northing) = converter.latlon2mgrs(lat, lon);
    itr.x = static_cast<float>(easting);
    itr.y = static_cast<float>(northing);
    const pcl::PointXYZ &p = itr;

    bool push_flag = false;
    for (auto v_itr = grids.begin(); v_itr != grids.end(); ++v_itr)
    {
      if (code == v_itr->mgrs_code)
      {
        v_itr->cloud.points.push_back(p);
        ++v_itr->size;
        push_flag = true;
        break;
      }
    }
    // NEW mgrs_code
    pcl::PointCloud<pcl::PointXYZ> a;  // null for initialize
    if (!push_flag)
    {
      pcd_xyz_grid new_grid = { code, a, 0 };
      grids.push_back(new_grid);
      grids[grids.size() - 1].cloud.points.push_back(p);
      ++grids[grids.size() - 1].size;
    }
  }

  // save PCD to "MGRS_code_time.pcd"
  for (auto v_itr = grids.begin(); v_itr != grids.end(); ++v_itr)
  {
    std::string file_name = path + v_itr->mgrs_code + "_" + std::to_string(std::time(NULL)) + ".pcd";
    v_itr->cloud.header = input->header;
    v_itr->cloud.width = v_itr->size;
    v_itr->cloud.height = 1;
    pcl::io::savePCDFileASCII(file_name, v_itr->cloud);
    std::cout << "Saved " << v_itr->cloud.points.size() << " data points to " << file_name << std::endl;
  }
}

void pcd_jpxy2mgrs(pcl::PointCloud<pcl::PointXYZI>::Ptr input, std::string path, const int plane_zone)
{
  map_tools::MgrsConverter converter;
  std::vector<pcd_xyzi_grid> grids;

  for (auto &itr : *input)
  {
    std::string code;
    double easting, northing;
    double lat, lon, alt;
    converter.jpxy2latlon(static_cast<double>(itr.y), static_cast<double>(itr.x), static_cast<double>(itr.z),
                          plane_zone, lat, lon, alt);
    std::tie(code, easting, northing) = converter.latlon2mgrs(lat, lon);
    itr.x = static_cast<float>(easting);
    itr.y = static_cast<float>(northing);
    const pcl::PointXYZI &p = itr;

    bool push_flag = false;
    for (auto v_itr = grids.begin(); v_itr != grids.end(); ++v_itr)
    {
      if (code == v_itr->mgrs_code)
      {
        v_itr->cloud.points.push_back(p);
        ++v_itr->size;
        push_flag = true;
        break;
      }
    }
    // NEW mgrs_code
    pcl::PointCloud<pcl::PointXYZI> a;  // null for initialize
    if (!push_flag)
    {
      pcd_xyzi_grid new_grid = { code, a, 0 };
      grids.push_back(new_grid);
      grids[grids.size() - 1].cloud.points.push_back(p);
      ++grids[grids.size() - 1].size;
    }
  }

  // save PCD to "MGRS_code_time.pcd"
  for (auto v_itr = grids.begin(); v_itr != grids.end(); ++v_itr)
  {
    std::string file_name = path + v_itr->mgrs_code + "_" + std::to_string(std::time(NULL)) + ".pcd";
    v_itr->cloud.header = input->header;
    v_itr->cloud.width = v_itr->size;
    v_itr->cloud.height = 1;
    pcl::io::savePCDFileASCII(file_name, v_itr->cloud);
    std::cout << "Saved " << v_itr->cloud.points.size() << " data points to " << file_name << std::endl;
  }
}

void pcd_jpxy2mgrs(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, std::string path, const int plane_zone)
{
  map_tools::MgrsConverter converter;
  std::vector<pcd_xyzrgb_grid> grids;

  for (auto &itr : *input)
  {
    std::string code;
    double easting, northing;
    double lat, lon, alt;
    converter.jpxy2latlon(static_cast<double>(itr.y), static_cast<double>(itr.x), static_cast<double>(itr.z),
                          plane_zone, lat, lon, alt);
    std::tie(code, easting, northing) = converter.latlon2mgrs(lat, lon);
    itr.x = static_cast<float>(easting);
    itr.y = static_cast<float>(northing);
    const pcl::PointXYZRGB &p = itr;

    bool push_flag = false;
    for (auto v_itr = grids.begin(); v_itr != grids.end(); ++v_itr)
    {
      if (code == v_itr->mgrs_code)
      {
        v_itr->cloud.points.push_back(p);
        ++v_itr->size;
        push_flag = true;
        break;
      }
    }
    // NEW mgrs_code
    pcl::PointCloud<pcl::PointXYZRGB> a;  // null for initialize
    if (!push_flag)
    {
      pcd_xyzrgb_grid new_grid = { code, a, 0 };
      grids.push_back(new_grid);
      grids[grids.size() - 1].cloud.points.push_back(p);
      ++grids[grids.size() - 1].size;
    }
  }

  // save PCD to "MGRS_code_time.pcd"
  for (auto v_itr = grids.begin(); v_itr != grids.end(); ++v_itr)
  {
    std::string file_name = path + v_itr->mgrs_code + "_" + std::to_string(std::time(NULL)) + ".pcd";
    v_itr->cloud.header = input->header;
    v_itr->cloud.width = v_itr->size;
    v_itr->cloud.height = 1;
    pcl::io::savePCDFileASCII(file_name, v_itr->cloud);
    std::cout << "Saved " << v_itr->cloud.points.size() << " data points to " << file_name << std::endl;
  }
}

int main(int argc, char **argv)
{
  if (argc < 4)
  {
    std::cout << "\"[PointXYZ|PointXYZI|PointXYZRGB]\" \"JP plane_zone\" \"filename(***.pcd)\" " << std::endl;
    return -1;
  }

  std::string point_type = argv[1];
  int plane_zone = atoi(argv[2]);
  if (point_type != "PointXYZ" && point_type != "PointXYZI" && point_type != "PointXYZRGB")
  {
    std::cout << "\"[PointXYZ|PointXYZI|PointXYZRGB]\" \"JP plane_zone\" \"filename(***.pcd)\" " << std::endl;
    std::cout << "you have to input Point type" << std::endl;
    return -1;
  }

  // PointXYZ
  if (point_type == "PointXYZ")
  {
    pcl::PointCloud<pcl::PointXYZ> map;
    std::string path;
    // Loading all PCDs.
    for (int i = 4; i <= argc; i++)
    {
      std::string input = argv[i - 1];
      pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      if (pcl::io::loadPCDFile(input, *temp_cloud) == -1)
      {
        PCL_ERROR("Couldn't read file %s \n", input.c_str());
        return -1;
      };
      unsigned long path_i = input.rfind('/');
      path = input.substr(0, path_i + 1);

      map += *temp_cloud;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr input(new pcl::PointCloud<pcl::PointXYZ>(map));
    pcd_jpxy2mgrs(input, path, plane_zone);
  }

  // PointXYZI
  else if (point_type == "PointXYZI")
  {
    pcl::PointCloud<pcl::PointXYZI> map;
    std::string path;
    // Loading all PCDs.
    for (int i = 4; i <= argc; i++)
    {
      std::string input = argv[i - 1];
      pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>);
      if (pcl::io::loadPCDFile(input, *temp_cloud) == -1)
      {
        PCL_ERROR("Couldn't read file %s \n", input.c_str());
        return -1;
      };
      unsigned long path_i = input.rfind('/');
      path = input.substr(0, path_i + 1);

      map += *temp_cloud;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr input(new pcl::PointCloud<pcl::PointXYZI>(map));
    pcd_jpxy2mgrs(input, path, plane_zone);
  }

  // PointRGB
  else
  {
    pcl::PointCloud<pcl::PointXYZRGB> map;
    std::string path;
    // Loading all PCDs.
    for (int i = 4; i <= argc; i++)
    {
      std::string input = argv[i - 1];
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      if (pcl::io::loadPCDFile(input, *temp_cloud) == -1)
      {
        PCL_ERROR("Couldn't read file %s \n", input.c_str());
        return -1;
      };
      unsigned long path_i = input.rfind('/');
      path = input.substr(0, path_i + 1);

      map += *temp_cloud;
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input(new pcl::PointCloud<pcl::PointXYZRGB>(map));
    pcd_jpxy2mgrs(input, path, plane_zone);
  }

  return 0;
}

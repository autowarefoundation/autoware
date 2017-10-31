/*
Convert csv to pcd.
Yuki Kitsukawa
*/

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

std::vector<std::string> split(std::string &input, char delimiter)
{
	std::istringstream stream(input);
	std::string field;
	std::vector<std::string> result;
	while(std::getline(stream, field, delimiter)){
		result.push_back(field);
	}
	return result;
}

int main (int argc, char** argv)
{
  int i;

  if(argc < 2){
    std::cout << "Usage: rosrun map_tools csv2pcd '***.csv'" << std::endl;
    exit(-1);
  }

  for(i = 1; i < argc; i++){

    std::string input = argv[i];
    std::string output = argv[i];
    std::string::size_type pos = output.find("csv");
    output.replace(pos, 3, "pcd");

    std::cout << "Input: " << input << std::endl;

    std::ifstream ifs(input);

    pcl::PointXYZI p;
    pcl::PointCloud<pcl::PointXYZI> output_cloud;

    if(!ifs){
    	std::cout << "Can't read " << input << "." << std::endl;
    }

    // Read csv line by line.
    std::string line;
    while(std::getline(ifs, line)){
    	std::vector<std::string> str_vec = split(line, ',');

    	p.x = std::stof(str_vec.at(0));
    	p.y = std::stof(str_vec.at(1));
    	p.z = std::stof(str_vec.at(2));
    	p.intensity = std::stoi(str_vec.at(3));
    	output_cloud.push_back(p);
    }

    if(pcl::io::savePCDFileBinary(output, output_cloud) == -1){
      std::cout << "Failed saving " << output << std::endl;
      return -1;
    }
    std::cout << "Output: " << output << std::endl << std::endl;
  }
  return 0;

}

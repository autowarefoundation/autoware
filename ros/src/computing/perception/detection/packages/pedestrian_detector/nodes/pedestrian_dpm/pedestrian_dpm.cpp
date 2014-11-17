#include <string>
#include "ros/ros.h"
#include "dpm/ImageObjects.h"

extern int dpm_ocv_main(int argc, char* argv[],
			const std::string& detection_type);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pedestrian_dpm");

	ros::NodeHandle n;
	std::string detection_type("pedestrian");

	return dpm_ocv_main(argc, argv, detection_type);
}

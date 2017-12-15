
#include <ros/ros.h>
#include "ray_ground_filter.h"


int main(int argc, char **argv)
{
	ros::init(argc, argv, "ray_ground_filter");
	RayGroundFilter app;

	app.Run();

	return 0;

}

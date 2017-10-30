#include <ros/ros.h>

#include <csv_logger.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "csv_logger");

    csv_logger_namespace::csv_logger _csv_logger;
    _csv_logger.run();

    return 0;
}

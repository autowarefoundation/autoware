#include <ros/ros.h>
#include <iostream>
#include <string>
#include <cstdlib>
#include "ImagePreprocessor.h"
#include <opencv2/highgui/highgui.hpp>


using namespace std;


int main (int argc, char *argv[])
{
	ros::init(argc, argv, "imgprocx", ros::init_options::AnonymousName);
	ros::start();
	ros::NodeHandle nodeHandler ("~");

	string maskpath (argv[1]);
	cv::Mat mmask = cv::imread(maskpath, CV_LOAD_IMAGE_GRAYSCALE);
	ImagePreprocessor::ProcessMode processMode = (ImagePreprocessor::ProcessMode)atoi(argv[2]);

	string inputTopic, outputTopic;
	nodeHandler.getParam ("input_topic", inputTopic);
	nodeHandler.getParam ("output_topic", outputTopic);

	ImagePreprocessor imgProc (processMode, inputTopic, outputTopic, nodeHandler);
	imgProc.setMask (mmask);
	imgProc.setIAlpha(0.3975);
	ros::spin();

	ros::shutdown();
	return 0;
}

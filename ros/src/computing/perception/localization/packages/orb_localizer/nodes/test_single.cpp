#include <iostream>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


#include "System.h"


using namespace std;
using namespace ORB_SLAM2;


int main (int argc, char *argv[])
{
	const string orbVocabFile (ORB_SLAM_VOCABULARY);
	const string configFile ("/home/sujiwo/Tsukuba2016/ros/etc/orb-slam2-tc.yaml");
	const string mapPath ("/home/sujiwo/Tsukuba2016/data/nagoya/2016-11-30-13-24-41/orb.map");

	ORB_SLAM2::System Localizer (orbVocabFile,
		configFile,
		ORB_SLAM2::System::MONOCULAR,
		false,
		mapPath,
	System::LOCALIZATION);

	cv::Mat image = cv::imread (argv[1]);
	Transform3 tg = Localizer.getTracker()->LocalizeImage(image, 0);

	return 0;
}

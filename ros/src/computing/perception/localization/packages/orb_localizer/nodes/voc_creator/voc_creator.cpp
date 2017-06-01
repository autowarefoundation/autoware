#include <iostream>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <ORBextractor.h>
#include <ORBVocabulary.h>
#include <Converter.h>


using namespace std;
using ORB_SLAM2::ORBextractor;
using ORB_SLAM2::ORBVocabulary;
vector<vector<cv::Mat> > sceneDescriptors;
namespace enc = sensor_msgs::image_encodings;


const float orbScaleFactor = 1.2;
const int orbLevels = 8,
		initFast = 20,
		minFast = 5;

const int imgWidth = 800,
	imgHeight = 600;

const int vocabHeight = 10,
	vocabLevel = 6;

class VocCreator
{
public:

VocCreator (ros::NodeHandle &_nodeHandler) :
	nodeHandler (_nodeHandler),
	imageBuf (nodeHandler),
	orbVocabulary (vocabHeight, vocabLevel)
{
	int numOrbFeatures;
	nodeHandler.getParam("number_extraction", numOrbFeatures);
	orbExtractor = new ORBextractor (numOrbFeatures, orbScaleFactor, orbLevels, initFast, minFast);

	string imageTopic;
	nodeHandler.getParam ("image_topic", imageTopic);

	imageSub = imageBuf.subscribe (imageTopic, 1, &VocCreator::imageCallback, this);
	cv::namedWindow("XYZ");
}


void stopVocabulary ()
{
	cerr << "Saving, please wait...\n";
	orbVocabulary.create (sceneFeatures);
	orbVocabulary.saveToTextFile("/tmp/newvocabulary.txt");
	cerr << "Done\n";
}


void appendVocabulary (cv::Mat &image)
{
	cv::resize(image, image, cv::Size(imgWidth, imgHeight));

	vector<cv::KeyPoint> fKeys;
	cv::Mat fDescriptors;
	(*orbExtractor)(image, cv::Mat(), fKeys, fDescriptors);

	// convert
	vector<cv::Mat> frameDescriptor = ORB_SLAM2::Converter::toDescriptorVector (fDescriptors);
	sceneFeatures.push_back(frameDescriptor);

	// test
	cv::Mat imgshow;
	cv::cvtColor(image, imgshow, CV_GRAY2BGR);
	for (auto &kp: fKeys) {
		cv::circle (imgshow, kp.pt, 1, cv::Scalar(255,0,0),-1);
	}
	cv::imshow ("XYZ", imgshow);
	cv::waitKey(1);
}


void show (cv::Mat &c)
{

}


void imageCallback (const sensor_msgs::ImageConstPtr &msg)
{
	// Copy the ros image message to cv::Mat.
	cv_bridge::CvImageConstPtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvShare(msg);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	cv::Mat image;
	// Check if we need debayering
	if (enc::isBayer(msg->encoding)) {
		int code=-1;
		if (msg->encoding == enc::BAYER_RGGB8 ||
			msg->encoding == enc::BAYER_RGGB16) {
			code = cv::COLOR_BayerBG2GRAY;
		}
		else if (msg->encoding == enc::BAYER_BGGR8 ||
				 msg->encoding == enc::BAYER_BGGR16) {
			code = cv::COLOR_BayerRG2GRAY;
		}
		else if (msg->encoding == enc::BAYER_GBRG8 ||
				 msg->encoding == enc::BAYER_GBRG16) {
			code = cv::COLOR_BayerGR2GRAY;
		}
		else if (msg->encoding == enc::BAYER_GRBG8 ||
				 msg->encoding == enc::BAYER_GRBG16) {
			code = cv::COLOR_BayerGB2GRAY;
		}
		cv::cvtColor(cv_ptr->image, image, code);
	}
	else {
		cv::cvtColor (cv_ptr->image, image, cv::COLOR_RGB2GRAY);
	}

	// Got gray image, send to vocabulary processing
	appendVocabulary(image);
}


private:
ros::NodeHandle &nodeHandler;
ORBextractor *orbExtractor;
ORBVocabulary orbVocabulary;

image_transport::ImageTransport imageBuf;
image_transport::Subscriber imageSub;

vector<vector<DBoW2::FORB::TDescriptor> > sceneFeatures;
};		// VocCreator






int main (int argc, char *argv[])
{
	ros::init(argc, argv, "voc_creator", ros::init_options::AnonymousName);
	ros::start();
	ros::NodeHandle nodeHandler ("~");

	VocCreator vocabularyCreator (nodeHandler);

	ros::spin();

	vocabularyCreator.stopVocabulary();
	return 0;
}

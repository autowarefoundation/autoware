#include <boost/foreach.hpp>
#include <rosbag/view.h>
#include <tf/tfMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include <iostream>
#include <exception>
#include "common.h"
#include "Converter.h"


#define foreach BOOST_FOREACH


using std::cout;
using std::endl;
using std::exception;
using ORB_SLAM2::KeyFrame;


void equalizeImageHistogramFromMask (cv::Mat &input, cv::Mat &output, cv::Mat &mask)
{
	cv::Mat LUT = cv::Mat::zeros(1, 256, CV_8U);
	cv::MatND hist;
	int histSize = 256;
	float range[] = {0,255};
	const float *ranges[] = {range};
//	uint8_t
	cv::calcHist (&input, 1, 0, mask, hist, 1, &histSize, ranges, true, false);

	int i=0;
	while (!hist.at<uint8_t>(i))
		i++;
	int total = (int)input.total();

	int miz=(int)hist.at<float>(i);
	float scale = 255.0f / (total-miz);
	int sum = 0;
	for (LUT.at<uchar>(i++)=0; i<256; ++i) {
		sum += (int)hist.at<float>(i);
		LUT.at<uchar>(i) = cv::saturate_cast<uchar>(sum * scale);
	}

	cv::LUT(input, LUT, output);
//	cv::equalizeHist(output, output);
}


float detectCcdSmear (cv::Mat &colorInput)
{
	// 1. Convert image to HSV and take V channel -> V
	cv::Mat imgHsv, V;
	cv::cvtColor(colorInput, imgHsv, CV_BGR2HSV);
	cv::extractChannel(imgHsv, V, 2);
	// 2. Normalize V
	V.convertTo(V, CV_32F);
	V /= 255.0;
	// Sum all elements of V per column
	cv::Mat tv = cv::Mat::zeros(V.cols, 1, CV_32F);
	for (int i=0; i<V.cols; i++) {
		tv.at<float>(i) = cv::sum(V.col(i))[0];
	}
	tv /= (float)V.rows;
	// Count number of columns that are out of range
	const float th = 0.1;
	int nc = 0;
	for (int i=0; i<V.cols; i++) {
		if (tv.at<float>(i) >= 1.0-th)
			nc += 1;
	}
	// done
	if (nc < 2)
		return -1;
	else return 1.0;
	// XXX: how to quantize nc to scale 0~1.0 ?
}



void tfToCV(const tf::Transform &src, cv::Mat &position, cv::Mat &orientation)
{
	position = cv::Mat (3,1,CV_64F);
	tf::Vector3 p = src.getOrigin();
	position.at<double>(0) = p.x(),
		position.at<double>(1) = p.y(),
		position.at<double>(2) = p.z();

	orientation = cv::Mat (4,1,CV_64F);
	tf::Quaternion otn = src.getRotation();
	orientation.at<double>(0) = otn.x(),
		orientation.at<double>(1) = otn.y(),
		orientation.at<double>(2) = otn.z(),
		orientation.at<double>(3) = otn.w();
}


void recomputeNewCameraParameter (
	// Original
	double fx1, double fy1, double cx1, double cy1,
	// New
	double &fx2, double &fy2, double &cx2, double &cy2,
	int width1, int height1,
	int width2, int height2
)
{
	double ratio = (double)width1 / (double)width2;
	fx2 = fx1 / ratio;
	fy2 = fy1 / ratio;
	cx2 = cx1 / ratio;
	cy2 = cy1 / ratio;
}


void tf2positiondirection (const tf::Transform &pose, float positiondirection[6])
{
	// position
	positiondirection[0] = pose.getOrigin().x();
	positiondirection[1] = pose.getOrigin().y();
	positiondirection[2] = pose.getOrigin().z();
	float fdirx = pose.getRotation().x(),
		fdiry = pose.getRotation().y(),
		fdirz = pose.getRotation().z(),
		fdirnorm;
	fdirnorm = sqrtf(fdirx*fdirx + fdiry*fdiry + fdirz*fdirz);
	fdirx /= fdirnorm;
	fdiry /= fdirnorm;
	fdirz /= fdirnorm;
	positiondirection[3] = fdirx;
	positiondirection[4] = fdiry;
	positiondirection[5] = fdirz;
}


tf::Transform FramePose (ORB_SLAM2::Frame *cframe)
{
	cv::Mat Rwc = cframe->mTcw.rowRange(0,3).colRange(0,3).t();
	cv::Mat twc = -Rwc * cframe->mTcw.rowRange(0,3).col(3);
	tf::Matrix3x3 M(Rwc.at<float>(0,0),Rwc.at<float>(0,1),Rwc.at<float>(0,2),
					Rwc.at<float>(1,0),Rwc.at<float>(1,1),Rwc.at<float>(1,2),
					Rwc.at<float>(2,0),Rwc.at<float>(2,1),Rwc.at<float>(2,2));
	tf::Vector3 V(twc.at<float>(0), twc.at<float>(1), twc.at<float>(2));

	return tf::Transform(M, V);
}


tf::Transform KeyFramePoseToTf (KeyFrame *kf)
{
	tf::Transform kfpose;

	cv::Mat t = kf->GetCameraCenter();
	cv::Mat orient = kf->GetRotation().t();
	Eigen::Quaterniond q = ORB_SLAM2::Converter::toQuaternion(orient);

	kfpose.setOrigin(tf::Vector3(t.at<float>(0), t.at<float>(1), t.at<float>(2)));
	kfpose.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));

	return kfpose;
}


tf::Transform getKeyFrameExtPose (const KeyFrame *kf)
{
	tf::Transform Ext;

	if (kf->extPosition.empty() or kf->extOrientation.empty()) {
		Ext.setOrigin(tf::Vector3(NAN, NAN, NAN));
		Ext.setRotation(tf::Quaternion(NAN, NAN, NAN, NAN));
	}

	else {
		Ext.setOrigin (tf::Vector3(
			kf->extPosition.at<double>(0),
			kf->extPosition.at<double>(1),
			kf->extPosition.at<double>(2) ));
		Ext.setRotation(tf::Quaternion(
			kf->extOrientation.at<double>(0),
			kf->extOrientation.at<double>(1),
			kf->extOrientation.at<double>(2),
			kf->extOrientation.at<double>(3) ));
	}
	return Ext;
}


cv::Vec3d
tfToCv (const tf::Vector3 &pos)
{
	cv::Vec3d cvVec;
	cvVec[0] = pos.x();
	cvVec[1] = pos.y();
	cvVec[2] = pos.z();
	return cvVec;
}

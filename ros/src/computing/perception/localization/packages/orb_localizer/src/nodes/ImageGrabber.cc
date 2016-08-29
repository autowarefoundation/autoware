/*
 * ImageGrabber.cc
 *
 *  Created on: May 31, 2016
 *      Author: sujiwo
 */

#include "ImageGrabber.h"

// XXX: Change to appropriate custom message
#include <std_msgs/UInt32.h>
#include <std_msgs/Float64.h>

#include "boost/date_time/posix_time/posix_time.hpp"


using namespace std;
using namespace boost::posix_time;

using ORB_SLAM2::Frame;
namespace enc = sensor_msgs::image_encodings;


// define debugging topics to broadcast to
const char
	*framebufferDebugTopic		=	"/orbslamdebug/framebuffer",
	*internalTopic				=	"/orbslamdebug";


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


ImageGrabber::ImageGrabber(ORB_SLAM2::System* pSLAM, ros::NodeHandle *nh, bool runOffline) :
	mpSLAM(pSLAM),
	rosNode (nh),
	doStop (false),
	doDebayer (false),
	offlineMode (runOffline),
	mTfBr (NULL),
	extListener (NULL),
	imageTransport (NULL)
{
	// External localization
	extFrame1 = (string)mpSLAM->fsSettings["ExternalLocalization.frame1"];
	extFrame2 = (string)mpSLAM->fsSettings["ExternalLocalization.frame2"];
	cout << "External Reference: from " << extFrame1 << " to " << extFrame2 << endl;

	offsetKeyframe = (int)mpSLAM->fsSettings["ExternalLocalization.OffsetKeyframes"];

	// Initialize TF
	if (offlineMode==false) {
		tf::Transform tfT;
		tfT.setIdentity();
		mTfBr = new tf::TransformBroadcaster();

		mTfBr->sendTransform(tf::StampedTransform(tfT,ros::Time::now(), "/ORB_SLAM/World", "/ORB_SLAM/Camera"));
	}

	// start of debug preparation

	cout << std::fixed << setprecision(7);

	if (pSLAM->opMode==ORB_SLAM2::System::LOCALIZATION) {
		imageTransport = new image_transport::ImageTransport (*rosNode);
		visualDebugView = imageTransport->advertise(framebufferDebugTopic, 1);
		debugMsgPublisher = rosNode->advertise<orb_localizer::debug> (internalTopic, 1);
	}
}


ImageGrabber::~ImageGrabber()
{
	if (mTfBr != NULL)
		delete (mTfBr);
	if (extListener != NULL)
		delete (extListener);
	if (imageTransport != NULL)
		delete (imageTransport);

//	debugBag.close();
}


void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
	// Activate this timer if you need time logging
	ptime rT1, rT2;
	rT1 = microsec_clock::local_time();

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
//			cout << "BGR2BGR" << endl;
			code = cv::COLOR_BayerBG2BGR;
		}
		else if (msg->encoding == enc::BAYER_BGGR8 ||
				 msg->encoding == enc::BAYER_BGGR16) {
			code = cv::COLOR_BayerRG2BGR;
		}
		else if (msg->encoding == enc::BAYER_GBRG8 ||
				 msg->encoding == enc::BAYER_GBRG16) {
			code = cv::COLOR_BayerGR2BGR;
		}
		else if (msg->encoding == enc::BAYER_GRBG8 ||
				 msg->encoding == enc::BAYER_GRBG16) {
			code = cv::COLOR_BayerGB2BGR;
		}
		cv::cvtColor(cv_ptr->image, image, code);
	}
	else
		image = cv_ptr->image;

	const double imageTime = msg->header.stamp.toSec();
	lastImageTimestamp = imageTime;

	// Do Resizing and cropping here
	cv::resize(image, image,
		cv::Size(
			(int)mpSLAM->fsSettings["Camera.WorkingResolution.Width"],
			(int)mpSLAM->fsSettings["Camera.WorkingResolution.Height"]
		));
	image = image(
		cv::Rect(
			(int)mpSLAM->fsSettings["Camera.ROI.x0"],
			(int)mpSLAM->fsSettings["Camera.ROI.y0"],
			(int)mpSLAM->fsSettings["Camera.ROI.width"],
			(int)mpSLAM->fsSettings["Camera.ROI.height"]
		)).clone();

	cv::Mat tmpRs = mpSLAM->TrackMonocular(image, imageTime);

	// Reinsert TF publisher, but only for localization. Original ORB-SLAM2 removes it.
	bool tfOk = false;
	tf::Transform locRef;

	tf::StampedTransform tfMsg;
	tfMsg.stamp_ = ros::Time(imageTime);
	tfMsg.frame_id_ = (string)mpSLAM->fsSettings["ExternalLocalization.frame1"];
	tfMsg.child_frame_id_ = (string)mpSLAM->fsSettings["ExternalLocalization.frame2"];

	Frame &cframe = mpSLAM->getTracker()->mCurrentFrame;
	if (mpSLAM->opMode==ORB_SLAM2::System::LOCALIZATION and
		mpSLAM->getTracker()->trackingIsGood() and
		offlineMode == false
	) {

//		cout << "Got Tracking: Client" << endl;
//		cout << tmpRs << endl << "XXX\n";

		tf::Transform tfTcw = FramePose(&cframe);
		mTfBr->sendTransform(tf::StampedTransform(tfTcw, ros::Time(imageTime), "/ORB_SLAM/World", "/ORB_SLAM/Camera"));

//		 Here, we use offset of external localization from the keyframe
		if (mpSLAM->getTracker()->mbOnlyTracking==true) {
//			ORB_SLAM2::KeyFrame *kfRef = cframe.mpReferenceKF;
			try {
				locRef = localizeByReference(tfTcw);
				tfMsg.setData(locRef);
				mTfBr->sendTransform(tfMsg);
				tfOk = true;
			} catch (...) {}
		}

	} else {
//		cout << "Got Lost" << endl;
	}

	rT2 = microsec_clock::local_time();
	cputimeDebug = (rT2-rT1).total_microseconds() * 1e-6;

	publishDebug();
}


void ImageGrabber::publishDebug ()
{
	if (mpSLAM->opMode==ORB_SLAM2::System::LOCALIZATION) {
		mpSLAM->getFrameDrawer()->DrawFrame();
		framebufferDebug = mpSLAM->getFrameDrawer()->getLastFrame();

		cv_bridge::CvImage bagImage;
		bagImage.image = framebufferDebug;
		bagImage.header.stamp = ros::Time(lastImageTimestamp);
		bagImage.encoding = "bgr8";
		visualDebugView.publish(bagImage.toImageMsg());

		orb_localizer::debug internalDebugMsg;
		internalDebugMsg.header.stamp = ros::Time (lastImageTimestamp);
		internalDebugMsg.header.frame_id = "ORB_SLAM2";
		internalDebugMsg.keyframe_id = lastKeyframeId;
		internalDebugMsg.cputime = cputimeDebug;
		internalDebugMsg.tracking = mpSLAM->getTracker()->trackingIsGood();
		debugMsgPublisher.publish (internalDebugMsg);
	}
}


void ImageGrabber::externalLocalizerGrab()
{
	if (extListener==NULL)
		extListener = new tf::TransformListener ();

	ros::Rate fps((int)mpSLAM->fsSettings["Camera.fps"] * 2);

	while (ros::ok()) {

		if (doStop == true)
			break;

		try {

			extListener->lookupTransform (extFrame1, extFrame2, ros::Time(0), extPose);
			unique_lock<mutex> lock(ORB_SLAM2::KeyFrame::extPoseMutex);
			tfToCV (extPose, ORB_SLAM2::KeyFrame::extEgoPosition, ORB_SLAM2::KeyFrame::extEgoOrientation);

		} catch (tf::TransformException &e) {

			unique_lock<mutex> lock(ORB_SLAM2::KeyFrame::extPoseMutex);
			ORB_SLAM2::KeyFrame::extEgoPosition.release();
			ORB_SLAM2::KeyFrame::extEgoOrientation.release();

		}

		fps.sleep();
	}
}


tf::Transform ImageGrabber::localizeByReference (const tf::Transform &tfOrb, ORB_SLAM2::KeyFrame *kf)
{
	lastKeyframeId = kf->mnId;

	ORB_SLAM2::KeyFrame *kOffset = mpSLAM->getMap()->offsetKeyframe(kf, offsetKeyframe);
	if (kOffset==NULL)
		throw std::out_of_range("No offset keyframe found");

	if (kf->extPosition.empty() or kOffset->extPosition.empty())
		throw std::out_of_range("External reference of keyframe not found");

	tf::Transform kfTr = KeyFramePoseToTf(kf);
	tf::Transform extRef = getKeyFrameExtPose(kf);

	tf::Transform kfTrOffset = KeyFramePoseToTf(kOffset);
	tf::Transform extRefOffset = getKeyFrameExtPose(kOffset);
	return localizeByReference (tfOrb, kfTr, kfTrOffset, extRef, extRefOffset);
}


/*
 * Main routine for localization by reference
 */
tf::Transform ImageGrabber::localizeByReference (
    	const tf::Transform &tfOrb,
		const tf::Transform &tfOrbMap, const tf::Transform &tfOrbMapOffset,
    	const tf::Transform &realMapPose, const tf::Transform &realMapOffset)
{
	double offDistO = cv::norm(
		ImageGrabber::tfToCv(tfOrbMap.getOrigin()) -
		ImageGrabber::tfToCv(tfOrbMapOffset.getOrigin()));
	double offDistE = cv::norm(
		ImageGrabber::tfToCv(realMapPose.getOrigin()) -
		ImageGrabber::tfToCv(realMapOffset.getOrigin()));
	double scale = offDistE / offDistO;

	// change orientation from camera to velodyne
	tf::Transform flipAxes;
	flipAxes.setOrigin(tf::Vector3(0, 0, 0));
	tf::Quaternion fpq;
	fpq.setRPY(M_PI/2,0,0);
	fpq.normalize();
	flipAxes.setRotation(fpq);
//	flipAxes.setRotation (tf::Quaternion(-M_PI/2, M_PI/2, 0).normalize());

	tf::Transform orbRel = tfOrbMap.inverse() * tfOrb;

	tf::Transform scaledRel = orbRel;
	scaledRel.setOrigin(orbRel.getOrigin() * scale);

	tf::Transform tfResult = realMapPose * scaledRel;
	return tfResult*flipAxes;
}


tf::Transform ImageGrabber::localizeByReference(const tf::Transform &tfOrb)
{
	float fdirx = tfOrb.getRotation().x(),
		fdiry = tfOrb.getRotation().y(),
		fdirz = tfOrb.getRotation().z(),
		fdirnorm;
	fdirnorm = sqrtf(fdirx*fdirx + fdiry*fdiry + fdirz*fdirz);
	fdirx /= fdirnorm;
	fdiry /= fdirnorm;
	fdirz /= fdirnorm;

	ORB_SLAM2::KeyFrame *kfNear = mpSLAM->getMap()->getNearestKeyFrame(
		tfOrb.getOrigin().x(),
		tfOrb.getOrigin().y(),
		tfOrb.getOrigin().z(),
		fdirx, fdiry, fdirz);
	if (kfNear==NULL)
		throw std::out_of_range("No keyframe found");

	lastKeyframeId = kfNear->mnId;
	return localizeByReference (tfOrb, kfNear);
}


tf::Transform ImageGrabber::localizeByReference(Frame *sframe)
{
//	const tf::Transform
}


tf::Transform ImageGrabber::localizeByReference(const tf::Transform &tfOrb, ORB_SLAM2::Map *mapsrc, const int offsetNum)
{
	float positiondir[6];
	tf2positiondirection(tfOrb, positiondir);

	ORB_SLAM2::KeyFrame *kfNear = mapsrc->getNearestKeyFrame(
		positiondir[0], positiondir[1], positiondir[2],
		positiondir[3], positiondir[4], positiondir[5]);
	if (kfNear==NULL)
		throw std::out_of_range("No keyframe found");
	ORB_SLAM2::KeyFrame *kOffset = mapsrc->offsetKeyframe(kfNear, offsetNum);
	if (kOffset==NULL)
		throw std::out_of_range("No offset keyframe found");

	tf::Transform kfTr = KeyFramePoseToTf(kfNear);
	tf::Transform extRef = getKeyFrameExtPose(kfNear);

	tf::Transform kfTrOffset = KeyFramePoseToTf(kOffset);
	tf::Transform extRefOffset = getKeyFrameExtPose(kOffset);

	return localizeByReference (tfOrb, kfTr, kfTrOffset, extRef, extRefOffset);
}


tf2_msgs::TFMessage ImageGrabber::createTfMessage (const tf::Transform &srcTransform,
	const string &frameSrc, const string &frameTarget,
	double timestamp=-1)
{
	ros::Time msgTime;
	tf2_msgs::TFMessage tfretval;

	if (timestamp>0)
		msgTime = ros::Time(timestamp);
	else msgTime = ros::Time::now();

	geometry_msgs::TransformStamped newTransform;
	newTransform.header.stamp = msgTime;
	newTransform.header.frame_id = frameSrc;
	newTransform.child_frame_id = frameTarget;
	newTransform.transform.translation.x = srcTransform.getOrigin().x();
	newTransform.transform.translation.y = srcTransform.getOrigin().y();
	newTransform.transform.translation.z = srcTransform.getOrigin().z();
	newTransform.transform.rotation.x = srcTransform.getRotation().x();
	newTransform.transform.rotation.y = srcTransform.getRotation().y();
	newTransform.transform.rotation.z = srcTransform.getRotation().z();
	newTransform.transform.rotation.w = srcTransform.getRotation().w();
	tfretval.transforms.push_back (newTransform);

	return tfretval;
}


tf::Transform ImageGrabber::getKeyFrameExtPose (const KeyFrame *kf)
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


tf::Transform ImageGrabber::KeyFramePoseToTf (KeyFrame *kf)
{
	tf::Transform kfpose;

	cv::Mat t = kf->GetCameraCenter();
	cv::Mat orient = kf->GetRotation().t();
	vector<float> q = ORB_SLAM2::Converter::toQuaternion(orient);

	kfpose.setOrigin(tf::Vector3(t.at<float>(0), t.at<float>(1), t.at<float>(2)));
	kfpose.setRotation(tf::Quaternion(q[0], q[1], q[2], q[3]));

	return kfpose;
}


tf::Transform ImageGrabber::FramePose (Frame *cframe)
{
	cv::Mat Rwc = cframe->mTcw.rowRange(0,3).colRange(0,3).t();
	cv::Mat twc = -Rwc * cframe->mTcw.rowRange(0,3).col(3);
	tf::Matrix3x3 M(Rwc.at<float>(0,0),Rwc.at<float>(0,1),Rwc.at<float>(0,2),
					Rwc.at<float>(1,0),Rwc.at<float>(1,1),Rwc.at<float>(1,2),
					Rwc.at<float>(2,0),Rwc.at<float>(2,1),Rwc.at<float>(2,2));
	tf::Vector3 V(twc.at<float>(0), twc.at<float>(1), twc.at<float>(2));

	return tf::Transform(M, V);
}


cv::Vec3d ImageGrabber::tfToCv (const tf::Vector3 &pos)
{
	cv::Vec3d cvVec;
	cvVec[0] = pos.x();
	cvVec[1] = pos.y();
	cvVec[2] = pos.z();
	return cvVec;
}


cv::Mat ImageGrabber::tfToCv (const tf::Transform &tfsrc)
{
	cv::Mat rtval = cv::Mat::eye(4,4, CV_32F);
	rtval.rowRange(0, 3).col(3).at<float>(0) = tfsrc.getOrigin().x();
	rtval.rowRange(0, 3).col(3).at<float>(1) = tfsrc.getOrigin().y();
	rtval.rowRange(0, 3).col(3).at<float>(2) = tfsrc.getOrigin().z();

	tf::Matrix3x3 rot (tfsrc.getRotation());
	for (int i=0; i<3; i++) {
		for (int j=0; j<3; j++) {
			rtval.at<float>(i,j) = rot[i][j];
		}
	}
	return rtval;
}

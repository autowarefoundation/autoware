/*
 * Matcher.cpp
 *
 *  Created on: Feb 15, 2017
 *      Author: sujiwo
 */

#include "Matcher.h"
#include <visualization_msgs/MarkerArray.h>


const string orbGenericVocabFile = ORB_SLAM_VOCABULARY;
vector<KeyFrame*> keyframeSelectorDebug(15);
ros::Publisher keyframeDebugger;


cv::Mat
tfToCv (const tf::Transform &tfsrc)
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


tf::Transform
rotateAxes (const tf::Transform &cPose, const float roll=0, const float pitch=0, const float yaw=0)
{
	tf::Transform rotation;
	rotation.setOrigin (tf::Vector3 (0,0,0));
	rotation.setRotation (tf::Quaternion(roll, pitch, yaw).normalize());
	return cPose * rotation;
}


void debugKeyframeSelection ()
{
	visualization_msgs::MarkerArray keyArrows;
	int i = 0;

	for (auto &kf: keyframeSelectorDebug) {
		visualization_msgs::Marker kfMarker;
		tf::Transform kfPose = KeyFramePoseToTf (kf);
//		kfPose = rotateAxes(kfPose, 0, -M_PI_2, 0);
		kfMarker.header.frame_id = "ORB_SLAM/World";
		kfMarker.action = visualization_msgs::Marker::ADD;
		kfMarker.type = visualization_msgs::Marker::ARROW;
		kfMarker.ns = "keyframe";
		kfMarker.id = i;
		kfMarker.color.r = 0.803;
		kfMarker.color.g = 0.5;
		kfMarker.color.b = 0.196;
		kfMarker.color.a = 1.0;
		kfMarker.scale.x = 0.01;
		kfMarker.scale.y = 0.02;
		kfMarker.scale.z = 0;

		tf::Matrix3x3 mtkf (kfPose.getRotation());
		float
			dirx = mtkf[0][2],
			diry = mtkf[1][2],
			dirz = mtkf[2][2];
		float norm = sqrtf(dirx*dirx + diry*diry + dirz*dirz);
		dirx /= norm;
		diry /= norm;
		dirz /= norm;
		geometry_msgs::Point p0, p1;
		p0.x = kfPose.getOrigin().x();
		p0.y = kfPose.getOrigin().y();
		p0.z = kfPose.getOrigin().z();
		p1.x = p0.x + dirx;
		p1.y = p0.y + diry;
		p1.z = p0.z + dirz;
		kfMarker.points.push_back(p0);
		kfMarker.points.push_back(p1);

		keyArrows.markers.push_back (kfMarker);
		i+=1;
	}

	keyframeDebugger.publish(keyArrows);
}


Matcher::Matcher (ros::NodeHandle &nh, bool _doPublish) :
	rosnode (nh),
	lastImageTimestamp(0),
	gotFirstFrame (false),
	lastKeyframeId (0),
	cpuTimeDebug (0),
	doPublish (_doPublish),
	lastGoodLocalizationTimestamp (-1)
{
	currentRealPose.setOrigin(tf::Vector3(1e10, 1e10, 1e10));

	mTfBr = new tf::TransformBroadcaster();

	string mapPath;
	rosnode.getParam ("map_file", mapPath);

	string configFile;
	rosnode.getParam ("configuration_file", configFile);

	SLAMSystem = new ORB_SLAM2::System (
		orbGenericVocabFile,
		configFile,
		ORB_SLAM2::System::MONOCULAR,
		false,
		mapPath,
		ORB_SLAM2::System::LOCALIZATION);

	// Image Subscription
    if ((int)SLAMSystem->fsSettings["Camera.compressed"]==0) {
    	th = image_transport::TransportHints ("raw");
    }
    else if ((int)SLAMSystem->fsSettings["Camera.compressed"]==1) {
    	th = image_transport::TransportHints ("compressed");
    }
	imageBuf = new image_transport::ImageTransport(rosnode);

	externalFrameFixed = (string)SLAMSystem->fsSettings["Localization.frame1"];
	externalFrameMoving = (string)SLAMSystem->fsSettings["Localization.frame2"] + ros::this_node::getName();
	offsetKeyframe = SLAMSystem->fsSettings["ExternalLocalization.OffsetKeyframes"];

	cout << "TF: From " << externalFrameFixed << " to " << externalFrameMoving << endl;

	string imageTopic;
	rosnode.getParam("image_topic", imageTopic);
	imageSub = imageBuf->subscribe (imageTopic, 1, &Matcher::imageCallback, this, th);

	// Result Publishers
	string poseTopic;
	rosnode.getParam("pose_topic", poseTopic);
	posePublisher = rosnode.advertise<geometry_msgs::PoseStamped> (poseTopic, 1);
	poseCovPublisher = rosnode.advertise<geometry_msgs::PoseWithCovarianceStamped> (poseTopic+"_covariance", 1);
	mTfBr = new tf::TransformBroadcaster();

	// start of debug preparation
	cout << std::fixed << setprecision(7);
	visualDebugView = imageBuf->advertise("framebuffer", 1);
	debugMsgPublisher = rosnode.advertise<orb_localizer::debug> ("debug", 1);
	keyframeDebugger = rosnode.advertise<visualization_msgs::MarkerArray> ("keyframeDebug", 1);
}


Matcher::~Matcher()
{
	SLAMSystem->Shutdown();

	delete (imageBuf);
	delete (mTfBr);
	delete (SLAMSystem);
}


void Matcher::imageCallback(const sensor_msgs::ImageConstPtr &msg)
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

	if (gotFirstFrame==false) {
		double fx2, fy2, cx2, cy2;
		recomputeNewCameraParameter (
			(double)SLAMSystem->fsSettings["Camera.fx"],
			(double)SLAMSystem->fsSettings["Camera.fy"],
			(double)SLAMSystem->fsSettings["Camera.cx"],
			(double)SLAMSystem->fsSettings["Camera.cy"],
			fx2, fy2, cx2, cy2,
			msg->width, msg->height,
			(int)SLAMSystem->fsSettings["Camera.WorkingResolution.Width"],
			(int)SLAMSystem->fsSettings["Camera.WorkingResolution.Height"]);
		// send camera parameters to tracker
		SLAMSystem->getTracker()->ChangeCalibration (fx2, fy2, cx2, cy2);
		gotFirstFrame = true;
	}

	// Processing before sending image to tracker
	// Do Resizing and cropping here
	cv::resize(image, image,
		cv::Size(
			(int)SLAMSystem->fsSettings["Camera.WorkingResolution.Width"],
			(int)SLAMSystem->fsSettings["Camera.WorkingResolution.Height"]
		));
	image = image(
		cv::Rect(
			(int)SLAMSystem->fsSettings["Camera.ROI.x0"],
			(int)SLAMSystem->fsSettings["Camera.ROI.y0"],
			(int)SLAMSystem->fsSettings["Camera.ROI.width"],
			(int)SLAMSystem->fsSettings["Camera.ROI.height"]
		)).clone();

	SLAMSystem->TrackMonocular(image, imageTime);

	bool tfOk = false;
	tf::Transform locRef;

	Frame &cframe = SLAMSystem->getTracker()->mCurrentFrame;

	try {
		if (SLAMSystem->getTracker()->trackingIsGood())
		{

			tf::Transform tfTcw = FramePose(&cframe);
			currentOrbPose = tfTcw;

//			fprintf (stderr, "ORB Position: %f %f %f\n",
//				currentOrbPose.getOrigin().x(),
//				currentOrbPose.getOrigin().y(),
//				currentOrbPose.getOrigin().z());
			mTfBr->sendTransform(tf::StampedTransform(currentOrbPose, ros::Time(lastImageTimestamp), "/ORB_SLAM/World", "/ORB_SLAM/Camera"));

	//		 Here, we use offset of external localization from the keyframe
			locRef = localizeByReference(tfTcw);
			publishPose(&locRef);
			tfOk = true;
			lastGoodLocalizationTimestamp = imageTime;

		} else {
			locRef.setOrigin(tf::Vector3(1e10, 1e10, 1e10));
			throw OrbException ("Tracking Lost");
		}

	} catch (OrbException &e) {
		publishPose (NULL);
	}

	currentRealPose = locRef;

	rT2 = microsec_clock::local_time();
	cpuTimeDebug = (rT2-rT1).total_microseconds() * 1e-6;

	// Debugging
	SLAMSystem->getFrameDrawer()->DrawFrame();
	cv::Mat framebufferDbg = SLAMSystem->getFrameDrawer()->getLastFrame();
	cv_bridge::CvImage bagImage;
	bagImage.image = framebufferDbg;
	bagImage.header.stamp = ros::Time(lastImageTimestamp);
	bagImage.encoding = "bgr8";
	visualDebugView.publish(bagImage.toImageMsg());

	orb_localizer::debug debugMsg;
	debugMsg.header.stamp = ros::Time(lastImageTimestamp);
	debugMsg.keyframe_id = lastKeyframeId;
	debugMsg.cputime = cpuTimeDebug;
	debugMsg.tracking = SLAMSystem->getTracker()->trackingIsGood();
	debugMsgPublisher.publish (debugMsg);
}


void
Matcher::publishPose (const tf::Transform *pose)
{
	if (doPublish==false)
		return;

	geometry_msgs::PoseStamped gpose;
	geometry_msgs::PoseWithCovarianceStamped gvpose;
	gpose.header.frame_id = externalFrameFixed;
	gpose.header.stamp = ros::Time(lastImageTimestamp);

	if (pose==NULL) {
		gpose.pose.position.x =
		gpose.pose.position.y =
		gpose.pose.position.z = 1e15;
		gpose.pose.orientation.x = 1.0;
		gpose.pose.orientation.y = .0;
		gpose.pose.orientation.z = .0;
		gpose.pose.orientation.w = .0;
		for (int i=0; i<36; i++) gvpose.pose.covariance[i] = 1;
	}

	else {
		gpose.pose.position.x = pose->getOrigin().x();
		gpose.pose.position.y = pose->getOrigin().y();
		gpose.pose.position.z = pose->getOrigin().z();
		gpose.pose.orientation.x = pose->getRotation().x();
		gpose.pose.orientation.y = pose->getRotation().y();
		gpose.pose.orientation.z = pose->getRotation().z();
		gpose.pose.orientation.w = pose->getRotation().w();

		mTfBr->sendTransform (tf::StampedTransform(*pose, ros::Time(lastImageTimestamp), externalFrameFixed, externalFrameMoving));

		for (int i=0; i<36; i++) gvpose.pose.covariance[i] = 0;
	}

	gvpose.pose.pose = gpose.pose;
	poseCovPublisher.publish(gvpose);
	posePublisher.publish (gpose);
}


Eigen::Vector3f make_vector3(const float &x, const float &y, const float &z)
{
	Eigen::Vector3f v;
	v << x,y,z;
	return v;
}


Eigen::Vector3f make_vector3 (const tf::Vector3 &tfv)
{
	return make_vector3 (tfv.x(), tfv.y(), tfv.z());
}


Eigen::Quaternionf make_quaternion (const float &x, const float &y, const float &z, const float &w)
{
	Eigen::Quaternionf q(w, x, y, z);
	return q;
}


Eigen::Quaternionf make_quaternion (const tf::Quaternion &tfq)
{
	return make_quaternion(tfq.x(), tfq.y(), tfq.z(), tfq.w());
}


tf::Transform
Matcher::localizeByReference(const tf::Transform &tfOrb)
{
	tf::Matrix3x3 orient (tfOrb.getRotation());
	ORB_SLAM2::KeyFrame *kfNear = SLAMSystem->getMap()->getNearestKeyFrame(
		make_vector3(tfOrb.getOrigin()),
		make_quaternion(tfOrb.getRotation()),
		&keyframeSelectorDebug
	);
//	debugKeyframeSelection();

	if (kfNear==NULL) {
		lastKeyframeId = -1;
		cerr << "**\n";
		throw OrbException("No nearest keyframe found _ 1");
	}

	lastKeyframeId = kfNear->mnId;
	return localizeByReference (tfOrb, kfNear);
}


tf::Transform
Matcher::localizeByReference (
		const tf::Transform &orbOriginalResult,
		const tf::Transform &orbMap, const tf::Transform &orbMapOffset,
		const tf::Transform &realMapPose, const tf::Transform &realMapOffset)
{
	double offDistO = cv::norm(
		tfToCv(orbMap.getOrigin()) -
		tfToCv(orbMapOffset.getOrigin()));
	double offDistE = cv::norm(
		tfToCv(realMapPose.getOrigin()) -
		tfToCv(realMapOffset.getOrigin()));
	double scale = offDistE / offDistO;

	// change orientation from camera to velodyne
	tf::Transform flipAxes;
	flipAxes.setOrigin(tf::Vector3(0, 0, 0));
	flipAxes.setRotation (tf::Quaternion(M_PI/2, 0, -M_PI/2).normalize());

	tf::Transform orbRel = orbMap.inverse() * orbOriginalResult;

	tf::Transform scaledRel = orbRel;
	scaledRel.setOrigin(orbRel.getOrigin() * scale);
	scaledRel = flipAxes*scaledRel;

	tf::Transform tfResult = realMapPose * scaledRel;

	// Still need to rotate axes
	return rotateAxes (tfResult, 0, M_PI/2, M_PI/2);
}


tf::Transform
Matcher::localizeByReference (const tf::Transform &tfOrb, ORB_SLAM2::KeyFrame *kf)
{
	lastKeyframeId = kf->mnId;

	ORB_SLAM2::KeyFrame *kOffset = SLAMSystem->getMap()->offsetKeyframe(kf, offsetKeyframe);
	if (kOffset==NULL)
		throw OrbException("No offset keyframe found");

	if (kf->extPosition.empty() or kOffset->extPosition.empty())
		throw OrbException("External reference of keyframe not found");

	tf::Transform kfTr = KeyFramePoseToTf(kf);
	tf::Transform extRef = getKeyFrameExtPose(kf);

	tf::Transform kfTrOffset = KeyFramePoseToTf(kOffset);
	tf::Transform extRefOffset = getKeyFrameExtPose(kOffset);
	return localizeByReference (tfOrb, kfTr, kfTrOffset, extRef, extRefOffset);

}

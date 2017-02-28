/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  XXX: Licensing has not been cleared yet.
*/

#include <iostream>
#include <string>
#include <thread>
#include <cmath>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>
#include "boost/date_time/posix_time/posix_time.hpp"

#include "System.h"
#include "Converter.h"
#include "../common.h"


using namespace std;
using namespace ORB_SLAM2;
using namespace boost::posix_time;
namespace enc = sensor_msgs::image_encodings;


class ORB_Matcher
{
public:
	ORB_Matcher (ORB_SLAM2::System& pSL, ros::NodeHandle &nh) :
		SLAMSystem (pSL), rosnode(nh),
		lastImageTimestamp (0.0),
		gotFirstFrame (false),
		lastKeyframeId (0)
	{
		externalFrameFixed = (string)pSL.fsSettings["Localization.frame1"];
		externalFrameMoving = (string)pSL.fsSettings["Localization.frame2"];
		offsetKeyframe = SLAMSystem.fsSettings["ExternalLocalization.OffsetKeyframes"];

		cout << "TF: From " << externalFrameFixed << " to " << externalFrameMoving << endl;

		// Image Subscription
	    if ((int)SLAMSystem.fsSettings["Camera.compressed"]==0) {
	    	th = image_transport::TransportHints ("raw");
	    }
	    else if ((int)SLAMSystem.fsSettings["Camera.compressed"]==1) {
	    	th = image_transport::TransportHints ("compressed");
	    }
		imageBuf = new image_transport::ImageTransport(rosnode);

		// Result Publishers
		string poseTopic;
		rosnode.getParam("pose_topic", poseTopic);
		posePublisher = rosnode.advertise<geometry_msgs::PoseStamped> (poseTopic, 1);
		mTfBr = new tf::TransformBroadcaster();

		// start of debug preparation
		cout << std::fixed << setprecision(7);
		visualDebugView = imageBuf->advertise("framebuffer", 1);
	}


	~ORB_Matcher ()
	{
		delete (imageBuf);
		delete (mTfBr);
	}


	void imageCallback (const sensor_msgs::ImageConstPtr& msg)
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
				(double)SLAMSystem.fsSettings["Camera.fx"],
				(double)SLAMSystem.fsSettings["Camera.fy"],
				(double)SLAMSystem.fsSettings["Camera.cx"],
				(double)SLAMSystem.fsSettings["Camera.cy"],
				fx2, fy2, cx2, cy2,
				msg->width, msg->height,
				(int)SLAMSystem.fsSettings["Camera.WorkingResolution.Width"],
				(int)SLAMSystem.fsSettings["Camera.WorkingResolution.Height"]);
			// send camera parameters to tracker
			SLAMSystem.getTracker()->ChangeCalibration (fx2, fy2, cx2, cy2);
			gotFirstFrame = true;
		}

		// Processing before sending image to tracker
		// Do Resizing and cropping here
		cv::resize(image, image,
			cv::Size(
				(int)SLAMSystem.fsSettings["Camera.WorkingResolution.Width"],
				(int)SLAMSystem.fsSettings["Camera.WorkingResolution.Height"]
			));
		image = image(
			cv::Rect(
				(int)SLAMSystem.fsSettings["Camera.ROI.x0"],
				(int)SLAMSystem.fsSettings["Camera.ROI.y0"],
				(int)SLAMSystem.fsSettings["Camera.ROI.width"],
				(int)SLAMSystem.fsSettings["Camera.ROI.height"]
			)).clone();

		SLAMSystem.TrackMonocular(image, imageTime);

		// Reinsert TF publisher, but only for localization. Original ORB-SLAM2 removes it.
		bool tfOk = false;
		tf::Transform locRef;

		Frame &cframe = SLAMSystem.getTracker()->mCurrentFrame;
		if (SLAMSystem.getTracker()->trackingIsGood())
		{

			tf::Transform tfTcw = FramePose(&cframe);
			mTfBr->sendTransform(tf::StampedTransform(tfTcw, ros::Time(imageTime), "/ORB_SLAM/World", "/ORB_SLAM/Camera"));

	//		 Here, we use offset of external localization from the keyframe

				try {
					locRef = localizeByReference(tfTcw);
					publishPose(&locRef);
					tfOk = true;
//					lastDebugMessage = "";
				} catch (exception &e) {
//					lastDebugMessage = e.what();
					publishPose(NULL);
				}

		} else {
			publishPose (NULL);
		}

		rT2 = microsec_clock::local_time();
//		cputimeDebug = (rT2-rT1).total_microseconds() * 1e-6;

		// Debugging
		SLAMSystem.getFrameDrawer()->DrawFrame();
		cv::Mat framebufferDbg = SLAMSystem.getFrameDrawer()->getLastFrame();
		cv_bridge::CvImage bagImage;
		bagImage.image = framebufferDbg;
		bagImage.header.stamp = ros::Time(lastImageTimestamp);
		bagImage.encoding = "bgr8";
		visualDebugView.publish(bagImage.toImageMsg());
	}

private:
	tf::Transform localizeByReference (const tf::Transform &tfOrb, ORB_SLAM2::KeyFrame *kf)
	{
		lastKeyframeId = kf->mnId;

		ORB_SLAM2::KeyFrame *kOffset = SLAMSystem.getMap()->offsetKeyframe(kf, offsetKeyframe);
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


	tf::Transform rotateAxes (const tf::Transform &cPose, const float roll=0, const float pitch=0, const float yaw=0)
	{
		tf::Transform rotation;
		rotation.setOrigin (tf::Vector3 (0,0,0));
		rotation.setRotation (tf::Quaternion(roll, pitch, yaw).normalize());
		return cPose * rotation;
	}


	/*
	 * Main routine for localization by reference
	 */
	tf::Transform localizeByReference (
	    	const tf::Transform &tfOrb,
			const tf::Transform &tfOrbMap, const tf::Transform &tfOrbMapOffset,
	    	const tf::Transform &realMapPose, const tf::Transform &realMapOffset)
	{
		double offDistO = cv::norm(
			tfToCv(tfOrbMap.getOrigin()) -
			tfToCv(tfOrbMapOffset.getOrigin()));
		double offDistE = cv::norm(
			tfToCv(realMapPose.getOrigin()) -
			tfToCv(realMapOffset.getOrigin()));
		double scale = offDistE / offDistO;

		// change orientation from camera to velodyne
		tf::Transform flipAxes;
		flipAxes.setOrigin(tf::Vector3(0, 0, 0));
		flipAxes.setRotation (tf::Quaternion(M_PI/2, 0, -M_PI/2).normalize());

		tf::Transform orbRel = tfOrbMap.inverse() * tfOrb;

		tf::Transform scaledRel = orbRel;
		scaledRel.setOrigin(orbRel.getOrigin() * scale);
		scaledRel = flipAxes*scaledRel;

		tf::Transform tfResult = realMapPose * scaledRel;

		// Still need to rotate axes
		return rotateAxes (tfResult, 0, M_PI/2, M_PI/2);
	}


	tf::Transform localizeByReference(const tf::Transform &tfOrb)
	{
		float fdirx = tfOrb.getRotation().x(),
			fdiry = tfOrb.getRotation().y(),
			fdirz = tfOrb.getRotation().z(),
			fdirnorm;
		fdirnorm = sqrtf(fdirx*fdirx + fdiry*fdiry + fdirz*fdirz);
		fdirx /= fdirnorm;
		fdiry /= fdirnorm;
		fdirz /= fdirnorm;

		ORB_SLAM2::KeyFrame *kfNear = SLAMSystem.getMap()->getNearestKeyFrame(
			tfOrb.getOrigin().x(),
			tfOrb.getOrigin().y(),
			tfOrb.getOrigin().z(),
			fdirx, fdiry, fdirz);
		if (kfNear==NULL)
			throw std::out_of_range("No nearest keyframe found");

		lastKeyframeId = kfNear->mnId;
		return localizeByReference (tfOrb, kfNear);
	}


	tf::Transform localizeByReference(Frame *sframe)
	{
	//	const tf::Transform
	}


	tf::Transform localizeByReference(const tf::Transform &tfOrb, ORB_SLAM2::Map *mapsrc, const int offsetNum)
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


	tf2_msgs::TFMessage createTfMessage (const tf::Transform &srcTransform,
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


	tf::Transform KeyFramePoseToTf (KeyFrame *kf)
	{
		tf::Transform kfpose;

		cv::Mat t = kf->GetCameraCenter();
		cv::Mat orient = kf->GetRotation().t();
		vector<float> q = ORB_SLAM2::Converter::toQuaternion(orient);

		kfpose.setOrigin(tf::Vector3(t.at<float>(0), t.at<float>(1), t.at<float>(2)));
		kfpose.setRotation(tf::Quaternion(q[0], q[1], q[2], q[3]));

		return kfpose;
	}


	cv::Vec3d tfToCv (const tf::Vector3 &pos)
	{
		cv::Vec3d cvVec;
		cvVec[0] = pos.x();
		cvVec[1] = pos.y();
		cvVec[2] = pos.z();
		return cvVec;
	}


	cv::Mat tfToCv (const tf::Transform &tfsrc)
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


	void publishPose (const tf::Transform *pose)
	{
		geometry_msgs::PoseStamped gpose;
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
		}

		posePublisher.publish (gpose);
	}


	void publishPoseWithCovariance (const tf::Transform *pose)
	{
		geometry_msgs::PoseWithCovarianceStamped gpose;
		gpose.header.frame_id = externalFrameFixed;
		gpose.header.stamp = ros::Time(lastImageTimestamp);

		// XXX: check how we can set the covariance

		if (pose==NULL) {
			gpose.pose.pose.position.x =
			gpose.pose.pose.position.y =
			gpose.pose.pose.position.z = 1e15;
			gpose.pose.pose.orientation.x = 1.0;
			gpose.pose.pose.orientation.y = .0;
			gpose.pose.pose.orientation.z = .0;
			gpose.pose.pose.orientation.w = .0;
		}

		else {
			gpose.pose.pose.position.x = pose->getOrigin().x();
			gpose.pose.pose.position.y = pose->getOrigin().y();
			gpose.pose.pose.position.z = pose->getOrigin().z();
			gpose.pose.pose.orientation.x = pose->getRotation().x();
			gpose.pose.pose.orientation.y = pose->getRotation().y();
			gpose.pose.pose.orientation.z = pose->getRotation().z();
			gpose.pose.pose.orientation.w = pose->getRotation().w();

			tf::StampedTransform tfMsg;
			tfMsg.stamp_ = ros::Time(lastImageTimestamp);
			tfMsg.frame_id_ = externalFrameFixed;
			tfMsg.child_frame_id_ = externalFrameMoving;
			tfMsg.setData(*pose);
			mTfBr->sendTransform(tfMsg);
		}

		posePublisher.publish(gpose);
	}


/*
 * Variable members
 */
private:
	tf::StampedTransform extPose;

	ros::NodeHandle &rosnode;
	ORB_SLAM2::System &SLAMSystem;
	ros::Publisher posePublisher;
	tf::TransformBroadcaster *mTfBr;

	// Debug
	image_transport::Publisher visualDebugView;

	string externalFrameFixed;
	string externalFrameMoving;

	double lastImageTimestamp;
	bool gotFirstFrame;

	// Logging
	uint32_t lastKeyframeId;
	int offsetKeyframe;

public:
	image_transport::TransportHints th;
	image_transport::ImageTransport *imageBuf;
	image_transport::Subscriber imageSub;

};




int main (int argc, char *argv[])
{
//	const string mapPath = (argc==3) ? argv[2] : string();
	const string orbVocabFile (ORB_SLAM_VOCABULARY);
//	const string configFile = argv[1];

	ros::init(argc, argv, "orb_matching", ros::init_options::AnonymousName);
	ros::start();
	ros::NodeHandle nodeHandler ("~");

	string mapPath;
	nodeHandler.getParam("map_file", mapPath);

	string configFile;
	nodeHandler.getParam("configuration_file", configFile);

	ORB_SLAM2::System SLAM(orbVocabFile,
		configFile,
		ORB_SLAM2::System::MONOCULAR,
		false,
		mapPath,
		System::LOCALIZATION);

	ORB_Matcher Matcher (SLAM, nodeHandler);
	string imageTopic;
	nodeHandler.getParam("image_topic", imageTopic);
	Matcher.imageSub = Matcher.imageBuf->subscribe (imageTopic, 1, &ORB_Matcher::imageCallback, &Matcher, Matcher.th);
	cerr << "ORB Localizer ready" << endl;
	ros::spin();

	SLAM.Shutdown();
	ros::shutdown();

	return 0;
}

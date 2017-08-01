/*
 * place_recognizer.cpp
 *
 *  Created on: May 18, 2017
 *      Author: sujiwo
 */

#include <iostream>
#include <vector>
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <boost/filesystem.hpp>
#include <tf/tf.h>
#include "System.h"
#include "Map.h"
#include "Frame.h"
#include "ORBmatcher.h"
#include "MapPoint.h"
#include "PnPsolver.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include "Optimizer.h"
#include "../common.h"


using namespace std;
using namespace ORB_SLAM2;
namespace enc = sensor_msgs::image_encodings;



//struct RecognizerOutput {
//	KeyFrame *kf;
//	geometry_msgs::PoseWithCovariance keyframePose;
//	double imageTimestamp;
//};


const string orbGenericVocabFile = ORB_SLAM_VOCABULARY;

// ROS Publishers
image_transport::Publisher recognizerImageDebug;
ros::Publisher vehiclePosePub;

cv::FileStorage sysConfig;
KeyFrameDatabase *keyframeDB;
ORBVocabulary *keyVocab;
ORBextractor *orbExtractor;
ORB_SLAM2::Map *sourceMap;

// Camera Parameters
cv::Mat CameraParam;

// Distortion Coefficients
cv::Mat DistCoef;

// ROS Launch Parameters
string
	mapPath
		("/data/Meidai2017/logging/2016-12-26-13-21-10/map/orbx.map"),
	configFile
		("/data/Meidai2017/ros/etc/orb-slam2.yaml"),
	imageTopic
		("/camera1/image_raw");


string kfImageDir;
cv::Mat PostRecognition (Frame &frame, KeyFrame *kf)
{
	string imagePath = kfImageDir + "/" + std::to_string(kf->mnId) + ".jpg";
	cv::Mat kfImage = cv::imread (imagePath, CV_LOAD_IMAGE_GRAYSCALE);
	if (kfImage.empty())
		throw std::invalid_argument("KeyFrame image not found: " + std::to_string(kf->mnId));

	return kfImage;
}


// We eliminate full initialization of SLAM System
// XXX: No exception handling here
void SlamSystemPrepare (
	const string &mapFilename,
	const string &configPath,
	cv::FileStorage &sconf,
	Map **sMap,
	ORBVocabulary **vocab,
	KeyFrameDatabase **kfdb,
	ORBextractor **orbext
	)
{
	// open Configuration file
	sconf = cv::FileStorage (configPath.c_str(), cv::FileStorage::READ);

	// Vocabulary
	*vocab = new ORBVocabulary ();
	cout << endl << "Loading Custom ORB Vocabulary... " ;
	string mapVoc = mapFilename + ".voc";
	bool vocload = (*vocab)->loadFromTextFile (mapVoc);
	cout << "Vocabulary loaded!" << endl << endl;

	// Create KeyFrame Database
	*kfdb = new KeyFrameDatabase(**vocab);

	// Map
	*sMap = new Map ();
	(*sMap)->loadFromDisk(mapFilename, *kfdb, true);

	// ORB Extractor
	*orbext = new ORBextractor(
		2 * (int)sconf["ORBextractor.nFeatures"],
		(float)sconf["ORBextractor.scaleFactor"],
		(int)sconf["ORBextractor.nLevels"],
		(int)sconf["ORBextractor.iniThFAST"],
		(int)sconf["ORBextractor.minThFAST"]);

	// Camera Parameters
    float fx = sconf["Camera.fx"];
    float fy = sconf["Camera.fy"];
    float cx = sconf["Camera.cx"];
    float cy = sconf["Camera.cy"];
    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(CameraParam);

    // Distortion Coefficients
    DistCoef = cv::Mat (4,1,CV_32F);
    DistCoef.at<float>(0) = sconf["Camera.k1"];
    DistCoef.at<float>(1) = sconf["Camera.k2"];
    DistCoef.at<float>(2) = sconf["Camera.p1"];
    DistCoef.at<float>(3) = sconf["Camera.p2"];
    const float k3 = sconf["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }

}


ORB_SLAM2::Frame
monocularFrame (const cv::Mat& inputGray, const double timestamp)
{
	float mbf = 0.0, thDepth = 0.0;

	return ORB_SLAM2::Frame (inputGray, timestamp, orbExtractor, keyVocab, CameraParam, DistCoef, mbf, thDepth);
}


KeyFrame* relocalize (Frame &frame)
{
//	KeyFrameDatabase *kfDB = SLAMSystem->getKeyFrameDB();
	frame.ComputeBoW();

	vector<KeyFrame*> vpCandidateKFs
		= keyframeDB->DetectRelocalizationCandidatesSimple(&frame);

	if (vpCandidateKFs.empty())
		return NULL;

	vector<bool> vcDiscarded;
	const int nKFs = vpCandidateKFs.size();
	vcDiscarded.resize(nKFs);
	ORBmatcher matcher (0.75, true);

	vector<vector<MapPoint*> > vvpMapPointMatches;
	vvpMapPointMatches.resize(nKFs);

	vector<PnPsolver*> vpPnPsolvers;
	vpPnPsolvers.resize(nKFs);

	int nCandidates=0;

    for(int i=0; i<nKFs; i++)
    {
        KeyFrame* pKF = vpCandidateKFs[i];
        if(pKF->isBad())
        	vcDiscarded[i] = true;
        else
        {
            int nmatches = matcher.SearchByBoW (pKF, frame, vvpMapPointMatches[i]);
            if(nmatches<15)
            {
            	vcDiscarded[i] = true;
                continue;
            }
            else
            {
            	return pKF;
                PnPsolver* pSolver = new PnPsolver (frame, vvpMapPointMatches[i]);
                pSolver->SetRansacParameters(0.99,10,300,4,0.5,5.991);
                vpPnPsolvers[i] = pSolver;
                nCandidates++;
            }
        }
    }

    cout << "#Candidates: " << nCandidates << endl;

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    bool bMatch = false;
    ORBmatcher matcher2(0.9,true);
    KeyFrame *kfRef;

	while(nCandidates>0 && !bMatch)
	{
		for(int i=0; i<nKFs; i++)
		{
			if(vcDiscarded[i])
				continue;

			// Perform 5 Ransac Iterations
			vector<bool> vbInliers;
			int nInliers;
			bool bNoMore;

			PnPsolver* pSolver = vpPnPsolvers[i];
			cv::Mat Tcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

			// If Ransac reachs max. iterations discard keyframe
			if(bNoMore)
			{
				vcDiscarded[i]=true;
				nCandidates--;
			}

			// If a Camera Pose is computed, optimize
			if(!Tcw.empty())
			{
				Tcw.copyTo(frame.mTcw);

				set<MapPoint*> sFound;

				const int np = vbInliers.size();

				for(int j=0; j<np; j++)
				{
					if(vbInliers[j])
					{
						frame.mvpMapPoints[j]=vvpMapPointMatches[i][j];
						sFound.insert(vvpMapPointMatches[i][j]);
					}
					else
						frame.mvpMapPoints[j]=NULL;
				}

				int nGood = Optimizer::PoseOptimization(&frame);

				if(nGood<10)
					continue;

				for(int io =0; io<frame.N; io++)
					if(frame.mvbOutlier[io])
						frame.mvpMapPoints[io]=static_cast<MapPoint*>(NULL);

				// If few inliers, search by projection in a coarse window and optimize again
				if(nGood<50)
				{
					int nadditional =matcher2.SearchByProjection(frame, vpCandidateKFs[i],sFound,10,100);

					if(nadditional+nGood>=50)
					{
						nGood = Optimizer::PoseOptimization(&frame);

						// If many inliers but still not enough, search by projection again in a narrower window
						// the camera has been already optimized with many points
						if(nGood>30 && nGood<50)
						{
							sFound.clear();
							for(int ip =0; ip<frame.N; ip++)
								if(frame.mvpMapPoints[ip])
									sFound.insert(frame.mvpMapPoints[ip]);
							nadditional = matcher2.SearchByProjection(frame, vpCandidateKFs[i], sFound, 3, 64);

							// Final optimization
							if(nGood+nadditional>=50)
							{
								nGood = Optimizer::PoseOptimization(&frame);

								for(int io =0; io<frame.N; io++)
									if(frame.mvbOutlier[io])
										frame.mvpMapPoints[io]=NULL;
							}
						}
					}
				}


				// If the pose is supported by enough inliers stop ransacs and continue
				if(nGood>=50)
				{
					bMatch = true;
					kfRef = vpCandidateKFs[i];
					break;
				}
			}
		}
	}

	return (bMatch==true ? kfRef : NULL);
}


void imageCallback (const sensor_msgs::ImageConstPtr &imageMsg)
{
//	cout << "Imaged\n";

	cv::Mat imageGray = createImageFromRosMessage(imageMsg,
		sysConfig["Camera.WorkingResolution.Width"],
		sysConfig["Camera.WorkingResolution.Height"],
		sysConfig["Camera.ROI.x0"],
		sysConfig["Camera.ROI.y0"],
		sysConfig["Camera.ROI.width"],
		sysConfig["Camera.ROI.height"],
		true
	);
	const double imageTime = imageMsg->header.stamp.toSec();

	Frame cframe = monocularFrame (imageGray, imageTime);

//	RecognizerOutput frameRecognizerOutput;
	try {
		KeyFrame *kfFound = relocalize(cframe);
		if (kfFound==NULL)
			cout << "No KF\n";
		else {
			cout << "KF: " << kfFound->mnId << endl;
			cv::Mat renderFrame = PostRecognition(cframe, kfFound);
			sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", renderFrame).toImageMsg();
			recognizerImageDebug.publish (msg);

			// Publish KF pose
			tf::Transform keyPose = getKeyFrameExtPose(kfFound);
			geometry_msgs::PoseStamped kfPose;
			kfPose.header.stamp = imageMsg->header.stamp;
			kfPose.header.frame_id = "world";
			kfPose.pose.position.x = keyPose.getOrigin().x();
			kfPose.pose.position.y = keyPose.getOrigin().y();
			kfPose.pose.position.z = keyPose.getOrigin().z();
			kfPose.pose.orientation.x = keyPose.getRotation().x();
			kfPose.pose.orientation.y = keyPose.getRotation().y();
			kfPose.pose.orientation.z = keyPose.getRotation().z();
			kfPose.pose.orientation.w = keyPose.getRotation().w();
			vehiclePosePub.publish(kfPose);

		}
	} catch (exception &e) {
		cout << "Error in recognizer: " << e.what() << endl;
	}
}


int main (int argc, char *argv[])
{
	ros::init(argc, argv, "place_recognizer", ros::init_options::AnonymousName);
	ros::start();
	ros::NodeHandle nodeHandler ("~");

	nodeHandler.getParam ("map_file", mapPath);
	nodeHandler.getParam ("configuration_file", configFile);
	cout << "Config: " << configFile << endl;
	nodeHandler.getParam ("image_topic", imageTopic);

	SlamSystemPrepare(mapPath, configFile, sysConfig, &sourceMap, &keyVocab, &keyframeDB, &orbExtractor);

	kfImageDir = boost::filesystem::path(mapPath).parent_path().string() + "/keyframe_images";
	cout << "Image directory: " << kfImageDir << endl;

	image_transport::TransportHints th ("raw");
	image_transport::ImageTransport imageBuf (nodeHandler);
	image_transport::Subscriber imageSub = imageBuf.subscribe (imageTopic, 1, &imageCallback, th);

	// Prepare publishers
	image_transport::ImageTransport imgPub (nodeHandler);
	recognizerImageDebug = imgPub.advertise ("image", 1);
	vehiclePosePub = nodeHandler.advertise<geometry_msgs::PoseStamped> ("pose", 1);

	cout << "Place Recognizer Ready\n";

	ros::spin();

	ros::shutdown();
	delete(sourceMap);

	return 0;
}

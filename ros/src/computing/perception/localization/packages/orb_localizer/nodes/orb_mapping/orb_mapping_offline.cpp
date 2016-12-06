#include <iostream>
#include <string>
#include <algorithm>
#include <thread>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <dirent.h>
#include <sys/types.h>
#include <cstdio>
#include <unistd.h>

#include "System.h"
#include "../common.h"


using namespace std;
using namespace ORB_SLAM2;


typedef Eigen::Transform<float,3,Eigen::Affine> Transform3;

struct imagePose {
	vector<string> imagePaths;
	vector<double> timestamps;
	vector<Transform3> poses;
};



void loadData (const string &dataDir, imagePose& dataset)
{
	DIR *fd = opendir(dataDir.c_str());
	if (fd==NULL) return;
//		throw exception ("Invalid directory");
	const string poseFn = dataDir + "/pose.csv";
	FILE *posefd = fopen(poseFn.c_str(), "r");

	dirent *de;
	while (true) {
		de = readdir (fd);
		if (de==NULL) break;
		string fname (de->d_name);

		// check if png
		string ext;
		try {
			ext = fname.substr(fname.size()-3);
		} catch (out_of_range &e) {
			continue;
		}
		if (ext != "png")
			continue;

		dataset.imagePaths.push_back(dataDir + '/' + fname);
	}

	closedir(fd);
	std::sort(dataset.imagePaths.begin(), dataset.imagePaths.end());

	// XXX: load pose
	while (!feof(posefd)) {
		Transform3 cpose;
		float row[8];
		float timestamp;
		int p = fscanf (posefd, "%f %f %f %f %f %f %f %f",
				&row[0],
				&row[1],
				&row[2],
				&row[3],
				&row[4],
				&row[5],
				&row[6],
				&row[7]);
		timestamp = row[0];
		Eigen::Quaternionf qf (row[7], row[4], row[5], row[6]);
		cpose = qf;
		cpose.translation().x() = row[1];
		cpose.translation().y() = row[2];
		cpose.translation().z() = row[3];

		dataset.timestamps.push_back(timestamp);
		dataset.poses.push_back(cpose);
	}

	fclose (posefd);
}


int main (int argc, char *argv[])
{
	const string orbVocabFile (ORB_SLAM_VOCABULARY);
	string configFile = argv[1];
	string mapPath = argv[2];
	string dataDir = argv[3];

	imagePose mDataSet;

	loadData (dataDir, mDataSet);

	ORB_SLAM2::System SLAM(orbVocabFile,
		configFile,
		ORB_SLAM2::System::MONOCULAR,
		true,
		mapPath,
		System::MAPPING, true);

	double fx2, fy2, cx2, cy2;
	recomputeNewCameraParameter (
		(double)SLAM.fsSettings["Camera.fx"],
		(double)SLAM.fsSettings["Camera.fy"],
		(double)SLAM.fsSettings["Camera.cx"],
		(double)SLAM.fsSettings["Camera.cy"],
		fx2, fy2, cx2, cy2,
		1920, 1440,
		(int)SLAM.fsSettings["Camera.WorkingResolution.Width"],
		(int)SLAM.fsSettings["Camera.WorkingResolution.Height"]);
	// send camera parameters to tracker
	SLAM.getTracker()->ChangeCalibration (fx2, fy2, cx2, cy2);


	for (int i=0; i<mDataSet.poses.size(); i++) {

		const string &path = mDataSet.imagePaths[i];
		Transform3 pose = mDataSet.poses[i];
		double timestamp = mDataSet.timestamps[i];

		cv::Mat image = cv::imread (path);
		// Processing before sending image to tracker
		// Do Resizing and cropping here
		cv::resize(image, image,
			cv::Size(
				(int)SLAM.fsSettings["Camera.WorkingResolution.Width"],
				(int)SLAM.fsSettings["Camera.WorkingResolution.Height"]
			));
		image = image(
			cv::Rect(
				(int)SLAM.fsSettings["Camera.ROI.x0"],
				(int)SLAM.fsSettings["Camera.ROI.y0"],
				(int)SLAM.fsSettings["Camera.ROI.width"],
				(int)SLAM.fsSettings["Camera.ROI.height"]
			)).clone();

		{
			Transform3 &p = mDataSet.poses[i];
			unique_lock <mutex> lock (ORB_SLAM2::KeyFrame::extPoseMutex);
			ORB_SLAM2::KeyFrame::extEgoPosition = cv::Mat (3,1,CV_64F);
			ORB_SLAM2::KeyFrame::extEgoPosition.at<double>(0) = p.translation().x();
			ORB_SLAM2::KeyFrame::extEgoPosition.at<double>(1) = p.translation().y();
			ORB_SLAM2::KeyFrame::extEgoPosition.at<double>(2) = p.translation().z();

			ORB_SLAM2::KeyFrame::extEgoOrientation = cv::Mat (4,1,CV_64F);
			Eigen::Quaternionf rp (p.rotation().matrix());
			ORB_SLAM2::KeyFrame::extEgoOrientation.at<double>(0) = rp.x();
			ORB_SLAM2::KeyFrame::extEgoOrientation.at<double>(1) = rp.y();
			ORB_SLAM2::KeyFrame::extEgoOrientation.at<double>(2) = rp.z();
			ORB_SLAM2::KeyFrame::extEgoOrientation.at<double>(3) = rp.w();
//			cout << ORB_SLAM2::KeyFrame::extEgoOrientation << endl;
		}

		SLAM.TrackMonocular (image, timestamp);
//		usleep (2e5);

	}


	SLAM.Shutdown();
	return 0;
}

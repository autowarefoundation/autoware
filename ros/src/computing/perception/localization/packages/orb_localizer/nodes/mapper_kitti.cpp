/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<iomanip>

#include<opencv2/core/core.hpp>

#include"System.h"
#include "tf/tf.h"
#include "utils.h"

using namespace std;

void LoadImageNames(const string &strSequence, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

void LoadPoses (const string &posePath, vector<tf::Transform> &poses);

void putPose (const tf::Transform &pose);


int main(int argc, char **argv)
{
	if(argc != 5)
	{
		cerr << endl << "Usage: mapper_kitti path_to_settings path_to_map path_to_dataset sequence_number\n" << endl;
		return 1;
	}

	// Retrieve paths to images and poses
	const string kittiDataPath = argv[3];
	const string sequenceNumber = argv[4];
	const string poseGroundTruth = kittiDataPath + "/poses/" + sequenceNumber + ".txt";

	vector<string> vstrImageFilenames;
	vector<double> vTimestamps;
	vector<tf::Transform> poseGt;
	LoadImageNames(kittiDataPath+"/sequences/"+sequenceNumber, vstrImageFilenames, vTimestamps);
	LoadPoses(poseGroundTruth, poseGt);

	int nImages = vstrImageFilenames.size();
	const string &mapPath = argv[2];

	// Create SLAM system. It initializes all system threads and gets ready to process frames.
	ORB_SLAM2::System SLAM (ORB_SLAM_VOCABULARY,
		argv[1],
		ORB_SLAM2::System::MONOCULAR,
		true,
		mapPath,
		ORB_SLAM2::System::MAPPING);
	//    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

	// Vector for tracking time statistics
	vector<float> vTimesTrack;
	vTimesTrack.resize(nImages);

	cout << endl << "-------" << endl;
	cout << "Start processing sequence ..." << endl;
	cout << "Images in the sequence: " << nImages << endl << endl;

	// Main loop
	cv::Mat im;
	for(int ni=0; ni<nImages; ni++)
	{
		// Read image from file
		im = cv::imread(vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
		double tframe = vTimestamps[ni];
		tf::Transform &currentPose = poseGt[ni];

		if(im.empty())
		{
			cerr << endl << "Failed to load image at: " << vstrImageFilenames[ni] << endl;
			return 1;
		}

		std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

		cout << "Frame #: " << ni << endl;
		// Pass the image to the SLAM system
//		putPose (currentPose);
		SLAM.TrackMonocular (im, tframe,
			cv::Vec3f (currentPose.getOrigin().x(),
				currentPose.getOrigin().y(),
				currentPose.getOrigin().z()),
			cv::Vec4f (currentPose.getRotation().x(),
				currentPose.getRotation().y(),
				currentPose.getRotation().z(),
				currentPose.getRotation().w())
		);

		std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

		double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

		vTimesTrack[ni]=ttrack;

		// Wait to load the next frame
		cin.ignore();
		/*double T=0;
		if(ni<nImages-1)
			T = vTimestamps[ni+1]-tframe;
		else if(ni>0)
			T = tframe-vTimestamps[ni-1];

		if(ttrack<T)
			usleep((T-ttrack)*1e6);*/
	}

	// Stop all threads
	SLAM.Shutdown();

	// Tracking time statistics
	sort(vTimesTrack.begin(),vTimesTrack.end());
	float totaltime = 0;
	for(int ni=0; ni<nImages; ni++)
	{
		totaltime+=vTimesTrack[ni];
	}
	cout << "-------" << endl << endl;
	cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
	cout << "mean tracking time: " << totaltime/nImages << endl;

	// Save camera trajectory
	SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

	return 0;
}


void LoadImageNames(const string &strPathToSequence, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    string strPrefixLeft = strPathToSequence + "/image_0/";

    const int nTimes = vTimestamps.size();
    vstrImageFilenames.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageFilenames[i] = strPrefixLeft + ss.str() + ".png";
    }
}


#include <cstdio>
void LoadPoses (const string &posePath, vector<tf::Transform> &poses)
{
	FILE *fd = fopen (posePath.c_str(), "r");
	float row[12];

	while (true) {
		int p = fscanf (fd, "%f %f %f %f %f %f %f %f %f %f %f %f",
			&row[0],
			&row[1],
			&row[2],
			&row[3],
			&row[4],
			&row[5],
			&row[6],
			&row[7],
			&row[8],
			&row[9],
			&row[10],
			&row[11]);
		if (p==EOF)
			break;

		tf::Transform ctf;
		ctf.setOrigin (tf::Vector3(row[3], row[7], row[11]));
		ctf.setBasis (tf::Matrix3x3(
			row[0], row[1], row[2],
			row[4], row[5], row[6],
			row[8], row[9], row[10]
		));
		poses.push_back(ctf);
	}

	fclose (fd);
}


void putPose (const tf::Transform &pose)
{
	tfToCV (pose, ORB_SLAM2::KeyFrame::extEgoPosition, ORB_SLAM2::KeyFrame::extEgoOrientation);
}

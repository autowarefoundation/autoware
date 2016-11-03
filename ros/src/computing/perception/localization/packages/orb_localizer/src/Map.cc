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

#include "Map.h"
#include "KeyFrame.h"
#include "MapPoint.h"
#include "KeyFrameDatabase.h"
#include <mutex>
#include <cstdio>
#include <exception>
#include <string>

#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include "MapObjectSerialization.h"


using std::string;


namespace ORB_SLAM2
{

Map::Map():
	mnMaxKFid(0),
	mbMapUpdated(false)
{
}

void Map::AddKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.insert(pKF);
    if(pKF->mnId>mnMaxKFid)
        mnMaxKFid=pKF->mnId;
}

void Map::AddMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.insert(pMP);
}

void Map::EraseMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.erase(pMP);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::EraseKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.erase(pKF);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
    unique_lock<mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

vector<KeyFrame*> Map::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}

vector<MapPoint*> Map::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
}

long unsigned int Map::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspMapPoints.size();
}

long unsigned int Map::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspKeyFrames.size();
}

vector<MapPoint*> Map::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}

long unsigned int Map::GetMaxKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFid;
}

void Map::clear()
{
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
        delete *sit;

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
        delete *sit;

    mspMapPoints.clear();
    mspKeyFrames.clear();
    mnMaxKFid = 0;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
}


KeyFrame* Map::offsetKeyframe (KeyFrame* kfSrc, int offset)
{
	try {
		int p = kfMapSortedId.at(kfSrc);
		p -= offset;
		return kfListSorted.at(p);
	} catch (...) {return NULL;}
}


const char *signature = "ORBSLAM";


void Map::saveToDisk(const string &filename, KeyFrameDatabase *keyframeDatabase)
{
	MapFileHeader header;
	memcpy (header.signature, signature, sizeof(header.signature));
	header.numOfKeyFrame = this->mspKeyFrames.size();
	header.numOfMapPoint = this->mspMapPoints.size();
	header.numOfReferencePoint = this->mvpReferenceMapPoints.size();

	fstream mapFileFd;
	mapFileFd.open(filename.c_str(), fstream::out | fstream::trunc);
	if (!mapFileFd.is_open())
		throw MapFileException();
	mapFileFd.write ((const char*)&header, sizeof(header));

	cout << "Header written: " << header.numOfKeyFrame << " keyframes, " << header.numOfMapPoint << " points" << endl;

	boost::archive::binary_oarchive mapArchive (mapFileFd);

	int p = 0;
	for (set<KeyFrame*>::const_iterator kfit=mspKeyFrames.begin(); kfit!=mspKeyFrames.end(); kfit++) {
		const KeyFrame *kf = *kfit;
		if (kf==NULL) {
			cerr << endl << "NULL KF found" << endl;
			continue;
		}
		mapArchive << *kf;
		cout << "Keyframes: " << ++p << "/" << mspKeyFrames.size() << "\r";
	}
	cout << endl;

	for (set<MapPoint*>::iterator mpit=mspMapPoints.begin(); mpit!=mspMapPoints.end(); mpit++) {
		const MapPoint *mp = *mpit;
		if (mp==NULL) {
			cerr << endl << "NULL MP found" << endl;
			continue;
		}
		mapArchive << *mp;
	}

	vector<idtype> vmvpReferenceMapPoints = createIdList (mvpReferenceMapPoints);
	mapArchive << vmvpReferenceMapPoints;

	mapArchive << *keyframeDatabase;

	mapFileFd.close();
}


struct _keyframeTimestampSortComparator {
	bool operator() (KeyFrame *kf1, KeyFrame *kf2)
	{ return kf1->mTimeStamp < kf2->mTimeStamp; }
} keyframeTimestampSortComparator;


#define MapOctreeResolution 1.0

void Map::loadFromDisk(const string &filename, KeyFrameDatabase *kfMemDb)
{
	MapFileHeader header;

	cout << "Opening " << filename << " ...\n";
	fstream mapFileFd;
	mapFileFd.open (filename.c_str(), fstream::in);
	if (!mapFileFd.is_open())
		throw BadMapFile();
	mapFileFd.read ((char*)&header, sizeof(header));

	if (strcmp(header.signature, signature) !=0)
		throw BadMapFile();
	cout << "Keyframes: " << header.numOfKeyFrame << ", MapPoint: " << header.numOfMapPoint << endl;

	boost::archive::binary_iarchive mapArchive (mapFileFd);

	for (unsigned int p=0; p<header.numOfKeyFrame; p++) {
		KeyFrame *kf = new KeyFrame;
		mapArchive >> *kf;
		if (!kf->isBad())
			mspKeyFrames.insert (kf);
		kfListSorted.push_back(kf);

		// XXX: Need to increase KeyFrame::nNextId
		// Also, adjust Frame::nNextId
		if (kf->mnId > KeyFrame::nNextId) {
			KeyFrame::nNextId = kf->mnId + 2;
			Frame::nNextId = kf->mnId + 3;
		}

	}

	std::sort(kfListSorted.begin(), kfListSorted.end(), keyframeTimestampSortComparator);
	for (unsigned int p=0; p<kfListSorted.size(); p++) {
		KeyFrame *kf = kfListSorted[p];
		kfMapSortedId[kf] = p;
	}

	for (unsigned int p=0; p<header.numOfMapPoint; p++) {
		try {

			MapPoint *mp = new MapPoint;
			mapArchive >> *mp;
			// Only insert if mapPoint has reference keyframe
			if (mp->mpRefKF != NULL)
				mspMapPoints.insert (mp);
			else
				delete (mp);
			if (mp->mnId > MapPoint::nNextId) {
				MapPoint::nNextId = mp->mnId + 2;
			}

		} catch (boost::archive::archive_exception &ae) {
			cout << "Archive exception at " << p << ": " << ae.code << endl;
		} catch (std::exception &e) {
			cout << "Unknown exception at " << p << ": " << e.what() << endl;
		}
	}

	// Set KeyFrame::nNextId and Frame::nNextId as next largest

	if (kfMemDb==NULL)
		return;

	for (set<KeyFrame*>::iterator kfset=mspKeyFrames.begin(); kfset!=mspKeyFrames.end(); kfset++) {
		(*kfset)->fixConnections (this, kfMemDb);
	}

	for (set<MapPoint*>::iterator mpset=mspMapPoints.begin(); mpset!=mspMapPoints.end(); mpset++) {
		(*mpset)->fixConnections (this);
	}

	vector<idtype> vmvpReferenceMapPoints;
	mapArchive >> vmvpReferenceMapPoints;
	mvpReferenceMapPoints = createObjectList<MapPoint> (vmvpReferenceMapPoints);

	mapArchive >> *kfMemDb;

	mapFileFd.close();
	mbMapUpdated = true;
	cout << "Done restoring map" << endl;

	/* Point Cloud Reconstruction */
	kfCloud = pcl::PointCloud<KeyFramePt>::Ptr (new pcl::PointCloud<KeyFramePt>);
	kfCloud->width = mspKeyFrames.size();
	kfCloud->height = 1;
	kfCloud->resize(kfCloud->width);
	int p=0;
	for (set<KeyFrame*>::iterator kfi=mspKeyFrames.begin(); kfi!=mspKeyFrames.end(); kfi++) {
		KeyFrame *kf = *kfi;
		cv::Mat pos = kf->GetCameraCenter();
		kfCloud->at(p).x = pos.at<float>(0);
		kfCloud->at(p).y = pos.at<float>(1);
		kfCloud->at(p).z = pos.at<float>(2);
		kfCloud->at(p).kf = kf;
		p++;
	}
	kfOctree = pcl::octree::OctreePointCloudSearch<KeyFramePt>::Ptr (new pcl::octree::OctreePointCloudSearch<KeyFramePt> (MapOctreeResolution));
	kfOctree->setInputCloud(kfCloud);
	kfOctree->addPointsFromInputCloud();
//	cout << "Done restoring Octree" << endl;
}


// we expect direction vector has been normalized,
// as returned by Frame::getDirectionVector()
KeyFrame* Map::getNearestKeyFrame (const float &x, const float &y, const float &z,
	const float fdir_x, const float fdir_y, const float fdir_z)
{
	KeyFramePt queryPoint;
	queryPoint.x = x, queryPoint.y = y, queryPoint.z = z;

	const int k = 10;
	vector<int> idcs;
	vector<float> sqrDist;
	idcs.resize(k);
	sqrDist.resize(k);

	int r = kfOctree->nearestKSearch(queryPoint, k, idcs, sqrDist);
	if (r==0)
		return NULL;

	for (auto ip: idcs) {
		float dirx, diry, dirz, cosT;
		KeyFrame *ckf = kfCloud->at(ip).kf;
		ckf->getDirectionVector(dirx, diry, dirz);
		cosT = dirx*fdir_x + diry*fdir_y + dirz*fdir_z;
		if (cosT <= 0.86)
			continue;
		else
			return ckf;
	}

	return NULL;
//	KeyFrame *kfn = kfCloud->at(idcs[0]).kf;
//	return kfn;
}


} //namespace ORB_SLAM

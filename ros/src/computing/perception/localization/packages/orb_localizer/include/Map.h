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

#ifndef MAP_H
#define MAP_H

#include <set>
#include <string>
#include <opencv2/opencv.hpp>
#include <exception>
#include <vector>
#include <map>
#include <mutex>

// For Fast keyframe search
#define PCL_NO_PRECOMPILE
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/impl/octree_search.hpp>


namespace ORB_SLAM2
{

class MapPoint;
class KeyFrame;
class KeyFrameDatabase;


/*
 * Warning: please do NOT modify this point structure
 * We have observed significant performance reduction.
 */
struct KeyFramePt {
	PCL_ADD_POINT4D;
	ORB_SLAM2::KeyFrame *kf;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16 ;


class Map
{
public:
    Map();

    void AddKeyFrame(KeyFrame* pKF);
    void AddMapPoint(MapPoint* pMP);
    void EraseMapPoint(MapPoint* pMP);
    void EraseKeyFrame(KeyFrame* pKF);
    void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs);

    std::vector<KeyFrame*> GetAllKeyFrames();
    std::vector<MapPoint*> GetAllMapPoints();
    std::vector<MapPoint*> GetReferenceMapPoints();

    long unsigned int MapPointsInMap();
    long unsigned  KeyFramesInMap();

    long unsigned int GetMaxKFid();

    void clear();

    std::vector<KeyFrame*> mvpKeyFrameOrigins;

    std::mutex mMutexMapUpdate;

    // This avoid that two points are created simultaneously in separate threads (id conflict)
    std::mutex mMutexPointCreation;


// Map Storage Handlers
    class BadMapFile : public std::exception
	{};
    class MapFileException : public std::exception
	{};

    void saveToDisk (const std::string &filename, KeyFrameDatabase *kfMemDb);
    void loadFromDisk (const std::string &filename, KeyFrameDatabase *kfMemDb=NULL);

	struct MapFileHeader {
		char signature[7];
		long unsigned int
			numOfKeyFrame,
			numOfMapPoint,
			numOfReferencePoint;
	};

	KeyFrame* getNearestKeyFrame (const float &x, const float &y, const float &z, const float fdir_x, const float fdir_y, const float fdir_z);
	KeyFrame* offsetKeyframe (KeyFrame* kfSrc, int offset);
//	KeyFrame* offsetKeyframe (KeyFrame* kfSrc, float offset);

	// These are used for augmented localization
	std::vector<KeyFrame*> kfListSorted;
	std::map<KeyFrame*, int> kfMapSortedId;

	bool mbMapUpdated;

protected:
    std::set<MapPoint*> mspMapPoints;
    std::set<KeyFrame*> mspKeyFrames;

    std::vector<MapPoint*> mvpReferenceMapPoints;

    long unsigned int mnMaxKFid;

    std::mutex mMutexMap;

    pcl::PointCloud<KeyFramePt>::Ptr kfCloud;
    pcl::octree::OctreePointCloudSearch<KeyFramePt>::Ptr kfOctree;

};

} //namespace ORB_SLAM

#endif // MAP_H

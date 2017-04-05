/*
 * MapObjectSerialization.h
 *
 *  Created on: Nov 15, 2015
 *      Author: sujiwo
 */

#ifndef INCLUDE_MAPOBJECTSERIALIZATION_H_
#define INCLUDE_MAPOBJECTSERIALIZATION_H_


#include <set>
#include <vector>
#include <list>
#include <map>
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/split_member.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/map.hpp>
#include "cvmat_serialization.h"
#include <boost/serialization/base_object.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>



#include "KeyFrame.h"
#include "MapPoint.h"
#include "KeyFrameDatabase.h"



using std::set;
using std::vector;
using std::map;
using std::list;
using ORB_SLAM2::KeyFrame;
using ORB_SLAM2::MapPoint;
using ORB_SLAM2::KeyFrameDatabase;
using std::out_of_range;


// comment this if you don't want to save images along with keyframes
//#define MAP_SAVE_IMAGE 1


/*
 * createIdList functions convert list of map objects (in list, set, vector and map) into list of ID
 * createObjectList functions do their reverse
 */

template<typename MapObject>
set<idtype> createIdList(const set<MapObject*> &mapObjectList)
{
	typedef typename set<MapObject*>::const_iterator ciMapObj;
	set<idtype> idList;

	for (ciMapObj it=mapObjectList.begin(); it!=mapObjectList.end(); it++) {
		MapObject *obj = *it;
		if (obj==NULL) continue;
		idList.insert (obj->mnId);
	}

	return idList;
}


template<typename MapObject>
set<MapObject*> createObjectList (const set<idtype> &mapIdList)
{
	set<MapObject*> objSet;

	for (set<idtype>::const_iterator it=mapIdList.begin(); it!=mapIdList.end(); it++) {
		try {
			MapObject *obj = MapObject::objectListLookup.at (*it);
			objSet.insert (obj);
		} catch (out_of_range &e) {
			continue;
		}
	}

	return objSet;
}


template<typename MapObject>
vector<idtype> createIdList(const vector<MapObject*> &mapObjectList)
{
	vector<idtype> idList = vector<idtype> (mapObjectList.size(), static_cast<idtype> (-1));

	int p = 0;
	for (typename vector<MapObject*>::const_iterator it=mapObjectList.begin(); it!=mapObjectList.end(); it++) {
		MapObject *obj = *it;
		if (obj != NULL)
			idList[p] = obj->mnId;
		p++;
	}

	return idList;
}


template<typename MapObject>
vector<MapObject*> createObjectList (const vector<idtype> &mapIdList)
{
	vector<MapObject*> objVect = vector<MapObject*> (mapIdList.size(), static_cast<MapObject*> (NULL));

	int p = 0;
	for (vector<idtype>::const_iterator it=mapIdList.begin(); it!=mapIdList.end(); it++) {
		MapObject *obj;
		try {
			obj = MapObject::objectListLookup.at (*it);
		} catch (out_of_range &e) {
			obj = NULL;
		}
		objVect[p] = obj;
		p++;
	}

	return objVect;
}


template<typename MapObject>
vector<MapObject*> purgeNull (const vector<MapObject*> &mapObjectList)
{
	vector<MapObject*> newObjectVector;

	for (typename vector<MapObject*>::const_iterator it=mapObjectList.begin(); it!=mapObjectList.end(); it++) {
		MapObject *obj = *it;
		if (obj==NULL)
			continue;
		newObjectVector.push_back(obj);
	}

	return newObjectVector;
}


template<typename MapObject>
list<idtype> createIdList(const list<MapObject*> &mapObjectList)
{
	typedef typename list<MapObject*>::const_iterator ciMapObj;
	list<idtype> idList;

	for (ciMapObj it=mapObjectList.begin(); it!=mapObjectList.end(); it++) {
		MapObject *obj = *it;
		if (obj==NULL) continue;
		idList.push_back (obj->mnId);
	}

	return idList;
}


template<typename MapObject>
list<MapObject*> createObjectList (const list<idtype> &mapIdList)
{
	list<MapObject*> objVect;

	for (list<idtype>::const_iterator it=mapIdList.begin(); it!=mapIdList.end(); it++) {
		try {
			MapObject *obj = MapObject::objectListLookup.at (*it);
			objVect.push_back (obj);
		} catch (out_of_range &e) {
			continue;
		}
	}

	return objVect;
}


template<typename MapObject, typename W>
map<idtype, W> createIdList(const map<MapObject*, W> &mapObjectList)
{
	typedef typename map<MapObject*, W>::const_iterator ciMapObj;
	map<idtype, W> idList;

	for (ciMapObj it=mapObjectList.begin(); it!=mapObjectList.end(); it++) {
		MapObject *obj = it->first;
		if (obj==NULL) continue;
		idList[obj->mnId] = it->second;
	}

	return idList;
}


template<typename MapObject, typename W>
map<MapObject*, W> createObjectList (const map<idtype, W> &mapIdList)
{
	map<MapObject*, W> objMap;
	typedef typename map<idtype, W>::const_iterator ci_t;

	for (ci_t it=mapIdList.begin(); it!=mapIdList.end(); it++) {
		try {
			MapObject *obj = MapObject::objectListLookup.at (it->first);
			if (obj==NULL)
				continue;
			objMap [obj] = it->second;
		} catch (out_of_range &e) {
			continue;
		}
	}
	return objMap;
}



template<typename T>
bool debugSerialization (const T &src)
{
	stringstream ssOut;
	boost::archive::binary_oarchive outarc (ssOut);
	outarc << src;
	string outStr = ssOut.str();

	string outStrX = outStr;
	stringstream ssIn (outStr);
	boost::archive::binary_iarchive inarc (ssIn);
	T tgt;
	inarc >> tgt;

	return true;
}



BOOST_SERIALIZATION_SPLIT_FREE (DBoW2::BowVector);
BOOST_SERIALIZATION_SPLIT_FREE (DBoW2::FeatureVector);
BOOST_SERIALIZATION_SPLIT_FREE (ORB_SLAM2::KeyFrame);
BOOST_SERIALIZATION_SPLIT_FREE (ORB_SLAM2::MapPoint);
BOOST_SERIALIZATION_SPLIT_FREE (ORB_SLAM2::KeyFrameDatabase);


namespace boost {
namespace serialization {


// XXX: Need to modify code for ORB_SLAM2
template <class Archive>
void save (Archive & ar, const ORB_SLAM2::KeyFrame &keyframe, const unsigned int version)
{
	ar &
		keyframe.mnId &
		keyframe.mnFrameId &
		keyframe.mTimeStamp &
		keyframe.mnGridCols &
		keyframe.mnGridRows &
		keyframe.mfGridElementWidthInv &
		keyframe.mfGridElementHeightInv &
		keyframe.mnTrackReferenceForFrame &
		keyframe.mnFuseTargetForKF &
		keyframe.mnBALocalForKF &
		keyframe.mnBAFixedForKF &
		keyframe.mnLoopQuery &
		keyframe.mnLoopWords &
		keyframe.mLoopScore &
		keyframe.mnRelocQuery &
		keyframe.mnRelocWords &
		keyframe.mRelocScore;
	ar &
		keyframe.mTcwGBA &
		keyframe.mTcwBefGBA &
		keyframe.mnBAGlobalForKF;
	ar &
		keyframe.fx & keyframe.fy & keyframe.cx & keyframe.cy &
		keyframe.invfx & keyframe.invfy &
		keyframe.mbf & keyframe.mb &
		keyframe.mThDepth;
	ar &
		keyframe.N;
	ar &
		keyframe.mvKeys &
		keyframe.mvKeysUn &
		keyframe.mvuRight &
		keyframe.mvDepth &
		keyframe.mDescriptors;
	ar &
		keyframe.mBowVec &
		keyframe.mFeatVec;
	ar &
		keyframe.mTcp;
	ar &
		keyframe.mnScaleLevels &
		keyframe.mfScaleFactor &
		keyframe.mfLogScaleFactor &
		keyframe.mvScaleFactors &
		keyframe.mvLevelSigma2 &
		keyframe.mvInvLevelSigma2;
	ar &
		keyframe.mnMinX &
		keyframe.mnMinY &
		keyframe.mnMaxX &
		keyframe.mnMaxY &
		keyframe.mK;

#ifdef MAP_SAVE_IMAGE
	ar & keyframe.im;
#endif

	ar &
		keyframe.Tcw &
		keyframe.Twc &
		keyframe.Ow &
		keyframe.Cw;

	vector<idtype> mapPointIdList = createIdList(keyframe.mvpMapPoints);
	ar & mapPointIdList;

	ar &
		keyframe.mGrid;

	map<idtype,int> imConnectedKeyFrameWeights = createIdList(keyframe.mConnectedKeyFrameWeights);
	ar & imConnectedKeyFrameWeights;

	vector<idtype> vmvpOrderedConnectedKeyFrames = createIdList (keyframe.mvpOrderedConnectedKeyFrames);
	ar & vmvpOrderedConnectedKeyFrames;

	ar & keyframe.mvOrderedWeights;
	ar & keyframe.mbFirstConnection;

	int parentId = (keyframe.mpParent==NULL) ? -1 : keyframe.mpParent->mnId;
	ar & parentId;

	set<idtype> vmspChildrens = createIdList (keyframe.mspChildrens);
	ar & vmspChildrens;

	set<idtype> vmspLoopEdges = createIdList (keyframe.mspLoopEdges);
	ar & vmspLoopEdges;

	ar &
		keyframe.mbNotErase &
		keyframe.mbToBeErased &
		keyframe.mbBad &
		keyframe.mHalfBaseline;

//	Enable these when you're sure external localization does work
	ar & keyframe.extPosition & keyframe.extOrientation;
}


// XXX: Need to modify code for ORB_SLAM2
template <class Archive>
void load (Archive & ar, ORB_SLAM2::KeyFrame &keyframe, const unsigned int version)
{
	ar &
		keyframe.mnId &
		keyframe.mnFrameId &
		keyframe.mTimeStamp &
		keyframe.mnGridCols &
		keyframe.mnGridRows &
		keyframe.mfGridElementWidthInv &
		keyframe.mfGridElementHeightInv &
		keyframe.mnTrackReferenceForFrame &
		keyframe.mnFuseTargetForKF &
		keyframe.mnBALocalForKF &
		keyframe.mnBAFixedForKF &
		keyframe.mnLoopQuery &
		keyframe.mnLoopWords &
		keyframe.mLoopScore &
		keyframe.mnRelocQuery &
		keyframe.mnRelocWords &
		keyframe.mRelocScore;
	ar &
		keyframe.mTcwGBA &
		keyframe.mTcwBefGBA &
		keyframe.mnBAGlobalForKF;
	ar &
		keyframe.fx & keyframe.fy & keyframe.cx & keyframe.cy &
		keyframe.invfx & keyframe.invfy &
		keyframe.mbf & keyframe.mb &
		keyframe.mThDepth;
	ar &
		keyframe.N;
	ar &
		keyframe.mvKeys &
		keyframe.mvKeysUn &
		keyframe.mvuRight &
		keyframe.mvDepth &
		keyframe.mDescriptors;
	ar &
		keyframe.mBowVec &
		keyframe.mFeatVec;
	ar &
		keyframe.mTcp;
	ar &
		keyframe.mnScaleLevels &
		keyframe.mfScaleFactor &
		keyframe.mfLogScaleFactor &
		keyframe.mvScaleFactors &
		keyframe.mvLevelSigma2 &
		keyframe.mvInvLevelSigma2;
	ar &
		keyframe.mnMinX &
		keyframe.mnMinY &
		keyframe.mnMaxX &
		keyframe.mnMaxY &
		keyframe.mK;

#ifdef MAP_SAVE_IMAGE
	ar & keyframe.im;
#endif

	ar &
		keyframe.Tcw &
		keyframe.Twc &
		keyframe.Ow &
		keyframe.Cw;

	ar & keyframe._mapPointIdList;

	ar &
		keyframe.mGrid;

	ar & keyframe._imConnectedKeyFrameWeights;

	ar & keyframe._vmvpOrderedConnectedKeyFrames;

	ar & keyframe.mvOrderedWeights;
	ar & keyframe.mbFirstConnection;

	ar & keyframe._parentId;

	ar & keyframe._vmspChildrens;

	ar & keyframe._vmspLoopEdges;

	ar &
		keyframe.mbNotErase &
		keyframe.mbToBeErased &
		keyframe.mbBad &
		keyframe.mHalfBaseline;

	KeyFrame::objectListLookup[keyframe.mnId] = &keyframe;

//	Enable these when you're sure external localization does work
	ar & keyframe.extPosition & keyframe.extOrientation;
//	keyframe.extPosition
}


template <class Archive>
void save (Archive & ar, const MapPoint &mapPoint, const unsigned int version)
{
	ar &
		mapPoint.mnId &
		mapPoint.mnFirstKFid &
		mapPoint.mnFirstFrame &
		mapPoint.nObs &
		mapPoint.mTrackProjX &
		mapPoint.mTrackProjY &
		mapPoint.mTrackProjXR;
//	ar & mapPoint.mbTrackInView;
	ar & mapPoint.mnTrackScaleLevel &
		mapPoint.mTrackViewCos &
		mapPoint.mnTrackReferenceForFrame &
		mapPoint.mnLastFrameSeen &
		mapPoint.mnBALocalForKF &
		mapPoint.mnFuseCandidateForKF &
		mapPoint.mnLoopPointForKF &
		mapPoint.mnCorrectedByKF &
		mapPoint.mnCorrectedReference;
	ar &
		mapPoint.mPosGBA &
		mapPoint.mnBAGlobalForKF;

	ar &
		mapPoint.mWorldPos;

	map<idtype,size_t> kfObservation = createIdList (mapPoint.mObservations);
	ar & kfObservation;

	ar &
		mapPoint.mNormalVector &
		mapPoint.mDescriptor;

	int refKfId = (mapPoint.mpRefKF==NULL ? -1 : mapPoint.mpRefKF->mnId);
	ar & refKfId;

	ar &
		mapPoint.mnVisible &
		mapPoint.mnFound &
		mapPoint.mbBad;

	int _mpReplaced = (mapPoint.mpReplaced==NULL ? -1 : mapPoint.mpReplaced->mnId);
	ar & _mpReplaced;

	ar &
		mapPoint.mfMinDistance &
		mapPoint.mfMaxDistance;
}


template <class Archive>
void load (Archive & ar, MapPoint &mapPoint, const unsigned int version)
{
	ar &
		mapPoint.mnId &
		mapPoint.mnFirstKFid &
		mapPoint.mnFirstFrame &
		mapPoint.nObs &
		mapPoint.mTrackProjX &
		mapPoint.mTrackProjY &
		mapPoint.mTrackProjXR;
//	ar & mapPoint.mbTrackInView;
	mapPoint.mbTrackInView = false;
	ar & mapPoint.mnTrackScaleLevel &
		mapPoint.mTrackViewCos &
		mapPoint.mnTrackReferenceForFrame &
		mapPoint.mnLastFrameSeen &
		mapPoint.mnBALocalForKF &
		mapPoint.mnFuseCandidateForKF &
		mapPoint.mnLoopPointForKF &
		mapPoint.mnCorrectedByKF &
		mapPoint.mnCorrectedReference;
	ar &
		mapPoint.mPosGBA &
		mapPoint.mnBAGlobalForKF;

	ar &
		mapPoint.mWorldPos;

	map<idtype,size_t> kfObservation;
	ar & kfObservation;

	ar &
		mapPoint.mNormalVector &
		mapPoint.mDescriptor;

	int refKfId;
	ar & refKfId;

	ar &
		mapPoint.mnVisible &
		mapPoint.mnFound &
		mapPoint.mbBad;

	int _mpReplaced;
	ar & _mpReplaced;

	ar &
		mapPoint.mfMinDistance &
		mapPoint.mfMaxDistance;

	// XXX: Should we refactor the above part of this function? it looks the same as
	// saving part
	MapPoint::mpReplacement[mapPoint.mnId] = _mpReplaced;
	mapPoint.mObservations = createObjectList<KeyFrame> (kfObservation);
	mapPoint.mpRefKF = (refKfId==-1 ?
			NULL :
			KeyFrame::objectListLookup[refKfId]);
	MapPoint::objectListLookup[mapPoint.mnId] = &mapPoint;
}


template <class Archive>
void save (Archive & ar, const KeyFrameDatabase &kfdb, const unsigned int version)
{
	vector<list<idtype> > _mvInvertedFile;
	_mvInvertedFile.resize (kfdb.mvInvertedFile.size());
	for (int i=0; i<kfdb.mvInvertedFile.size(); i++) {
		_mvInvertedFile[i] = createIdList(kfdb.mvInvertedFile[i]);
	}
	ar & _mvInvertedFile;
}


template <class Archive>
void load (Archive & ar, KeyFrameDatabase &kfdb, const unsigned int version)
{
	vector<list<idtype> > _mvInvertedFile;
	ar & _mvInvertedFile;
	kfdb.mvInvertedFile.resize(_mvInvertedFile.size());

	for (int i=0; i<_mvInvertedFile.size(); i++) {
		kfdb.mvInvertedFile[i] = createObjectList<KeyFrame> (_mvInvertedFile[i]);
	}
}


}
}


void debugList (map<idtype,int> &targetList, const char *filename);



#endif /* INCLUDE_MAPOBJECTSERIALIZATION_H_ */

#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "KeyFrameDatabase.h"
#include "Converter.h"

#include <string>
#include <vector>


using namespace std;
using boost::python::vector_indexing_suite;


vector<int> doAdd (int p, int q)
{
	vector<int> pqr;
	pqr.push_back (p);
	pqr.push_back (q);
	return pqr;
}


class MapPointWrapper
{
public:

	MapPointWrapper (ORB_SLAM2::MapPoint *p) :
		_mp(p)
	{
		cv::Mat wpos = p->GetWorldPos();
		x = wpos.at<double>(0);
		y = wpos.at<double>(1);
		z = wpos.at<double>(2);
	}

	bool operator == (const MapPointWrapper &x) const
	{ return this->_mp == x._mp; }

	bool operator != (const MapPointWrapper &x) const
	{ return this->_mp != x._mp; }

	double x, y, z;

private:
	ORB_SLAM2::MapPoint *_mp;
};


class KeyFrameWrapper
{
public:
	KeyFrameWrapper (ORB_SLAM2::KeyFrame *kf) :
		_kf (kf)
	{
		// Pose in ORB-SLAM2 coordinate
		cv::Mat t = kf->GetCameraCenter();
		cv::Mat orient = kf->GetRotation().t();
		vector<float> q = ORB_SLAM2::Converter::toQuaternion(orient);

		x = t.at<float>(0);
		y = t.at<float>(0);
		z = t.at<float>(0);
		qx = q[0]; qy = q[1]; qz = q[2]; qw = q[3];

		// Pose in Metric coordinate
		if (kf->extPosition.empty() or kf->extOrientation.empty()) {
			xr = yr = zr = qxr = qyr = qzr = qwr = NAN;
		}

		else {
			xr = kf->extPosition.at<double>(0);
			yr = kf->extPosition.at<double>(1);
			zr = kf->extPosition.at<double>(2);
			qxr = kf->extOrientation.at<double>(0);
			qyr = kf->extOrientation.at<double>(1);
			qzr = kf->extOrientation.at<double>(2);
			qwr = kf->extOrientation.at<double>(3);
		}
	}

	bool operator == (const KeyFrameWrapper &x) const
	{ return this->_kf == x._kf; }

	bool operator != (const KeyFrameWrapper &x) const
	{ return this->_kf != x._kf; }

	double timestamp() const
	{ return _kf->mTimeStamp; }

	// Coordinate in ORB-SLAM system
	double x, y, z,
		qx, qy, qz, qw;

	// coordinate in metric system
	double xr, yr, zr,
		qxr, qyr, qzr, qwr;

private:
	ORB_SLAM2::KeyFrame *_kf;

};


class MapWrapper
{
public:
	MapWrapper (const string &loadPath)
	{
		fakeVocab = new ORB_SLAM2::ORBVocabulary ();
		kfdb = new ORB_SLAM2::KeyFrameDatabase (*fakeVocab);
		maprf = new ORB_SLAM2::Map ();
		maprf->loadFromDisk(loadPath, kfdb);
	}


	vector<KeyFrameWrapper> getKeyFrames ()
	{
		vector<KeyFrameWrapper> kfList;
		vector<ORB_SLAM2::KeyFrame*> kfsList = maprf->kfListSorted;
		kfList.reserve(kfsList.size());

		for (auto kfs: kfsList) {
			if (!kfs->isBad()) {
				KeyFrameWrapper kfw (kfs);
				kfList.push_back (kfw);
			}
		}

		return kfList;
	}


	vector<MapPointWrapper> getMapPoints ()
	{
		vector<MapPointWrapper> mpList;
		vector<ORB_SLAM2::MapPoint*> srcMpList = maprf->GetAllMapPoints();
		mpList.reserve(srcMpList.size());

		for (auto mp: srcMpList) {
			if (!mp->isBad()) {
				MapPointWrapper mpp (mp);
				mpList.push_back(mpp);
			}
		}

		return mpList;
	}


private:
	ORB_SLAM2::Map *maprf;
	ORB_SLAM2::KeyFrameDatabase *kfdb;
	ORB_SLAM2::ORBVocabulary *fakeVocab;
};


BOOST_PYTHON_MODULE (_orb_slam2)
{
	using namespace boost::python;

	class_ <vector<int> > ("IntArray")
		.def (vector_indexing_suite<vector<int> > ());
	def ("doAdd", &doAdd);

	class_ <MapPointWrapper> ("MapPoint", no_init)
		.def_readonly("x", &MapPointWrapper::x)
		.def_readonly("y", &MapPointWrapper::y)
		.def_readonly("z", &MapPointWrapper::z);

	class_ <KeyFrameWrapper> ("KeyFrame", no_init)
		.def_readonly("x", &KeyFrameWrapper::x)
		.def_readonly("y", &KeyFrameWrapper::y)
		.def_readonly("z", &KeyFrameWrapper::z)
		.def_readonly("qx", &KeyFrameWrapper::qx)
		.def_readonly("qy", &KeyFrameWrapper::qy)
		.def_readonly("qz", &KeyFrameWrapper::qz)
		.def_readonly("qw", &KeyFrameWrapper::qw)
		.def_readonly("xr", &KeyFrameWrapper::xr)
		.def_readonly("yr", &KeyFrameWrapper::yr)
		.def_readonly("zr", &KeyFrameWrapper::zr)
		.def_readonly("qxr", &KeyFrameWrapper::qxr)
		.def_readonly("qyr", &KeyFrameWrapper::qyr)
		.def_readonly("qzr", &KeyFrameWrapper::qzr)
		.def_readonly("qwr", &KeyFrameWrapper::qwr)
		.def ("timestamp", &KeyFrameWrapper::timestamp);

	class_ <vector<KeyFrameWrapper> > ("KeyFrameList")
		.def (vector_indexing_suite<vector<KeyFrameWrapper> > ());

	class_ <vector<MapPointWrapper> > ("MapPointList")
		.def (vector_indexing_suite<vector<MapPointWrapper> > ());

	class_ <MapWrapper> ("Map", init<const string &>())
		.def ("getKeyFrames", &MapWrapper::getKeyFrames)
		.def ("getMapPoints", &MapWrapper::getMapPoints);

}

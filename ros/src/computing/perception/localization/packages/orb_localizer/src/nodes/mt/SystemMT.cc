/*
 * SystemMT.cpp
 *
 *  Created on: Jul 22, 2016
 *      Author: sujiwo
 */

#include <string>

#include "SystemMT.h"



using namespace std;
using ORB_SLAM2::ORBVocabulary;


const double orbError = 0.5;


SystemMT::SystemMT (ros::NodeHandle &nh, const vector<string> &mapPaths, const string &vocabPath, const string &_settingsPath) :

	settingPath (_settingsPath),

	imgIsNew (false),

	rosnode (nh),

	// XXX: we silently assume that map loading is always successful
	readyCheck (mapPaths.size()+1)

{
	fSetting = cv::FileStorage (settingPath.c_str(), cv::FileStorage::READ);

	sVocab = new ORBVocabulary();
    if (vocabPath.empty() == false) {
    	cout << "Loading vocabulary ... ";
		bool bVocLoad = sVocab->loadFromTextFile(vocabPath);
		if(!bVocLoad)
		{
			cerr << "Wrong path to vocabulary. " << endl;
			cerr << "Failed to open at: " << vocabPath << endl;
			exit(-1);
		}
		cout << "Done" << endl << endl;
    }

    int i = 1;
    for (auto mp: mapPaths) {
    	string ids = std::to_string (i);
    	TrackingThread *pth = new TrackingThread (mp, this, ids);
    	children.push_back(pth);
    	i += 1;
    }

    pfilter = new PF::ParticleFilter<CamState, tf::Transform, double> (NUMBER_OF_PARTICLE, vehicleModel);

    // Wait until all workers ready
    cout << "Waiting for all workers to be ready... " << endl;
    readyCheck.wait();
    cout << "All workers ready" << endl;
}


/*
 * XXX: this destructor does not work correctly
 */
SystemMT::~SystemMT ()
{
	{
		boost::lock_guard<boost::mutex> lock (imgLock);
		for (auto proct: children)
			proct->stop();
	}
	imgMon.notify_all();

	for (auto proct: children)
		delete (proct);
//	for (auto proct: children) {
//		proct->stop();
//		cout << "Stopping worker #" << proct->identity << endl;
//		delete (proct);
//	}

	delete (pfilter);
}


/*
 * We expect colorful image, unbayered
 */
void SystemMT::Track (const cv::Mat &srcImage, const double timestamp)
{
	currentTimestamp = timestamp;

	{
		boost::lock_guard<boost::mutex> lock (imgLock);

		currentImage = srcImage.clone();
		// resize and crop
		// Do Resizing and cropping here
		cv::resize(currentImage, currentImage,
			cv::Size(
				(int)fSetting["Camera.WorkingResolution.Width"],
				(int)fSetting["Camera.WorkingResolution.Height"]
			));
		currentImage = currentImage(
			cv::Rect(
				(int)fSetting["Camera.ROI.x0"],
				(int)fSetting["Camera.ROI.y0"],
				(int)fSetting["Camera.ROI.width"],
				(int)fSetting["Camera.ROI.height"]
			)).clone();

		imgIsNew = true;
		currentTimestamp = timestamp;
	}
	// notify worker threads to begin
	imgMon.notify_all();

//	filter ();

	readyCheck.wait();
	imgIsNew = false;

}


void SystemMT::filter()
{
	if (!vehicleModel.isInitialized()) {
		for (auto proct: children) {
			if (!proct->poseIsValid())
				continue;
			// we found valid pose; set this as particle initialization
			vehicleModel.preinitialize(proct->getCurrent(), currentTimestamp);
			break;
		}
		pfilter->initializeParticles();
		return;
	}
//	prev

}


OrbMapFusion::OrbMapFusion() :
	initialized (false),
	prevTimestamp (0)
{}


void OrbMapFusion::preinitialize(const tf::Transform &iPose, const double ts)
{
	prevTimestamp = ts;
	initPose = iPose;
	initialized = true;
}


CamState OrbMapFusion::initializeParticleState() const
{
	CamState m0 (initPose);
	double ds = PF::nrand(orbError),
		xn = m0.getOrigin().x(),
		yn = m0.getOrigin().y(),
		zn = m0.getOrigin().z();
	m0.setOrigin(tf::Vector3(xn, yn, zn));
	return m0;
}


/*
 * XXX: this motion model is untested !
 * Need to initialize velocity
 */
CamState OrbMapFusion::motionModel(const CamState &vstate, const double &t) const
{
	CamState nstate;

	double dt = t - prevTimestamp;
	double x = vstate.getOrigin().x() + vstate.velocity.x()*dt + PF::nrand(orbError);
	double y = vstate.getOrigin().y() + vstate.velocity.y()*dt + PF::nrand(orbError);
	double z = vstate.getOrigin().z() + vstate.velocity.z()*dt + PF::nrand(orbError);
	double vx = (x-vstate.getOrigin().x()) / dt;
	double vy = (y-vstate.getOrigin().y()) / dt;
	double vz = (z-vstate.getOrigin().z()) / dt;

	nstate.setOrigin(tf::Vector3(x, y, z));
	nstate.velocity = tf::Vector3 (vx, vy, vz);
	return nstate;
}


double OrbMapFusion::measurementModel(const CamState &state, const vector<tf::Transform> &observations) const
{
	vector<double> obsWeight;

	for (const auto &pose: observations) {
		double wo,
			xt = pose.getOrigin().x(),
			yt = pose.getOrigin().y(),
			zt = pose.getOrigin().z(),
			xs = state.getOrigin().x(),
			ys = state.getOrigin().y(),
			zs = state.getOrigin().z();
		wo = exp (-(pow(xt-xs,2) / (2*orbError*orbError) +
			pow(yt-ys,2) / (2*orbError*orbError)));
		obsWeight.push_back(wo);
	}

	double w = *std::max_element(obsWeight.begin(), obsWeight.end());
	return max(w, 0.01);
}

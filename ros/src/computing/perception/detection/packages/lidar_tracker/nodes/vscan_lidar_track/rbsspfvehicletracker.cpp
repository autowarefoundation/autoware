#include "rbsspfvehicletracker.h"
#include "moc_rbsspfvehicletracker.cpp"

RBSSPFVehicleTrackerInstance::RBSSPFVehicleTrackerInstance(int vehicleID,
		QThread *thread) :
		QObject(NULL)
{
	id = vehicleID;
	tracker_instance_state_ = InitGeometry;
	cuda_OpenTracker(tracker_data_container_);
	this->moveToThread(thread);
	connect(thread, SIGNAL(finished()), this, SLOT(deleteLater()));
	thread->start();
}

RBSSPFVehicleTrackerInstance::~RBSSPFVehicleTrackerInstance()
{
	cuda_CloseTracker(tracker_data_container_);
}

//this thing is called on every thread and synced using that initFlag
void RBSSPFVehicleTrackerInstance::slotCheckInitState(int initNum,
		VehicleState * initState, bool * initFlag)
{
	for (int i = 0; i < initNum; i++)
	{
		double tmpdx = initState[i].x - tracker_result_container_.estimate.x;
		double tmpdy = initState[i].y - tracker_result_container_.estimate.y;
		double dx = tmpdx * cos(-tracker_result_container_.estimate.theta)
				- tmpdy * sin(-tracker_result_container_.estimate.theta);
		double dy = tmpdx * sin(-tracker_result_container_.estimate.theta)
				+ tmpdy * cos(-tracker_result_container_.estimate.theta);
		if ((dx <= tracker_result_container_.estimate.wl + MARGIN2)
				&& (dx >= -tracker_result_container_.estimate.wr - MARGIN2)
				&& (dy <= tracker_result_container_.estimate.lf + MARGIN2)
				&& (dy >= -tracker_result_container_.estimate.lb - MARGIN2))
		{
			initFlag[i] = 0; //matched
		}
	}
	//std::cout << "Calling slotCheckInitStateFinish from tracker >" << id << std::endl;
	emit signalCheckInitStateFinish();
	return;
}

void RBSSPFVehicleTrackerInstance::slotUpdateTracker(
		QMap<int, VehicleState> * initStateMap)
{
	switch (tracker_instance_state_)
	{
	case InitGeometry:
		//std::cout << "InitGeometry->InitMotion >" << id << std::endl;
		tracker_instance_state_ = InitMotion;
		tracker_result_container_.estimate = (*initStateMap)[id];
		cuda_InitGeometry(tracker_data_container_, tracker_result_container_);
		break;
	case InitMotion:
		//std::cout << "InitMotion->UpdateTracker >" << id << std::endl;
		tracker_instance_state_ = UpdateTracker;
		cuda_InitMotion(tracker_data_container_, tracker_result_container_);
		break;
	case UpdateTracker:
		//std::cout << "UpdateTracker >" << id << std::endl;
//        cuda_InitMotion(trackerdatacontainer,trackerresultcontainer);
		cuda_UpdateTracker(tracker_data_container_, tracker_result_container_);
		break;
	default:
		return;
	}
	emit signalUpdateTrackerFinish(id, &tracker_result_container_);
	return;
}

RBSSPFVehicleTracker::RBSSPFVehicleTracker() :
		QObject(NULL)
{
	//qRegisterMetaType<TrackerResultContainer>("TrackerResultContainer");
	tracker_state_ = NoLaserData;
	cuda_InitLaserScan();
}

RBSSPFVehicleTracker::~RBSSPFVehicleTracker()
{
	QList<QThread *> threadlist = trackers_thread_container_map_.values();
	int i, n = threadlist.size();
	for (i = 0; i < n; i++)
	{
		threadlist[i]->exit();
		threadlist[i]->wait();
	}
	cuda_FreeLaserScan();
}

void RBSSPFVehicleTracker::addTrackerData(LaserScan & scan,
		QVector<VehicleState> & initState)
{
	switch (tracker_state_)
	{
	case NoLaserData:
		//std::cout << "NoLaserData->OneLaserData >" << tracker_id_ << std::endl;
		tracker_state_ = OneLaserData;
		cuda_SetLaserScan(scan);
		break;
	case OneLaserData:
		//std::cout << "OneLaserData->ReadyForTracking >" << tracker_id_ << std::endl;
		tracker_state_ = ReadyForTracking;
		cuda_SetLaserScan(scan);
		break;
	case ReadyForTracking:
		//std::cout << "ReadyForTracking->Processing >" << tracker_id_ << std::endl;
		tracker_state_ = Processing;
		cuda_SetLaserScan(scan);
		current_laserscan_ = scan;
		detection_num = initState.size();
		if (detection_num > 0) //if there are detections
		{
			detections_ = initState;
			detections_matched_.fill(1, detection_num); //1 means not matched, number of detections
			tracker_count_ = trackers_thread_container_map_.size();
			if (tracker_count_ > 0) //if there are trackers and detections
			{
				//std::cout << "  TrackersAndDetectionsAvailable =" << tracker_count_ << ", " << detection_num << std::endl;
				emit signalCheckInitState(detection_num, detections_.data(),
						detections_matched_.data());
			}
			else //if there are detections but NO trackers running
			{
				//std::cout << "  DetectionsAvailable NoTrackers. Calling slotCheckInitStateFinish from Tracker> " << tracker_id_ << std::endl;
				slotCheckInitStateFinish();
			}
		}
		else if (trackers_thread_container_map_.size() > 0) //if no detections but trackers running
		{
			//std::cout << "  OnlyTrackersAvailable >" << tracker_id_ << std::endl;
			detections_unmatched_.clear();
			tracker_count_ = trackers_thread_container_map_.size();
			emit signalUpdateTracker(&detections_unmatched_); //update trackers with empty detection queue
		}
		else //no trackers, no detections
		{
			//std::cout << "  NoTrackers NoDetections>" << tracker_id_ << std::endl;
			tracker_state_ = ReadyForTracking;
			emit signalUpdateTrackerFinish(current_laserscan_,
					tracker_result_container_map_);
		}
		break;
	case Processing:
		//std::cout << "Processing>" << tracker_id_ << std::endl;
		if (laserscan_processing_queue_.size()<5)//max 10 frames in queue
		{
			laserscan_processing_queue_.push_back(scan);
			detections_processing_queue_.push_back(initState);
		}
		break;
	}
}

void RBSSPFVehicleTracker::slotCheckInitStateFinish()
{
	tracker_count_--;
	//std::cout << "ReducingLifespan slotCheckInitStateFinish>" << tracker_id_ << std::endl;
	if (tracker_count_ <= 0)
	{
		detections_unmatched_.clear();
		int i, n = detections_matched_.size();
		for (i = 0; i < n; i++)
		{
			if (detections_matched_[i]) //not matched
			{
				detections_unmatched_.insert(tracker_id_, detections_[i]); //add unmatched detection to the queue

				if (trackers_thread_container_map_.size() < 10)
				{
					QThread * thread = new QThread;
					RBSSPFVehicleTrackerInstance * trackerinstance =
							new RBSSPFVehicleTrackerInstance(tracker_id_, thread);
					tracker_result_container_map_.insert(tracker_id_,
							TrackerResultContainer());
					trackers_thread_container_map_.insert(tracker_id_, thread);
					tracker_lifespan_map_.insert(tracker_id_, 0);

					connect(this,
							SIGNAL(signalCheckInitState(int,VehicleState*,bool*)),
							trackerinstance,
							SLOT(slotCheckInitState(int,VehicleState*,bool*)));
					connect(this,
							SIGNAL(signalUpdateTracker(QMap<int,VehicleState>*)),
							trackerinstance,
							SLOT(slotUpdateTracker(QMap<int,VehicleState>*)));

					connect(trackerinstance, SIGNAL(signalCheckInitStateFinish()),
							this, SLOT(slotCheckInitStateFinish()));
					connect(trackerinstance,
							SIGNAL(
									signalUpdateTrackerFinish(int,TrackerResultContainer *)),
							this,
							SLOT(
									slotUpdateTrackerFinish(int,TrackerResultContainer *)));

					tracker_id_++;
				}
			}
		}

		tracker_count_ = trackers_thread_container_map_.size();
		emit signalUpdateTracker(&detections_unmatched_); //update trackers with the unmatched detections
	}
}

void RBSSPFVehicleTracker::slotUpdateTrackerFinish(int vehicleID,
		TrackerResultContainer * trackerResult)
{
	tracker_count_--;
	//std::cout << "ReducingLifespan slotUpdateTrackerFinish >" << vehicleID << std::endl;
	tracker_result_container_map_[vehicleID] = *trackerResult;
	if (trackerResult->estimate.dx > 1 || trackerResult->estimate.dy > 1
			|| trackerResult->estimate.dtheta > DEG2RAD(20)
			|| trackerResult->estimate.count < 1)
	{
		tracker_lifespan_map_[vehicleID]++;
	}
	else
	{
		tracker_lifespan_map_[vehicleID]--;
	}
	if (tracker_lifespan_map_[vehicleID] > 10)//max 10 trackers
	{
		//std::cout << "RemovingTracker >" << tracker_id_ << std::endl;
		trackers_thread_container_map_[vehicleID]->exit();
		tracker_result_container_map_.remove(vehicleID);
		trackers_thread_container_map_.remove(vehicleID);
		tracker_lifespan_map_.remove(vehicleID);
	}


	if (tracker_count_ <= 0)
	{
		emit signalUpdateTrackerFinish(current_laserscan_,
				tracker_result_container_map_);
		if (laserscan_processing_queue_.size() > 0)
		{
			LaserScan scan = laserscan_processing_queue_.front();
			current_laserscan_ = scan;
			laserscan_processing_queue_.pop_front();
			cuda_SetLaserScan(scan);

			detections_ = detections_processing_queue_.front();
			detections_processing_queue_.pop_front();

			detection_num = detections_.size();
			if (detection_num > 0) //detections available
			{
				detections_matched_.fill(1, detection_num);
				tracker_count_ = trackers_thread_container_map_.size();
				emit signalCheckInitState(detection_num, detections_.data(),
						detections_matched_.data());
			}
			else
			{
				detections_unmatched_.clear();
				tracker_count_ = trackers_thread_container_map_.size();
				emit signalUpdateTracker(&detections_unmatched_);
			}
		}
		else
		{
			tracker_state_ = ReadyForTracking;
		}
	}
}

/*
 * SimpleTracker.cpp
 *
 *  Created on: Aug 11, 2016
 *      Author: hatem
 */

#include "SimpleTracker.h"
#include "MatrixOperations.h"
#include "PlanningHelpers.h"
#include "UtilityH.h"

#include <iostream>
#include <vector>
#include <cstdio>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>

namespace SimulationNS
{

using namespace PlannerHNS;

SimpleTracker::SimpleTracker()
{
	iTracksNumber = 1;
	m_DT = 0.1;
	m_MAX_ASSOCIATION_DISTANCE = 3.0;
	m_MAX_TRACKS_AFTER_LOSING = 10;
	m_MaxKeepTime = 2; // seconds
	m_bUseCenterOnly = true;
	m_bFirstCall = true;
	UtilityHNS::UtilityH::GetTickCount(m_TrackTimer);
}

void SimpleTracker::InitSimpleTracker()
{
	UtilityHNS::UtilityH::GetTickCount(m_TrackTimer);
	InitializeInterestRegions(TRACKING_HORIZON, 5, 5, m_InterestRegions);
}

SimpleTracker::~SimpleTracker()
{
	for(unsigned int i = 0; i < m_InterestRegions.size(); i++)
	{
		delete m_InterestRegions.at(i);
	}
	m_InterestRegions.clear();

	for(unsigned int i = 0; i < m_Tracks.size(); i++)
	{
		delete m_Tracks.at(i);
	}
	m_Tracks.clear();

}

void SimpleTracker::InitializeInterestRegions(double horizon, double init_raduis, double init_time, std::vector<InterestCircle*>& regions)
{
	//initialize interest regions
	double distance = 0;
	while(distance <= horizon)
	{
		InterestCircle* pCir = new InterestCircle(regions.size()+1);
		if(regions.size() == 0)
		{
			pCir->radius = 5;
			//pCir->pPrevCircle = 0;
			pCir->forget_time = NEVER_GORGET_TIME;
			pCir->forget_time = m_MaxKeepTime*2.0;
			pCir->forget_time = m_MaxKeepTime;
			regions.push_back(pCir);
			std::cout << "Region No: " << regions.size() << ", Radius: " << pCir->radius << ", F time: " << pCir->forget_time << std::endl;
		}
		else
		{
			int iPrev = regions.size()-1;
			if(regions.size() > 1)
				pCir->radius = regions.at(iPrev)->radius + regions.at(iPrev-1)->radius;
			else
				pCir->radius = regions.at(iPrev)->radius + regions.at(iPrev)->radius;

//			regions.at(iPrev)->pNextCircle = pCir;
//			pCir->pPrevCircle = regions.at(iPrev);
			pCir->forget_time = m_MaxKeepTime-iPrev-2;
			pCir->forget_time = m_MaxKeepTime;
			if(pCir->forget_time <= 0 )
				pCir->forget_time = 0.2;
			regions.push_back(pCir);

			std::cout << "Region No: " << regions.size() << ", Radius: " << pCir->radius << ", F time: " << pCir->forget_time << std::endl;
		}

		distance = pCir->radius;
	}
}

void SimpleTracker::AssociateToRegions(KFTrackV& detectedObject)
{
	for(unsigned int i = 0; i < m_InterestRegions.size(); i++)
	{
		m_InterestRegions.at(i)->pTrackers.clear();
		if(detectedObject.obj.distance_to_center <= m_InterestRegions.at(i)->radius)
		{
			detectedObject.region_id = m_InterestRegions.at(i)->id;
			detectedObject.forget_time = m_InterestRegions.at(i)->forget_time;
			//std::cout << "Associate Object: " << detectedObject.obj.id << ", With Region: " << detectedObject.region_id << ", And Time: " << detectedObject.forget_time << std::endl;
			return;
		}
	}

	if(m_InterestRegions.size() > 0)
	{
		detectedObject.region_id = m_InterestRegions.at(m_InterestRegions.size()-1)->id;
		detectedObject.forget_time = m_InterestRegions.at(m_InterestRegions.size()-1)->forget_time;
		//std::cout << "Associate Object: " << detectedObject.obj.id << ", With Region: " << detectedObject.region_id << ", And Time: " << detectedObject.forget_time << std::endl;
	}
}

void SimpleTracker::AssociateAndTrack()
{
	DetectedObject* prev_obj;
	DetectedObject* curr_obj;

	std::vector<CostRecordSet> matching_matrix;

	for(unsigned int i = 0 ; i < m_DetectedObjects.size(); i++)
	{
		double minCost = 99999999;
		double minID = -1;

		curr_obj = &m_DetectedObjects.at(i);
		curr_obj->center.cost = 0;

		matching_matrix.push_back(CostRecordSet(i, -1, 0));

		for(unsigned int j = 0; j < m_Tracks.size(); j++)
		{
			prev_obj = &m_Tracks.at(j)->obj;
			double expected_d = prev_obj->center.v * m_DT;

			if(m_bUseCenterOnly)
			{
				curr_obj->center.cost = fabs(hypot(curr_obj->center.pos.y- prev_obj->center.pos.y, curr_obj->center.pos.x- prev_obj->center.pos.x) - expected_d);
			}
			else
			{
				for(unsigned int k = 0; k < curr_obj->contour.size(); k++)
					for(unsigned int pk = 0; pk < prev_obj->contour.size(); pk++)
						curr_obj->center.cost += fabs(hypot(curr_obj->contour.at(k).y -prev_obj->contour.at(pk).y, curr_obj->contour.at(k).x -prev_obj->contour.at(pk).x) - expected_d);

				curr_obj->center.cost = curr_obj->center.cost/(double)(curr_obj->contour.size()*prev_obj->contour.size());
			}

			if(DEBUG_TRACKER)
				std::cout << "Cost Cost (" << i << "), " << prev_obj->center.pos.ToString() << ","
						<< curr_obj->center.cost <<  ", contour: " << curr_obj->contour.size()
						<< ", " << curr_obj->center.pos.ToString() << std::endl;

			if(curr_obj->center.cost < minCost)
			{
				minCost = curr_obj->center.cost;
				minID = j;
			}
		}

		bool bSkip = false;

		for(unsigned int k = 0; k < matching_matrix.size(); k++)
		{
			if(matching_matrix.at(k).prevObj == minID)
			{
				if(minCost < matching_matrix.at(k).cost)
				{
					matching_matrix.at(k).prevObj = minID;
					matching_matrix.at(k).cost = minCost;
				}
				bSkip = true;
				break;
			}
		}

		if(!bSkip && minID >= 0 && minCost <= m_MAX_ASSOCIATION_DISTANCE)
		{
			matching_matrix.at(i).prevObj = minID;
			matching_matrix.at(i).cost = minCost;
		}
	}

	for(unsigned int i = 0 ; i < matching_matrix.size(); i ++)
	{
		curr_obj =  &m_DetectedObjects.at(matching_matrix.at(i).currobj);

		if(matching_matrix.at(i).prevObj == -1)
		{
			 CreateTrackV2(*curr_obj);
			if(DEBUG_TRACKER)
				std::cout << "New Matching Index: " << matching_matrix.at(i).currobj << ", "<< matching_matrix.at(i).cost<< ", " << iTracksNumber << std::endl;
		}
		else
		{
			curr_obj->id = m_Tracks.at(matching_matrix.at(i).prevObj)->obj.id;
			curr_obj->center.pos.a = m_Tracks.at(matching_matrix.at(i).prevObj)->obj.center.pos.a;
			m_Tracks.at(matching_matrix.at(i).prevObj)->obj = *curr_obj;
			AssociateToRegions(*m_Tracks.at(matching_matrix.at(i).prevObj));
			if(DEBUG_TRACKER)
				std::cout << "ObjIndex: " <<  matching_matrix.at(i).currobj <<  ", Matched with ID  " << prev_obj->id << ", "<< matching_matrix.at(i).cost << std::endl;
		}
	}
}

void SimpleTracker::AssociateSimply()
{
	std::vector<KFTrackV> newObjects;
	for(unsigned int j = 0; j < m_DetectedObjects.size(); j++)
	{
		double iCloseset = 0;
		double dCloseset = 9999999;
		for(unsigned int i = 0; i < m_TrackSimply.size(); i++)
		{
			double d = hypot(m_DetectedObjects.at(j).center.pos.y-m_TrackSimply.at(i).obj.center.pos.y, m_DetectedObjects.at(j).center.pos.x-m_TrackSimply.at(i).obj.center.pos.x);
			if(d < dCloseset)
			{
				dCloseset = d;
				iCloseset = i;
			}
		}

		if(dCloseset <= m_MAX_ASSOCIATION_DISTANCE)
		{
			m_DetectedObjects.at(j).id = m_TrackSimply.at(iCloseset).obj.id;
			m_DetectedObjects.at(j).center.pos.a = m_TrackSimply.at(iCloseset).obj.center.pos.a;
			m_TrackSimply.at(iCloseset).obj = m_DetectedObjects.at(j);
			newObjects.push_back(m_TrackSimply.at(iCloseset));
		}
		else
		{
			iTracksNumber = iTracksNumber + 1;
			m_DetectedObjects.at(j).id = iTracksNumber;
			KFTrackV track(m_DetectedObjects.at(j).center.pos.x, m_DetectedObjects.at(j).center.pos.y,m_DetectedObjects.at(j).center.pos.a, m_DetectedObjects.at(j).id, m_DT);
			track.obj = m_DetectedObjects.at(j);
			newObjects.push_back(track);
		}
	}

	m_TrackSimply = newObjects;

	for(unsigned int i =0; i< m_TrackSimply.size(); i++)
		m_TrackSimply.at(i).UpdateTracking(m_DT, m_TrackSimply.at(i).obj, m_TrackSimply.at(i).obj);

	m_DetectedObjects.clear();
	for(unsigned int i = 0; i< m_TrackSimply.size(); i++)
		m_DetectedObjects.push_back(m_TrackSimply.at(i).obj);
}

void SimpleTracker::CreateTrackV2(DetectedObject& o)
{
	iTracksNumber = iTracksNumber + 1;
	o.id = iTracksNumber;
	KFTrackV* pT = new KFTrackV(o.center.pos.x, o.center.pos.y,o.center.pos.a, o.id, m_DT);
	pT->obj = o;
	AssociateToRegions(*pT);
	m_Tracks.push_back(pT);
}

void SimpleTracker::TrackV2()
{
	for(unsigned int i =0; i< m_Tracks.size(); i++)
	{
		m_Tracks.at(i)->UpdateTracking(m_DT, m_Tracks.at(i)->obj, m_Tracks.at(i)->obj);

		//std::cout<< "Obj ID: " << m_Tracks.at(i)->GetTrackID() << ", Remaining Time: " <<  m_Tracks.at(i)->forget_time  << std::endl;
	}
}

void SimpleTracker::CleanOldTracks()
{
	m_DetectedObjects.clear();
	for(int i = 0; i< (int)m_Tracks.size(); i++)
	{
		if(m_Tracks.at(i)->forget_time < 0 && m_Tracks.at(i)->forget_time > NEVER_GORGET_TIME)
		{
			delete m_Tracks.at(i);
			m_Tracks.erase(m_Tracks.begin()+i);
			i--;
		}
		else if(m_Tracks.at(i)->m_iLife > MIN_EVIDENCE_NUMBER)
		{
			m_DetectedObjects.push_back(m_Tracks.at(i)->obj);
		}
	}
}

void SimpleTracker::DoOneStep(const WayPoint& currPose, const std::vector<DetectedObject>& obj_list)
{
	if(!m_bFirstCall)
		m_DT = UtilityHNS::UtilityH::GetTimeDiffNow(m_TrackTimer);
	else
		m_bFirstCall = false;

	UtilityHNS::UtilityH::GetTickCount(m_TrackTimer);

	//std::cout << " Tracking Time : " << m_DT << std::endl;
	m_DetectedObjects = obj_list;

	AssociateSimply();

//	AssociateAndTrack();
//	TrackV2();
//	CleanOldTracks();



//	AssociateObjects();
//	Track(m_DetectedObjects);
//	m_PrevDetectedObjects = m_DetectedObjects;
//	m_PrevState = currPose;
}

void SimpleTracker::AssociateObjects()
{
	std::vector<DetectedObject> hidden_list;
	DetectedObject* prev_obj;
	DetectedObject* curr_obj;

	for(unsigned int i = 0 ; i < m_DetectedObjects.size(); i++)
	{
		double minCost = 99999999;
		double minID = -1;

		curr_obj = &m_DetectedObjects.at(i);
		curr_obj->center.cost = 0;

		for(unsigned int j = 0; j < m_PrevDetectedObjects.size(); j++)
		{
			prev_obj = &m_PrevDetectedObjects.at(j);

			if(m_bUseCenterOnly)
			{
				curr_obj->center.cost = hypot(curr_obj->center.pos.y- prev_obj->center.pos.y, curr_obj->center.pos.x- prev_obj->center.pos.x);
			}
			else
			{
				for(unsigned int k = 0; k < curr_obj->contour.size(); k++)
					for(unsigned int pk = 0; pk < prev_obj->contour.size(); pk++)
						curr_obj->center.cost += hypot(curr_obj->contour.at(k).y -prev_obj->contour.at(pk).y, curr_obj->contour.at(k).x -prev_obj->contour.at(pk).x);

				curr_obj->center.cost = curr_obj->center.cost/(double)(curr_obj->contour.size()*prev_obj->contour.size());
			}

//			if(DEBUG_TRACKER)
//				std::cout << "Cost (" << i << "), " << prev_obj->center.pos.ToString() << ", Cost: "
//						<< curr_obj->center.cost <<  ", contour: " << curr_obj->contour.size()
//						<< ", " << curr_obj->center.pos.ToString() << std::endl;

			if(curr_obj->center.cost < minCost)
			{
				minCost = curr_obj->center.cost;
				minID = prev_obj->id;
			}
		}

		if(minID <= 0 || minCost > m_MAX_ASSOCIATION_DISTANCE) // new Object enter the scene
		{
			iTracksNumber = iTracksNumber + 1;
			 curr_obj->id = iTracksNumber;
			if(DEBUG_TRACKER)
				std::cout << "New Matching " << curr_obj->id << ", "<< minCost<< ", " << iTracksNumber << std::endl;
		}
		else
		{
			 curr_obj->id = minID;
			if(DEBUG_TRACKER)
				std::cout << "Matched with ID  " << curr_obj->id << ", "<< minCost<< std::endl;
		}
	}

	for(unsigned int i = 0 ; i < m_PrevDetectedObjects.size(); i++)
	{
		prev_obj = &m_PrevDetectedObjects.at(i);
		bool bFound = false;
		for(unsigned int j = 0; j < m_DetectedObjects.size(); j++)
		{
			if(prev_obj->id == m_DetectedObjects.at(j).id)
			{
				bFound = true;
				break;
			}
		}

		if(!bFound && prev_obj->predicted_center.cost < m_MAX_TRACKS_AFTER_LOSING)
		{
			prev_obj->predicted_center.cost++;
			hidden_list.push_back(*prev_obj);
		}
	}

	m_DetectedObjects.insert(m_DetectedObjects.begin(), hidden_list.begin(), hidden_list.end());
}

void SimpleTracker::CreateTrack(DetectedObject& o)
{
	KFTrackV* pT = new KFTrackV(o.center.pos.x, o.center.pos.y,o.center.pos.a, o.id, m_DT);
	o.id = pT->GetTrackID();
	pT->UpdateTracking(m_DT, o, o);
	m_Tracks.push_back(pT);
}

KFTrackV* SimpleTracker::FindTrack(long index)
{
	for(unsigned int i=0; i< m_Tracks.size(); i++)
	{
		if(m_Tracks[i]->GetTrackID() == index)
			return m_Tracks[i];
	}

	return 0;
}

void SimpleTracker::Track(std::vector<DetectedObject>& objects_list)
{
	for(unsigned int i =0; i<objects_list.size(); i++)
	{
		if(objects_list[i].id >= 0)
		{
			KFTrackV* pT = FindTrack(objects_list[i].id);
			if(pT)
			{
//				pT->UpdateTracking(m_DT, objects_list[i].center.pos.x, objects_list//[i].center.pos.y, objects_list[i].center.pos.a,
						//objects_list[i].center.pos.x, objects_list[i].center.pos.y, //objects_list[i].center.pos.a,
//						objects_list[i].center.v);
				//std::cout << "Update Current Track " << objects_list[i].id << std::endl;
			}
			else
			{
				CreateTrack(objects_list[i]);
				//std::cout << "Create New Track " << objects_list[i].id << std::endl;
			}
		}
	}
}

void SimpleTracker::CoordinateTransform(const WayPoint& refCoordinate, DetectedObject& obj)
{
	Mat3 rotationMat(-refCoordinate.pos.a);
	Mat3 translationMat(-refCoordinate.pos.x, -refCoordinate.pos.y);
	obj.center.pos = translationMat*obj.center.pos;
	obj.center.pos = rotationMat*obj.center.pos;
	for(unsigned int j = 0 ; j < obj.contour.size(); j++)
	{
		obj.contour.at(j) = translationMat*obj.contour.at(j);
		obj.contour.at(j) = rotationMat*obj.contour.at(j);
	}
}

void SimpleTracker::CoordinateTransformPoint(const WayPoint& refCoordinate, GPSPoint& obj)
{
	Mat3 rotationMat(-refCoordinate.pos.a);
	Mat3 translationMat(-refCoordinate.pos.x, -refCoordinate.pos.y);
	obj = translationMat*obj;
	obj = rotationMat*obj;
}

} /* namespace BehaviorsNS */

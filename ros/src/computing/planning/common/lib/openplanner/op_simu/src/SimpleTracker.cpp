/*
 * SimpleTracker.cpp
 *
 *  Created on: Aug 11, 2016
 *      Author: hatem
 */

#include "SimpleTracker.h"
#include "MatrixOperations.h"
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
	m_dt = 0.1;
	m_MAX_ASSOCIATION_DISTANCE = 2.0;
	m_MAX_ASSOCIATION_SIZE_DIFF = 1.0;
	m_MAX_ASSOCIATION_ANGLE_DIFF = 0.44;
	m_MaxKeepTime = 2; // seconds
	m_bUseCenterOnly = true;
	m_bFirstCall = true;
	m_nMinTrustAppearances = 5;
	m_Horizon = 100.0;
	m_CirclesResolution = 5.0;
	UtilityHNS::UtilityH::GetTickCount(m_TrackTimer);
}

void SimpleTracker::InitSimpleTracker()
{
	UtilityHNS::UtilityH::GetTickCount(m_TrackTimer);
	InitializeInterestRegions(m_InterestRegions);
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

void SimpleTracker::InitializeInterestRegions(std::vector<InterestCircle*>& regions)
{
	//initialize interest regions
	double next_raduis = m_CirclesResolution;
	double next_foget_time = m_MaxKeepTime;
	while(1)
	{
		InterestCircle* pCir = new InterestCircle(regions.size()+1);
		if(regions.size() == 0)
		{
			pCir->radius = next_raduis;
			pCir->forget_time = next_foget_time;
			regions.push_back(pCir);
			std::cout << "Region No: " << regions.size() << ", Radius: " << pCir->radius << ", F time: " << pCir->forget_time << std::endl;
		}
		else
		{
			pCir->radius = next_raduis;
			pCir->forget_time = next_foget_time;
			regions.push_back(pCir);

			std::cout << "Region No: " << regions.size() << ", Radius: " << pCir->radius << ", F time: " << pCir->forget_time << std::endl;
		}

		if(next_raduis >= m_Horizon)
			break;

		next_raduis += next_raduis * 0.8;
		if(next_raduis > m_Horizon)
			next_raduis = m_Horizon;
		next_foget_time -= next_foget_time * 0.25;
		if(next_foget_time < 0.1)
			next_foget_time = 0.1;
	}
}

void SimpleTracker::DoOneStep(const WayPoint& currPose, const std::vector<DetectedObject>& obj_list, const TRACKING_TYPE& type)
{
	if(!m_bFirstCall)
	{
		m_dt = UtilityHNS::UtilityH::GetTimeDiffNow(m_TrackTimer);
		m_StateDiff.pos.x = m_PrevState.pos.x - currPose.pos.x ;
		m_StateDiff.pos.y = m_PrevState.pos.y - currPose.pos.y;
		m_StateDiff.pos.a = UtilityHNS::UtilityH::AngleBetweenTwoAnglesPositive(currPose.pos.a, m_PrevState.pos.a) * UtilityHNS::UtilityH::GetSign(m_PrevState.pos.a - currPose.pos.a);
		//std::cout << "(" << m_StateDiff.pos.x << ", " << m_StateDiff.pos.y << ", " << m_StateDiff.pos.a << std::endl;
	}
	else
		m_bFirstCall = false;

	UtilityHNS::UtilityH::GetTickCount(m_TrackTimer);

	m_DetectedObjects = obj_list;

	if(type == ASSOCIATE_ONLY)
	{
		AssociateOnly();
	}
	else if (type == SIMPLE_TRACKER)
	{
		AssociateSimply();
	}
	else
	{
		AssociateAndTrack();
		CleanOldTracks();
	}

	m_PrevState = currPose;

}

void SimpleTracker::MatchClosest()
{
	newObjects.clear();

	while(m_DetectedObjects.size() > 0)
	{
		double iCloseset_track = -1;
		double iCloseset_obj = -1;
		double dCloseset = 99999999;
		bool bFoundMatch = false;
		double min_size = -1;
		//std::cout << "DetObjSize: " <<  m_DetectedObjects.size() <<  ", TracksSize: " << m_TrackSimply.size() << std::endl;
		for(unsigned int jj = 0; jj < m_DetectedObjects.size(); jj++)
		{
			double object_size = hypot(m_DetectedObjects.at(jj).w, m_DetectedObjects.at(jj).l);

			for(unsigned int i = 0; i < m_TrackSimply.size(); i++)
			{
				double old_size = hypot(m_TrackSimply.at(i).obj.w, m_TrackSimply.at(i).obj.l);
				double obj_diff = fabs(object_size - old_size);
				double d = hypot(m_DetectedObjects.at(jj).center.pos.y-m_TrackSimply.at(i).obj.center.pos.y, m_DetectedObjects.at(jj).center.pos.x-m_TrackSimply.at(i).obj.center.pos.x);

				bool bInsidePolygon = (InsidePolygon(m_TrackSimply.at(i).obj.contour, m_DetectedObjects.at(jj).center.pos) == 1 || InsidePolygon(m_DetectedObjects.at(jj).contour, m_TrackSimply.at(i).obj.center.pos) == 1);
				bool bDirectionMatch = false;
				if(m_TrackSimply.at(i).obj.bDirection)
				{
					double diff_y = m_DetectedObjects.at(jj).center.pos.y - m_TrackSimply.at(i).obj.center.pos.y;
					double diff_x = m_DetectedObjects.at(jj).center.pos.x - m_TrackSimply.at(i).obj.center.pos.x;
					double angle_diff = 100;
					if(hypot(diff_y, diff_x) > 0.2)
					{
						angle_diff = UtilityHNS::UtilityH::FixNegativeAngle(atan2(diff_y, diff_x));
						if(UtilityHNS::UtilityH::AngleBetweenTwoAnglesPositive(angle_diff,m_TrackSimply.at(i).obj.center.pos.a) < m_MAX_ASSOCIATION_ANGLE_DIFF)
							bDirectionMatch = true;
					}
				}

				//std::cout << "Test: " << m_TrackSimply.at(i).obj.id << ", MinD: " << d << ", ObjS: " << object_size << ", ObjI: " << jj << ", TrackS: " << old_size << ", TrackI: " << i << std::endl;

				if(obj_diff < m_MAX_ASSOCIATION_SIZE_DIFF &&  bInsidePolygon)
				 {
					 bFoundMatch = true;
					 iCloseset_track = i;
					 iCloseset_obj = jj;
					 break;
				 }

				if(obj_diff < m_MAX_ASSOCIATION_SIZE_DIFF && d < dCloseset)
				{
					dCloseset = d;
					iCloseset_track = i;
					iCloseset_obj = jj;
					min_size = obj_diff;
				}
			}

			if(bFoundMatch)
				break;
		}



		if(iCloseset_obj != -1 && iCloseset_track != -1 && (dCloseset <= m_MAX_ASSOCIATION_DISTANCE || bFoundMatch == true))
		{
			//std::cout << "MatchObj: " << m_TrackSimply.at(iCloseset_track).obj.id << ", MinD: " << dCloseset << ", MinS: " << min_size << ", ObjI" << iCloseset_obj <<", TrackI: " << iCloseset_track << ", ContourMatch: " << bFoundMatch << std::endl;
			m_DetectedObjects.at(iCloseset_obj).id = m_TrackSimply.at(iCloseset_track).obj.id;
			MergeObjectAndTrack(m_TrackSimply.at(iCloseset_track), m_DetectedObjects.at(iCloseset_obj));
			newObjects.push_back(m_TrackSimply.at(iCloseset_track));
			m_TrackSimply.erase(m_TrackSimply.begin()+iCloseset_track);
			m_DetectedObjects.erase(m_DetectedObjects.begin()+iCloseset_obj);
		}
		else
		{
			iTracksNumber = iTracksNumber + 1;
			if(iCloseset_obj != -1)
			{
				m_DetectedObjects.at(iCloseset_obj).id = iTracksNumber;
				KFTrackV track(m_DetectedObjects.at(iCloseset_obj).center.pos.x, m_DetectedObjects.at(iCloseset_obj).center.pos.y,m_DetectedObjects.at(iCloseset_obj).actual_yaw, m_DetectedObjects.at(iCloseset_obj).id, m_dt, m_nMinTrustAppearances);
				track.obj = m_DetectedObjects.at(iCloseset_obj);
				newObjects.push_back(track);
				//std::cout << "NewObj: " << iTracksNumber  << ", MinD: " << dCloseset << ", MinS: " << min_size << ", ObjI:" << iCloseset_obj <<", TrackI: " << iCloseset_track << ", ContMatch: " << bFoundMatch << std::endl;
				m_DetectedObjects.erase(m_DetectedObjects.begin()+iCloseset_obj);
			}
			else
			{
				m_DetectedObjects.at(0).id = iTracksNumber;
				KFTrackV track(m_DetectedObjects.at(0).center.pos.x, m_DetectedObjects.at(0).center.pos.y,m_DetectedObjects.at(0).actual_yaw, m_DetectedObjects.at(0).id, m_dt, m_nMinTrustAppearances);
				track.obj = m_DetectedObjects.at(0);
				newObjects.push_back(track);
				//std::cout << "NewObj: " << iTracksNumber  << ", MinD: " << dCloseset << ", MinS: " << min_size << ", ObjI:" << 0 <<", TrackI: " << iCloseset_track << ", ContMatch: " << bFoundMatch << std::endl;
				m_DetectedObjects.erase(m_DetectedObjects.begin()+0);
			}
		}
	}

	m_TrackSimply = newObjects;
}

void SimpleTracker::AssociateOnly()
{
	MatchClosest();

	for(unsigned int i =0; i< m_TrackSimply.size(); i++)
		m_TrackSimply.at(i).UpdateAssociateOnly(m_dt, m_TrackSimply.at(i).obj, m_TrackSimply.at(i).obj);

	m_DetectedObjects.clear();
	for(unsigned int i = 0; i< m_TrackSimply.size(); i++)
		m_DetectedObjects.push_back(m_TrackSimply.at(i).obj);
}

void SimpleTracker::AssociateSimply()
{
	for(unsigned int i = 0; i < m_TrackSimply.size(); i++)
		m_TrackSimply.at(i).m_bUpdated = false;

	MatchClosest();

	for(unsigned int i =0; i< m_TrackSimply.size(); i++)
		m_TrackSimply.at(i).UpdateTracking(m_dt, m_TrackSimply.at(i).obj, m_TrackSimply.at(i).obj);

	m_DetectedObjects.clear();
	for(unsigned int i = 0; i< m_TrackSimply.size(); i++)
		m_DetectedObjects.push_back(m_TrackSimply.at(i).obj);
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
			return;
		}
	}

	if(m_InterestRegions.size() > 0)
	{
		detectedObject.region_id = m_InterestRegions.at(m_InterestRegions.size()-1)->id;
		detectedObject.forget_time = m_InterestRegions.at(m_InterestRegions.size()-1)->forget_time;
	}
}

void SimpleTracker::AssociateAndTrack()
{
	for(unsigned int i = 0; i < m_TrackSimply.size(); i++)
		m_TrackSimply.at(i).m_bUpdated = false;


	std::cout << std::endl;
	while(m_DetectedObjects.size() > 0)
	{
		double iCloseset_track = -1;
		double iCloseset_obj = -1;
		double dCloseset = 99999999;
		bool bFoundMatch = false;
		double min_size = -1;
		std::cout << "DetObjSize: " <<  m_DetectedObjects.size() <<  ", TracksSize: " << m_TrackSimply.size() << std::endl;
		for(unsigned int jj = 0; jj < m_DetectedObjects.size(); jj++)
		{
			double object_size = hypot(m_DetectedObjects.at(jj).w, m_DetectedObjects.at(jj).l);

			for(unsigned int i = 0; i < m_TrackSimply.size(); i++)
			{
				double old_size = hypot(m_TrackSimply.at(i).obj.w, m_TrackSimply.at(i).obj.l);
				double obj_diff = fabs(object_size - old_size);
				double d = hypot(m_DetectedObjects.at(jj).center.pos.y-m_TrackSimply.at(i).obj.center.pos.y, m_DetectedObjects.at(jj).center.pos.x-m_TrackSimply.at(i).obj.center.pos.x);

				std::cout << "Test: " << m_TrackSimply.at(i).obj.id << ", MinD: " << d << ", ObjS: " << object_size << ", ObjI: " << jj << ", TrackS: " << old_size << ", TrackI: " << i << std::endl;

				if(obj_diff < m_MAX_ASSOCIATION_SIZE_DIFF && (InsidePolygon(m_TrackSimply.at(i).obj.contour, m_DetectedObjects.at(jj).center.pos) == 1 || InsidePolygon(m_DetectedObjects.at(jj).contour, m_TrackSimply.at(i).obj.center.pos) == 1) )
				 {
					 bFoundMatch = true;
					 iCloseset_track = i;
					 iCloseset_obj = jj;
					 break;
				 }

				if(obj_diff < m_MAX_ASSOCIATION_SIZE_DIFF && d < dCloseset)
				{
					dCloseset = d;
					iCloseset_track = i;
					iCloseset_obj = jj;
					min_size = obj_diff;
				}
			}

			if(bFoundMatch)
				break;
		}



		if(iCloseset_obj != -1 && iCloseset_track != -1 && (dCloseset <= m_MAX_ASSOCIATION_DISTANCE || bFoundMatch == true))
		{
			std::cout << "MatchObj: " << m_TrackSimply.at(iCloseset_track).obj.id << ", MinD: " << dCloseset << ", MinS: " << min_size << ", ObjI" << iCloseset_obj <<", TrackI: " << iCloseset_track << ", ContourMatch: " << bFoundMatch << std::endl;
			m_DetectedObjects.at(iCloseset_obj).id = m_TrackSimply.at(iCloseset_track).obj.id;
			MergeObjectAndTrack(m_TrackSimply.at(iCloseset_track), m_DetectedObjects.at(iCloseset_obj));
			AssociateToRegions(m_TrackSimply.at(iCloseset_track));
			m_DetectedObjects.erase(m_DetectedObjects.begin()+iCloseset_obj);
		}
		else
		{
			iTracksNumber = iTracksNumber + 1;
			if(iCloseset_obj != -1)
			{
				m_DetectedObjects.at(iCloseset_obj).id = iTracksNumber;
				KFTrackV track(m_DetectedObjects.at(iCloseset_obj).center.pos.x, m_DetectedObjects.at(iCloseset_obj).center.pos.y,m_DetectedObjects.at(iCloseset_obj).actual_yaw, m_DetectedObjects.at(iCloseset_obj).id, m_dt, m_nMinTrustAppearances);
				track.obj = m_DetectedObjects.at(iCloseset_obj);
				AssociateToRegions(track);
				m_TrackSimply.push_back(track);
				std::cout << "UnMachedObj: " << iTracksNumber  << ", MinD: " << dCloseset << ", MinS: " << min_size << ", ObjI:" << iCloseset_obj <<", TrackI: " << iCloseset_track << ", ContMatch: " << bFoundMatch << std::endl;
				m_DetectedObjects.erase(m_DetectedObjects.begin()+iCloseset_obj);
			}
			else
			{
				m_DetectedObjects.at(0).id = iTracksNumber;
				KFTrackV track(m_DetectedObjects.at(0).center.pos.x, m_DetectedObjects.at(0).center.pos.y,m_DetectedObjects.at(0).actual_yaw, m_DetectedObjects.at(0).id, m_dt, m_nMinTrustAppearances);
				track.obj = m_DetectedObjects.at(0);
				AssociateToRegions(track);
				m_TrackSimply.push_back(track);
				std::cout << "NewObj: " << iTracksNumber  << ", MinD: " << dCloseset << ", MinS: " << min_size << ", ObjI:" << 0 <<", TrackI: " << iCloseset_track << ", ContMatch: " << bFoundMatch << std::endl;
				m_DetectedObjects.erase(m_DetectedObjects.begin()+0);
			}
		}
	}

	for(unsigned int i =0; i< m_TrackSimply.size(); i++)
	{
		//if(m_TrackSimply.at(i).m_bUpdated)
			m_TrackSimply.at(i).UpdateTracking(m_dt, m_TrackSimply.at(i).obj, m_TrackSimply.at(i).obj);
//		else
//		{
////			double dx = 0;
////			double dy = 0;
////
////			if(m_TrackSimply.at(i).obj.bVelocity && m_TrackSimply.at(i).obj.center.v > 3)
////			{
////				dx = m_TrackSimply.at(i).obj.center.v * cos(m_TrackSimply.at(i).obj.center.pos.a) * m_dt;
////				dy = m_TrackSimply.at(i).obj.center.v * sin(m_TrackSimply.at(i).obj.center.pos.a) * m_dt;
////			}
////
////			m_TrackSimply.at(i).obj.center.pos.x += dx;
////			m_TrackSimply.at(i).obj.center.pos.y += dy;
////
////			for(unsigned int k=0; k < m_TrackSimply.at(i).obj.contour.size(); k++)
////			{
////				m_TrackSimply.at(i).obj.contour.at(k).x += dx;
////				m_TrackSimply.at(i).obj.contour.at(k).y += dy;
////			}
//
//			m_TrackSimply.at(i).PredictTracking(m_dt, m_TrackSimply.at(i).obj, m_TrackSimply.at(i).obj);
//		}
	}
}

void SimpleTracker::CleanOldTracks()
{
	m_DetectedObjects.clear();
	for(int i = 0; i< (int)m_TrackSimply.size(); i++)
	{
		if(m_TrackSimply.at(i).forget_time < 0 && m_TrackSimply.at(i).forget_time != NEVER_GORGET_TIME)
		{
			m_TrackSimply.erase(m_TrackSimply.begin()+i);
			i--;
		}
		else if(m_TrackSimply.at(i).m_iLife > m_nMinTrustAppearances)
		{
			m_DetectedObjects.push_back(m_TrackSimply.at(i).obj);
		}
	}
}

void SimpleTracker::MergeObjectAndTrack(KFTrackV& track, PlannerHNS::DetectedObject& obj)
{
	obj.centers_list = track.obj.centers_list;
	track.obj = obj;
	track.m_bUpdated = true;
}

int SimpleTracker::InsidePolygon(const std::vector<GPSPoint>& polygon,const GPSPoint& p)
{
	int counter = 0;
	int i;
	double xinters;
	GPSPoint p1,p2;
	int N = polygon.size();
	if(N <=0 ) return -1;

	p1 = polygon.at(0);
	for (i=1;i<=N;i++)
	{
		p2 = polygon.at(i % N);

		if (p.y > MIN(p1.y,p2.y))
		{
			if (p.y <= MAX(p1.y,p2.y))
			{
				if (p.x <= MAX(p1.x,p2.x))
				{
					if (p1.y != p2.y)
					{
						xinters = (p.y-p1.y)*(p2.x-p1.x)/(p2.y-p1.y)+p1.x;
						if (p1.x == p2.x || p.x <= xinters)
							counter++;
					}
				}
			}
		}
		p1 = p2;
	}

	if (counter % 2 == 0)
		return 0;
	else
		return 1;
}
}

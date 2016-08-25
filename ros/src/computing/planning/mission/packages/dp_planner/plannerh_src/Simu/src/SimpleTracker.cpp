/*
 * SimpleTracker.cpp
 *
 *  Created on: Aug 11, 2016
 *      Author: hatem
 */

#include "SimpleTracker.h"
#include "MatrixOperations.h"
#include "PlanningHelpers.h"

namespace SimulationNS
{

using namespace PlannerHNS;

SimpleTracker::SimpleTracker()
{
	m_pCarTracker = 0;
	iTracksNumber = 1;
	m_Car.center.cost = 0;
	m_Car.l = 4.5;
	m_Car.w = 1.9;
	m_Car.id = 0;
	m_Car.t = CAR;
}

SimpleTracker::~SimpleTracker()
{

}

void SimpleTracker::DoOneStep(const WayPoint& currPose, const std::vector<DetectedObject>& obj_list)
{
	//2- Associate with previous detected Obstacles
	//	- Transform To Prev Car Pose
	//	- find closest previous detected object
	//	- assign track id
	//3- Apply Track Step using objects in car coordinate
	//4- Transfere back to Global Coordinate

	//Transform Previous scan
//	bool bEqual = false;
//	if(m_DetectedObjects.size() == obj_list.size())
//	{
//		for(unsigned int i = 0; i < obj_list.size(); i++)
//		{
//			if(obj_list.at(i).center.pos.x == m_DetectedObjects.at(i).center.pos.x
//					&& obj_list.at(i).center.pos.y == m_DetectedObjects.at(i).center.pos.y
//					&& obj_list.at(i).center.pos.a == m_DetectedObjects.at(i).center.pos.a)
//				bEqual = true;
//
//		}
//	}
	m_DetectedObjects = obj_list;
	m_Car.center = currPose;

	AssociateObjects();

	Track(m_DetectedObjects);

	m_PrevDetectedObjects = m_DetectedObjects;
	m_PrevState = currPose;
}

void SimpleTracker::AssociateObjects()
{
	std::vector<DetectedObject> hidden_list;
	for(unsigned int i = 0 ; i < m_DetectedObjects.size(); i++)
	{
		double minCost = 99999999;
		double minID = -1;
//		if(m_DetectedObjects.size()>1)
//			minID = 0;

		DetectedObject trans_obj = m_DetectedObjects.at(i);
//		WayPoint prevStateTrans = m_PrevState;
//		CoordinateTransformPoint(m_Car.center, prevStateTrans.pos);
//		CoordinateTransform(prevStateTrans, trans_obj);

		for(unsigned int j = 0; j < m_PrevDetectedObjects.size(); j++)
		{
			m_DetectedObjects.at(i).center.cost = distance2points(trans_obj.center.pos, m_PrevDetectedObjects.at(j).center.pos);
//			for(unsigned int k = 0; k < trans_obj.contour.size(); k++)
//			{
//				//if(k < m_PrevDetectedObjects.at(j).contour.size())
//					m_DetectedObjects.at(i).center.cost += distance2points(trans_obj.contour.at(k), m_PrevDetectedObjects.at(j).contour.at(k));
//			}

			std::cout << "Cost Cost (" << i << "), " << m_PrevDetectedObjects.at(j).center.pos.ToString() << ","
					<< m_DetectedObjects.at(i).center.cost <<  ", contour: " << trans_obj.contour.size()
					<< ", " << trans_obj.center.pos.ToString() << std::endl;

			//m_DetectedObjects.at(i).center.cost = m_DetectedObjects.at(i).center.cost/(double)(trans_obj.contour.size()+1);

			if(m_DetectedObjects.at(i).center.cost < minCost)
			{
				minCost = m_DetectedObjects.at(i).center.cost;
				minID = m_PrevDetectedObjects.at(j).id;
			}
		}

		if(minID <= 0 || minCost > 1.5) // new Object enter the scene
		{
			iTracksNumber = iTracksNumber + 1;
			m_DetectedObjects.at(i).id = iTracksNumber;
			std::cout << "New Matching " << m_DetectedObjects.at(i).id << ", "<< minCost<< ", " << iTracksNumber << std::endl;
		}
		else
		{
			m_DetectedObjects.at(i).id = minID;
			std::cout << "Matched with ID  " << m_DetectedObjects.at(i).id << ", "<< minCost<< std::endl;
		}
	}

	for(unsigned int i = 0 ; i < m_PrevDetectedObjects.size(); i++)
	{
		DetectedObject* obj = &m_PrevDetectedObjects.at(i);
		bool bFound = false;
		for(unsigned int j = 0; j < m_DetectedObjects.size(); j++)
		{
			if(m_PrevDetectedObjects.at(i).id == m_DetectedObjects.at(j).id)
			{
				bFound = true;
				break;
			}
		}

		if(!bFound && obj->predicted_center.cost < 300)
		{
			obj->predicted_center.cost++;
			hidden_list.push_back(*obj);
		}
	}

	m_DetectedObjects.insert(m_DetectedObjects.begin(), hidden_list.begin(), hidden_list.end());
}

void SimpleTracker::Initialize(const WayPoint& currPose)
{
	//iTracksNumber = 1;
	m_Car.center.pos = currPose.pos;
	m_pCarTracker = new KFTrack(currPose.pos.x, currPose.pos.y, currPose.pos.a, 1);
	m_pCarTracker->UpdateTracking(m_Car.center.pos.x, m_Car.center.pos.y,m_Car.center.pos.a,
			m_Car.center.pos.x, m_Car.center.pos.y, m_Car.center.pos.a, m_Car.center.v);

}

void SimpleTracker::CreateTrack(DetectedObject& o)
{
	KFTrack* pT = new KFTrack(o.center.pos.x, o.center.pos.y,o.center.pos.a, o.id);
	o.id = pT->GetTrackID();
	pT->UpdateTracking(o.center.pos.x, o.center.pos.y, o.center.pos.a,
			o.center.pos.x, o.center.pos.y,o.center.pos.a, o.center.v);
	m_Tracks.push_back(pT);
}

KFTrack* SimpleTracker::FindTrack(long index)
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
	m_pCarTracker->UpdateTracking(m_Car.center.pos.x, m_Car.center.pos.y, m_Car.center.pos.a,
				m_Car.center.pos.x, m_Car.center.pos.y, m_Car.center.pos.a, m_Car.center.v);

	for(unsigned int i =0; i<objects_list.size(); i++)
	{
		if(objects_list[i].id >= 0)
		{
			KFTrack* pT = FindTrack(objects_list[i].id);
			if(pT)
			{
				pT->UpdateTracking(objects_list[i].center.pos.x, objects_list[i].center.pos.y, objects_list[i].center.pos.a,
						objects_list[i].center.pos.x, objects_list[i].center.pos.y, objects_list[i].center.pos.a,
						objects_list[i].center.v);
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
//	for(unsigned int j = 0 ; j < obj.contour.size(); j++)
//	{
//		obj.contour.at(j) = translationMat*obj.contour.at(j);
//		obj.contour.at(j) = rotationMat*obj.contour.at(j);
//	}
}

void SimpleTracker::CoordinateTransformPoint(const WayPoint& refCoordinate, GPSPoint& obj)
{
	Mat3 rotationMat(-refCoordinate.pos.a);
	Mat3 translationMat(-refCoordinate.pos.x, -refCoordinate.pos.y);
	obj = translationMat*obj;
	obj = rotationMat*obj;
}

} /* namespace BehaviorsNS */

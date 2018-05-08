/*
 * PolygonGenerator.h
 *
 *  Created on: Nov 2, 2016
 *      Author: ai-driver
 */

#ifndef POLYGONGENERATOR_H_
#define POLYGONGENERATOR_H_

#include "op_planner/RoadNetwork.h"
#include "op_planner/PlanningHelpers.h"
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

namespace PlannerXNS
{

using namespace PlannerHNS;

#define QUARTERS_NUMBER 8
#define MIN_POINTS_PER_QUARTER 1
#define MIN_DISTANCE_BETWEEN_CORNERS 0.5

class QuarterView
{
public:
	int id;
	int min_ang;
	int max_ang;
	std::vector<WayPoint> vectorsFromCenter;

	QuarterView(const int& min_a, const int& max_a, const int& index)
	{
		min_ang = min_a;
		max_ang = max_a;
		id = index;
	}

	void InitQuarterView(const int& min_a, const int& max_a, const int& index)
	{
		min_ang = min_a;
		max_ang = max_a;
		id = index;
	}

	bool UpdateQuarterView(const WayPoint& v)
	{
		if(v.pos.a <= min_ang || v.pos.a > max_ang)
			return false;

		bool bGreaterFound = false;
		unsigned int greaterIndex = 0;
		for(unsigned int i=0; i< vectorsFromCenter.size(); i++)
		{
			if(vectorsFromCenter.at(i).cost > v.cost)
			{
				bGreaterFound = true;
				greaterIndex = i;
				break;
			}
		}

		if(bGreaterFound)
		{
			if(greaterIndex < vectorsFromCenter.size())
				vectorsFromCenter.insert(vectorsFromCenter.begin()+greaterIndex, v);
			else
				vectorsFromCenter.push_back(v);
		}
		else
			vectorsFromCenter.push_back(v);

		return true;

	}

	bool GetMaxPoint(WayPoint& maxPoint)
	{
		if(vectorsFromCenter.size()==0)
			return false;

		maxPoint = vectorsFromCenter.at(vectorsFromCenter.size()-1);
		return vectorsFromCenter.size();
	}

	int GetPointsNumber()
	{
		return vectorsFromCenter.size();
	}
};

class PolygonGenerator
{

public:

	GPSPoint m_Centroid;
	PolygonGenerator();
	virtual ~PolygonGenerator();
	void CheckConvexPoligon(std::vector<WayPoint>& polygon);
	GPSPoint CalculateCentroid(const pcl::PointCloud<pcl::PointXYZ>& cluster);
	std::vector<QuarterView> CreateQuarterViews(const int& nResolution);
	std::vector<GPSPoint> EstimateClusterPolygon(const pcl::PointCloud<pcl::PointXYZ>& cluster, const GPSPoint& original_centroid );
};

} /* namespace PlannerXNS */

#endif /* POLYGONGENERATOR_H_ */

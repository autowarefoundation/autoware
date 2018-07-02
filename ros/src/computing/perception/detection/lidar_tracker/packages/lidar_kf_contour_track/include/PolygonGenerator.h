
/// \file  PolygonGenerator.h
/// \brief Generate convex hull from point cloud cluster of detected object
/// \author Hatem Darweesh
/// \date Nov 2, 2016

#ifndef OP_POLYGONGENERATOR_H_
#define OP_POLYGONGENERATOR_H_

#include "op_planner/RoadNetwork.h"
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

namespace ContourTrackerNS
{

class QuarterView
{
public:
	int id;
	int min_ang;
	int max_ang;
	PlannerHNS::WayPoint max_from_center;
	bool bFirst;

	QuarterView(const int& min_a, const int& max_a, const int& index)
	{
		min_ang = min_a;
		max_ang = max_a;
		id = index;
		bFirst = true;
	}

	void InitQuarterView(const int& min_a, const int& max_a, const int& index)
	{
		min_ang = min_a;
		max_ang = max_a;
		id = index;
		bFirst = true;
	}

	void ResetQuarterView()
	{
		bFirst = true;
	}

	bool UpdateQuarterView(const PlannerHNS::WayPoint& v)
	{
		if(v.pos.a <= min_ang || v.pos.a > max_ang)
			return false;

		if(bFirst)
		{
			max_from_center = v;
			bFirst = false;
		}
		else if(v.cost > max_from_center.cost)
			max_from_center = v;

		return true;
	}

	bool GetMaxPoint(PlannerHNS::WayPoint& maxPoint)
	{
		if(bFirst)
			return false;
		else
			maxPoint = max_from_center;

		return true;
	}
};

class PolygonGenerator
{

public:

	PlannerHNS::GPSPoint m_Centroid;
	std::vector<QuarterView> m_Quarters;
	std::vector<PlannerHNS::GPSPoint> m_Polygon;

	PolygonGenerator(int nQuarters);
	virtual ~PolygonGenerator();
	std::vector<QuarterView> CreateQuarterViews(const int& nResolution);
	std::vector<PlannerHNS::GPSPoint> EstimateClusterPolygon(const pcl::PointCloud<pcl::PointXYZ>& cluster, const PlannerHNS::GPSPoint& original_centroid, PlannerHNS::GPSPoint& new_centroid, const double& polygon_resolution = 1.0);
};

} /* namespace PlannerXNS */

#endif /* OP_POLYGONGENERATOR_H_ */

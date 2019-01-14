
/// \file PlanningHelpers.cpp
/// \brief Helper functions for planning algorithms
/// \author Hatem Darweesh
/// \date Jun 16, 2016


#include "op_planner/PlanningHelpers.h"
#include "op_planner/MatrixOperations.h"
#include <string>
#include <float.h>

using namespace UtilityHNS;
using namespace std;

namespace PlannerHNS
{

std::vector<std::pair<GPSPoint, GPSPoint> > PlanningHelpers::m_TestingClosestPoint;

PlanningHelpers::PlanningHelpers()
{
}

PlanningHelpers::~PlanningHelpers()
{
}

bool PlanningHelpers::GetRelativeInfoRange(const std::vector<std::vector<WayPoint> >& trajectories, const WayPoint& p,const double& searchDistance, RelativeInfo& info)
{
	if(trajectories.size() == 0) return false;

	vector<RelativeInfo> infos;
	for(unsigned int i=0; i < trajectories.size(); i++)
	{
		RelativeInfo info_item;
		GetRelativeInfo(trajectories.at(i), p, info_item);
		double angle_diff = UtilityH::AngleBetweenTwoAnglesPositive(info_item.perp_point.pos.a, p.pos.a)*RAD2DEG;
		if(angle_diff < 75)
		{
			info_item.iGlobalPath = i;
			infos.push_back(info_item);
		}
	}

	if(infos.size() == 0)
		return false;
	else if(infos.size() == 1)
	{
		info = infos.at(0);
		return true;
	}

	double minCost = DBL_MAX;
	int min_index = 0;

	for(unsigned int i=0 ; i< infos.size(); i++)
	{
		if(searchDistance > 0)
		{
			double laneChangeCost = trajectories.at(infos.at(i).iGlobalPath).at(infos.at(i).iFront).laneChangeCost;
			if(fabs(infos.at(i).perp_distance) < searchDistance && laneChangeCost < minCost)
			{
				min_index = i;
				minCost = laneChangeCost;
			}
		}
		else
		{
			if(fabs(infos.at(i).perp_distance) < minCost)
			{
				min_index = i;
				minCost = infos.at(i).perp_distance;
			}
		}
	}

	info = infos.at(min_index);

	return true;
}


bool PlanningHelpers::GetRelativeInfo(const std::vector<WayPoint>& trajectory, const WayPoint& p, RelativeInfo& info, const int& prevIndex )
{
	if(trajectory.size() < 2) return false;

	WayPoint p0, p1;
	if(trajectory.size()==2)
	{
		p0 = trajectory.at(0);
		p1 = WayPoint((trajectory.at(0).pos.x+trajectory.at(1).pos.x)/2.0,
					  (trajectory.at(0).pos.y+trajectory.at(1).pos.y)/2.0,
					  (trajectory.at(0).pos.z+trajectory.at(1).pos.z)/2.0, trajectory.at(0).pos.a);
		info.iFront = 1;
		info.iBack = 0;
	}
	else
	{
		info.iFront = GetClosestNextPointIndexFast(trajectory, p, prevIndex);
//		WayPoint p2 = p;
//		int old_index = GetClosestNextPointIndex(trajectory, p2, prevIndex);
//		if(old_index != info.iFront)
//			cout << " Alert Alert !!!! fast: " << info.iFront << ", slow: " << old_index  << endl;

		if(info.iFront > 0)
			info.iBack = info.iFront -1;
		else
			info.iBack = 0;

		if(info.iFront == 0)
		{
			p0 = trajectory.at(info.iFront);
			p1 = trajectory.at(info.iFront+1);
		}
		else if(info.iFront > 0 && info.iFront < trajectory.size()-1)
		{
			p0 = trajectory.at(info.iFront-1);
			p1 = trajectory.at(info.iFront);
		}
		else
		{
			p0 = trajectory.at(info.iFront-1);
			p1 = WayPoint((p0.pos.x+trajectory.at(info.iFront).pos.x)/2.0, (p0.pos.y+trajectory.at(info.iFront).pos.y)/2.0, (p0.pos.z+trajectory.at(info.iFront).pos.z)/2.0, p0.pos.a);
		}
	}

	WayPoint prevWP = p0;
	Mat3 rotationMat(-p1.pos.a);
	Mat3 translationMat(-p.pos.x, -p.pos.y);
	Mat3 invRotationMat(p1.pos.a);
	Mat3 invTranslationMat(p.pos.x, p.pos.y);

	p0.pos = translationMat*p0.pos;
	p0.pos = rotationMat*p0.pos;

	p1.pos = translationMat*p1.pos;
	p1.pos = rotationMat*p1.pos;

	double m = (p1.pos.y-p0.pos.y)/(p1.pos.x-p0.pos.x);
	info.perp_distance = p1.pos.y - m*p1.pos.x; // solve for x = 0

	if(std::isnan(info.perp_distance) || std::isinf(info.perp_distance)) info.perp_distance = 0;

	info.to_front_distance = fabs(p1.pos.x); // distance on the x axes


	info.perp_point = p1;
	info.perp_point.pos.x = 0; // on the same y axis of the car
	info.perp_point.pos.y = info.perp_distance; //perp distance between the car and the trajectory

	info.perp_point.pos = invRotationMat  * info.perp_point.pos;
	info.perp_point.pos = invTranslationMat  * info.perp_point.pos;

	info.from_back_distance = hypot(info.perp_point.pos.y - prevWP.pos.y, info.perp_point.pos.x - prevWP.pos.x);

	info.angle_diff = UtilityH::AngleBetweenTwoAnglesPositive(p1.pos.a, p.pos.a)*RAD2DEG;

	return true;
}

bool PlanningHelpers::GetRelativeInfoLimited(const std::vector<WayPoint>& trajectory, const WayPoint& p, RelativeInfo& info, const int& prevIndex )
{
	if(trajectory.size() < 2) return false;

	WayPoint p0, p1;

	if(trajectory.size()==2)
	{
		vector<WayPoint> _trajectory;
		p0 = trajectory.at(0);
		p1 = p0;
		p1 = WayPoint((trajectory.at(0).pos.x+trajectory.at(1).pos.x)/2.0,
					  (trajectory.at(0).pos.y+trajectory.at(1).pos.y)/2.0,
					  (trajectory.at(0).pos.z+trajectory.at(1).pos.z)/2.0, trajectory.at(0).pos.a);
		_trajectory.push_back(p0);
		_trajectory.push_back(p1);
		_trajectory.push_back(trajectory.at(1));

		info.iFront = GetClosestNextPointIndexFast(_trajectory, p, prevIndex);
		if(info.iFront > 0)
			info.iBack = info.iFront -1;
		else
			info.iBack = 0;

		if(info.iFront == 0)
		{
			p0 = _trajectory.at(info.iFront);
			p1 = _trajectory.at(info.iFront+1);
		}
		else if(info.iFront > 0 && info.iFront < _trajectory.size()-1)
		{
			p0 = _trajectory.at(info.iFront-1);
			p1 = _trajectory.at(info.iFront);
		}
		else
		{
			p0 = _trajectory.at(info.iFront-1);
			p1 = WayPoint((p0.pos.x+_trajectory.at(info.iFront).pos.x)/2.0, (p0.pos.y+_trajectory.at(info.iFront).pos.y)/2.0, (p0.pos.z+_trajectory.at(info.iFront).pos.z)/2.0, p0.pos.a);
		}

		WayPoint prevWP = p0;
		Mat3 rotationMat(-p1.pos.a);
		Mat3 translationMat(-p.pos.x, -p.pos.y);
		Mat3 invRotationMat(p1.pos.a);
		Mat3 invTranslationMat(p.pos.x, p.pos.y);

		p0.pos = translationMat*p0.pos;
		p0.pos = rotationMat*p0.pos;

		p1.pos = translationMat*p1.pos;
		p1.pos = rotationMat*p1.pos;

		double m = (p1.pos.y-p0.pos.y)/(p1.pos.x-p0.pos.x);
		info.perp_distance = p1.pos.y - m*p1.pos.x; // solve for x = 0

		if(std::isnan(info.perp_distance) || std::isinf(info.perp_distance)) info.perp_distance = 0;

		info.to_front_distance = fabs(p1.pos.x); // distance on the x axes

		info.perp_point = p1;
		info.perp_point.pos.x = 0; // on the same y axis of the car
		info.perp_point.pos.y = info.perp_distance; //perp distance between the car and the _trajectory

		info.perp_point.pos = invRotationMat  * info.perp_point.pos;
		info.perp_point.pos = invTranslationMat  * info.perp_point.pos;

		info.from_back_distance = hypot(info.perp_point.pos.y - prevWP.pos.y, info.perp_point.pos.x - prevWP.pos.x);

		info.angle_diff = UtilityH::AngleBetweenTwoAnglesPositive(p1.pos.a, p.pos.a)*RAD2DEG;

		info.bAfter = false;
		info.bBefore = false;

		if(info.iFront == 0)
		{
			info.bBefore = true;
		}
		else if(info.iFront == _trajectory.size()-1)
		{
			int s = _trajectory.size();
			double angle_befor_last = UtilityH::FixNegativeAngle(atan2(_trajectory.at(s-2).pos.y - _trajectory.at(s-1).pos.y, _trajectory.at(s-2).pos.x - _trajectory.at(s-1).pos.x));
			double angle_from_perp = UtilityH::FixNegativeAngle(atan2(info.perp_point.pos.y - _trajectory.at(s-1).pos.y, info.perp_point.pos.x - _trajectory.at(s-1).pos.x));
			double diff_last_perp = UtilityH::AngleBetweenTwoAnglesPositive(angle_befor_last, angle_from_perp);
			info.after_angle = diff_last_perp;
			if(diff_last_perp > M_PI_2)
			{
				info.bAfter = true;
			}

		}
	}
	else
	{
		info.iFront = GetClosestNextPointIndexFast(trajectory, p, prevIndex);
		if(info.iFront > 0)
			info.iBack = info.iFront -1;
		else
			info.iBack = 0;

		if(info.iFront == 0)
		{
			p0 = trajectory.at(info.iFront);
			p1 = trajectory.at(info.iFront+1);
		}
		else if(info.iFront > 0 && info.iFront < trajectory.size()-1)
		{
			p0 = trajectory.at(info.iFront-1);
			p1 = trajectory.at(info.iFront);
		}
		else
		{
			p0 = trajectory.at(info.iFront-1);
			p1 = WayPoint((p0.pos.x+trajectory.at(info.iFront).pos.x)/2.0, (p0.pos.y+trajectory.at(info.iFront).pos.y)/2.0, (p0.pos.z+trajectory.at(info.iFront).pos.z)/2.0, p0.pos.a);
		}

		WayPoint prevWP = p0;
		Mat3 rotationMat(-p1.pos.a);
		Mat3 translationMat(-p.pos.x, -p.pos.y);
		Mat3 invRotationMat(p1.pos.a);
		Mat3 invTranslationMat(p.pos.x, p.pos.y);

		p0.pos = translationMat*p0.pos;
		p0.pos = rotationMat*p0.pos;

		p1.pos = translationMat*p1.pos;
		p1.pos = rotationMat*p1.pos;

		double m = (p1.pos.y-p0.pos.y)/(p1.pos.x-p0.pos.x);
		info.perp_distance = p1.pos.y - m*p1.pos.x; // solve for x = 0

		if(std::isnan(info.perp_distance) || std::isinf(info.perp_distance)) info.perp_distance = 0;

		info.to_front_distance = fabs(p1.pos.x); // distance on the x axes

		info.perp_point = p1;
		info.perp_point.pos.x = 0; // on the same y axis of the car
		info.perp_point.pos.y = info.perp_distance; //perp distance between the car and the trajectory

		info.perp_point.pos = invRotationMat  * info.perp_point.pos;
		info.perp_point.pos = invTranslationMat  * info.perp_point.pos;

		info.from_back_distance = hypot(info.perp_point.pos.y - prevWP.pos.y, info.perp_point.pos.x - prevWP.pos.x);

		info.angle_diff = UtilityH::AngleBetweenTwoAnglesPositive(p1.pos.a, p.pos.a)*RAD2DEG;

		info.bAfter = false;
		info.bBefore = false;

		if(info.iFront == 0)
		{
			info.bBefore = true;
		}
		else if(info.iFront == trajectory.size()-1)
		{
			int s = trajectory.size();
			double angle_befor_last = UtilityH::FixNegativeAngle(atan2(trajectory.at(s-2).pos.y - trajectory.at(s-1).pos.y, trajectory.at(s-2).pos.x - trajectory.at(s-1).pos.x));
			double angle_from_perp = UtilityH::FixNegativeAngle(atan2(info.perp_point.pos.y - trajectory.at(s-1).pos.y, info.perp_point.pos.x - trajectory.at(s-1).pos.x));
			double diff_last_perp = UtilityH::AngleBetweenTwoAnglesPositive(angle_befor_last, angle_from_perp);
			info.after_angle = diff_last_perp;
			if(diff_last_perp > M_PI_2)
			{
				info.bAfter = true;
			}

		}
	}

	return true;
}

bool PlanningHelpers::GetThreePointsInfo(const WayPoint& p0, const WayPoint& p1, const WayPoint& p2, WayPoint& perp_p, double& long_d, double lat_d)
{

	if(p0.pos.x == p1.pos.x && p0.pos.y == p1.pos.y) return false;
	if(p0.pos.x == p2.pos.x && p0.pos.y == p2.pos.y) return false;
	if(p1.pos.x == p2.pos.x && p1.pos.y == p2.pos.y) return false;

	perp_p = p1;
	WayPoint first_p = p0;
	double angle_x = atan2(p1.pos.y-p0.pos.y, p1.pos.x-p0.pos.x);

	Mat3 rotationMat(-angle_x);
	Mat3 translationMat(-p2.pos.x, -p2.pos.y);
	Mat3 invRotationMat(angle_x);
	Mat3 invTranslationMat(p2.pos.x, p2.pos.y);

	first_p.pos = translationMat*first_p.pos;
	first_p.pos = rotationMat*first_p.pos;

	perp_p.pos = translationMat*perp_p.pos;
	perp_p.pos = rotationMat*perp_p.pos;

	if(perp_p.pos.x-first_p.pos.x == 0) return false;

	double m = (perp_p.pos.y-first_p.pos.y)/(perp_p.pos.x-first_p.pos.x);
	lat_d = perp_p.pos.y - m*perp_p.pos.x; // solve for x = 0

	if(std::isnan(lat_d) || std::isinf(lat_d)) return false;

	if(perp_p.pos.x < 0)
		return false;

	perp_p.pos.x = 0; // on the same y axis of the car
	perp_p.pos.y = lat_d; //perp distance between the car and the trajectory

	perp_p.pos = invRotationMat  * perp_p.pos;
	perp_p.pos = invTranslationMat  * perp_p.pos;

	long_d = hypot(perp_p.pos.y - first_p.pos.y, perp_p.pos.x - first_p.pos.x);

	return true;
}

WayPoint PlanningHelpers::GetFollowPointOnTrajectory(const std::vector<WayPoint>& trajectory, const RelativeInfo& init_p, const double& distance, unsigned int& point_index)
{
	WayPoint follow_point;

	if(trajectory.size()==0) return follow_point;

	//condition 1, if far behind the first point on the trajectory
	int local_i = init_p.iFront;

	if(init_p.iBack == 0 && init_p.iBack == init_p.iFront && init_p.from_back_distance > distance)
	{
		follow_point = trajectory.at(init_p.iFront);
		follow_point.pos.x = init_p.perp_point.pos.x + distance * cos(follow_point.pos.a);
		follow_point.pos.y = init_p.perp_point.pos.y + distance * sin(follow_point.pos.a);
	}
	//condition 2, if far after the last point on the trajectory
	else if(init_p.iFront == (int)trajectory.size() - 1)
	{
		follow_point = trajectory.at(init_p.iFront);
		follow_point.pos.x = init_p.perp_point.pos.x + distance * cos(follow_point.pos.a);
		follow_point.pos.y = init_p.perp_point.pos.y + distance * sin(follow_point.pos.a);
	}
	else
	{
		double d = init_p.to_front_distance;
		while(local_i < (int)trajectory.size()-1 && d < distance)
		{
			local_i++;
			d += hypot(trajectory.at(local_i).pos.y - trajectory.at(local_i-1).pos.y, trajectory.at(local_i).pos.x - trajectory.at(local_i-1).pos.x);
		}

		double d_part = distance - d;

		follow_point = trajectory.at(local_i);
		follow_point.pos.x = follow_point.pos.x + d_part * cos(follow_point.pos.a);
		follow_point.pos.y = follow_point.pos.y + d_part * sin(follow_point.pos.a);
	}

	point_index = local_i;

	return follow_point;
}

double PlanningHelpers::GetExactDistanceOnTrajectory(const std::vector<WayPoint>& trajectory, const RelativeInfo& p1,const RelativeInfo& p2)
{
	if(trajectory.size() == 0) return 0;

	if(p2.iFront == p1.iFront && p2.iBack == p1.iBack)
	{
		return p2.to_front_distance - p1.to_front_distance;
	}
	else if(p2.iBack >= p1.iFront)
	{
		double d_on_path = p1.to_front_distance + p2.from_back_distance;
		for(int i = p1.iFront; i < p2.iBack; i++)
			d_on_path += hypot(trajectory.at(i+1).pos.y - trajectory.at(i).pos.y, trajectory.at(i+1).pos.x - trajectory.at(i).pos.x);

		return d_on_path;
	}
	else if(p2.iFront <= p1.iBack)
	{
		double d_on_path = p1.from_back_distance + p2.to_front_distance;
		for(int i = p2.iFront; i < p1.iBack; i++)
			d_on_path += hypot(trajectory.at(i+1).pos.y - trajectory.at(i).pos.y, trajectory.at(i+1).pos.x - trajectory.at(i).pos.x);

		return -d_on_path;
	}
	else
	{
		return 0;
	}
}

int PlanningHelpers::GetClosestNextPointIndex_obsolete(const vector<WayPoint>& trajectory, const WayPoint& p,const int& prevIndex )
{
	if(trajectory.size() == 0 || prevIndex < 0) return 0;

	double d = 0, minD = DBL_MAX;
	int min_index  = prevIndex;

	for(unsigned int i=prevIndex; i< trajectory.size(); i++)
	{
		d  = distance2pointsSqr(trajectory.at(i).pos, p.pos);
		if(d < minD)
		{
			min_index = i;
			minD = d;
		}
	}

//	cout << "Slow=> Start: " << 0;
//	cout << ", End: " << trajectory.size();
//	cout << ", Minimum Before: " << min_index;

	if(min_index < (int)trajectory.size()-2)
	{
		GPSPoint curr, next;
		curr = trajectory.at(min_index).pos;
		next = trajectory.at(min_index+1).pos;
		GPSPoint v_1(p.pos.x - curr.x   ,p.pos.y - curr.y,0,0);
		double norm1 = pointNorm(v_1);
		GPSPoint v_2(next.x - curr.x,next.y - curr.y,0,0);
		double norm2 = pointNorm(v_2);
		double dot_pro = v_1.x*v_2.x + v_1.y*v_2.y;
		double a = UtilityH::FixNegativeAngle(acos(dot_pro/(norm1*norm2)));
		if(a <= M_PI_2)
			min_index = min_index+1;
	}

//	cout << ", Minimum After: " << min_index << ", Big O: " << trajectory.size() << endl;

	return min_index;
}

int PlanningHelpers::GetClosestNextPointIndexFastV2(const vector<WayPoint>& trajectory, const WayPoint& p,const int& prevIndex)
{

	int size = (int)trajectory.size();

		if(size < 2 || prevIndex < 0) return 0;

		double d = 0, minD = DBL_MAX;


		double resolution = hypot(trajectory[1].pos.y -trajectory[0].pos.y , trajectory[1].pos.x -trajectory[0].pos.x);
		double d_to_zero = hypot(p.pos.y -trajectory[0].pos.y , p.pos.x - trajectory[0].pos.x);
		double d_to_size = hypot(trajectory[size-1].pos.y - p.pos.y , trajectory[size-1].pos.x - p.pos.x);

		int iStart = d_to_zero / resolution;
		WayPoint perp_p;
		double lat_d = 0;
		double long_d = 0;

		if(iStart > 0 && iStart < size-1 &&  GetThreePointsInfo(trajectory.at(0), trajectory.at(iStart), p, perp_p, long_d, lat_d))
		{
	//		m_TestingClosestPoint.push_back(make_pair(trajectory.at(0).pos, p.pos));
	//		m_TestingClosestPoint.push_back(make_pair(trajectory.at(iStart).pos, p.pos));
			iStart = long_d / resolution;
		}

		if(iStart>0)
			iStart--;

		int iEnd = size - (d_to_size / resolution);

		if(iEnd >= 0 && iEnd < size-2 &&  GetThreePointsInfo(trajectory.at(size-1), trajectory.at(iEnd), p, perp_p, long_d, lat_d))
		{
	//		m_TestingClosestPoint.push_back(make_pair(trajectory.at(size-1).pos, p.pos));
	//		m_TestingClosestPoint.push_back(make_pair(trajectory.at(iEnd).pos, p.pos));
			iEnd = size - (long_d / resolution);
		}

		if(iEnd < size-1)
			iEnd++;

	//	cout << "Fast=>";
	//	cout << " Start: " << iStart;
	//	cout << ", End: " << iEnd;

		double d_from_start = d_to_zero;
		if(iStart < size)
			d_from_start = hypot(trajectory[iStart].pos.y - p.pos.y , trajectory[iStart].pos.x - p.pos.x);

		double d_from_end = d_to_size;
		if(iEnd >= 0)
			d_from_end = hypot(trajectory[iEnd].pos.y - p.pos.y , trajectory[iEnd].pos.x - p.pos.x);

		if(iStart >= size && iEnd < 0)
		{
			if(d_to_zero < d_to_size)
			{
				iStart = 0;
				iEnd = size/2 -1;
			}
			else
			{
				iStart = size/2;
				iEnd = size - 1;
			}
		}
		else
		{
			if(iStart >=size || (d_from_start > d_to_zero))
				iStart = 0;

			if(iEnd < 0 || (d_from_end > d_to_size))
				iEnd = size-1;
		}

		if(iStart > iEnd)
			iEnd = size-1;

		int min_index  =  iStart;

		int ncout = 0;
		for(int i=iStart; i<= iEnd; i++)
		{
			d  = distance2pointsSqr(trajectory[i].pos, p.pos);
			if(d < minD)
			{
				min_index = i;
				minD = d;
			}
			ncout++;
		}

	//	cout << ", Minimum Before: " << min_index;

		if(min_index < size-2)
		{
			GPSPoint curr, next;
			curr = trajectory[min_index].pos;
			next = trajectory[min_index+1].pos;
			GPSPoint v_1(p.pos.x - curr.x   ,p.pos.y - curr.y,0,0);
			double norm1 = pointNorm(v_1);
			GPSPoint v_2(next.x - curr.x,next.y - curr.y,0,0);
			double norm2 = pointNorm(v_2);
			double dot_pro = v_1.x*v_2.x + v_1.y*v_2.y;
			double a = UtilityH::FixNegativeAngle(acos(dot_pro/(norm1*norm2)));
			if(a <= M_PI_2)
				min_index = min_index+1;
		}

	//	m_TestingClosestPoint.push_back(make_pair(trajectory.at(min_index).pos, p.pos));

	//	cout << ", Minimum After: " << min_index << ", Big O: " << ncout << endl;
	//	cout << "d_zero: " << d_to_zero << ", d_start: " << d_from_start << endl;
	//	cout << "d_size: " << d_to_size << ", d_end: " << d_from_end << endl;
		return min_index;


}

int PlanningHelpers::GetClosestNextPointIndexFast(const vector<WayPoint>& trajectory, const WayPoint& p,const int& prevIndex )
{
	int size = (int)trajectory.size();

		if(size < 2 || prevIndex < 0) return 0;

		double d = 0, minD = DBL_MAX;
		int min_index  = prevIndex;
		int iStart = prevIndex;
		int iEnd = size;
		double resolution = hypot(trajectory[1].pos.y -trajectory[0].pos.y , trajectory[1].pos.x -trajectory[0].pos.x);

		//divide every 5 meters
		int skip_factor = 5;
		if(resolution > skip_factor)
			resolution = skip_factor;


		int skip = 1;
		if(resolution > 0)
			skip = skip_factor/resolution;

		for(int i=0; i< size; i+=skip)
		{
			if(i+skip/2 < size)
				d  = (distance2pointsSqr(trajectory[i].pos, p.pos) + distance2pointsSqr(trajectory[i+skip/2].pos, p.pos))/2.0;
			else
				d  = distance2pointsSqr(trajectory[i].pos, p.pos);
			if(d < minD)
			{
				iStart = i-skip;
				iEnd = i+skip;
				minD = d;
				min_index = i;
			}
		}

		if((size - skip/2 - 1) > 0)
			d  = (distance2pointsSqr(trajectory[size-1].pos, p.pos) + distance2pointsSqr(trajectory[size - skip/2 -1 ].pos, p.pos))/2.0;
		else
			d  = distance2pointsSqr(trajectory[size-1].pos, p.pos);

		if(d < minD)
		{
			iStart = size-skip;
			iEnd = size+skip;
			minD = d;
			min_index = size-1;
		}

		if(iStart < 0) iStart = 0;
		if(iEnd >= size) iEnd = size -1;

		for(int i=iStart; i< iEnd; i++)
		{
			d  = distance2pointsSqr(trajectory[i].pos, p.pos);
			if(d < minD)
			{
				min_index = i;
				minD = d;
			}
		}

		if(min_index < size-1)
		{
			GPSPoint curr, next;
			curr = trajectory[min_index].pos;
			next = trajectory[min_index+1].pos;
			GPSPoint v_1(p.pos.x - curr.x   ,p.pos.y - curr.y,0,0);
			double norm1 = pointNorm(v_1);
			GPSPoint v_2(next.x - curr.x,next.y - curr.y,0,0);
			double norm2 = pointNorm(v_2);
			double dot_pro = v_1.x*v_2.x + v_1.y*v_2.y;
			double a = UtilityH::FixNegativeAngle(acos(dot_pro/(norm1*norm2)));
			if(a <= M_PI_2)
				min_index = min_index+1;
		}

		//m_TestingClosestPoint.push_back(make_pair(trajectory.at(min_index).pos, p.pos));

		return min_index;
}

int PlanningHelpers::GetClosestNextPointIndexDirectionFast(const vector<WayPoint>& trajectory, const WayPoint& p,const int& prevIndex )
{
	int size = (int)trajectory.size();

	if(size < 2 || prevIndex < 0) return 0;

	double d = 0, minD = DBL_MAX;
	int min_index  = prevIndex;

	for(unsigned int i=prevIndex; i< size; i++)
	{
		d  = distance2pointsSqr(trajectory[i].pos, p.pos);
		double angle_diff = UtilityH::AngleBetweenTwoAnglesPositive(trajectory[i].pos.a, p.pos.a)*RAD2DEG;

		if(d < minD && angle_diff < 45)
		{
			min_index = i;
			minD = d;
		}
	}

	if(min_index < (int)trajectory.size()-2)
	{
		GPSPoint curr, next;
		curr = trajectory.at(min_index).pos;
		next = trajectory.at(min_index+1).pos;
		GPSPoint v_1(p.pos.x - curr.x   ,p.pos.y - curr.y,0,0);
		double norm1 = pointNorm(v_1);
		GPSPoint v_2(next.x - curr.x,next.y - curr.y,0,0);
		double norm2 = pointNorm(v_2);
		double dot_pro = v_1.x*v_2.x + v_1.y*v_2.y;
		double a = UtilityH::FixNegativeAngle(acos(dot_pro/(norm1*norm2)));
		if(a <= M_PI_2)
			min_index = min_index+1;
	}

	return min_index;
}

int PlanningHelpers::GetClosestPointIndex_obsolete(const vector<WayPoint>& trajectory, const WayPoint& p,const int& prevIndex )
{
	if(trajectory.size() == 0 || prevIndex < 0) return 0;

	double d = 0, minD = DBL_MAX;
	int min_index  = prevIndex;

	for(unsigned int i=prevIndex; i< trajectory.size(); i++)
	{
		d  = distance2pointsSqr(trajectory.at(i).pos, p.pos);
		if(d < minD)
		{
			min_index = i;
			minD = d;
		}
	}

	return min_index;
}

WayPoint PlanningHelpers::GetPerpendicularOnTrajectory_obsolete(const vector<WayPoint>& trajectory, const WayPoint& p, double& distance, const int& prevIndex )
{
	if(trajectory.size() < 2) return p;

	WayPoint p0, p1, p2, perp;
	if(trajectory.size()==2)
	{
		p0 = trajectory.at(0);
		p1 = WayPoint((trajectory.at(0).pos.x+trajectory.at(1).pos.x)/2.0,
					  (trajectory.at(0).pos.y+trajectory.at(1).pos.y)/2.0,
					  (trajectory.at(0).pos.z+trajectory.at(1).pos.z)/2.0, trajectory.at(0).pos.a);
		p2 = trajectory.at(1);
	}
	else
	{
		int next_index = GetClosestNextPointIndex_obsolete(trajectory, p, prevIndex);

		if(next_index == 0)
		{
			p0 = trajectory[next_index];
			p1 = trajectory[next_index+1];
			p2 = trajectory[next_index+2];
		}
		else if(next_index > 0 && next_index < trajectory.size()-1)
		{
			p0 = trajectory[next_index-1];
			p1 = trajectory[next_index];
			p2 = trajectory[next_index+1];
		}
		else
		{
			p0 = trajectory[next_index-1];
			p2 = trajectory[next_index];

			p1 = WayPoint((p0.pos.x+p2.pos.x)/2.0, (p0.pos.y+p2.pos.y)/2.0, (p0.pos.z+p2.pos.z)/2.0, p0.pos.a);

		}
	}

	Mat3 rotationMat(-p1.pos.a);
	Mat3 translationMat(-p.pos.x, -p.pos.y);
	Mat3 invRotationMat(p1.pos.a);
	Mat3 invTranslationMat(p.pos.x, p.pos.y);

	p0.pos = translationMat*p0.pos;
	p0.pos = rotationMat*p0.pos;

	p1.pos = translationMat*p1.pos;
	p1.pos = rotationMat*p1.pos;

	p2.pos = translationMat*p2.pos;
	p2.pos= rotationMat*p2.pos;

	double m = (p1.pos.y-p0.pos.y)/(p1.pos.x-p0.pos.x);
	double d = p1.pos.y - m*p1.pos.x; // solve for x = 0
	distance = p1.pos.x; // distance on the x axes

	perp = p1;
	perp.pos.x = 0; // on the same y axis of the car
	perp.pos.y = d; //perp distance between the car and the trajectory

	perp.pos = invRotationMat  * perp.pos;
	perp.pos = invTranslationMat  * perp.pos;

	return perp;
}

double PlanningHelpers::GetPerpDistanceToTrajectorySimple_obsolete(const vector<WayPoint>& trajectory, const WayPoint& p,const int& prevIndex)
{

	if(trajectory.size() < 2)
		return 0;

	WayPoint p0, p1, p2;
	int next_index = 0;
	if(trajectory.size()==2)
	{
		p0 = trajectory.at(0);
		p2 = trajectory.at(1);
		p1 = WayPoint((p0.pos.x+p2.pos.x)/2.0, (p0.pos.y+p2.pos.y)/2.0, (p0.pos.z+p2.pos.z)/2.0, p0.pos.a);

	}
	else
	{
		next_index = GetClosestNextPointIndexFast(trajectory, p, prevIndex);
		if(next_index == 0)
		{
			p0 = trajectory[next_index];
			p1 = trajectory[next_index+1];
			p2 = trajectory[next_index+2];
		}
		else if(next_index > 0 && next_index < trajectory.size()-1)
		{
			p0 = trajectory[next_index-1];
			p1 = trajectory[next_index];
			p2 = trajectory[next_index+1];
		}
		else
		{
			p0 = trajectory[next_index-1];
			p2 = trajectory[next_index];

			p1 = WayPoint((p0.pos.x+p2.pos.x)/2.0, (p0.pos.y+p2.pos.y)/2.0, (p0.pos.z+p2.pos.z)/2.0, p0.pos.a);

		}

	}


	Mat3 rotationMat(-p1.pos.a);
	Mat3 translationMat(-p.pos.x, -p.pos.y);

	p0.pos = translationMat*p0.pos;
	p0.pos = rotationMat*p0.pos;

	p1.pos = translationMat*p1.pos;
	p1.pos = rotationMat*p1.pos;

	p2.pos = translationMat*p2.pos;
	p2.pos = rotationMat*p2.pos;

	double m = (p1.pos.y-p0.pos.y)/(p1.pos.x-p0.pos.x);
	double d = p1.pos.y - m*p1.pos.x;

	if(std::isnan(d) || std::isinf(d))
	{
	  //assert(false);
	  d = 0;
	}

	return d;
}

double PlanningHelpers::GetPerpDistanceToVectorSimple_obsolete(const WayPoint& point1, const WayPoint& point2, const WayPoint& pose)
{
	WayPoint p1 = point1, p2 = point2;
	Mat3 rotationMat(-p1.pos.a);
	Mat3 translationMat(-pose.pos.x, -pose.pos.y);

	p1.pos = translationMat*p1.pos;
	p1.pos = rotationMat*p1.pos;

	p2.pos = translationMat*p2.pos;
	p2.pos = rotationMat*p2.pos;

	double m = (p2.pos.y-p1.pos.y)/(p2.pos.x-p1.pos.x);
	double d = p2.pos.y - m*p2.pos.x;

	if(std::isnan(d) || std::isinf(d))
	{
	  //assert(false);
	  d = 0;
	}

	return d;
}

WayPoint PlanningHelpers::GetNextPointOnTrajectory_obsolete(const vector<WayPoint>& trajectory, const double& distance, const int& currIndex)
{
	assert(trajectory.size()>0);

	int local_currIndex = currIndex;

	if(local_currIndex < 0 || local_currIndex >= trajectory.size())
		return trajectory.at(0);

	WayPoint p1 = trajectory.at(local_currIndex);
	WayPoint p2;

	double d = 0;
	while(local_currIndex < (trajectory.size()-1) && d < distance)
	{
		local_currIndex++;
		p2 = p1;
		p1 = trajectory.at(local_currIndex);
		d += distance2points(p1.pos, p2.pos);
	}

	if(local_currIndex >= trajectory.size()-1)
	  return p1;

	double distance_diff = distance -  d;

	p2 = trajectory.at(local_currIndex);
	p1 = trajectory.at(local_currIndex+1);

	GPSPoint uv(p1.pos.x - p2.pos.x, p1.pos.y - p2.pos.y ,0,0);
	double v_norm = pointNorm(uv);

	assert(v_norm != 0);

	uv.x = (uv.x / v_norm) * distance_diff;
	uv.y = (uv.y / v_norm) * distance_diff;

	double ydiff = p1.pos.y-p2.pos.y;
	double xdiff = p1.pos.x-p2.pos.x;
	double a =  atan2(ydiff,xdiff);

	WayPoint abs_waypoint = p2;

	abs_waypoint.pos.x = p2.pos.x + uv.x;
	abs_waypoint.pos.y = p2.pos.y + uv.y;
	abs_waypoint.pos.a = a;

	return abs_waypoint;
}

double PlanningHelpers::GetDistanceOnTrajectory_obsolete(const std::vector<WayPoint>& path, const int& start_index, const WayPoint& p)
{

	int end_point_index = GetClosestPointIndex_obsolete(path, p);
	if(end_point_index > 0)
		end_point_index--;

	double padding_distance = distance2points(path.at(end_point_index).pos, p.pos);

	double d_on_path = 0;
	if(end_point_index >= start_index)
	{
		for(unsigned int i = start_index; i < end_point_index; i++)
			d_on_path += distance2points(path.at(i).pos, path.at(i+1).pos);

		d_on_path += padding_distance;
	}
	else
	{
		for(unsigned int i = start_index; i > end_point_index; i--)
			d_on_path -= distance2points(path.at(i).pos, path.at(i-1).pos);
	}

	return d_on_path;
}

bool PlanningHelpers::CompareTrajectories(const std::vector<WayPoint>& path1, const std::vector<WayPoint>& path2)
{
	if(path1.size() != path2.size())
		return false;

	for(unsigned int i=0; i< path1.size(); i++)
	{
		if(path1.at(i).v != path2.at(i).v || path1.at(i).pos.x != path2.at(i).pos.x || path1.at(i).pos.y != path2.at(i).pos.y || path1.at(i).pos.alt != path2.at(i).pos.alt || path1.at(i).pos.lon != path2.at(i).pos.lon)
			return false;
	}

	return true;
}

double PlanningHelpers::GetDistanceToClosestStopLineAndCheck(const std::vector<WayPoint>& path, const WayPoint& p, const double& giveUpDistance, int& stopLineID, int& stopSignID, int& trafficLightID, const int& prevIndex)
{
	trafficLightID = stopSignID = stopLineID = -1;

	RelativeInfo info;
	GetRelativeInfo(path, p, info, prevIndex);

	for(unsigned int i=info.iBack; i<path.size(); i++)
	{
		if(path.at(i).stopLineID > 0 && path.at(i).pLane)
		{

			for(unsigned int j = 0; j < path.at(i).pLane->stopLines.size(); j++)
			{

				if(path.at(i).pLane->stopLines.at(j).id == path.at(i).stopLineID)
				{
					stopLineID = path.at(i).stopLineID;

					RelativeInfo stop_info;
					WayPoint stopLineWP ;
					stopLineWP.pos = path.at(i).pLane->stopLines.at(j).points.at(0);
					GetRelativeInfo(path, stopLineWP, stop_info);
					double localDistance = GetExactDistanceOnTrajectory(path, info, stop_info);

					if(localDistance > giveUpDistance)
					{
						stopSignID = path.at(i).pLane->stopLines.at(j).stopSignID;
						trafficLightID = path.at(i).pLane->stopLines.at(j).trafficLightID;
						return localDistance;
					}
				}
			}
		}
	}

	return -1;
}

void PlanningHelpers::CreateManualBranchFromTwoPoints(WayPoint& p1,WayPoint& p2 , const double& distance, const DIRECTION_TYPE& direction, std::vector<WayPoint>& path)
{
	WayPoint endWP, midWP;

	double branch_angle = 0;
	if(direction == FORWARD_RIGHT_DIR)
	{
		branch_angle = p1.pos.a-M_PI_2;
	}
	else if(direction == FORWARD_LEFT_DIR)
	{
		branch_angle = p1.pos.a+M_PI_2;
	}
	endWP.pos.y = p2.pos.y + distance*sin(branch_angle);
	endWP.pos.x = p2.pos.x + distance*cos(branch_angle);

	midWP = p2;
	midWP.pos.x = (p1.pos.x+p2.pos.x)/2.0;
	midWP.pos.y = (p1.pos.y+p2.pos.y)/2.0;
	endWP.bDir = midWP.bDir = direction;

	path.clear();
	path.push_back(p1);
	path.push_back(p2);
	path.push_back(endWP);

	//PlanningHelpers::SmoothPath(path, 0.4, 0.1);
	PlanningHelpers::FixPathDensity(path, 1);
	PlanningHelpers::SmoothPath(path, 0.4, 0.25);
	PlanningHelpers::FixPathDensity(path, 0.5);
	PlanningHelpers::SmoothPath(path, 0.25, 0.4);

	for(unsigned int i=0; i < path.size(); i++)
	{
		if(direction == FORWARD_LEFT_DIR)
		{
			path.at(i).state = INITIAL_STATE;
			path.at(i).beh_state = BEH_BRANCH_LEFT_STATE;
			path.at(i).laneId = -2;
		}
		if(direction == FORWARD_RIGHT_DIR)
		{
			path.at(i).state = INITIAL_STATE;
			path.at(i).beh_state = BEH_BRANCH_RIGHT_STATE;
			path.at(i).laneId = -3;
		}
	}
}

void PlanningHelpers::CreateManualBranch(std::vector<WayPoint>& path, const int& degree, const DIRECTION_TYPE& direction)
{
	if(path.size() < 5) return;

	//start branch point
	WayPoint branch_start = path.at(path.size()-5);
	WayPoint last_wp = path.at(path.size()-1);


	WayPoint endWP;
	vector<WayPoint> goal_path;
	double branch_angle = 0;
	if(direction == FORWARD_RIGHT_DIR)
	{
		branch_angle = last_wp.pos.a-M_PI_2;
	}
	else if(direction == FORWARD_LEFT_DIR)
	{
		branch_angle = last_wp.pos.a+M_PI_2;
	}
	endWP.pos.y = last_wp.pos.y + 10*sin(branch_angle);
	endWP.pos.x = last_wp.pos.x + 10*cos(branch_angle);

	WayPoint wp = last_wp;
	wp.pos.x = (last_wp.pos.x+endWP.pos.x)/2.0;
	wp.pos.y = (last_wp.pos.y+endWP.pos.y)/2.0;
	endWP.bDir = wp.bDir = direction;
	goal_path.push_back(wp);
	goal_path.push_back(endWP);

	goal_path.insert(goal_path.begin(), path.end()-5, path.end());
	PlanningHelpers::SmoothPath(goal_path, 0.25, 0.25);
	PlanningHelpers::FixPathDensity(goal_path, 0.75);
	PlanningHelpers::SmoothPath(goal_path, 0.25, 0.35);

	path.erase(path.end()-5, path.end());
	path.insert(path.end(), goal_path.begin(), goal_path.end());

	PlanningHelpers::CalcAngleAndCost(path);

	for(unsigned int i=0; i < path.size(); i++)
	{
		if(direction == FORWARD_LEFT_DIR)
		{
			path.at(i).state = INITIAL_STATE;
			path.at(i).beh_state = BEH_BRANCH_LEFT_STATE;
		}
		if(direction == FORWARD_RIGHT_DIR)
		{
			path.at(i).state = INITIAL_STATE;
			path.at(i).beh_state = BEH_BRANCH_RIGHT_STATE;
		}
	}


}

void PlanningHelpers::FixPathDensity(vector<WayPoint>& path, const double& distanceDensity)
{
	if(path.size() == 0 || distanceDensity==0) return;

	double d = 0, a = 0;
	double margin = distanceDensity*0.01;
	double remaining = 0;
	int nPoints = 0;
	vector<WayPoint> fixedPath;
	fixedPath.push_back(path.at(0));
	for(unsigned int si = 0, ei=1; ei < path.size(); )
	{
		d += hypot(path.at(ei).pos.x- path.at(ei-1).pos.x, path.at(ei).pos.y- path.at(ei-1).pos.y) + remaining;
		a = atan2(path.at(ei).pos.y - path.at(si).pos.y, path.at(ei).pos.x - path.at(si).pos.x);

		if(d < distanceDensity - margin ) // skip
		{
			ei++;
			remaining = 0;
		}
		else if(d > (distanceDensity +  margin)) // skip
		{
			WayPoint pm = path.at(si);
			nPoints = d  / distanceDensity;
			for(int k = 0; k < nPoints; k++)
			{
				pm.pos.x = pm.pos.x + distanceDensity * cos(a);
				pm.pos.y = pm.pos.y + distanceDensity * sin(a);
				fixedPath.push_back(pm);
			}
			remaining = d - nPoints*distanceDensity;
			si++;
			path.at(si).pos = pm.pos;
			d = 0;
			ei++;
		}
		else
		{
			d = 0;
			remaining = 0;
			fixedPath.push_back(path.at(ei));
			ei++;
			si = ei - 1;
		}
	}

	path = fixedPath;
}

void PlanningHelpers::SmoothPath(vector<WayPoint>& path, double weight_data,
		double weight_smooth, double tolerance)
{

	if (path.size() <= 2 )
	{
		//cout << "Can't Smooth Path, Path_in Size=" << path.size() << endl;
		return;
	}

	const vector<WayPoint>& path_in = path;
	vector<WayPoint> smoothPath_out =  path_in;

	double change = tolerance;
	double xtemp, ytemp;
	int nIterations = 0;

	int size = path_in.size();

	while (change >= tolerance)
	{
		change = 0.0;
		for (int i = 1; i < size - 1; i++)
		{
//			if (smoothPath_out[i].pos.a != smoothPath_out[i - 1].pos.a)
//				continue;

			xtemp = smoothPath_out[i].pos.x;
			ytemp = smoothPath_out[i].pos.y;

			smoothPath_out[i].pos.x += weight_data
					* (path_in[i].pos.x - smoothPath_out[i].pos.x);
			smoothPath_out[i].pos.y += weight_data
					* (path_in[i].pos.y - smoothPath_out[i].pos.y);

			smoothPath_out[i].pos.x += weight_smooth
					* (smoothPath_out[i - 1].pos.x + smoothPath_out[i + 1].pos.x
							- (2.0 * smoothPath_out[i].pos.x));
			smoothPath_out[i].pos.y += weight_smooth
					* (smoothPath_out[i - 1].pos.y + smoothPath_out[i + 1].pos.y
							- (2.0 * smoothPath_out[i].pos.y));

			change += fabs(xtemp - smoothPath_out[i].pos.x);
			change += fabs(ytemp - smoothPath_out[i].pos.y);

		}
		nIterations++;
	}

	path = smoothPath_out;
}

void PlanningHelpers::PredictConstantTimeCostForTrajectory(std::vector<PlannerHNS::WayPoint>& path, const PlannerHNS::WayPoint& currPose, const double& minVelocity, const double& minDist)
{
	if(path.size() == 0) return;

	for(unsigned int i = 0 ; i < path.size(); i++)
		path.at(i).timeCost = -1;

	if(currPose.v == 0 || currPose.v < minVelocity) return;

	RelativeInfo info;
	PlanningHelpers::GetRelativeInfo(path, currPose, info);

	double total_distance = 0;
	double accum_time = 0;

	path.at(info.iFront).timeCost = 0;
	if(info.iFront == 0 ) info.iFront++;

	for(unsigned int i=info.iFront; i<path.size(); i++)
	{
		total_distance += hypot(path.at(i).pos.x- path.at(i-1).pos.x,path.at(i).pos.y- path.at(i-1).pos.y);
		accum_time = total_distance/currPose.v;
		path.at(i).timeCost = accum_time;
	}
}

void PlanningHelpers::FixAngleOnly(std::vector<WayPoint>& path)
{
	if(path.size() <= 2) return;

	path[0].pos.a = UtilityH::FixNegativeAngle(atan2(path[1].pos.y - path[0].pos.y, path[1].pos.x - path[0].pos.x ));

	for(int j = 1; j < path.size()-1; j++)
		path[j].pos.a 		= UtilityH::FixNegativeAngle(atan2(path[j+1].pos.y - path[j].pos.y, path[j+1].pos.x - path[j].pos.x ));

	int j = (int)path.size()-1;

	path[j].pos.a = path[j-1].pos.a;

	for(int j = 0; j < path.size()-1; j++)
	{
		if(path.at(j).pos.x == path.at(j+1).pos.x && path.at(j).pos.y == path.at(j+1).pos.y)
			path.at(j).pos.a = path.at(j+1).pos.a;
	}
}

double PlanningHelpers::CalcAngleAndCost(vector<WayPoint>& path, const double& lastCost, const bool& bSmooth)
{
	if(path.size() < 2) return 0;
	if(path.size() == 2)
	{
		path[0].pos.a = UtilityH::FixNegativeAngle(atan2(path[1].pos.y - path[0].pos.y, path[1].pos.x - path[0].pos.x ));
		path[0].cost = lastCost;
		path[1].pos.a = path[0].pos.a;
		path[1].cost = path[0].cost +  distance2points(path[0].pos, path[1].pos);
		return path[1].cost;
	}

	path[0].pos.a = UtilityH::FixNegativeAngle(atan2(path[1].pos.y - path[0].pos.y, path[1].pos.x - path[0].pos.x ));
	path[0].cost = lastCost;

	for(int j = 1; j < path.size()-1; j++)
	{
		path[j].pos.a 		= UtilityH::FixNegativeAngle(atan2(path[j+1].pos.y - path[j].pos.y, path[j+1].pos.x - path[j].pos.x ));
		path[j].cost 	= path[j-1].cost +  distance2points(path[j-1].pos, path[j].pos);
	}

	int j = (int)path.size()-1;

	path[j].pos.a 		= path[j-1].pos.a;
	path[j].cost 	= path[j-1].cost + distance2points(path[j-1].pos, path[j].pos);

	for(int j = 0; j < path.size()-1; j++)
	{
		if(path.at(j).pos.x == path.at(j+1).pos.x && path.at(j).pos.y == path.at(j+1).pos.y)
			path.at(j).pos.a = path.at(j+1).pos.a;
	}

	return path[j].cost;
}

double PlanningHelpers::CalcAngleAndCostAndCurvatureAnd2D(vector<WayPoint>& path, const double& lastCost)
{
	if(path.size() < 2) return -1;

	path[0].pos.a 	= atan2(path[1].pos.y - path[0].pos.y, path[1].pos.x - path[0].pos.x );
	path[0].cost 	= lastCost;

	double k = 0;
	GPSPoint center;

	for(unsigned int j = 1; j < path.size()-1; j++)
	{
		k =  CalcCircle(path[j-1].pos,path[j].pos, path[j+1].pos, center);
		if(k > 150.0 || std::isnan(k))
			k = 150.0;

		if(k<1.0)
			path[j].cost = 0;
		else
			path[j].cost = 1.0-1.0/k;

		path[j].pos.a 	= atan2(path[j+1].pos.y - path[j].pos.y, path[j+1].pos.x - path[j].pos.x );
	}
	unsigned int j = path.size()-1;

	path[0].cost    = path[1].cost;
	path[j].cost 	= path[j-1].cost;
	path[j].pos.a 	= path[j-1].pos.a;
	path[j].cost 	= path[j-1].cost ;

	return path[j].cost;
}

double PlanningHelpers::CalcCircle(const GPSPoint& pt1, const GPSPoint& pt2, const GPSPoint& pt3, GPSPoint& center)
{
	double yDelta_a= pt2.y - pt1.y;
	double xDelta_a= pt2.x - pt1.x;
	double yDelta_b= pt3.y - pt2.y;
	double xDelta_b= pt3.x - pt2.x;

	if (fabs(xDelta_a) <= 0.000000000001 && fabs(yDelta_b) <= 0.000000000001)
	{
		center.x= 0.5*(pt2.x + pt3.x);
		center.y= 0.5*(pt1.y + pt2.y);
		return distance2points(center,pt1);
	}

	double aSlope=yDelta_a/xDelta_a;
	double bSlope=yDelta_b/xDelta_b;
	if (fabs(aSlope-bSlope) <= 0.000000000001)
	{
		return 100000;
	}

	center.x= (aSlope*bSlope*(pt1.y - pt3.y) + bSlope*(pt1.x + pt2 .x) - aSlope*(pt2.x+pt3.x) )/(2.0* (bSlope-aSlope) );
	center.y = -1.0*(center.x - (pt1.x+pt2.x)/2.0)/aSlope +  (pt1.y+pt2.y)/2.0;

	return  distance2points(center,pt1);
}

void PlanningHelpers::ExtractPartFromPointToDistance(const vector<WayPoint>& originalPath, const WayPoint& pos, const double& minDistance,
		const double& pathDensity, vector<WayPoint>& extractedPath, const double& SmoothDataWeight, const double& SmoothWeight, const double& SmoothTolerance)
{
	extractedPath.clear();
	unsigned int close_index = GetClosestNextPointIndexDirectionFast(originalPath, pos);
//	int i_slow = GetClosestNextPointIndexDirection(originalPath, pos);
//	if(close_index != i_slow)
//		cout << "Aler Alert !!! fast: " << close_index << ", slow: " << i_slow  << endl;
	//vector<WayPoint> tempPath;
	double d_limit = 0;
	if(close_index >= 2) close_index -=2;
	else close_index = 0;

	for(unsigned int i=close_index; i< originalPath.size(); i++)
	{
		extractedPath.push_back(originalPath.at(i));

		if(i>0)
			d_limit += hypot(originalPath.at(i).pos.y - originalPath.at(i-1).pos.y, originalPath.at(i).pos.x - originalPath.at(i-1).pos.x);

		if(d_limit > minDistance)
			break;
	}

	if(extractedPath.size() < 2)
	{
		cout << endl << "### Planner Z . Extracted Rollout Path is too Small, Size = " << extractedPath.size() << endl;
		return;
	}

	FixPathDensity(extractedPath, pathDensity);
	SmoothPath(extractedPath, SmoothDataWeight, SmoothWeight , SmoothTolerance);
	CalcAngleAndCost(extractedPath);

	//extractedPath = tempPath;
	//tempPath.clear();
	//TestQuadraticSpline(extractedPath, tempPath);
}

void PlanningHelpers::ExtractPartFromPointToDistanceDirectionFast(const vector<WayPoint>& originalPath, const WayPoint& pos, const double& minDistance,
		const double& pathDensity, vector<WayPoint>& extractedPath)
{
	if(originalPath.size() < 2 ) return;

	extractedPath.clear();

	int close_index = GetClosestNextPointIndexDirectionFast(originalPath, pos);
	double d = 0;

	if(close_index + 1 >= originalPath.size())
		close_index = originalPath.size() - 2;

	for(int i=close_index; i >=  0; i--)
	{
		extractedPath.insert(extractedPath.begin(),  originalPath.at(i));
		if(i < originalPath.size())
			d += hypot(originalPath.at(i).pos.y - originalPath.at(i+1).pos.y, originalPath.at(i).pos.x - originalPath.at(i+1).pos.x);
		if(d > 10)
			break;
	}

	//extractedPath.push_back(info.perp_point);
	d = 0;
	for(int i=close_index+1; i < (int)originalPath.size(); i++)
	{
		extractedPath.push_back(originalPath.at(i));
		if(i > 0)
			d += hypot(originalPath.at(i).pos.y - originalPath.at(i-1).pos.y, originalPath.at(i).pos.x - originalPath.at(i-1).pos.x);
		if(d > minDistance)
			break;
	}

	if(extractedPath.size() < 2)
	{
		cout << endl << "### Planner Z . Extracted Rollout Path is too Small, Size = " << extractedPath.size() << endl;
		return;
	}

	FixPathDensity(extractedPath, pathDensity);
	CalcAngleAndCost(extractedPath);
}

void PlanningHelpers::ExtractPartFromPointToDistanceFast(const vector<WayPoint>& originalPath, const WayPoint& pos, const double& minDistance,
		const double& pathDensity, vector<WayPoint>& extractedPath, const double& SmoothDataWeight, const double& SmoothWeight, const double& SmoothTolerance)
{
	extractedPath.clear();
	RelativeInfo info;
	GetRelativeInfo(originalPath, pos, info);
	double d = 0;
	if(info.iBack > 0)
		info.iBack--;

	for(int i=info.iBack; i >=  0; i--)
	{
		extractedPath.insert(extractedPath.begin(),  originalPath.at(i));
		if(i < originalPath.size())
			d += hypot(originalPath.at(i).pos.y - originalPath.at(i+1).pos.y, originalPath.at(i).pos.x - originalPath.at(i+1).pos.x);
		if(d > 10)
			break;
	}

	//extractedPath.push_back(info.perp_point);
	d = 0;
	for(int i=info.iBack+1; i < (int)originalPath.size(); i++)
	{
		extractedPath.push_back(originalPath.at(i));
		if(i > 0)
			d += hypot(originalPath.at(i).pos.y - originalPath.at(i-1).pos.y, originalPath.at(i).pos.x - originalPath.at(i-1).pos.x);
		if(d > minDistance)
			break;
	}

	if(extractedPath.size() < 2)
	{
		cout << endl << "### Planner Z . Extracted Rollout Path is too Small, Size = " << extractedPath.size() << endl;
		return;
	}

	FixPathDensity(extractedPath, pathDensity);
	CalcAngleAndCost(extractedPath);
}

void PlanningHelpers::CalculateRollInTrajectories(const WayPoint& carPos, const double& speed, const vector<WayPoint>& originalCenter, int& start_index,
		int& end_index, vector<double>& end_laterals ,
		vector<vector<WayPoint> >& rollInPaths, const double& max_roll_distance,
		const double& maxSpeed, const double&  carTipMargin, const double& rollInMargin,
		const double& rollInSpeedFactor, const double& pathDensity, const double& rollOutDensity,
		const int& rollOutNumber, const double& SmoothDataWeight, const double& SmoothWeight,
		const double& SmoothTolerance, const bool& bHeadingSmooth,
		std::vector<WayPoint>& sampledPoints)
{
	WayPoint p;
	double dummyd = 0;

	int iLimitIndex = (carTipMargin/0.3)/pathDensity;
	if(iLimitIndex >= originalCenter.size())
		iLimitIndex = originalCenter.size() - 1;

	//Get Closest Index
	RelativeInfo info;
	GetRelativeInfo(originalCenter, carPos, info);
	double remaining_distance = 0;
	int close_index = info.iBack;
	for(unsigned int i=close_index; i< originalCenter.size()-1; i++)
	  {
		if(i>0)
			remaining_distance += distance2points(originalCenter[i].pos, originalCenter[i+1].pos);
	  }

	double initial_roll_in_distance = info.perp_distance ; //GetPerpDistanceToTrajectorySimple(originalCenter, carPos, close_index);


	vector<WayPoint> RollOutStratPath;
	///***   Smoothing From Car Heading Section ***///
//	if(bHeadingSmooth)
//	{
//		unsigned int num_of_strait_points = carTipMargin / pathDensity;
//		int closest_for_each_iteration = 0;
//		WayPoint np = GetPerpendicularOnTrajectory_obsolete(originalCenter, carPos, dummyd, closest_for_each_iteration);
//		np.pos = carPos.pos;
//
//		RollOutStratPath.push_back(np);
//		for(unsigned int i = 0; i < num_of_strait_points; i++)
//		{
//			p = RollOutStratPath.at(i);
//			p.pos.x = p.pos.x +  pathDensity*cos(p.pos.a);
//			p.pos.y = p.pos.y +  pathDensity*sin(p.pos.a);
//			np = GetPerpendicularOnTrajectory_obsolete(originalCenter, p, dummyd, closest_for_each_iteration);
//			np.pos = p.pos;
//			RollOutStratPath.push_back(np);
//		}
//
//		initial_roll_in_distance = GetPerpDistanceToTrajectorySimple_obsolete(originalCenter, RollOutStratPath.at(RollOutStratPath.size()-1), close_index);
//	}
	///***   -------------------------------- ***///


	//printf("\n Lateral Distance: %f" , initial_roll_in_distance);

	//calculate the starting index
	double d_limit = 0;
	unsigned int far_index = close_index;

	//calculate end index
	double start_distance = rollInSpeedFactor*speed+rollInMargin;
	if(start_distance > remaining_distance)
		start_distance = remaining_distance;

	d_limit = 0;
	for(unsigned int i=close_index; i< originalCenter.size(); i++)
	  {
		  if(i>0)
			  d_limit += distance2points(originalCenter[i].pos, originalCenter[i-1].pos);

		  if(d_limit >= start_distance)
		  {
			  far_index = i;
			  break;
		  }
	  }

	int centralTrajectoryIndex = rollOutNumber/2;
	vector<double> end_distance_list;
	for(int i=0; i< rollOutNumber+1; i++)
	  {
		  double end_roll_in_distance = rollOutDensity*(i - centralTrajectoryIndex);
		  end_distance_list.push_back(end_roll_in_distance);
	  }

	start_index = close_index;
	end_index = far_index;
	end_laterals = end_distance_list;

	//calculate the actual calculation starting index
	d_limit = 0;
	unsigned int smoothing_start_index = start_index;
	unsigned int smoothing_end_index = end_index;

	for(unsigned int i=smoothing_start_index; i< originalCenter.size(); i++)
	{
		if(i > 0)
			d_limit += distance2points(originalCenter[i].pos, originalCenter[i-1].pos);
		if(d_limit > carTipMargin)
			break;

		smoothing_start_index++;
	}

	d_limit = 0;
	for(unsigned int i=end_index; i< originalCenter.size(); i++)
	{
		if(i > 0)
			d_limit += distance2points(originalCenter[i].pos, originalCenter[i-1].pos);
		if(d_limit > carTipMargin)
			break;

		smoothing_end_index++;
	}

	int nSteps = end_index - smoothing_start_index;


	vector<double> inc_list;
	rollInPaths.clear();
	vector<double> inc_list_inc;
	for(int i=0; i< rollOutNumber+1; i++)
	{
		double diff = end_laterals.at(i)-initial_roll_in_distance;
		inc_list.push_back(diff/(double)nSteps);
		rollInPaths.push_back(vector<WayPoint>());
		inc_list_inc.push_back(0);
	}



	vector<vector<WayPoint> > execluded_from_smoothing;
	for(unsigned int i=0; i< rollOutNumber+1 ; i++)
		execluded_from_smoothing.push_back(vector<WayPoint>());



	//Insert First strait points within the tip of the car range
	for(unsigned int j = start_index; j < smoothing_start_index; j++)
	{
		p = originalCenter.at(j);
		double original_speed = p.v;
	  for(unsigned int i=0; i< rollOutNumber+1 ; i++)
	  {
		  p.pos.x = originalCenter.at(j).pos.x -  initial_roll_in_distance*cos(p.pos.a + M_PI_2);
		  p.pos.y = originalCenter.at(j).pos.y -  initial_roll_in_distance*sin(p.pos.a + M_PI_2);
		  if(i!=centralTrajectoryIndex)
			  p.v = original_speed * LANE_CHANGE_SPEED_FACTOR;
		  else
			  p.v = original_speed ;

		  if(j < iLimitIndex)
			  execluded_from_smoothing.at(i).push_back(p);
		  else
			  rollInPaths.at(i).push_back(p);

		  sampledPoints.push_back(p);
	  }
	}

	for(unsigned int j = smoothing_start_index; j < end_index; j++)
	  {
		  p = originalCenter.at(j);
		  double original_speed = p.v;
		  for(unsigned int i=0; i< rollOutNumber+1 ; i++)
		  {
			  inc_list_inc[i] += inc_list[i];
			  double d = inc_list_inc[i];
			  p.pos.x = originalCenter.at(j).pos.x -  initial_roll_in_distance*cos(p.pos.a + M_PI_2) - d*cos(p.pos.a+ M_PI_2);
			  p.pos.y = originalCenter.at(j).pos.y -  initial_roll_in_distance*sin(p.pos.a + M_PI_2) - d*sin(p.pos.a+ M_PI_2);
			  if(i!=centralTrajectoryIndex)
				  p.v = original_speed * LANE_CHANGE_SPEED_FACTOR;
			  else
				  p.v = original_speed ;

			  rollInPaths.at(i).push_back(p);

			  sampledPoints.push_back(p);
		  }
	  }

	//Insert last strait points to make better smoothing
	for(unsigned int j = end_index; j < smoothing_end_index; j++)
	{
		p = originalCenter.at(j);
		double original_speed = p.v;
	  for(unsigned int i=0; i< rollOutNumber+1 ; i++)
	  {
		  double d = end_laterals.at(i);
		  p.pos.x  = originalCenter.at(j).pos.x - d*cos(p.pos.a + M_PI_2);
		  p.pos.y  = originalCenter.at(j).pos.y - d*sin(p.pos.a + M_PI_2);
		  if(i!=centralTrajectoryIndex)
			  p.v = original_speed * LANE_CHANGE_SPEED_FACTOR;
		  else
			  p.v = original_speed ;
		  rollInPaths.at(i).push_back(p);

		  sampledPoints.push_back(p);
	  }
	}

	for(unsigned int i=0; i< rollOutNumber+1 ; i++)
		rollInPaths.at(i).insert(rollInPaths.at(i).begin(), execluded_from_smoothing.at(i).begin(), execluded_from_smoothing.at(i).end());

	///***   Smoothing From Car Heading Section ***///
//	if(bHeadingSmooth)
//	{
//		for(unsigned int i=0; i< rollOutNumber+1 ; i++)
//		{
//			unsigned int cut_index = GetClosestNextPointIndex(rollInPaths.at(i), RollOutStratPath.at(RollOutStratPath.size()-1));
//			rollInPaths.at(i).erase(rollInPaths.at(i).begin(), rollInPaths.at(i).begin()+cut_index);
//			rollInPaths.at(i).insert(rollInPaths.at(i).begin(), RollOutStratPath.begin(), RollOutStratPath.end());
//		}
//	}
	///***   -------------------------------- ***///



	d_limit = 0;
	for(unsigned int j = smoothing_end_index; j < originalCenter.size(); j++)
	  {
		if(j > 0)
			d_limit += distance2points(originalCenter.at(j).pos, originalCenter.at(j-1).pos);

		if(d_limit > max_roll_distance)
			break;

			p = originalCenter.at(j);
			double original_speed = p.v;
		  for(unsigned int i=0; i< rollInPaths.size() ; i++)
		  {
			  double d = end_laterals.at(i);
			  p.pos.x  = originalCenter.at(j).pos.x - d*cos(p.pos.a + M_PI_2);
			  p.pos.y  = originalCenter.at(j).pos.y - d*sin(p.pos.a + M_PI_2);

			  if(i!=centralTrajectoryIndex)
				  p.v = original_speed * LANE_CHANGE_SPEED_FACTOR;
			  else
				  p.v = original_speed ;

			  rollInPaths.at(i).push_back(p);

			  sampledPoints.push_back(p);
		  }
	  }

	for(unsigned int i=0; i< rollOutNumber+1 ; i++)
	{
		SmoothPath(rollInPaths.at(i), SmoothDataWeight, SmoothWeight, SmoothTolerance);
	}

//	for(unsigned int i=0; i< rollInPaths.size(); i++)
//		CalcAngleAndCost(rollInPaths.at(i));
}

bool PlanningHelpers::FindInList(const std::vector<int>& list,const int& x)
{
	for(unsigned int i = 0 ; i < list.size(); i++)
	{
		if(list.at(i) == x)
			return true;
	}
	return false;
}

void PlanningHelpers::RemoveWithValue(std::vector<int>& list,const int& x)
{
	for(unsigned int i = 0 ; i < list.size(); i++)
	{
		if(list.at(i) == x)
		{
			list.erase(list.begin()+i);
		}
	}
}

std::vector<int> PlanningHelpers::GetUniqueLeftRightIds(const std::vector<WayPoint>& path)
{
	 vector<int> sideLanes;
	for(unsigned int iwp = 0; iwp < path.size(); iwp++)
	 {
		 if(path.at(iwp).LeftPointId>0)
		 {
			 bool bFound = false;
			 for(unsigned int is = 0 ; is < sideLanes.size(); is++)
			 {
				 if(sideLanes.at(is) == path.at(iwp).LeftPointId)
				 {
					 bFound = true;
					 break;
				 }
			 }

			 if(!bFound)
				 sideLanes.push_back(path.at(iwp).LeftPointId);
		 }

		 if(path.at(iwp).RightPointId>0)
		 {
			 bool bFound = false;
			 for(unsigned int is = 0 ; is < sideLanes.size(); is++)
			 {
				 if(sideLanes.at(is) == path.at(iwp).RightPointId)
				 {
					 bFound = true;
					 break;
				 }
			 }

			 if(!bFound)
				 sideLanes.push_back(path.at(iwp).RightPointId);
		 }

		 //RemoveWithValue(sideLanes, path.at(iwp).laneId);
	 }
	return sideLanes;
}

void PlanningHelpers::SmoothSpeedProfiles(vector<WayPoint>& path_in, double weight_data, double weight_smooth, double tolerance	)
{

	if (path_in.size() <= 1)
		return;
	vector<WayPoint> newpath = path_in;

	double change = tolerance;
	double xtemp;
	int nIterations = 0;
	int size = newpath.size();

	while (change >= tolerance)
	{
		change = 0.0;
		for (int i = 1; i < size -1; i++)
		{
			xtemp = newpath[i].v;
			newpath[i].v += weight_data * (path_in[i].v - newpath[i].v);
			newpath[i].v += weight_smooth * (newpath[i - 1].v + newpath[i + 1].v - (2.0 * newpath[i].v));
			change += fabs(xtemp - newpath[i].v);

		}
		nIterations++;
	}

	path_in = newpath;
}

void PlanningHelpers::SmoothCurvatureProfiles(vector<WayPoint>& path_in, double weight_data, double weight_smooth, double tolerance)
{
	if (path_in.size() <= 1)
			return;
	vector<WayPoint> newpath = path_in;

	double change = tolerance;
	double xtemp;
	int nIterations = 0;
	int size = newpath.size();

	while (change >= tolerance)
	{
		change = 0.0;
		for (int i = 1; i < size -1; i++)
		{
			xtemp = newpath[i].cost;
			newpath[i].cost += weight_data * (path_in[i].cost - newpath[i].cost);
			newpath[i].cost += weight_smooth * (newpath[i - 1].cost + newpath[i + 1].cost - (2.0 * newpath[i].cost));
			change += fabs(xtemp - newpath[i].cost);

		}
		nIterations++;
	}
	path_in = newpath;
}

void PlanningHelpers::SmoothWayPointsDirections(vector<WayPoint>& path_in, double weight_data, double weight_smooth, double tolerance	)
{

	if (path_in.size() <= 1)
		return;

	vector<WayPoint> newpath = path_in;

	double change = tolerance;
	double xtemp;
	int nIterations = 0;
	int size = newpath.size();

	while (change >= tolerance)
	{
		change = 0.0;
		for (int i = 1; i < size -1; i++)
		{
			xtemp = newpath[i].pos.a;
			newpath[i].pos.a += weight_data * (path_in[i].pos.a - newpath[i].pos.a);
			newpath[i].pos.a += weight_smooth * (newpath[i - 1].pos.a + newpath[i + 1].pos.a - (2.0 * newpath[i].pos.a));
			change += fabs(xtemp - newpath[i].pos.a);

		}
		nIterations++;
	}
	path_in = newpath;
}

void PlanningHelpers::SmoothGlobalPathSpeed(vector<WayPoint>& path)
{
	CalcAngleAndCostAndCurvatureAnd2D(path);
	SmoothSpeedProfiles(path, 0.45,0.25, 0.01);
}

void PlanningHelpers::GenerateRecommendedSpeed(vector<WayPoint>& path, const double& max_speed, const double& speedProfileFactor)
{
	CalcAngleAndCostAndCurvatureAnd2D(path);
	SmoothCurvatureProfiles(path, 0.4, 0.3, 0.01);
	double v = 0;

	for(unsigned int i = 0 ; i < path.size(); i++)
	{
		double k_ratio = path.at(i).cost*10.0;
		double local_max = (path.at(i).v >= 0 && max_speed > path.at(i).v) ? path.at(i).v : max_speed;

		if(k_ratio >= 9.5)
			v = local_max;
		else if(k_ratio <= 8.5)
			v = 1.0*speedProfileFactor;
		else
		{
			k_ratio = k_ratio - 8.5;
			v = (local_max - 1.0) * k_ratio + 1.0;
			v = v*speedProfileFactor;
		}

		if(v > local_max)
			path.at(i).v = local_max;
		else
			path.at(i).v = v;

	}

	SmoothSpeedProfiles(path, 0.4,0.3, 0.01);
}

WayPoint* PlanningHelpers::BuildPlanningSearchTreeV2(WayPoint* pStart,
		const WayPoint& goalPos,
		const vector<int>& globalPath,
		const double& DistanceLimit,
		const bool& bEnableLaneChange,
		vector<WayPoint*>& all_cells_to_delete)
{
	if(!pStart) return NULL;

	vector<pair<WayPoint*, WayPoint*> >nextLeafToTrace;

	WayPoint* pZero = 0;
	WayPoint* wp    = new WayPoint();
	*wp = *pStart;
	nextLeafToTrace.push_back(make_pair(pZero, wp));
	all_cells_to_delete.push_back(wp);

	double 		distance 		= 0;
	double 		before_change_distance	= 0;
	WayPoint* 	pGoalCell 		= 0;
	double 		nCounter 		= 0;


	while(nextLeafToTrace.size()>0)
	{
		nCounter++;

		unsigned int min_cost_index = 0;
		double min_cost = DBL_MAX;

		for(unsigned int i=0; i < nextLeafToTrace.size(); i++)
		{
			if(nextLeafToTrace.at(i).second->cost < min_cost)
			{
				min_cost = nextLeafToTrace.at(i).second->cost;
				min_cost_index = i;
			}
		}

		WayPoint* pH 	= nextLeafToTrace.at(min_cost_index).second;

		assert(pH != 0);

		nextLeafToTrace.erase(nextLeafToTrace.begin()+min_cost_index);

		double distance_to_goal = distance2points(pH->pos, goalPos.pos);
		double angle_to_goal = UtilityH::AngleBetweenTwoAnglesPositive(UtilityH::FixNegativeAngle(pH->pos.a), UtilityH::FixNegativeAngle(goalPos.pos.a));
		if( distance_to_goal <= 0.1 && angle_to_goal < M_PI_4)
		{
			cout << "Goal Found, LaneID: " << pH->laneId <<", Distance : " << distance_to_goal << ", Angle: " << angle_to_goal*RAD2DEG << endl;
			pGoalCell = pH;
			break;
		}
		else
		{

			if(pH->pLeft && !CheckLaneExits(all_cells_to_delete, pH->pLeft->pLane) && !CheckNodeExits(all_cells_to_delete, pH->pLeft) && bEnableLaneChange && before_change_distance > LANE_CHANGE_MIN_DISTANCE)
			{
				wp = new WayPoint();
				*wp = *pH->pLeft;
				double d = hypot(wp->pos.y - pH->pos.y, wp->pos.x - pH->pos.x);
				distance += d;
				before_change_distance = -LANE_CHANGE_MIN_DISTANCE*3;

				for(unsigned int a = 0; a < wp->actionCost.size(); a++)
				{
					//if(wp->actionCost.at(a).first == LEFT_TURN_ACTION)
						d += wp->actionCost.at(a).second;
				}

				wp->cost = pH->cost + d;
				wp->pRight = pH;
				wp->pLeft = 0;

				nextLeafToTrace.push_back(make_pair(pH, wp));
				all_cells_to_delete.push_back(wp);
			}

			if(pH->pRight && !CheckLaneExits(all_cells_to_delete, pH->pRight->pLane) && !CheckNodeExits(all_cells_to_delete, pH->pRight) && bEnableLaneChange && before_change_distance > LANE_CHANGE_MIN_DISTANCE)
			{
				wp = new WayPoint();
				*wp = *pH->pRight;
				double d = hypot(wp->pos.y - pH->pos.y, wp->pos.x - pH->pos.x);
				distance += d;
				before_change_distance = -LANE_CHANGE_MIN_DISTANCE*3;

				for(unsigned int a = 0; a < wp->actionCost.size(); a++)
				{
					//if(wp->actionCost.at(a).first == RIGHT_TURN_ACTION)
						d += wp->actionCost.at(a).second;
				}

				wp->cost = pH->cost + d ;
				wp->pLeft = pH;
				wp->pRight = 0;
				nextLeafToTrace.push_back(make_pair(pH, wp));
				all_cells_to_delete.push_back(wp);
			}

			for(unsigned int i =0; i< pH->pFronts.size(); i++)
			{
				if(CheckLaneIdExits(globalPath, pH->pLane) && pH->pFronts.at(i) && !CheckNodeExits(all_cells_to_delete, pH->pFronts.at(i)))
				{
					wp = new WayPoint();
					*wp = *pH->pFronts.at(i);

					double d = hypot(wp->pos.y - pH->pos.y, wp->pos.x - pH->pos.x);
					distance += d;
					before_change_distance += d;

					for(unsigned int a = 0; a < wp->actionCost.size(); a++)
					{
						//if(wp->actionCost.at(a).first == FORWARD_ACTION)
							d += wp->actionCost.at(a).second;
					}

					wp->cost = pH->cost + d;
					wp->pBacks.push_back(pH);

					nextLeafToTrace.push_back(make_pair(pH, wp));
					all_cells_to_delete.push_back(wp);
				}
			}
		}

		if(distance > DistanceLimit && globalPath.size()==0)
		{
			//if(!pGoalCell)
			cout << "Goal Not Found, LaneID: " << pH->laneId <<", Distance : " << distance << endl;
			pGoalCell = pH;
			break;
		}

		//pGoalCell = pH;
	}

	while(nextLeafToTrace.size()!=0)
		nextLeafToTrace.pop_back();
	//closed_nodes.clear();

	return pGoalCell;
}

WayPoint* PlanningHelpers::BuildPlanningSearchTreeStraight(WayPoint* pStart,
		const double& DistanceLimit,
		vector<WayPoint*>& all_cells_to_delete)
{
	if(!pStart) return NULL;

	vector<pair<WayPoint*, WayPoint*> >nextLeafToTrace;

	WayPoint* pZero = 0;
	WayPoint* wp    = new WayPoint();
	*wp = *pStart;
	wp->cost = 0;
	nextLeafToTrace.push_back(make_pair(pZero, wp));
	all_cells_to_delete.push_back(wp);

	double 		distance 		= 0;
	WayPoint* 	pGoalCell 		= 0;
	double 		nCounter 		= 0;

	while(nextLeafToTrace.size()>0)
	{
		nCounter++;

		unsigned int min_cost_index = 0;
		double min_cost = DBL_MAX;

		for(unsigned int i=0; i < nextLeafToTrace.size(); i++)
		{
			if(nextLeafToTrace.at(i).second->cost < min_cost)
			{
				min_cost = nextLeafToTrace.at(i).second->cost;
				min_cost_index = i;
			}
		}

		WayPoint* pH 	= nextLeafToTrace.at(min_cost_index).second;
		assert(pH != 0);

		nextLeafToTrace.erase(nextLeafToTrace.begin()+min_cost_index);

		for(unsigned int i =0; i< pH->pFronts.size(); i++)
		{
			if(pH->pFronts.at(i) && !CheckNodeExits(all_cells_to_delete, pH->pFronts.at(i)))
			{
				wp = new WayPoint();
				*wp = *pH->pFronts.at(i);

				double d = hypot(wp->pos.y - pH->pos.y, wp->pos.x - pH->pos.x);
				distance += d;

//				for(unsigned int a = 0; a < wp->actionCost.size(); a++)
//				{
//					//if(wp->actionCost.at(a).first == FORWARD_ACTION)
//						d += wp->actionCost.at(a).second;
//				}

				wp->cost = pH->cost + d;
				wp->pBacks.push_back(pH);
				if(wp->cost < DistanceLimit)
				{
					nextLeafToTrace.push_back(make_pair(pH, wp));
					all_cells_to_delete.push_back(wp);
				}
				else
					delete wp;
			}
		}

//		if(pH->pLeft && !CheckLaneExits(all_cells_to_delete, pH->pLeft->pLane))
//		{
//			wp = new WayPoint();
//			*wp = *pH->pLeft;
//			double d = hypot(wp->pos.y - pH->pos.y, wp->pos.x - pH->pos.x);
//
//			for(unsigned int a = 0; a < wp->actionCost.size(); a++)
//			{
//				//if(wp->actionCost.at(a).first == LEFT_TURN_ACTION)
//					d += wp->actionCost.at(a).second;
//			}
//
//			wp->cost = pH->cost + d + LANE_CHANGE_COST;
//			wp->pRight = pH;
//			wp->pRight = 0;
//
//			nextLeafToTrace.push_back(make_pair(pH, wp));
//			all_cells_to_delete.push_back(wp);
//		}
//
//		if(pH->pRight && !CheckLaneExits(all_cells_to_delete, pH->pRight->pLane))
//		{
//			wp = new WayPoint();
//			*wp = *pH->pRight;
//			double d = hypot(wp->pos.y - pH->pos.y, wp->pos.x - pH->pos.x);;
//
//			for(unsigned int a = 0; a < wp->actionCost.size(); a++)
//			{
//				//if(wp->actionCost.at(a).first == RIGHT_TURN_ACTION)
//					d += wp->actionCost.at(a).second;
//			}
//
//			wp->cost = pH->cost + d + LANE_CHANGE_COST;
//			wp->pLeft = pH;
//			wp->pRight = 0;
//			nextLeafToTrace.push_back(make_pair(pH, wp));
//			all_cells_to_delete.push_back(wp);
//		}

		pGoalCell = pH;
	}

	while(nextLeafToTrace.size()!=0)
		nextLeafToTrace.pop_back();

	return pGoalCell;
}

int PlanningHelpers::PredictiveIgnorIdsDP(WayPoint* pStart, const double& DistanceLimit,
		vector<WayPoint*>& all_cells_to_delete,vector<WayPoint*>& end_waypoints, std::vector<int>& lanes_ids)
{
	if(!pStart) return 0;

		vector<pair<WayPoint*, WayPoint*> >nextLeafToTrace;

		WayPoint* pZero = 0;
		WayPoint* wp    = new WayPoint();
		*wp = *pStart;
		wp->cost = 0;
		wp->pLeft = 0;
		wp->pRight = 0;
		nextLeafToTrace.push_back(make_pair(pZero, wp));
		all_cells_to_delete.push_back(wp);

		double 		distance 		= 0;
		end_waypoints.clear();
		double 		nCounter 		= 0;

		while(nextLeafToTrace.size()>0)
		{
			nCounter++;

			WayPoint* pH 	= nextLeafToTrace.at(0).second;

			assert(pH != 0);

			nextLeafToTrace.erase(nextLeafToTrace.begin()+0);

			for(unsigned int i =0; i< pH->pFronts.size(); i++)
			{
				if(pH->pFronts.at(i) && !CheckNodeExits(all_cells_to_delete, pH->pFronts.at(i)))
				{
					if(pH->cost < DistanceLimit)
					{
						wp = new WayPoint();
						*wp = *pH->pFronts.at(i);

						double d = distance2points(wp->pos, pH->pos);
						distance += d;
						wp->cost = pH->cost + d;
						wp->pBacks.push_back(pH);
						wp->pLeft = 0;
						wp->pRight = 0;

						bool bFoundLane = false;
						for(unsigned int k = 0 ; k < lanes_ids.size(); k++)
						{
							if(wp->laneId == lanes_ids.at(k))
							{
								bFoundLane = true;
								break;
							}
						}

						if(!bFoundLane)
							nextLeafToTrace.push_back(make_pair(pH, wp));
						all_cells_to_delete.push_back(wp);
					}
					else
					{
						end_waypoints.push_back(pH);
					}
				}
			}
		}

		while(nextLeafToTrace.size()!=0)
			nextLeafToTrace.pop_back();
		//closed_nodes.clear();

		return end_waypoints.size();
}

int PlanningHelpers::PredictiveDP(WayPoint* pStart, const double& DistanceLimit,
		vector<WayPoint*>& all_cells_to_delete,vector<WayPoint*>& end_waypoints)
{
	if(!pStart) return 0;

	vector<pair<WayPoint*, WayPoint*> >nextLeafToTrace;

	WayPoint* pZero = 0;
	WayPoint* wp    = new WayPoint();
	*wp = *pStart;
	wp->pLeft = 0;
	wp->pRight = 0;
	nextLeafToTrace.push_back(make_pair(pZero, wp));
	all_cells_to_delete.push_back(wp);

	double 		distance 		= 0;
	end_waypoints.clear();
	double 		nCounter 		= 0;

	while(nextLeafToTrace.size()>0)
	{
		nCounter++;

		WayPoint* pH 	= nextLeafToTrace.at(0).second;

		assert(pH != 0);

		nextLeafToTrace.erase(nextLeafToTrace.begin()+0);

		for(unsigned int i =0; i< pH->pFronts.size(); i++)
		{
			if(pH->pFronts.at(i) && !CheckNodeExits(all_cells_to_delete, pH->pFronts.at(i)))
			{
				if(pH->cost < DistanceLimit)
				{
					wp = new WayPoint();
					*wp = *pH->pFronts.at(i);

					double d = distance2points(wp->pos, pH->pos);
					distance += d;
					wp->cost = pH->cost + d;
					wp->pBacks.push_back(pH);
					wp->pLeft = 0;
					wp->pRight = 0;

					nextLeafToTrace.push_back(make_pair(pH, wp));
					all_cells_to_delete.push_back(wp);
				}
				else
				{
					end_waypoints.push_back(pH);
				}
			}
		}
	}

	while(nextLeafToTrace.size()!=0)
		nextLeafToTrace.pop_back();
	//closed_nodes.clear();

	return end_waypoints.size();
}

bool PlanningHelpers::CheckLaneIdExits(const std::vector<int>& lanes, const Lane* pL)
{
	if(lanes.size()==0) return true;

	for(unsigned int i=0; i< lanes.size(); i++)
	{
		if(lanes.at(i) == pL->id)
			return true;
	}

	return false;
}

WayPoint* PlanningHelpers::CheckLaneExits(const vector<WayPoint*>& nodes, const Lane* pL)
{
	if(nodes.size()==0) return nullptr;

	for(unsigned int i=0; i< nodes.size(); i++)
	{
		if(nodes.at(i)->pLane == pL)
			return nodes.at(i);
	}

	return nullptr;
}

WayPoint* PlanningHelpers::CheckNodeExits(const vector<WayPoint*>& nodes, const WayPoint* pL)
{
	if(nodes.size()==0) return nullptr;

	for(unsigned int i=0; i< nodes.size(); i++)
	{
		if(nodes.at(i)->laneId == pL->laneId && nodes.at(i)->id == pL->id)
			return nodes.at(i);
	}

	return nullptr;
}

WayPoint* PlanningHelpers::CreateLaneHeadCell(Lane* pLane, WayPoint* pLeft, WayPoint* pRight,
		WayPoint* pBack)
{
	if(!pLane) return nullptr;
	if(pLane->points.size()==0) return nullptr;

	WayPoint* c = new WayPoint;
	c->pLane 		= pLane;
	c->pos 			= pLane->points.at(0).pos;
	c->v			= pLane->speed;
	c->laneId  		= pLane->id;
	c->pLeft 		= pLeft;
	if(pLeft)
		c->cost		= pLeft->cost;

	c->pRight		= pRight;
	if(pRight)
		c->cost = pRight->cost;

	if(pBack)
	{
		pBack->pFronts.push_back(c);
		c->pBacks.push_back(pBack);
		c->cost = pBack->cost + distance2points(c->pos, pBack->pos);

		for(unsigned int i=0; i< c->pBacks.size(); i++)
		{
				if(c->pBacks.at(i)->cost < c->cost)
					c->cost = c->pBacks.at(i)->cost;
		}
	}
	return c;
}

double PlanningHelpers::GetLanePoints(Lane* l, const WayPoint& prevWayPointIndex,
		const double& minDistance , const double& prevCost, vector<WayPoint>& points)
{
	if(l == NULL || minDistance<=0) return 0;

	int index = 0;
	WayPoint  p1, p2;
	WayPoint idx;

	p2 = p1 = l->points.at(index);
	p1.pLane = l;
	p1.cost = prevCost;
	p2.cost = p1.cost + distance2points(p1.pos, p2.pos);

	points.push_back(p1);

	for(unsigned int i=index+1; i<l->points.size(); i++)
	{

		p2 = l->points.at(i);
		p2.pLane = l;
		p2.cost = p1.cost + distance2points(p1.pos, p2.pos);
		points.push_back(p2);

		if(p2.cost >= minDistance)
				break;
		p1 = p2;
	}
	return p2.cost;
}

WayPoint* PlanningHelpers::GetMinCostCell(const vector<WayPoint*>& cells, const vector<int>& globalPathIds)
{
	if(cells.size() == 1)
	{
//		for(unsigned int j = 0; j < cells.at(0)->actionCost.size(); j++)
//			cout << "Cost (" << cells.at(0)->laneId << ") of going : " << cells.at(0)->actionCost.at(j).first << ", is : " << cells.at(0)->actionCost.at(j).second << endl;
		return cells.at(0);
	}

	WayPoint* pC = cells.at(0); //cost is distance
	for(unsigned int i=1; i < cells.size(); i++)
	{
		bool bFound = false;
		if(globalPathIds.size()==0)
			bFound = true;

		int iLaneID = cells.at(i)->id;
		for(unsigned int j=0; j < globalPathIds.size(); j++)
		{
			if(globalPathIds.at(j) == iLaneID)
			{
				bFound = true;
				break;
			}
		}

//		for(unsigned int j = 0; j < cells.at(0)->actionCost.size(); j++)
//			cout << "Cost ("<< i <<") of going : " << cells.at(0)->actionCost.at(j).first << ", is : " << cells.at(0)->actionCost.at(j).second << endl;


		if(cells.at(i)->cost < pC->cost && bFound == true)
			pC = cells.at(i);
	}


	return pC;
}

void PlanningHelpers::ExtractPlanAlernatives(const std::vector<WayPoint>& singlePath, std::vector<std::vector<WayPoint> >& allPaths)
{
	if(singlePath.size() == 0)
		return;

	allPaths.clear();
	std::vector<WayPoint> path;
	path.push_back(singlePath.at(0));
	double skip_distance = 8;
	double d = 0;
	bool bStartSkip = false;
	for(unsigned int i= 1; i < singlePath.size(); i++)
	{
		if(singlePath.at(i).bDir != FORWARD_DIR && singlePath.at(i).pLane && singlePath.at(i).pFronts.size() > 0)
		{

			bStartSkip = true;
			WayPoint start_point = singlePath.at(i-1);

			cout << "Current Velocity = " << start_point.v << endl;

			RelativeInfo start_info;
			PlanningHelpers::GetRelativeInfo(start_point.pLane->points, start_point, start_info);
			vector<WayPoint*> local_cell_to_delete;
			PlannerHNS::WayPoint* pStart = &start_point.pLane->points.at(start_info.iFront);
			WayPoint* pLaneCell =  PlanningHelpers::BuildPlanningSearchTreeStraight(pStart, BACKUP_STRAIGHT_PLAN_DISTANCE, local_cell_to_delete);
			if(pLaneCell)
			{
				vector<WayPoint> straight_path;
				vector<vector<WayPoint> > tempCurrentForwardPathss;
				vector<int> globalPathIds;
				PlanningHelpers::TraversePathTreeBackwards(pLaneCell, pStart, globalPathIds, straight_path, tempCurrentForwardPathss);
				if(straight_path.size() > 2)
				{
					straight_path.insert(straight_path.begin(), path.begin(), path.end());
					for(unsigned int ic = 0; ic < straight_path.size(); ic++)
						straight_path.at(ic).laneChangeCost = 1;
					allPaths.push_back(straight_path);
				}
			}
		}

		if(bStartSkip)
		{
			d += hypot(singlePath.at(i).pos.y - singlePath.at(i-1).pos.y, singlePath.at(i).pos.x - singlePath.at(i-1).pos.x);
			if(d > skip_distance)
			{
				d = 0;
				bStartSkip = false;
			}
		}

		if(!bStartSkip)
			path.push_back(singlePath.at(i));
	}

	allPaths.push_back(path);
}

void PlanningHelpers::TraversePathTreeBackwards(WayPoint* pHead, WayPoint* pStartWP,const vector<int>& globalPathIds,
		vector<WayPoint>& localPath, std::vector<std::vector<WayPoint> >& localPaths)
{
	if(pHead != NULL && pHead->id != pStartWP->id)
	{
		if(pHead->pBacks.size()>0)
		{
			localPaths.push_back(localPath);
			TraversePathTreeBackwards(GetMinCostCell(pHead->pBacks, globalPathIds),pStartWP, globalPathIds, localPath, localPaths);
			pHead->bDir = FORWARD_DIR;
			localPath.push_back(*pHead);
		}
		else if(pHead->pLeft && pHead->cost > 0)
		{
			//vector<Vector2D> forward_path;
			//TravesePathTreeForwards(pHead->pLeft, forward_path, FORWARD_RIGHT);
			//localPaths.push_back(forward_path);
			cout << "Global Lane Change  Right " << endl;
			TraversePathTreeBackwards(pHead->pLeft,pStartWP, globalPathIds, localPath, localPaths);
			pHead->bDir = FORWARD_RIGHT_DIR;
			localPath.push_back(*pHead);
		}
		else if(pHead->pRight && pHead->cost > 0)
		{
			//vector<Vector2D> forward_path;
			//TravesePathTreeForwards(pHead->pRight, forward_path, FORWARD_LEFT);
			//localPaths.push_back(forward_path);

			cout << "Global Lane Change  Left " << endl;
			TraversePathTreeBackwards(pHead->pRight,pStartWP, globalPathIds, localPath, localPaths);
			pHead->bDir = FORWARD_LEFT_DIR;
			localPath.push_back(*pHead);
		}
//		else
//			cout << "Err: PlannerZ -> NULL Back Pointer " << pHead;
	}
	else
		assert(pHead);
}

ACTION_TYPE PlanningHelpers::GetBranchingDirection(WayPoint& currWP, WayPoint& nextWP)
{
	ACTION_TYPE t = FORWARD_ACTION;

//	//first Get the average of the next 3 waypoint directions
//	double angle = 0;
//	if(nextWP.pLane->id == 487)
//		angle = 11;
//
//	int counter = 0;
//	angle = 0;
//
//	for(unsigned int i=0; i < nextWP.pLane->points.size() && counter < 10; i++, counter++)
//	{
//		angle += nextWP.pLane->points.at(i).pos.a;
//	}
//	angle = angle / counter;
//
//	//Get Circular angle for correct subtraction
//	double circle_angle = UtilityH::GetCircularAngle(currWP.pos.a, angle);
//
//	if( currWP.pos.a - circle_angle > (7.5*DEG2RAD))
//	{
//		t = RIGHT_TURN_ACTION;
//		cout << "Right Lane, Average Angle = " << angle*RAD2DEG << ", Circle Angle = " << circle_angle*RAD2DEG << ", currAngle = " << currWP.pos.a*RAD2DEG << endl;
//	}
//	else if( currWP.pos.a - circle_angle < (-7.5*DEG2RAD))
//	{
//		t = LEFT_TURN_ACTION;
//		cout << "Left Lane, Average Angle = " << angle*RAD2DEG << ", Circle Angle = " << circle_angle*RAD2DEG << ", currAngle = " << currWP.pos.a*RAD2DEG << endl;
//	}

	return t;
}

void PlanningHelpers::CalcContourPointsForDetectedObjects(const WayPoint& currPose, vector<DetectedObject>& obj_list, const double& filterDistance)
{
	vector<DetectedObject> res_list;
	for(unsigned int i = 0; i < obj_list.size(); i++)
	{
		GPSPoint center = obj_list.at(i).center.pos;
		double distance = distance2points(center, currPose.pos);

		if(distance < filterDistance)
		{
			DetectedObject obj = obj_list.at(i);

			Mat3 rotationMat(center.a);
			Mat3 translationMat(center.x, center.y);
			double w2 = obj.w/2.0;
			double h2 = obj.l/2.0;
			double z = center.z + obj.h/2.0;

			GPSPoint left_bottom(-w2, -h2, z,0);
			GPSPoint right_bottom(w2,-h2, z,0);
			GPSPoint right_top(w2,h2, z,0);
			GPSPoint left_top(-w2,h2, z,0);

			left_bottom 	= rotationMat * left_bottom;
			right_bottom 	= rotationMat * right_bottom;
			right_top 		= rotationMat * right_top;
			left_top 		= rotationMat * left_top;

			left_bottom 	= translationMat * left_bottom;
			right_bottom 	= translationMat * right_bottom;
			right_top 		= translationMat * right_top;
			left_top 		= translationMat * left_top;

			obj.contour.clear();
			obj.contour.push_back(left_bottom);
			obj.contour.push_back(right_bottom);
			obj.contour.push_back(right_top);
			obj.contour.push_back(left_top);

			res_list.push_back(obj);
		}
	}

	obj_list = res_list;
}

double PlanningHelpers::GetVelocityAhead(const std::vector<WayPoint>& path, const RelativeInfo& info, int& prev_index, const double& reasonable_brake_distance)
{
	if(path.size()==0) return 0;


	double min_v = path.at(info.iBack).v;
	double d = info.to_front_distance;

	int local_i = info.iFront;
	while(local_i < path.size()-1 && d < reasonable_brake_distance)
	{
		local_i++;
		d += hypot(path.at(local_i).pos.y - path.at(local_i-1).pos.y, path.at(local_i).pos.x - path.at(local_i-1).pos.x);
		if(path.at(local_i).v < min_v)
			min_v = path.at(local_i).v;
	}

	if(local_i < prev_index && prev_index < path.size())
	{
		min_v = path.at(prev_index).v;
	}
	else
		prev_index = local_i;

	return min_v;
}

void PlanningHelpers::WritePathToFile(const string& fileName, const vector<WayPoint>& path)
{
	DataRW  dataFile;
	ostringstream str_header;
	str_header << "laneID" << "," << "wpID"  << "," "x" << "," << "y" << "," << "a"<<","<< "cost" << "," << "Speed" << "," ;
	vector<string> dataList;
	 for(unsigned int i=0; i<path.size(); i++)
	 {
		 ostringstream strwp;
		 strwp << path.at(i).laneId << "," << path.at(i).id <<","<<path.at(i).pos.x<<","<< path.at(i).pos.y
				 <<","<< path.at(i).pos.a << "," << path.at(i).cost << "," << path.at(i).v << ",";
		 dataList.push_back(strwp.str());
	 }

	 dataFile.WriteLogData("", fileName, str_header.str(), dataList);
}

LIGHT_INDICATOR PlanningHelpers::GetIndicatorsFromPath(const std::vector<WayPoint>& path, const WayPoint& pose,  const double& seachDistance)
{
	if(path.size() < 2)
		return INDICATOR_NONE;

	LIGHT_INDICATOR ind = INDICATOR_NONE;
	RelativeInfo info;
	PlanningHelpers::GetRelativeInfo(path, pose, info);

	if(info.perp_point.actionCost.size() > 0)
	{
		if(info.perp_point.actionCost.at(0).first == LEFT_TURN_ACTION)
			ind = INDICATOR_LEFT;
		else if(info.perp_point.actionCost.at(0).first == RIGHT_TURN_ACTION)
			ind = INDICATOR_RIGHT;
	}

	double total_d = 0;
	for(unsigned int i=info.iFront; i < path.size()-2; i++)
	{

		total_d+= hypot(path.at(i+1).pos.y - path.at(i).pos.y, path.at(i+1).pos.x - path.at(i).pos.x);
		if(path.at(i).actionCost.size() > 0)
		{
			if(path.at(i).actionCost.at(0).first == LEFT_TURN_ACTION)
				return INDICATOR_LEFT;
			else if(path.at(i).actionCost.at(0).first == RIGHT_TURN_ACTION)
				return INDICATOR_RIGHT;
		}

		if(total_d > seachDistance)
			break;
	}

	return ind;
}

PlannerHNS::WayPoint PlanningHelpers::GetRealCenter(const PlannerHNS::WayPoint& currState, const double& wheel_base)
{
	PlannerHNS::WayPoint pose_center = currState;
	PlannerHNS::Mat3 rotationMat(-currState.pos.a);
	PlannerHNS::Mat3 translationMat(-currState.pos.x, -currState.pos.y);

	PlannerHNS::Mat3 rotationMatInv(currState.pos.a);
	PlannerHNS::Mat3 translationMatInv(currState.pos.x, currState.pos.y);

	pose_center.pos = translationMat*pose_center.pos;
	pose_center.pos = rotationMat*pose_center.pos;

	pose_center.pos.x += wheel_base/3.0;

	pose_center.pos = rotationMatInv*pose_center.pos;
	pose_center.pos = translationMatInv*pose_center.pos;

	return pose_center;
}

double PlanningHelpers::frunge ( double x )
{
  double fx;

  fx = 1.0 / ( 1.0 + 25.0 * x * x );

  return fx;
}

double PlanningHelpers::fprunge ( double x )
{
  double bot;
  double fx;

  bot = 1.0 + 25.0 * x * x;
  fx = -50.0 * x / ( bot * bot );

  return fx;
}

double PlanningHelpers::fpprunge ( double x )
{
  double bot;
  double fx;

  bot = 1.0 + 25.0 * x * x;
  fx = ( -50.0 + 3750.0 * x * x ) / ( bot * bot * bot );

  return fx;
}


} /* namespace PlannerHNS */

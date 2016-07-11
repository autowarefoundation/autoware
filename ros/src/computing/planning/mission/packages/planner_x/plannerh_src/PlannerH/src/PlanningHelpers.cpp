/*
 * PlanningHelpers.cpp
 *
 *  Created on: Jun 16, 2016
 *      Author: hatem
 */

#include "PlanningHelpers.h"
#include "MatrixOperations.h"



using namespace UtilityHNS;
using namespace std;



namespace PlannerHNS {



PlanningHelpers::PlanningHelpers()
{
}

PlanningHelpers::~PlanningHelpers()
{
}

int PlanningHelpers::GetClosestPointIndex(const vector<WayPoint>& trajectory, const WayPoint& p,const int& prevIndex )
{
	if(trajectory.size() == 0)
		return 0;

	double d = 0, minD = 9999999999;
	if(prevIndex < 0) return 0;

	double min_index  = prevIndex;

	for(unsigned int i=prevIndex; i< trajectory.size(); i++)
	{
		d  = distance2pointsSqr(trajectory.at(i).pos, p.pos);
		//if((p.pLane == 0 || trajectory.at(i).pLane == p.pLane) && d < minD)
		if(d < minD)
		{
			min_index = i;
			minD = d;
		}
	}

	return min_index;
}

int PlanningHelpers::GetClosestNextPointIndex(const vector<WayPoint>& trajectory, const WayPoint& p,const int& prevIndex )
{
	if(trajectory.size() == 0)
		return 0;

	double d = 0, minD = 9999999999;
	if(prevIndex < 0) return 0;

	double min_index  = prevIndex;

	for(unsigned int i=prevIndex; i< trajectory.size(); i++)
	{
		d  = distance2pointsSqr(trajectory.at(i).pos, p.pos);
		//if((p.pLane == 0 || trajectory.at(i).pLane == p.pLane) && d < minD)
		if(d < minD)
		{
			min_index = i;
			minD = d;
		}
	}

	if(min_index < trajectory.size()-2)
	{
		GPSPoint curr, next;
		curr = trajectory.at(min_index).pos;
		next = trajectory.at(min_index+1).pos;
		POINT2D v_1(p.pos.x - curr.x   ,p.pos.y - curr.y);
		double norm1 = pointNorm(v_1);
		POINT2D v_2(next.x - curr.x,next.y - curr.y);
		double norm2 = pointNorm(v_2);

		double dot_pro = v_1.x*v_2.x + v_1.y*v_2.y;

		double a = UtilityH::FixNegativeAngle(acos(dot_pro/(norm1*norm2)));

		if(a <= M_PI_2)
			min_index = min_index+1;
	}

	return min_index;
}

WayPoint PlanningHelpers::GetPerpendicularOnTrajectory(const vector<WayPoint>& trajectory, const WayPoint& p, double& distance, const int& prevIndex )
{

	if(trajectory.size() < 2)
		return p;

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
		int next_index = GetClosestNextPointIndex(trajectory, p, prevIndex);

		if(next_index == 0)
		{
			p0 = trajectory[next_index];
			p1 = trajectory[next_index+1];
			p2 = trajectory[next_index+2];
		}
		else if(next_index > 0)
		{
			p0 = trajectory[next_index-1];
			p1 = trajectory[next_index];
			p2 = trajectory[next_index+1];
		}
	}

	Mat3 rotationMat(-p.pos.a);
	Mat3 translationMat(-p.pos.x, -p.pos.y);
	Mat3 invRotationMat(p.pos.a);
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

double PlanningHelpers::GetPerpDistanceToTrajectorySimple(const vector<WayPoint>& trajectory, const WayPoint& p,const int& prevIndex)
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
		next_index = GetClosestNextPointIndex(trajectory, p, prevIndex);
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

	if(isnan(d) || isinf(d))
	{
	  //assert(false);
	  d = 0;
	}

	return d;
}

WayPoint PlanningHelpers::GetNextPointOnTrajectory(const vector<WayPoint>& trajectory, const double& distance, const int& currIndex)
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

	POINT2D uv(p1.pos.x - p2.pos.x, p1.pos.y - p2.pos.y);
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

void PlanningHelpers::FixPathDensity(vector<WayPoint>& path, const double& distanceDensity)
{
	double d2 = distanceDensity*2.0;
	for(unsigned int i=1; i<path.size(); i++)
	{
		double d = distance2points(path.at(i).pos , path.at(i-1).pos);
		if(d <= distanceDensity)
		{
			path.erase(path.begin()+i);
			i--;
		}
	}
	bool bDoneAll = false;
	while(!bDoneAll)
	{
		bDoneAll = true;
		for(unsigned int i=0; i<path.size()-1; i++)
		{
			double d = distance2points(path.at(i).pos , path.at(i+1).pos);
			if(d > d2)
			{
				WayPoint pm = path.at(i);
				pm.pos.x = (pm.pos.x + path.at(i+1).pos.x) / 2.0;
				pm.pos.y = (pm.pos.y + path.at(i+1).pos.y) / 2.0;
				path.insert(path.begin()+i+1, pm);
				bDoneAll = false;
			}
		}
	}
}

void PlanningHelpers::SmoothPath(vector<WayPoint>& path, double weight_data,
		double weight_smooth, double tolerance)
{

	if (path.size() <= 2 )
	{
		cout << "Can't Smooth Path, Path_in Size=" << path.size() << endl;
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

double PlanningHelpers::CalcAngleAndCost(vector<WayPoint>& path, const double& lastCost, const bool& bSmooth)
{
	path[0].pos.a = atan2(path[1].pos.y - path[0].pos.y, path[1].pos.x - path[0].pos.x );
	path[0].cost = lastCost;

	for(unsigned int j = 1; j < path.size()-1; j++)
	{
		path[j].pos.a 		= atan2(path[j+1].pos.y - path[j].pos.y, path[j+1].pos.x - path[j].pos.x );
		path[j].cost 	= path[j-1].cost +  distance2points(path[j-1].pos, path[j].pos);
	}
	unsigned int j = path.size()-1;

	path[j].pos.a 		= path[j-1].pos.a;
	path[j].cost 	= path[j-1].cost + distance2points(path[j-1].pos, path[j].pos);

	if(bSmooth)
	{
		//SmoothWayPointsDirections(path, 0.1, 0.4, 0.1);
	}

	return path[j].cost;
}

double PlanningHelpers::CalcAngleAndCostAndCurvatureAnd2D(vector<WayPoint>& path, const double& lastCost, const bool& bSmooth)
{
	path[0].pos.a 		= atan2(path[1].pos.y - path[0].pos.y, path[1].pos.x - path[0].pos.x );
	path[0].cost 	= lastCost;
	path[0].pos.z		= 0;

	double k = 0;
	GPSPoint center;

	for(unsigned int j = 1; j < path.size()-1; j++)
	{
		k =  CalcCircle(path[j-1].pos,path[j].pos, path[j+1].pos, center);
		if(k > 250 || isnan(k))
			k = 250;

		if(k<1)
			path[j].cost = 0;
		else
			path[j].cost 	= 1.0/k;

		path[j].pos.a 		= atan2(path[j+1].pos.y - path[j].pos.y, path[j+1].pos.x - path[j].pos.x );
		path[j].cost 	= path[j-1].cost + distance2points(path[j-1].pos, path[j].pos);
		path[j].pos.z 	= 0;
	}
	unsigned int j = path.size()-1;

	path[0].cost    = path[1].cost;
	path[j].cost 	= path[j-1].cost;
	path[j].pos.a 	= path[j-1].pos.a;
	path[j].cost 	= path[j-1].cost + distance2points(path[j-1].pos, path[j].pos);
	path[j].pos.z 	= 0;

	if(bSmooth)
	{
//		SmoothWayPointsDirections(path, 0.05, 0.45, 0.01);
//		SmoothCurvatureProfiles(path, 0.05, 0.45, 0.01);
	}

	return path[j].cost;
}

double PlanningHelpers::CalcCircle(const GPSPoint& pt1, const GPSPoint& pt2, const GPSPoint& pt3, GPSPoint& center)
{
	double yDelta_a= pt2.y - pt1.y;
	double xDelta_a= pt2.x - pt1.x;
	double yDelta_b= pt3.y - pt2.y;
	double xDelta_b= pt3.x - pt2.x;

	if (fabs(xDelta_a) <= 0.000000001 && fabs(yDelta_b) <= 0.000000001)
	{

		center.x= 0.5*(pt2.x + pt3.x);
		center.y= 0.5*(pt1.y + pt2.y);

		return distance2points(center,pt1);

	}

	 //IsPerpendicular() assure that xDelta(s) are not zero
	double aSlope=yDelta_a/xDelta_a; //
	double bSlope=yDelta_b/xDelta_b;
	if (fabs(aSlope-bSlope) <= 0.000000001)
	{
		return -1;
	}

	// calc center
	center.x= (aSlope*bSlope*(pt1.y - pt3.y) + bSlope*(pt1.x + pt2 .x)
		- aSlope*(pt2.x+pt3.x) )/(2* (bSlope-aSlope) );
	center.y = -1*(center.x - (pt1.x+pt2.x)/2)/aSlope +  (pt1.y+pt2.y)/2;
	//center.z= pt1.m_z;

	return  distance2points(center,pt1);		// calc. radius
}

void PlanningHelpers::ExtractPartFromPointToDistance(const vector<WayPoint>& originalPath, const WayPoint& pos, const double& minDistance,
		const double& pathDensity, vector<WayPoint>& extractedPath, const double& SmoothDataWeight, const double& SmoothWeight, const double& SmoothTolerance)
{
	extractedPath.clear();
	unsigned int close_index = GetClosestPointIndex(originalPath, pos);
	vector<WayPoint> tempPath;
	double d_limit = 0;
	if(close_index>=5) close_index -=5;
	else close_index = 0;

	for(unsigned int i=close_index; i< originalPath.size(); i++)
	{
		tempPath.push_back(originalPath.at(i));
		if(i+1 < originalPath.size())
			d_limit += distance2points(originalPath.at(i).pos, originalPath.at(i+1).pos);

		if(d_limit > minDistance)
		{
			if(i+1 < originalPath.size())
				tempPath.push_back(originalPath.at(i+1));
			break;
		}
	}

	if(tempPath.size() < 2)
	{
		cout << endl << "### Planner Z . Extracted Rollout Path is too Small, Size = " << tempPath.size() << endl;
		return;
	}

	FixPathDensity(tempPath, pathDensity);
	SmoothPath(tempPath, SmoothDataWeight, SmoothWeight , SmoothTolerance);
	CalcAngleAndCost(tempPath);

	extractedPath = tempPath;

	if(extractedPath.at(extractedPath.size()-1).cost < 35)
		cout << endl << "### Planner Z . Extracted Rollout Path is too Short, Distance =  " << extractedPath.at(extractedPath.size()-1).cost << endl;
}

void PlanningHelpers::CalculateRollInTrajectories(const WayPoint& carPos, const double& speed, const vector<WayPoint>& originalCenter, int& start_index,
		int& end_index, vector<double>& end_laterals ,
		vector<vector<WayPoint> >& rollInPaths, const double& max_roll_distance,
		const double& maxSpeed, const double&  carTipMargin, const double& rollInMargin,
		const double& rollInSpeedFactor, const double& pathDensity, const double& rollOutDensity,
		const int& rollOutNumber, const double& SmoothDataWeight, const double& SmoothWeight,
		const double& SmoothTolerance, const bool& bHeadingSmooth)
{

	WayPoint rearPos = carPos;
	WayPoint p;
	double dummyd = 0;
//	rearPos.pos.x -= m_pCarInfo->wheel_base/2.0 * cos(rearPos.a);
//	rearPos.pos.y -= m_pCarInfo->wheel_base/2.0 * sin(rearPos.a);

	int iLimitIndex = (carTipMargin/0.3)/pathDensity;
	if(iLimitIndex >= originalCenter.size())
		iLimitIndex = originalCenter.size() - 1;

	//Get Closest Index
	unsigned int close_index = GetClosestPointIndex(originalCenter, rearPos);
	double remaining_distance = 0;
	for(unsigned int i=close_index; i< originalCenter.size()-1; i++)
	  {
		if(i>0)
			remaining_distance += distance2points(originalCenter[i].pos, originalCenter[i+1].pos);
	  }

	double initial_roll_in_distance = GetPerpDistanceToTrajectorySimple(originalCenter, carPos, close_index);


	vector<WayPoint> RollOutStratPath;
	///***   Smoothing From Car Heading Section ***///
	if(bHeadingSmooth)
	{
		unsigned int num_of_strait_points = carTipMargin / pathDensity;
		int closest_for_each_iteration = 0;
		WayPoint np = GetPerpendicularOnTrajectory(originalCenter, rearPos, dummyd, closest_for_each_iteration);
		np.pos = rearPos.pos;

		RollOutStratPath.push_back(np);
		for(unsigned int i = 0; i < num_of_strait_points; i++)
		{
			p = RollOutStratPath.at(i);
			p.pos.x = p.pos.x +  pathDensity*cos(p.pos.a);
			p.pos.y = p.pos.y +  pathDensity*sin(p.pos.a);
			np = GetPerpendicularOnTrajectory(originalCenter, p, dummyd, closest_for_each_iteration);
			np.pos = p.pos;
			RollOutStratPath.push_back(np);
		}

		initial_roll_in_distance = GetPerpDistanceToTrajectorySimple(originalCenter, RollOutStratPath.at(RollOutStratPath.size()-1), close_index);
	}
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
	  }
	}

	for(unsigned int i=0; i< rollOutNumber+1 ; i++)
		rollInPaths.at(i).insert(rollInPaths.at(i).begin(), execluded_from_smoothing.at(i).begin(), execluded_from_smoothing.at(i).end());

	///***   Smoothing From Car Heading Section ***///
	if(bHeadingSmooth)
	{
		for(unsigned int i=0; i< rollOutNumber+1 ; i++)
		{
			unsigned int cut_index = GetClosestNextPointIndex(rollInPaths.at(i), RollOutStratPath.at(RollOutStratPath.size()-1));
			rollInPaths.at(i).erase(rollInPaths.at(i).begin(), rollInPaths.at(i).begin()+cut_index);
			rollInPaths.at(i).insert(rollInPaths.at(i).begin(), RollOutStratPath.begin(), RollOutStratPath.end());
		}
	}
	///***   -------------------------------- ***///

	for(unsigned int i=0; i< rollOutNumber+1 ; i++)
	{
		SmoothPath(rollInPaths.at(i), SmoothDataWeight, SmoothWeight, SmoothTolerance);
	}

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
		  }
	  }

//	for(unsigned int i=0; i< rollInPaths.size(); i++)
//		CalcAngleAndCost(rollInPaths.at(i));
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

void PlanningHelpers::GenerateRecommendedSpeed(vector<WayPoint>& path, const double& max_speed, const double& speedProfileFactor)
{
	vector<WayPoint> totalPath, tempPath;
	unsigned int latestIndex = 0;
	double latestDistance = 0;

	CalcAngleAndCostAndCurvatureAnd2D(path);
	//process every 150 meters
	while(latestIndex < path.size())
	{
		tempPath.clear();

		for(;latestIndex < path.size(); latestIndex++)
		{
			if((path.at(latestIndex).cost - latestDistance) >= 150)
			{
				latestDistance = path.at(latestIndex).cost;
				break;
			}
			tempPath.push_back(path.at(latestIndex));
		}

		double prev_a = tempPath.at(0).pos.a;
		double a = prev_a;

		for(unsigned int i = 1 ; i < tempPath.size(); i++)
		{
			a  = UtilityH::GetCircularAngle(tempPath.at(i-1).pos.a, tempPath.at(i).pos.a);

			double diff = (a - prev_a)*speedProfileFactor;
			if(speedProfileFactor == 0)
				tempPath.at(i).v = max_speed;
			else if(abs(diff) > 1.0 && max_speed > 2) // hard turn , speed = 3.0 m/s
				tempPath.at(i).v = 2.0;
			else if(abs(diff) > 0.9 && max_speed > 3) // hard turn , speed = 3.0 m/s
				tempPath.at(i).v = 3.0;
			else if(abs(diff) > 0.8 && max_speed > 4) // hard turn , speed = 3.0 m/s
				tempPath.at(i).v = 4.0;
			else if(abs(diff) > 0.7 && max_speed > 5) // hard turn , speed = 3.0 m/s
				tempPath.at(i).v = 5.0;
			else if(abs(diff) > 0.6 && max_speed > 6) // hard turn , speed = 3.0 m/s
				tempPath.at(i).v = 6.0;
			else if(abs(diff) > 0.5 && max_speed > 7) // hard turn , speed = 3.0 m/s
				tempPath.at(i).v = 7.0;
			else if(abs(diff) > 0.4 && max_speed > 8) // medium turn , speed = 6.0 m/s
				tempPath.at(i).v = 8.0;
			else if(abs(diff) > 0.3 && max_speed > 9) // medium turn , speed = 6.0 m/s
				tempPath.at(i).v = 9.0;
			else if(abs(diff) > 0.2 && max_speed > 10) // medium turn , speed = 6.0 m/s
				tempPath.at(i).v = 10.0;
//			else if(abs(diff) > 0.25) // light turn , speed = 6.0 m/s
//				tempPath.at(i).vteer = 8.0;
			else // max
				tempPath.at(i).v = max_speed;

			//tempPath.at(i).vteer = abs(diff);
			//tempPath.at(i).vteer = tempPath.at(i).v / (tempPath.at(i).cost - tempPath.at(i-1).cost);
			if(i==1)
			{
				//tempPath.at(i-1).steer = tempPath.at(i).steer;
				//tempPath.at(i-1).a = tempPath.at(i).a;
				tempPath.at(i-1).v = tempPath.at(i).v;
			}

			prev_a = a;
		}

		totalPath.insert(totalPath.end(), tempPath.begin(), tempPath.end());
	}

	path = totalPath;
}

WayPoint* PlanningHelpers::BuildPlanningSearchTree(Lane* l, const WayPoint& prevWayPointIndex,
		const WayPoint& startPos, const WayPoint& goalPos,
		const vector<int>& globalPath, const double& DistanceLimit,
		int& nMaxLeftBranches, int& nMaxRightBranches,
		vector<WayPoint*>& all_cells_to_delete )
{
	if(!l) return NULL;

	all_cells_to_delete.clear();
	vector<WayPoint> currentLanePoints;
	vector<pair<Lane*, WayPoint*> > closed_lanes;
	vector<pair<Lane*, WayPoint*> > nextLeafToTrace;


	WayPoint* pHead = new WayPoint();
	all_cells_to_delete.push_back(pHead);
	pHead->pos 		= startPos.pos;
	pHead->pLane 	= l;
	pHead->v		= l->speed;
	pHead->laneId 	= l->id;

	nextLeafToTrace.push_back(make_pair(l,pHead));
	closed_lanes.push_back(make_pair(l,pHead));

	double 		distance 		= 0;
	nMaxLeftBranches 			= 0;
	nMaxRightBranches 			= 0;
	int 		nCurrBranches 	= 0;
	WayPoint* 	pc 				= 0;
	WayPoint* 	pGoalCell 		= 0;
	bool 		bGoalReached 	= false;
	double 		d 				= 0;
	double 		nCounter 		= 0;
	WayPoint*   pNewCell 		= 0;

	while(nextLeafToTrace.size()>0)
	{
		nCounter++;
		Lane* currL		= nextLeafToTrace.at(0).first;
		WayPoint* pH 	= nextLeafToTrace.at(0).second;

		assert(pH != 0);
		assert(currL !=0);

		nextLeafToTrace.erase(nextLeafToTrace.begin()+0);

		//Get the points of the lane ,
		currentLanePoints.clear();
		d = GetLanePoints(currL, prevWayPointIndex, DistanceLimit, pH->cost, currentLanePoints);

		WayPoint* pCurrentFirstHead = pH;
		double closest_to_the_goal = 9999999;

		for(unsigned int i=0; i < currentLanePoints.size(); i++)
		{
			if(i>0)
			{
				pc 				= new WayPoint;
				pc->pos 		= currentLanePoints.at(i).pos;
				pc->pLane 		= currL;
				pc->pBacks.push_back(pH);
				pH->pFronts.push_back(pc);

				all_cells_to_delete.push_back(pc);

				pH = pc;
			}

			double distance_to_goal = distance2points(pH->pos, goalPos.pos);
			if(distance_to_goal < closest_to_the_goal)
				closest_to_the_goal = distance_to_goal;

			if( distance_to_goal <= 5.0)
			{
				pGoalCell = pH;
				bGoalReached = true;
			}
		}

		if(!bGoalReached)
		{

			if(currL->pLeftLane && !CheckLaneExits(closed_lanes, currL->pLeftLane) && CheckLaneIdExits(globalPath, currL) && globalPath.size() > 0)
			{
				pNewCell = CreateLaneHeadCell(currL->pLeftLane, 0,pCurrentFirstHead, 0);
				all_cells_to_delete.push_back(pNewCell);
				nextLeafToTrace.push_back(make_pair(currL->pLeftLane, pNewCell));
				closed_lanes.push_back(make_pair(currL->pLeftLane, pNewCell));
			}
			if(currL->pRightLane && !CheckLaneExits(closed_lanes, currL->pRightLane) && CheckLaneIdExits(globalPath, currL) && globalPath.size() > 0)
			{
				pNewCell = CreateLaneHeadCell(currL->pRightLane, pCurrentFirstHead, 0 , 0);
				all_cells_to_delete.push_back(pNewCell);
				nextLeafToTrace.push_back(make_pair(currL->pRightLane, pNewCell));
				closed_lanes.push_back(make_pair(currL->pRightLane, pNewCell));
			}

			for(unsigned int i =0; i< currL->toLanes.size(); i++)
			{
				if(CheckLaneIdExits(globalPath, currL->toLanes.at(i)))
				{
					WayPoint* pClosedNextHead = CheckLaneExits(closed_lanes, currL->toLanes.at(i));
					if(!pClosedNextHead)
					{
						pNewCell = CreateLaneHeadCell(currL->toLanes.at(i), 0, 0, pH);
						all_cells_to_delete.push_back(pNewCell);
						nextLeafToTrace.push_back(make_pair(currL->toLanes.at(i), pNewCell));
						closed_lanes.push_back(make_pair(currL->toLanes.at(i), pNewCell));
					}
					else
					{
						pClosedNextHead->pBacks.push_back(pH);
						for(unsigned int j=0; j< pClosedNextHead->pBacks.size(); j++)
						{
								if(pClosedNextHead->pBacks.at(j)->cost < pClosedNextHead->cost)
									pClosedNextHead->cost = pClosedNextHead->pBacks.at(j)->cost;
						}
					}
				}
			}
		}

		distance+= d;

		if(distance > DistanceLimit && globalPath.size()==0)
		{
			//if(!pGoalCell)
			pGoalCell = pH;
			break;
		}

		pGoalCell = pH;
	}

	currentLanePoints.clear();
	while(nextLeafToTrace.size()!=0)
		nextLeafToTrace.pop_back();
	closed_lanes.clear();

	return pGoalCell;
}

WayPoint* PlanningHelpers::BuildPlanningSearchTreeV2(WayPoint* pStart, const WayPoint& prevWayPointIndex,
		const WayPoint& startPos, const WayPoint& goalPos,
		const vector<int>& globalPath, const double& DistanceLimit,
		int& nMaxLeftBranches, int& nMaxRightBranches,
		vector<WayPoint*>& all_cells_to_delete )
{
	if(!pStart) return NULL;

	vector<WayPoint*> closed_nodes;
	vector<pair<WayPoint*, WayPoint*> >nextLeafToTrace;

	WayPoint* pZero = 0;
	WayPoint* wp    = new WayPoint();
	*wp = *pStart;
	nextLeafToTrace.push_back(make_pair(pZero, wp));
	closed_nodes.push_back(wp);
	all_cells_to_delete.push_back(wp);

	double 		distance 		= 0;
	nMaxLeftBranches 			= 0;
	nMaxRightBranches 			= 0;
	int 		nCurrBranches 	= 0;
	WayPoint* 	pGoalCell 		= 0;
	double 		nCounter 		= 0;


	while(nextLeafToTrace.size()>0)
	{
		nCounter++;
		WayPoint* pH 	= nextLeafToTrace.at(0).second;
		WayPoint* pPrev 	= nextLeafToTrace.at(0).first;

		assert(pH != 0);

		nextLeafToTrace.erase(nextLeafToTrace.begin()+0);

		if(pPrev == 0) // first point
			pH->cost  = 0;
		else
			pH->cost += distance2points(pPrev->pos, pH->pos);

		distance+= pH->cost;

		double distance_to_goal = distance2points(pH->pos, goalPos.pos);
		if( distance_to_goal <= 5.0)
		{
			pGoalCell = pH;
			break;
		}
		else
		{

			if(pH->pLeft && !CheckNodeExits(closed_nodes, pH->pLeft) && CheckLaneIdExits(globalPath, pH->pLane) && globalPath.size() > 0)
			{
				wp = new WayPoint();
				*wp = *pH->pLeft;
				nextLeafToTrace.push_back(make_pair(pH, wp));
				closed_nodes.push_back(wp);
				all_cells_to_delete.push_back(wp);
			}
			if(pH->pRight && !CheckNodeExits(closed_nodes, pH->pRight) && CheckLaneIdExits(globalPath, pH->pLane) && globalPath.size() > 0)
			{
				wp = new WayPoint();
				*wp = *pH->pRight;
				nextLeafToTrace.push_back(make_pair(pH, wp));
				closed_nodes.push_back(wp);
				all_cells_to_delete.push_back(wp);
			}

			for(unsigned int i =0; i< pH->pFronts.size(); i++)
			{
				if(CheckLaneIdExits(globalPath, pH->pLane))
				{
					WayPoint* pClosedNextHead = CheckNodeExits(closed_nodes, pH->pFronts.at(i));
					if(!pClosedNextHead && pH->pFronts.at(i))
					{
						if(pH->pBacks.size()==0)
							pClosedNextHead = 0;

						wp = new WayPoint();
						*wp = *pH->pFronts.at(i);
						wp->pBacks.push_back(pH);
						nextLeafToTrace.push_back(make_pair(pH, wp));
						closed_nodes.push_back(wp);
						all_cells_to_delete.push_back(wp);
					}
//					else
//					{
//						pClosedNextHead->pBacks.push_back(pH);
//						for(unsigned int j=0; j< pClosedNextHead->pBacks.size(); j++)
//						{
//								if(pClosedNextHead->pBacks.at(j)->cost < pClosedNextHead->cost)
//									pClosedNextHead->cost = pClosedNextHead->pBacks.at(j)->cost;
//						}
//					}
				}
			}
		}

		if(distance > DistanceLimit && globalPath.size()==0)
		{
			//if(!pGoalCell)
			pGoalCell = pH;
			break;
		}

		pGoalCell = pH;
	}

	while(nextLeafToTrace.size()!=0)
		nextLeafToTrace.pop_back();
	closed_nodes.clear();

	return pGoalCell;
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

WayPoint* PlanningHelpers::CheckLaneExits(const vector<pair<Lane*, WayPoint*> >& lanes, const Lane* pL)
{
	if(lanes.size()==0) return 0;

	for(unsigned int i=0; i< lanes.size(); i++)
	{
		if(lanes.at(i).first==pL)
			return lanes.at(i).second;
	}

	return 0;
}

WayPoint* PlanningHelpers::CheckNodeExits(const vector<WayPoint*>& nodes, const WayPoint* pL)
{
	if(nodes.size()==0) return 0;

	for(unsigned int i=0; i< nodes.size(); i++)
	{
		if(nodes.at(i)==pL)
			return nodes.at(i);
	}

	return 0;
}

WayPoint* PlanningHelpers::CreateLaneHeadCell(Lane* pLane, WayPoint* pLeft, WayPoint* pRight,
		WayPoint* pBack)
{
	if(!pLane) return 0;
	if(pLane->points.size()==0) return 0;

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
		return cells.at(0);

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

		if(cells.at(i)->cost < pC->cost && bFound == true)
			pC = cells.at(i);
	}


	return pC;
}

void PlanningHelpers::TravesePathTreeBackwards(WayPoint* pHead, WayPoint* pStartWP,const vector<int>& globalPathIds,
vector<WayPoint>& localPath, std::vector<std::vector<WayPoint> >& localPaths)
{
	if(pHead != NULL && pHead != pStartWP)
	{
		if(pHead->pBacks.size()>0)
		{
			localPaths.push_back(localPath);

//			int check = 0;
//			if(localPaths.size()>=30)
//				check = 100;

			TravesePathTreeBackwards(GetMinCostCell(pHead->pBacks, globalPathIds),pStartWP, globalPathIds, localPath, localPaths);
			pHead->bDir = FORWARD_DIR;
			localPath.push_back(*pHead);


		}
//		else if(pHead->pLeft)
//		{
//			//vector<Vector2D> forward_path;
//			//TravesePathTreeForwards(pHead->pLeft, forward_path, FORWARD_RIGHT);
//			//localPaths.push_back(forward_path);
//
//			TravesePathTreeBackwards(pHead->pLeft,globalPathIds, localPath, localPaths);
//			pHead->bDir = FORWARD_RIGHT_DIR;
//			localPath.push_back(*pHead);
//		}
//		else if(pHead->pRight)
//		{
//			//vector<Vector2D> forward_path;
//			//TravesePathTreeForwards(pHead->pRight, forward_path, FORWARD_LEFT);
//			//localPaths.push_back(forward_path);
//
//			TravesePathTreeBackwards(pHead->pRight,globalPathIds, localPath, localPaths);
//			pHead->bDir = FORWARD_LEFT_DIR;
//			localPath.push_back(*pHead);
//		}
//		else
//			cout << "Err: PlannerZ -> NULL Back Pointer " << pHead;
	}
	else
		assert(pHead);
}

} /* namespace PlannerHNS */

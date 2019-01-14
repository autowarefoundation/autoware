
/// \file  BehaviorPrediction.cpp
/// \brief Predict detected vehicles's possible trajectories, these trajectories extracted from the vector map.
/// \author Hatem Darweesh
/// \date Jul 6, 2017



#include "op_planner/BehaviorPrediction.h"
#include "op_planner/MappingHelpers.h"
#include "op_planner/PlanningHelpers.h"
#include "op_planner/MatrixOperations.h"


namespace PlannerHNS
{

BehaviorPrediction::BehaviorPrediction()
{
	m_MaxLaneDetectionDistance = 0.5;
	m_PredictionDistance = 20.0;
	m_bGenerateBranches = false;
	m_bUseFixedPrediction = true;
	m_bStepByStep = false;
	m_bCanDecide = true;
	m_bParticleFilter = false;
	UtilityHNS::UtilityH::GetTickCount(m_GenerationTimer);
	UtilityHNS::UtilityH::GetTickCount(m_ResamplingTimer);
	m_bFirstMove = true;
	m_bDebugOut = false;
}

BehaviorPrediction::~BehaviorPrediction()
{
}

void BehaviorPrediction::FilterObservations(const std::vector<DetectedObject>& obj_list, RoadNetwork& map, std::vector<DetectedObject>& filtered_list)
{
	for(unsigned int i=0; i < obj_list.size(); i++)
	{
		if(obj_list.at(i).t == SIDEWALK || obj_list.at(i).center.v < 1.0)
			continue;

		bool bFound = false;
		int found_index = 0;
		for(unsigned int ip=0; ip < filtered_list.size(); ip++)
		{
			if(filtered_list.at(ip).id == obj_list.at(i).id)
			{
				found_index = ip;
				bFound = true;
				break;
			}
		}

		if(bFound)
			filtered_list.at(found_index) = obj_list.at(i);
		else
			filtered_list.push_back(obj_list.at(i));
	}

	for(int ip=0; ip < (int)filtered_list.size(); ip++)
	{
		//check for cleaning
		bool bRevFound = false;
		for(unsigned int ic=0; ic < obj_list.size(); ic++)
		{
			if(filtered_list.at(ip).id == obj_list.at(ic).id)
			{
				bRevFound = true;
				break;
			}
		}

		if(!bRevFound)
		{
			filtered_list.erase(filtered_list.begin()+ip);
			ip--;
		}
	}
}

void BehaviorPrediction::DoOneStep(const std::vector<DetectedObject>& obj_list, const WayPoint& currPose, const double& minSpeed, const double& maxDeceleration, RoadNetwork& map)
{
	if(!m_bUseFixedPrediction && maxDeceleration !=0)
		m_PredictionDistance = -pow(currPose.v, 2)/(maxDeceleration);

	ExtractTrajectoriesFromMap(obj_list, map, m_ParticleInfo_II);
	CalculateCollisionTimes(minSpeed);

	if(m_bParticleFilter)
	{
		ParticleFilterSteps(m_ParticleInfo_II);
	}
}

void BehaviorPrediction::CalculateCollisionTimes(const double& minSpeed)
{
	for(unsigned int i=0; i < m_ParticleInfo_II.size(); i++)
	{
		for(unsigned int j=0; j < m_ParticleInfo_II.at(i)->obj.predTrajectories.size(); j++)
		{
			PlannerHNS::PlanningHelpers::PredictConstantTimeCostForTrajectory(m_ParticleInfo_II.at(i)->obj.predTrajectories.at(j), m_ParticleInfo_II.at(i)->obj.center, minSpeed, m_PredictionDistance);
//			PlannerHNS::PlanningHelpers::CalcAngleAndCost(m_PredictedObjects.at(i).predTrajectories.at(j));
		}
	}
}

void BehaviorPrediction::ExtractTrajectoriesFromMap(const std::vector<DetectedObject>& curr_obj_list,RoadNetwork& map, std::vector<ObjParticles*>& old_obj_list)
{
	PlannerH planner;
	m_temp_list_ii.clear();

	std::vector<ObjParticles*> delete_me_list = old_obj_list;

	for(unsigned int i=0; i < curr_obj_list.size(); i++)
	{
		bool bMatch = false;
		for(unsigned int ip=0; ip < old_obj_list.size(); ip++)
		{
			if(old_obj_list.at(ip)->obj.id == curr_obj_list.at(i).id)
			{
				bool bFound = false;
				for(unsigned int k=0; k < m_temp_list_ii.size(); k++)
				{
					if(m_temp_list_ii.at(k) == old_obj_list.at(ip))
					{
						bFound = true;
						break;
					}
				}

				if(!bFound)
				{
					old_obj_list.at(ip)->obj = curr_obj_list.at(i);
					m_temp_list_ii.push_back(old_obj_list.at(ip));
				}

				DeleteFromList(delete_me_list, old_obj_list.at(ip));

				old_obj_list.erase(old_obj_list.begin()+ip);
				bMatch = true;
				break;
			}
		}

		if(!bMatch)
		{
			ObjParticles* pNewObj = new  ObjParticles();
			pNewObj->obj = curr_obj_list.at(i);
			m_temp_list_ii.push_back(pNewObj);
		}
	}

	DeleteTheRest(delete_me_list);
	old_obj_list.clear();
	old_obj_list = m_temp_list_ii;

	//m_PredictedObjects.clear();
	for(unsigned int ip=0; ip < old_obj_list.size(); ip++)
	{
		PredictCurrentTrajectory(map, old_obj_list.at(ip));
		//m_PredictedObjects.push_back(old_obj_list.at(ip)->obj);
		old_obj_list.at(ip)->MatchTrajectories();
	}

}

void BehaviorPrediction::CalPredictionTimeForObject(ObjParticles* pCarPart)
{
	if(pCarPart->obj.center.v > 0 )
		pCarPart->m_PredictionTime = (MIN_PREDICTION_TIME + 1.5*pCarPart->obj.center.v) / pCarPart->obj.center.v;
	else
		pCarPart->m_PredictionTime = MIN_PREDICTION_TIME;
}

void BehaviorPrediction::PredictCurrentTrajectory(RoadNetwork& map, ObjParticles* pCarPart)
{
	pCarPart->obj.predTrajectories.clear();
	PlannerH planner;
	if(pCarPart->obj.bDirection && pCarPart->obj.bVelocity)
	{
		PlannerHNS::WayPoint fake_pose = pCarPart->obj.center;
		pCarPart->obj.pClosestWaypoints = MappingHelpers::GetClosestWaypointsListFromMap(fake_pose, map, m_MaxLaneDetectionDistance, pCarPart->obj.bDirection);
		planner.PredictTrajectoriesUsingDP(fake_pose, pCarPart->obj.pClosestWaypoints, m_PredictionDistance, pCarPart->obj.predTrajectories, m_bGenerateBranches, pCarPart->obj.bDirection);
	}
	else
	{
		bool bLocalDirectionSearch = false;
		pCarPart->obj.pClosestWaypoints = MappingHelpers::GetClosestWaypointsListFromMap(pCarPart->obj.center, map, m_MaxLaneDetectionDistance, pCarPart->obj.bDirection);
		if(pCarPart->obj.pClosestWaypoints.size()>0)
		{
			pCarPart->obj.center.pos.a = pCarPart->obj.pClosestWaypoints.at(0)->pos.a;
			bLocalDirectionSearch = true;
		}
		planner.PredictTrajectoriesUsingDP(pCarPart->obj.center, pCarPart->obj.pClosestWaypoints, m_PredictionDistance, pCarPart->obj.predTrajectories, m_bGenerateBranches, pCarPart->obj.bDirection);
	}


	for(unsigned int t = 0; t < pCarPart->obj.predTrajectories.size(); t ++)
	{
//		std::ostringstream path_name;
//		path_name << "/home/hatem/autoware_openplanner_logs/TempPredLog/";
//		path_name << t ;
//		path_name << "_";
//		PlanningHelpers::GenerateRecommendedSpeed(pCarPart->obj.predTrajectories.at(t), 10, 1.0);
//		PlannerHNS::PlanningHelpers::WritePathToFile(path_name.str(), pCarPart->obj.predTrajectories.at(t));
		if(pCarPart->obj.predTrajectories.at(t).size() > 0)
			pCarPart->obj.predTrajectories.at(t).at(0).collisionCost = 0;
	}
}

void BehaviorPrediction::ParticleFilterSteps(std::vector<ObjParticles*>& part_info)
{
	for(unsigned int i=0; i < part_info.size(); i++)
	{
		SamplesFreshParticles(part_info.at(i));
		CollectParticles(part_info.at(i));
		MoveParticles(part_info.at(i));
		CalculateWeights(part_info.at(i));
		RemoveWeakParticles(part_info.at(i));
		CalculateAveragesAndProbabilities(part_info.at(i));
		FindBest(part_info.at(i));
	}
}

int BehaviorPrediction::FromIndicatorToNumber(const PlannerHNS::LIGHT_INDICATOR& indi)
{
	if(indi == PlannerHNS::INDICATOR_NONE)
		return 0;
	else if(indi == PlannerHNS::INDICATOR_LEFT)
	{
		return 1;
	}
	else if(indi == PlannerHNS::INDICATOR_RIGHT)
		return 2;
	else if(indi == PlannerHNS::INDICATOR_BOTH)
		return 3;
	else
		return 0;
}

PlannerHNS::LIGHT_INDICATOR BehaviorPrediction::FromNumbertoIndicator(const int& num)
{
	if(num == 0)
		return PlannerHNS::INDICATOR_NONE;
	else if(num == 1)
		return PlannerHNS::INDICATOR_LEFT;
	else if(num == 2)
		return PlannerHNS::INDICATOR_RIGHT;
	else if(num == 3)
		return PlannerHNS::INDICATOR_BOTH;
	else
		return PlannerHNS::INDICATOR_NONE;
}

double BehaviorPrediction::CalcIndicatorWeight(PlannerHNS::LIGHT_INDICATOR p_ind, PlannerHNS::LIGHT_INDICATOR obj_ind)
{
	if((obj_ind == PlannerHNS::INDICATOR_LEFT || obj_ind == PlannerHNS::INDICATOR_RIGHT || obj_ind == PlannerHNS::INDICATOR_NONE) && p_ind == obj_ind)
		return 0.99;
	else
		return 0.01;
}

double BehaviorPrediction::CalcAccelerationWeight(int p_acl, int obj_acl)
{
	if((p_acl > 0 && obj_acl > 0) || (p_acl < 0 && obj_acl < 0))
		return 0.99;
	else
		return 0.01;
}

void BehaviorPrediction::CalOnePartWeight(ObjParticles* pParts,Particle& p)
{
	if(p.bDeleted) return;

	//p.pose_w = exp(-(0.5*pow((p.pose.pos.x - pParts->obj.center.pos.x),2)/(2*MEASURE_POSE_ERROR*MEASURE_POSE_ERROR)+ pow((p.pose.pos.y - pParts->obj.center.pos.y),2)/(2*MEASURE_POSE_ERROR*MEASURE_POSE_ERROR)));
	p.pose_w = 1.0/hypot(0.5*(p.pose.pos.y - pParts->obj.center.pos.y), 0.5*(p.pose.pos.x - pParts->obj.center.pos.x));
	//p.dir_w  = exp(-(pow(fabs(UtilityHNS::UtilityH::AngleBetweenTwoAnglesPositive(p.pose.pos.a,  pParts->obj.center.pos.a)),2)/(2*MEASURE_ANGLE_ERROR*MEASURE_ANGLE_ERROR)));
	p.dir_w  = M_PI_2 - fabs(UtilityHNS::UtilityH::AngleBetweenTwoAnglesPositive(p.pose.pos.a,  pParts->obj.center.pos.a));
	p.vel_w  = exp(-(pow((p.vel - pParts->obj.center.v),2)/(2*MEASURE_VEL_ERROR*MEASURE_VEL_ERROR)));
	//p.vel_w = fabs(p.vel - pParts->obj.center.v);
	p.ind_w  = CalcIndicatorWeight(FromNumbertoIndicator(p.indicator), pParts->obj.indicator_state);
	p.ind_w  -= p.ind_w*MEASURE_IND_ERROR;
	p.acl_w = CalcAccelerationWeight(p.acc, pParts->obj.acceleration_desc);

	//std::cout << p.beh << "|" << p.vel_w <<"|" <<p.vel << "|" << pParts->obj.center.v;

	pParts->pose_w_t += p.pose_w;
	pParts->dir_w_t += p.dir_w;
	pParts->vel_w_t += p.vel_w;
	pParts->ind_w_t += p.ind_w;
	pParts->acl_w_t += p.acl_w;

	if(p.pose_w > pParts->pose_w_max)
		pParts->pose_w_max = p.pose_w;
	if(p.dir_w > pParts->dir_w_max)
		pParts->dir_w_max = p.dir_w;
	if(p.vel_w > pParts->vel_w_max)
		pParts->vel_w_max = p.vel_w;
	if(p.ind_w > pParts->ind_w_max)
		pParts->ind_w_max = p.ind_w;
	if(p.acl_w > pParts->acl_w_max)
		pParts->acl_w_max = p.acl_w;

	if(p.pose_w < pParts->pose_w_min)
		pParts->pose_w_min = p.pose_w;
	if(p.dir_w < pParts->dir_w_min)
		pParts->dir_w_min = p.dir_w;
	if(p.vel_w < pParts->vel_w_min)
		pParts->vel_w_min = p.vel_w;
	if(p.ind_w < pParts->ind_w_min)
		pParts->ind_w_min = p.ind_w;
	if(p.acl_w < pParts->acl_w_min)
		pParts->acl_w_min = p.acl_w;

	p.w_raw = p.pose_w*POSE_FACTOR + p.dir_w*DIRECTION_FACTOR + p.vel_w*VELOCITY_FACTOR + p.ind_w*INDICATOR_FACTOR + p.acl_w*ACCELERATE_FACTOR;

	if(p.w_raw >= pParts->max_w_raw)
		pParts->max_w_raw = p.w_raw;

	if(p.w_raw <= pParts->min_w_raw)
		pParts->min_w_raw = p.w_raw;
}

void BehaviorPrediction::NormalizeOnePartWeight(ObjParticles* pParts,Particle& p)
{
	if(p.bDeleted) return;

	double pose_diff  = pParts->pose_w_max-pParts->pose_w_min;
	double dir_diff = pParts->dir_w_max-pParts->dir_w_min;
	double vel_diff = pParts->vel_w_max-pParts->vel_w_min;
	double ind_diff = pParts->ind_w_max-pParts->ind_w_min;
	double acl_diff = pParts->acl_w_max-pParts->acl_w_min;

	double epsilon = 0.05;

	if(fabs(pose_diff) > epsilon)
		p.pose_w = p.pose_w/pose_diff;
	else
		p.pose_w = 0;

	if(p.pose_w > 1.0 ) p.pose_w = 1.0;

	if(fabs(dir_diff) > epsilon)
		p.dir_w = (p.dir_w - pParts->dir_w_min)/dir_diff;
	else
		p.dir_w = 0;

	if(p.dir_w > 1.0 ) p.dir_w = 1.0;

	if(fabs(vel_diff) > epsilon)
		p.vel_w = (p.vel_w-pParts->vel_w_min)/vel_diff;
	else
		p.vel_w = 0;

	if(p.vel_w > 1.0) p.vel_w = 1.0;

	if(fabs(ind_diff) > epsilon)
		p.ind_w = (p.ind_w - pParts->ind_w_min)/ind_diff;
	else
		p.ind_w = 0;

	if(p.ind_w > 1.0) p.ind_w = 1.0;

	if(fabs(acl_diff) > epsilon)
		p.acl_w = (p.acl_w - pParts->acl_w_min)/acl_diff;
	else
		p.acl_w = 0;

	if(p.acl_w > 1.0) p.acl_w = 1.0;

	p.w = p.pose_w*POSE_FACTOR + p.dir_w*DIRECTION_FACTOR + p.vel_w*VELOCITY_FACTOR + p.ind_w*INDICATOR_FACTOR + p.acl_w*ACCELERATE_FACTOR;
	//p.w = p.pose_w*0.1 + p.vel_w*0.2 + p.dir_w * 0.2 + p.ind_w + 0.5;

	if(p.w >= pParts->max_w)
		pParts->max_w = p.w;

	if(p.w <= pParts->min_w)
		pParts->min_w = p.w;

	  pParts->all_w += p.w;
}

void BehaviorPrediction::CalculateWeights(ObjParticles* pParts)
{
	pParts->all_w = 0;
	pParts->pose_w_t = 0;
	pParts->dir_w_t = 0;
	pParts->vel_w_t = 0;
	pParts->acl_w_t = 0;

	pParts->pose_w_max = -99999999;
	pParts->dir_w_max = -99999999;
	pParts->vel_w_max = -99999999;
	pParts->acl_w_max = -99999999;

	pParts->pose_w_min = 99999999;
	pParts->dir_w_min = 99999999;
	pParts->vel_w_min = 99999999;
	pParts->acl_w_min = 99999999;

	pParts->max_w_raw = DBL_MIN;
	pParts->min_w_raw = DBL_MAX;

	//std::cout << "Acceleration Diff: ";
	for(unsigned int i = 0 ; i < pParts->m_AllParticles.size(); i++)
	{
	  //  std::cout << "(" << pParts->m_AllParticles.at(i)->pTraj->index <<",";
	    CalOnePartWeight(pParts, *pParts->m_AllParticles.at(i));
	    //std::cout << ")";
	}

	//std::cout << "Befor Normalize: Max: " <<  pParts->acl_w_max << ", Min: " << pParts->acl_w_min << std::endl;
	//std::cout << std::endl;

	//if((pParts->m_TrajectoryTracker.size() > 1 && pParts->min_w_raw < 0.5) || pParts->max_w_raw == 0 || fabs(pParts->max_w_raw - pParts->min_w_raw) < 0.1 )
	if((pParts->max_w_raw == 0 || fabs(pParts->max_w_raw - pParts->min_w_raw) < 0.1 || pParts->min_w_raw > 0.5) && pParts->m_TrajectoryTracker.size() > 1)
		m_bCanDecide = false;
	else
		m_bCanDecide = true;

	//Normalize
	pParts->max_w = -9999999;
	pParts->min_w = 9999999;
	pParts->all_w = 0;

	for(unsigned int i = 0 ; i < pParts->m_AllParticles.size(); i++)
	{
		NormalizeOnePartWeight(pParts, *pParts->m_AllParticles.at(i));
	}
}

void BehaviorPrediction::CalculateAveragesAndProbabilities(ObjParticles* pParts)
{
	for(unsigned int t=0; t < pParts->m_TrajectoryTracker.size(); t++)
	{
		pParts->m_TrajectoryTracker.at(t)->CalcAverages();
		pParts->m_TrajectoryTracker.at(t)->CalcProbabilities();
	}
}

void BehaviorPrediction::RemoveWeakParticles(ObjParticles* pParts)
{

	double critical_val = pParts->min_w + (pParts->max_w - pParts->min_w)*KEEP_PERCENTAGE;
//	std::cout << "Behavior Weights ------------------------------------------------ " << std::endl;
//	std::cout << "Max Raw: " << pParts->max_w_raw << ", Min Raw: " << pParts->min_w_raw << std::endl;
//	std::cout << "Keep Percent Value: " << critical_val <<  std::endl;
//
//	if(pParts->obj.acceleration > 0 )
//		std::cout << "Accelerate vrooom vroom : " << std::endl;
//	else if(pParts->obj.acceleration  < 0 )
//		std::cout << "Brake Eeeeee Eeeeeeee: " << std::endl;

	for(int i =0 ; i < pParts->m_AllParticles.size(); i++)
	{
		//also delete far particle
		double d = hypot(pParts->obj.center.pos.y - pParts->m_AllParticles.at(i)->pose.pos.y, pParts->obj.center.pos.x - pParts->m_AllParticles.at(i)->pose.pos.x);

		if(pParts->m_AllParticles.at(i)->w < critical_val || d > m_PredictionDistance)
		{
			pParts->m_AllParticles.at(i)->pTraj->DeleteParticle(*(pParts->m_AllParticles.at(i)), pParts->m_AllParticles.at(i)->original_index);
			pParts->m_AllParticles.erase(pParts->m_AllParticles.begin()+i);
			i--;
		}
	}
}

void BehaviorPrediction::FindBest(ObjParticles* pParts)
{
	if(pParts->m_TrajectoryTracker.size() > 0)
	{
		pParts->best_beh_track = pParts->m_TrajectoryTracker.at(0);
		pParts->i_best_track = 0;
	}

	for(unsigned int t = 1; t < pParts->m_TrajectoryTracker.size(); t++)
	{
		if(pParts->m_TrajectoryTracker.at(t)->best_p > pParts->best_beh_track->best_p)
		{
			pParts->best_beh_track = pParts->m_TrajectoryTracker.at(t);
			pParts->i_best_track = t;
		}
	}


	if(m_bDebugOut )
	{
		std::cout << "Behavior Prob ------------------------------------------------ : " << pParts->m_TrajectoryTracker.size() << std::endl;
		std::cout << "Raw  Weights: Max: " <<  pParts->max_w_raw << ", Min: " << pParts->min_w_raw << std::endl;
		std::cout << "Norm Weights: Max: " <<  pParts->max_w << ", Min: " << pParts->min_w << std::endl;
	}

	for(unsigned int t=0; t < pParts->obj.predTrajectories.size() ; t++)
	{
		if(pParts->obj.predTrajectories.at(t).size()>0)
		{
			pParts->obj.predTrajectories.at(t).at(0).collisionCost = 0.25;
		}
	}

	if(m_bCanDecide && pParts->best_beh_track != nullptr)
	{
		std::string str_beh = "Unknown";
		if(pParts->best_beh_track->best_beh == BEH_STOPPING_STATE)
			str_beh = "Stopping";
		else if(pParts->best_beh_track->best_beh == BEH_FORWARD_STATE)
			str_beh = "Forward";

		if(m_bDebugOut )
			std::cout << "Trajectory (" << pParts->i_best_track << "), P: " << pParts->best_beh_track->best_p << " , Beh (" << pParts->best_beh_track->best_beh << ", " << str_beh << ")" << std::endl;

		if(pParts->best_beh_track->index < pParts->obj.predTrajectories.size())
		  pParts->obj.predTrajectories.at(pParts->best_beh_track->index).at(0).collisionCost = 1;

	}
	else
	{
		if(m_bDebugOut )
			std::cout << "Trajectory (" << -1 << "), P: " << 0 << " , Beh (" << -1 << ", " << "Can't Decide" << ")" << std::endl;
	}

	if(m_bDebugOut )
	{
		for(unsigned int t=0; t < pParts->m_TrajectoryTracker.size() ; t++)
		{
			if(pParts->m_TrajectoryTracker.at(t)->nAliveForward > 0)
				std::cout << t << ", Forward Particles:" << pParts->m_TrajectoryTracker.at(t)->nAliveForward << std::endl;
			if(pParts->m_TrajectoryTracker.at(t)->nAliveStop > 0)
				std::cout << t << ", Stoping Particles:" << pParts->m_TrajectoryTracker.at(t)->nAliveStop << std::endl;
		}

		std::cout << "------------------------------------------------ --------------" << std::endl<< std::endl;
	}
}

void BehaviorPrediction::SamplesFreshParticles(ObjParticles* pParts)
{
	timespec _time;
	UtilityHNS::UtilityH::GetTickCount(_time);
	srand(_time.tv_nsec);

	ENG eng(_time.tv_nsec);
	NormalDIST dist_x(0, MOTION_POSE_ERROR);
	VariatGEN gen_x(eng, dist_x);
	NormalDIST vel(MOTION_VEL_ERROR, MOTION_VEL_ERROR);
	VariatGEN gen_v(eng, vel);
	NormalDIST ang(0, MOTION_ANGLE_ERROR);
	VariatGEN gen_a(eng, ang);
//	NormalDIST acl(0, MEASURE_ACL_ERROR);
//	VariatGEN gen_acl(eng, acl);

	Particle p;
	p.pose = pParts->obj.center;
	p.vel = 0;
	p.acc = 0;
	p.indicator = 0;
	bool bRegenerate = true;

	if(UtilityHNS::UtilityH::GetTimeDiffNow(m_GenerationTimer) > 2)
	{
		UtilityHNS::UtilityH::GetTickCount(m_GenerationTimer);
		bRegenerate = true;
	}

//	for(unsigned int t=0; t < pParts->m_TrajectoryTracker.size(); t++)
//	{
//		PlanningHelpers::FixPathDensity(pParts->m_TrajectoryTracker.at(t)->trajectory, 0.5);
//		PlanningHelpers::CalcAngleAndCost(pParts->m_TrajectoryTracker.at(t)->trajectory);
//	}

	for(unsigned int t=0; t < pParts->m_TrajectoryTracker.size(); t++)
	{
		RelativeInfo info;
		PlanningHelpers::GetRelativeInfo(pParts->m_TrajectoryTracker.at(t)->trajectory, pParts->obj.center, info);
		unsigned int point_index = 0;
		p.pose = PlanningHelpers::GetFollowPointOnTrajectory(pParts->m_TrajectoryTracker.at(t)->trajectory, info, PREDICTION_DISTANCE_PERCENTAGE*m_PredictionDistance, point_index);

		if(pParts->m_TrajectoryTracker.at(t)->beh == PlannerHNS::BEH_FORWARD_STATE && pParts->m_TrajectoryTracker.at(t)->nAliveForward < BEH_PARTICLES_NUM)
		{
			p.beh = PlannerHNS::BEH_FORWARD_STATE;
			int nPs = BEH_PARTICLES_NUM - pParts->m_TrajectoryTracker.at(t)->nAliveForward;

			for(unsigned int i=0; i < nPs; i++)
			{
				Particle p_new = p;
				p_new.pose.pos.x += gen_x();
				p_new.pose.pos.y += gen_x();
				p_new.pose.pos.a += gen_a();
				p_new.vel = pParts->obj.center.v + fabs(gen_v());
				p_new.pose.v = p_new.vel;
				pParts->m_TrajectoryTracker.at(t)->InsertNewParticle(p_new);
			}
		}

		if(ENABLE_STOP_BEHAVIOR_GEN == 1 && pParts->m_TrajectoryTracker.at(t)->nAliveStop < 	BEH_PARTICLES_NUM)
		{
			p.beh = PlannerHNS::BEH_STOPPING_STATE;
			int nPs = BEH_PARTICLES_NUM - pParts->m_TrajectoryTracker.at(t)->nAliveStop;

			for(unsigned int i=0; i < nPs; i++)
			{
				Particle p_new = p;
				p_new.pose.pos.x += gen_x();
				p_new.pose.pos.y += gen_x();
				p_new.pose.pos.a += gen_a();
				p_new.vel = 0;
				p_new.pose.v = pParts->obj.center.v + fabs(gen_v());
				pParts->m_TrajectoryTracker.at(t)->InsertNewParticle(p_new);
			}
		}
	}
}

void BehaviorPrediction::CollectParticles(ObjParticles* pParts)
{
	pParts->m_AllParticles.clear();
	for(unsigned int t=0; t < pParts->m_TrajectoryTracker.size(); t++)
	{
		for(unsigned int i=0; i < pParts->m_TrajectoryTracker.at(t)->m_ForwardPart.size(); i++)
		{
			pParts->m_TrajectoryTracker.at(t)->m_ForwardPart.at(i).original_index = i;
			pParts->m_AllParticles.push_back(&pParts->m_TrajectoryTracker.at(t)->m_ForwardPart.at(i));
		}

		for(unsigned int i=0; i < pParts->m_TrajectoryTracker.at(t)->m_StopPart.size(); i++)
		{
			pParts->m_TrajectoryTracker.at(t)->m_StopPart.at(i).original_index = i;
			pParts->m_AllParticles.push_back(&pParts->m_TrajectoryTracker.at(t)->m_StopPart.at(i));
		}
	}
}

void BehaviorPrediction::MoveParticles(ObjParticles* pParts)
{
	double dt = 0.01;
	if(m_bStepByStep)
	{
		dt = 0.08;
	}
	else
	{
		dt = UtilityHNS::UtilityH::GetTimeDiffNow(m_ResamplingTimer);
		UtilityHNS::UtilityH::GetTickCount(m_ResamplingTimer);
		if(m_bFirstMove)
		{
			m_bFirstMove  = false;
			return;
		}
	}

	PlannerHNS::BehaviorState curr_behavior;
	PlannerHNS::ParticleInfo curr_part_info;
	PlannerHNS::VehicleState control_u;
	PassiveDecisionMaker decision_make;
	PlannerHNS::CAR_BASIC_INFO carInfo;
	carInfo.width = pParts->obj.w;
	carInfo.length = pParts->obj.l;
	carInfo.max_acceleration = 3.0;
	carInfo.max_deceleration = -3;
	carInfo.max_speed_forward = MAX_PREDICTION_SPEED;
	carInfo.min_speed_forward = 0;
	carInfo.max_steer_angle = 0.4;
	carInfo.min_steer_angle = -0.4;
	carInfo.turning_radius = 7.2;
	carInfo.wheel_base = carInfo.length*0.75;

	for(unsigned int t=0; t < pParts->m_TrajectoryTracker.size(); t++)
	{
		PlanningHelpers::GenerateRecommendedSpeed(pParts->m_TrajectoryTracker.at(t)->trajectory, carInfo.max_speed_forward, 1.0);
	}

//	std::cout << "Motion Status------ " << std::endl;
//	if(pParts->obj.acceleration_desc == 1)
//		std::cout << "Acceleration: " << pParts->obj.acceleration_raw << ", Accelerate: " << pParts->obj.acceleration_desc << std::endl;
//	else if(pParts->obj.acceleration_desc == -1)
//		std::cout << "Acceleration: " << pParts->obj.acceleration_raw << ", Braking   : " << pParts->obj.acceleration_desc << std::endl;
//	else
//		std::cout << "Acceleration: " << pParts->obj.acceleration_raw << ", Cruising  : " << pParts->obj.acceleration_desc << std::endl;

	for(unsigned int i=0; i < pParts->m_AllParticles.size(); i++)
	{
		Particle* p = pParts->m_AllParticles.at(i);

		if(USE_OPEN_PLANNER_MOVE == 0)
		  {
			p->pose.v = pParts->obj.center.v;
			curr_part_info = decision_make.MoveStepSimple(dt, p->pose, p->pTraj->trajectory,carInfo);
			if(p->prev_time_diff > ACCELERATION_CALC_TIME)
			{
				p->acc_raw = (curr_part_info.vel - p->vel_prev_big)/p->prev_time_diff;
				p->vel_prev_big = curr_part_info.vel;
				p->prev_time_diff = 0;
			}
			else
			{
				p->prev_time_diff += dt;
			}

			if(fabs(p->acc_raw) < ACCELERATION_DECISION_VALUE)
				p->acc = 0;
			else if(p->acc_raw > ACCELERATION_DECISION_VALUE)
				p->acc = 1;
			else if(p->acc_raw < -ACCELERATION_DECISION_VALUE)
				p->acc = -1;

			p->indicator = FromIndicatorToNumber(curr_part_info.indicator);

//			if(curr_behavior.state == PlannerHNS::STOPPING_STATE && p->beh == PlannerHNS::BEH_YIELDING_STATE)
//				p->vel += 1;
//			else if(p->beh == PlannerHNS::BEH_YIELDING_STATE)
//				p->vel = p->vel/2.0;
//			else if(curr_behavior.state != PlannerHNS::STOPPING_STATE && p->beh == PlannerHNS::BEH_STOPPING_STATE)
//				p->vel += 1;
//			else if(p->beh == PlannerHNS::BEH_STOPPING_STATE)
//				p->vel = 0;

//			if(curr_behavior.state != PlannerHNS::STOPPING_STATE && p->beh == PlannerHNS::BEH_STOPPING_STATE)
//				p->vel += 1;
			if(p->beh == PlannerHNS::BEH_STOPPING_STATE)
			{
				p->vel = 0;
				if(p->acc == 0)
					p->acc = -1;
				else if(p->acc == 1)
					p->acc = 0;
			}

		  }
		else
		  {
			curr_behavior = decision_make.MoveStep(dt, p->pose, p->pTraj->trajectory,carInfo);
			p->acc = UtilityHNS::UtilityH::GetSign(curr_behavior.maxVelocity - p->vel_prev_big);
			p->vel = curr_behavior.maxVelocity;
			if(fabs(p->vel - p->vel_prev_big) > 0.5)
				p->vel_prev_big = p->vel;
			p->indicator = FromIndicatorToNumber(curr_behavior.indicator);
//			if(p->indicator == 0)
//			  std::cout << ", Off";
//			else if(p->indicator == 1)
//			  std::cout << ", Left";
//			else if(p->indicator == 2)
//			  std::cout << ", Right";

			if(curr_behavior.state == PlannerHNS::STOPPING_STATE && p->beh == PlannerHNS::BEH_YIELDING_STATE)
				p->vel += 1;
			else if(p->beh == PlannerHNS::BEH_YIELDING_STATE)
				p->vel = p->vel/2.0;
			else if(curr_behavior.state != PlannerHNS::STOPPING_STATE && p->beh == PlannerHNS::BEH_STOPPING_STATE)
				p->vel += 1;
			else if(p->beh == PlannerHNS::BEH_STOPPING_STATE)
			{
				p->vel = 0;
			}

		  }
	}
	//std::cout << "End Motion Status ------ " << std::endl;
}

} /* namespace PlannerHNS */

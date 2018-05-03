
/// \file  BehaviorPrediction.h
/// \brief Predict detected vehicles's possible trajectories, these trajectories extracted from the vector map.
/// \author Hatem Darweesh
/// \date Jul 6, 2017


#ifndef BEHAVIORPREDICTION_H_
#define BEHAVIORPREDICTION_H_

#include "PlannerH.h"
#include "UtilityH.h"

namespace PlannerHNS
{

class BehaviorPrediction
{
public:
	BehaviorPrediction();
	virtual ~BehaviorPrediction();
	void DoOneStep(const std::vector<DetectedObject>& obj_list, const WayPoint& currPose, const double& minSpeed, const double& maxDeceleration, RoadNetwork& map);

public:
	double m_MaxLaneDetectionDistance;
	double m_PredictionDistance;
	bool m_bGenerateBranches;
	bool m_bUseFixedPrediction;
	std::vector<DetectedObject> m_PredictedObjects;
	std::vector<DetectedObject> m_temp_list;

protected:

	void FilterObservations(const std::vector<DetectedObject>& obj_list, RoadNetwork& map, std::vector<DetectedObject>& filtered_list);
	void ExtractTrajectoriesFromMap(const std::vector<DetectedObject>& obj_list, RoadNetwork& map, std::vector<DetectedObject>& old_list);

};



} /* namespace PlannerHNS */

#endif /* BEHAVIORPREDICTION_H_ */

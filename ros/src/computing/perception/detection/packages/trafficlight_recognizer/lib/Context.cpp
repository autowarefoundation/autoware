#include "Context.h"

Context::Context(cv::Point aRedCenter, cv::Point aYellowCenter, cv::Point aGreenCenter,
                 int aLampRadius, cv::Point aTopLeft, cv::Point aBotRight)
{
	redCenter = aRedCenter;
	yellowCenter = aYellowCenter;
	greenCenter = aGreenCenter;
	lampRadius = aLampRadius;
	topLeft = aTopLeft;
	botRight = aBotRight;
	leftTurnSignal = false;
	rightTurnSignal = false;
	closestLaneId = -1;
	newCandidateLightState = UNDEFINED;
	lightState = UNDEFINED;
}

/*
  define magnitude relationship of context
 */
bool Context::CompareContext(const Context in_context_a, const Context in_context_b)
{
	/* if lampRadius is bigger, context is smaller */
	return in_context_a.lampRadius >= in_context_b.lampRadius;
} /* static bool compareContext() */


void Context::SetContexts(std::vector<Context> &out_signal_contexts,
                          const autoware_msgs::Signals::ConstPtr &in_lamp_signals_positions,
                          const int in_image_height,
                          const int in_image_width)
{
	/* copy parts of data to local variable */
	std::vector<autoware_msgs::ExtractedPosition> signals_lamps;
	std::vector<autoware_msgs::ExtractedPosition>::iterator lamp_iterator;

	std::vector<int> lane_id_vector;

	for (unsigned int i = 0; i < in_lamp_signals_positions->Signals.size(); i++)
	{
		autoware_msgs::ExtractedPosition tmp_lamp_position;
		tmp_lamp_position.signalId = in_lamp_signals_positions->Signals.at(i).signalId;
		tmp_lamp_position.u = in_lamp_signals_positions->Signals.at(i).u;
		tmp_lamp_position.v = in_lamp_signals_positions->Signals.at(i).v;
		tmp_lamp_position.radius = in_lamp_signals_positions->Signals.at(i).radius;
		tmp_lamp_position.x = in_lamp_signals_positions->Signals.at(i).x;
		tmp_lamp_position.y = in_lamp_signals_positions->Signals.at(i).y;
		tmp_lamp_position.z = in_lamp_signals_positions->Signals.at(i).z;
		tmp_lamp_position.type = in_lamp_signals_positions->Signals.at(i).type;
		tmp_lamp_position.linkId = in_lamp_signals_positions->Signals.at(i).linkId;
		tmp_lamp_position.plId = in_lamp_signals_positions->Signals.at(i).plId;
		signals_lamps.push_back(tmp_lamp_position);

		lane_id_vector.push_back(tmp_lamp_position.linkId); //store lanes ids, to later identify signals contexts
	}

	//get unique lane ids
	std::sort(lane_id_vector.begin(), lane_id_vector.end());
	std::vector<int>::iterator new_end = std::unique(lane_id_vector.begin(), lane_id_vector.end());
	lane_id_vector.erase(new_end, lane_id_vector.end());

	std::vector<Context> final_signal_contexts;

	//one traffic signal per lane, check each lane and find the bulb belonging to this lane (this signal Context)
	for (unsigned int ctx_idx = 0; ctx_idx < lane_id_vector.size(); ctx_idx++)
	{
		Context current_signal_context;
		int min_radius = INT_MAX;
		int most_left = in_image_width;
		int most_top = in_image_height;
		int most_right = 0;
		int most_bottom = 0;
		current_signal_context.leftTurnSignal = false;
		current_signal_context.rightTurnSignal = false;
		current_signal_context.closestLaneId = -1;
		//check which lamps belong to this lane
		for (lamp_iterator = signals_lamps.begin(); lamp_iterator < signals_lamps.end(); lamp_iterator++)
		{
			int img_x = lamp_iterator->u;
			int img_y = lamp_iterator->v;
			double map_x = lamp_iterator->x;
			double map_y = lamp_iterator->y;
			double map_z = lamp_iterator->z;
			int radius = lamp_iterator->radius;
			if (lamp_iterator->linkId == lane_id_vector.at(ctx_idx) &&
			    0 < img_x - radius - 1.5 * radius && img_x + radius + 1.5 * radius < in_image_width &&
			    0 < img_y - radius - 1.5 * radius && img_y + radius + 1.5 * radius < in_image_height)
			{
				switch (lamp_iterator->type)
				{
					case 1:             /* RED */
						current_signal_context.redCenter = cv::Point(img_x, img_y);
						current_signal_context.redCenter3d = cv::Point3d(map_x, map_y, map_z);
						break;
					case 2:             /* GREEN */
						current_signal_context.greenCenter = cv::Point(img_x, img_y);
						current_signal_context.greenCenter3d = cv::Point3d(map_x, map_y, map_z);
						break;
					case 3:             /* YELLOW */
						current_signal_context.yellowCenter = cv::Point(img_x, img_y);
						current_signal_context.yellowCenter3d = cv::Point3d(map_x, map_y, map_z);
						// use yellow light bulb signalID as this context's representative
						current_signal_context.signalID = lamp_iterator->signalId;
						current_signal_context.closestLaneId = lamp_iterator->linkId;
						break;
					case 21:            /*RED LEFT*/
						current_signal_context.redCenter = cv::Point(img_x, img_y);
						current_signal_context.redCenter3d = cv::Point3d(map_x, map_y, map_z);
						current_signal_context.leftTurnSignal = true;
						break;
					case 22:            /*GREEN LEFT*/
						current_signal_context.greenCenter = cv::Point(img_x, img_y);
						current_signal_context.greenCenter3d = cv::Point3d(map_x, map_y, map_z);
						current_signal_context.leftTurnSignal = true;
						break;
					case 23:            /*YELLOW LEFT*/
						current_signal_context.yellowCenter = cv::Point(img_x, img_y);
						current_signal_context.yellowCenter3d = cv::Point3d(map_x, map_y, map_z);
						current_signal_context.leftTurnSignal = true;
						// use yellow light bulb signalID as this context's representative
						current_signal_context.signalID = lamp_iterator->signalId;
						current_signal_context.closestLaneId = lamp_iterator->linkId;
						break;
					default:          /* this signal is not for cars (for pedestrian or something) */
						continue;
				}
				min_radius = (min_radius > radius) ? radius : min_radius;
				most_left = (most_left > img_x - radius - 1.5 * min_radius) ? img_x - radius - 1.5 * min_radius
				                                                            : most_left;
				most_top = (most_top > img_y - radius - 1.5 * min_radius) ? img_y - radius - 1.5 * min_radius
				                                                          : most_top;
				most_right = (most_right < img_x + radius + 1.5 * min_radius) ? img_x + radius + 1.5 * min_radius
				                                                              : most_right;
				most_bottom = (most_bottom < img_y + radius + 1.5 * min_radius) ? img_y + radius + 1.5 * min_radius
				                                                                : most_bottom;
			}//end if check this lamp belong to this lane and visible
		}//end for to check if the lamp belongs to the lane

		current_signal_context.lampRadius = min_radius;
		current_signal_context.topLeft = cv::Point(most_left, most_top);
		current_signal_context.botRight = cv::Point(most_right, most_bottom);
		current_signal_context.lightState = UNDEFINED;
		current_signal_context.stateJudgeCount = 0;

		/* search whether this signal has already belonged in detector.out_signal_contexts */
		bool isInserted = false;
		std::vector<int> eraseCandidate;
		for (unsigned int i = 0; i < out_signal_contexts.size(); i++)
		{
			if (current_signal_context.signalID == out_signal_contexts.at(i).signalID &&
				current_signal_context.lampRadius != INT_MAX)
			{
				/* update to new information except to lightState */
				final_signal_contexts.push_back(current_signal_context);
				final_signal_contexts.back().lightState = out_signal_contexts.at(i).lightState;
				final_signal_contexts.back().stateJudgeCount = out_signal_contexts.at(i).stateJudgeCount;
				final_signal_contexts.back().newCandidateLightState = out_signal_contexts.at(i).newCandidateLightState;
				isInserted = true;
				break;
			}
		}

		if (isInserted == false && current_signal_context.lampRadius != INT_MAX)
		{
			final_signal_contexts.push_back(
					current_signal_context); // this current_signal_context is new in detector.out_signal_contexts
		}

	}//end for check each lane

	/* sort by lampRadius */
	std::sort(final_signal_contexts.begin(), final_signal_contexts.end(), CompareContext);

	/* reset detector.out_signal_contexts */
	out_signal_contexts.clear();
	out_signal_contexts.resize(final_signal_contexts.size());
	for (unsigned int i = 0; i < final_signal_contexts.size(); i++)
	{
		out_signal_contexts.at(i) = final_signal_contexts.at(i);
	}

} /* std::vector<Context> Context::SetContexts() */

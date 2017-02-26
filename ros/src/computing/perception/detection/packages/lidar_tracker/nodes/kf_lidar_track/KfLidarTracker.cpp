#include "KfLidarTracker.h"

// ---------------------------------------------------------------------------
// Tracker. Manage tracks. Create, remove, update.
// ---------------------------------------------------------------------------
KfLidarTracker::KfLidarTracker(float in_time_delta,	float in_acceleration_noise_magnitude, float in_distance_threshold, size_t maximum_allowed_skipped_frames, size_t maximum_trace_length, bool pose_estimation) :
	time_delta_(in_time_delta),
	acceleration_noise_magnitude_(in_acceleration_noise_magnitude),
	distance_threshold_(in_distance_threshold),
	maximum_allowed_skipped_frames_(maximum_allowed_skipped_frames),
	maximum_trace_length_(maximum_trace_length),
	next_track_id_(0),
	pose_estimation_(false)
{
}

void KfLidarTracker::Update(const lidar_tracker::CloudClusterArray& in_cloud_cluster_array, DistType in_match_method)
{
	size_t num_detections = in_cloud_cluster_array.clusters.size();
	size_t num_tracks = tracks.size();
	std::vector<int> track_assignments(num_tracks, -1);
	std::vector<size_t> detections_assignments;

	// If no trackers, new track for each detection
	if (tracks.size() == 0)
	{
		std::cout << "New tracks" << num_detections << std::endl;
		// If no tracks yet
		for (size_t i = 0; i < num_detections; ++i)
		{
			tracks.push_back(CTrack(in_cloud_cluster_array.clusters[i],
									time_delta_,
									acceleration_noise_magnitude_,
									next_track_id_++)
							);
		}
	}

	if (num_tracks > 0 && num_detections>0)
	{
		std::cout << "Trying to match " << num_tracks << " tracks with " << num_detections << std::endl;

		//calculate distances between objects
		for (size_t i = 0; i < num_detections; i++)
		{

			float current_distance_threshold = distance_threshold_;
			for (size_t j = 0; j < num_tracks; j++)
			{
				float current_distance = tracks[j].CalculateDistance(cv::Point2f(in_cloud_cluster_array.clusters[i].centroid_point.point.x, in_cloud_cluster_array.clusters[i].centroid_point.point.y));
				if(current_distance < current_distance_threshold)
				{//assign the closest detection
					std::cout << "best match so far "<< " dist:" << current_distance << std::endl;
					current_distance_threshold = current_distance;
					track_assignments[j] = i;//assign detection i to track j
					detections_assignments.push_back(j);///////////////////////////////////////
				}
			}
		}

		//check assignmets
		for (size_t i = 0; i< num_tracks; i++)
		{
			if (track_assignments[i]>0) //if this track was assigned, update kalman filter, reset remaining life
			{
				tracks[i].skipped_frames = 0;
				tracks[i].Update(in_cloud_cluster_array.clusters[track_assignments[i]],
								true,
								maximum_trace_length_);
				//detections_assignments.push_back(track_assignments[i]);
			}
			else				     // if not matched continue using predictions, and increase life
			{
				tracks[i].Update(lidar_tracker::CloudCluster(), false, maximum_trace_length_);
				tracks[i].skipped_frames++;
			}
		}

		// If track life is long, remove it.
		for (size_t i = 0; i < tracks.size(); i++)
		{
			if (tracks[i].skipped_frames > maximum_allowed_skipped_frames_)
			{
				tracks.erase(tracks.begin() + i);
				i--;
			}
		}
	}//endof matching

	// Search for unassigned detections and start new trackers.
	for (size_t i = 0; i < num_detections; ++i)
	{
		std::vector<size_t>::iterator it = find(detections_assignments.begin(), detections_assignments.end(), i);
		if (it != detections_assignments.end())//if detection not found in the already assigned ones, add new tracker
		{
			tracks.push_back(CTrack(in_cloud_cluster_array.clusters[i],
									time_delta_,
									acceleration_noise_magnitude_,
									next_track_id_++)
							);
		}
	}

}
// ---------------------------------------------------------------------------
//
// ---------------------------------------------------------------------------
/*void KfLidarTracker::Update(
	const lidar_tracker::CloudClusterArray& in_cloud_cluster_array,
	DistType distType
	)
{
	size_t detections_num = in_cloud_cluster_array.clusters.size();
	// -----------------------------------
	// If there is no tracks yet, then every cv::Point begins its own track.
	// -----------------------------------
	if (tracks.size() == 0)
	{
		std::cout << "New track" << std::endl;
		// If no tracks yet
		for (size_t i = 0; i < detections_num; ++i)
		{
			tracks.push_back(CTrack(in_cloud_cluster_array.clusters[i],
									time_delta_,
									acceleration_noise_magnitude_,
									next_track_id_++)
							);
		}
	}

	size_t N = tracks.size();
	size_t M = detections_num;

	std::vector<int> assignment;

	if (!tracks.empty())
	{
		std::cout << "Try to match" << std::endl;
		std::vector<float> cost_matrix(N * M);

		switch (distType)
		{
		case CentersDist:
			for (size_t i = 0; i < tracks.size(); i++)
			{
				for (size_t j = 0; j < detections_num; j++)
				{
					cost_matrix[i + j * N] = tracks[i].CalculateDistance(cv::Point2f(in_cloud_cluster_array.clusters[j].centroid_point.point.x, in_cloud_cluster_array.clusters[j].centroid_point.point.y));
				}
			}
			break;

		case RectsDist:
			for (size_t i = 0; i < tracks.size(); i++)
			{
				for (size_t j = 0; j < detections_num; j++)
				{
					cost_matrix[i + j * N] = tracks[i].CalculateDistance( cv::Rect_<float>(in_cloud_cluster_array.clusters[i].centroid_point.point.x - in_cloud_cluster_array.clusters[i].bounding_box.dimensions.x/2,
																			in_cloud_cluster_array.clusters[i].centroid_point.point.y - in_cloud_cluster_array.clusters[i].bounding_box.dimensions.y/2,
																			in_cloud_cluster_array.clusters[i].bounding_box.dimensions.x,
																			in_cloud_cluster_array.clusters[i].bounding_box.dimensions.y
																			)
														);
				}
			}
			break;
		}

		// -----------------------------------
		// Solving assignment problem (tracks and predictions of Kalman filter)
		// -----------------------------------
		AssignmentProblemSolver APS;
		std::cout << "Hungarian Algorithm Start"<< std::endl;
		APS.Solve(cost_matrix, N, M, assignment, AssignmentProblemSolver::optimal);
		std::cout << "Hungarian Algorithm End"<< std::endl;
		// -----------------------------------
		// clean assignment from pairs with large distance
		// -----------------------------------
		for (size_t i = 0; i < assignment.size(); i++)
		{
			if (assignment[i] != -1)
			{
				if (cost_matrix[i + assignment[i] * N] > distance_threshold_)
				{
					assignment[i] = -1;
					tracks[i].skipped_frames = 1;
					std::cout << "Existing track Not matched " << i << " Distance:" << cost_matrix[i + assignment[i] * N] << std::endl;
				}
			}
			else
			{
				// If track have no assigned detect, then increment skipped frames counter.
				tracks[i].skipped_frames++;
				std::cout << "Existing track Not matched " << i <<", dying " << tracks[i].skipped_frames << std::endl;
			}
		}

		// -----------------------------------
		// If track didn't get detects long time, remove it.
		// -----------------------------------
		for (size_t i = 0; i < tracks.size(); i++)
		{
			if (tracks[i].skipped_frames > maximum_allowed_skipped_frames_)
			{
				tracks.erase(tracks.begin() + i);
				assignment.erase(assignment.begin() + i);
				std::cout << "Died " << i << std::endl;
				i--;
			}
		}
	}

	// -----------------------------------
	// Search for unassigned detects and start new tracks for them.
	// -----------------------------------
	for (size_t i = 0; i < detections_num; ++i)
	{
		if (find(assignment.begin(), assignment.end(), i) == assignment.end())
		{
			std::cout << "New object, Not matched " << i << std::endl;
			tracks.push_back(CTrack(in_cloud_cluster_array.clusters[i],
									time_delta_,
									acceleration_noise_magnitude_,
									next_track_id_++)
							);
		}
	}

	// Update Kalman Filters state
	for (size_t i = 0; i<assignment.size(); i++)
	{
		// If track updated less than one time, than filter state is not correct.

		if (assignment[i] != -1) // If we have assigned detect, then update using its coordinates,
		{
			tracks[i].skipped_frames = 0;
			std::cout << "Matched. Kalman Update on " << i << std::endl;
			tracks[i].Update(	in_cloud_cluster_array.clusters[i],
								true,
								maximum_trace_length_);
		}
		else				     // if not continue using predictions
		{
			std::cout << "Not Matched. Kalman Update on " << i << std::endl;
			tracks[i].Update(lidar_tracker::CloudCluster(), false, maximum_trace_length_);
		}
	}

}*/
// ---------------------------------------------------------------------------
//
// ---------------------------------------------------------------------------
KfLidarTracker::~KfLidarTracker(void)
{
}

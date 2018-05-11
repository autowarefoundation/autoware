#include "lidar_kf_track.h"

// ---------------------------------------------------------------------------
// Tracker. Manage tracks. Create, remove, update.
// ---------------------------------------------------------------------------
KfLidarTracker::KfLidarTracker(float in_time_delta,
		float in_acceleration_noise_magnitude,
		float in_distance_threshold,
		float in_tracker_merging_threshold,
		size_t maximum_allowed_skipped_frames,
		size_t maximum_trace_length,
		bool pose_estimation,
		size_t maximum_track_id) :
	time_delta_(in_time_delta),
	acceleration_noise_magnitude_(in_acceleration_noise_magnitude),
	distance_threshold_(in_distance_threshold),
	tracker_merging_threshold_(in_tracker_merging_threshold),
	maximum_allowed_skipped_frames_(maximum_allowed_skipped_frames),
	maximum_trace_length_(maximum_trace_length),
	next_track_id_(0),
	pose_estimation_(false),
	maximum_track_id_(maximum_track_id)
{
}

void KfLidarTracker::CreatePolygonFromPoints(const geometry_msgs::Polygon& in_points, boost_polygon& out_polygon)
{
	std::vector< boost_point_xy > hull_detection_points;

	for (size_t k=0; k < in_points.points.size()/2; k++)
	{
		hull_detection_points.push_back(
					boost_point_xy(in_points.points[k].x,
							in_points.points[k].y)
					);
	}
	boost::geometry::assign_points(out_polygon, hull_detection_points);
}

void KfLidarTracker::Update(const autoware_msgs::CloudClusterArray& in_cloud_cluster_array, DistType in_match_method)
{
	size_t num_detections = in_cloud_cluster_array.clusters.size();
	size_t num_tracks = tracks_.size();

	std::vector<size_t> detections_assignments;
	std::vector<double> detections_areas(num_detections, 0.0f);

	std::vector< CTrack > final_tracks;

	// If no trackers, new track for each detection
	if (num_tracks == 0)
	{
		//std::cout << "New tracks" << num_detections << std::endl;
		// If no tracks yet
		for (size_t i = 0; i < num_detections; ++i)
		{
			tracks_.push_back(CTrack(in_cloud_cluster_array.clusters[i],
									time_delta_,
									acceleration_noise_magnitude_,
									next_track_id_++)
							);
		}
		num_tracks = tracks_.size();
	}
	std::vector<int> track_assignments(num_tracks, -1);
	std::vector< std::vector<size_t> > track_assignments_vector(num_tracks);

	//else
	{
		//std::cout << "Trying to match " << num_tracks << " tracks with " << num_detections << std::endl;

		//calculate distances between objects
		for (size_t i = 0; i < num_detections; i++)
		{

			float current_distance_threshold = distance_threshold_;

			//detection polygon
			boost_polygon hull_detection_polygon;
			CreatePolygonFromPoints(in_cloud_cluster_array.clusters[i].convex_hull.polygon, hull_detection_polygon);
			detections_areas[i] = boost::geometry::area(hull_detection_polygon);

			for (size_t j = 0; j < num_tracks; j++)
			{
				//float current_distance = tracks_[j].CalculateDistance(cv::Point2f(in_cloud_cluster_array.clusters[i].centroid_point.point.x, in_cloud_cluster_array.clusters[i].centroid_point.point.y));
				float current_distance = sqrt(
												pow(tracks_[j].GetCluster().centroid_point.point.x - in_cloud_cluster_array.clusters[i].centroid_point.point.x, 2) +
												pow(tracks_[j].GetCluster().centroid_point.point.y - in_cloud_cluster_array.clusters[i].centroid_point.point.y, 2)
										);

				//tracker polygon
				boost_polygon hull_track_polygon;
				CreatePolygonFromPoints(tracks_[j].GetCluster().convex_hull.polygon, hull_track_polygon);

				//if(current_distance < current_distance_threshold)
				if (!boost::geometry::disjoint(hull_detection_polygon, hull_track_polygon)
					||  (current_distance < current_distance_threshold)
					)
				{//assign the closest detection or overlapping
					current_distance_threshold = current_distance;
					track_assignments[j] = i;//assign detection i to track j
					track_assignments_vector[j].push_back(i);//add current detection as a match
					detections_assignments.push_back(j);///////////////////////////////////////
				}
			}
		}

		//check assignmets
		for (size_t i = 0; i< num_tracks; i++)
		{
			if (track_assignments[i]>=0) //if this track was assigned, update kalman filter, reset remaining life
			{
				//keep oldest
				tracks_[i].skipped_frames = 0;

				//join all assigned detections to update the tracker
				/*autoware_msgs::CloudClusterPtr summed_cloud_cluster(new autoware_msgs::CloudCluster());
				pcl::PointCloud<pcl::PointXYZ>::Ptr summed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
				for (size_t j = 0; j < track_assignments_vector[i].size(); j++)
				{
					pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
					pcl::fromROSMsg(in_cloud_cluster_array.clusters[track_assignments_vector[i][j]].cloud, *current_cloud_ptr);
					*summed_cloud_ptr += *current_cloud_ptr;
				}
				ClusterPtr merged_cluster(new Cluster());
				std::vector<int> indices(summed_cloud_ptr->points.size(), 0);
				for (size_t j=0; j<summed_cloud_ptr->points.size(); j++)
				{
					indices[j]=j;
				}
				merged_cluster->SetCloud(summed_cloud_ptr, indices, in_cloud_cluster_array.clusters[track_assignments[i]].header, i,255, 255, 255, "", pose_estimation_);

				merged_cluster->ToRosMessage(in_cloud_cluster_array.clusters[track_assignments[i]].header, *summed_cloud_cluster);*/

				tracks_[i].Update(in_cloud_cluster_array.clusters[track_assignments[i]],//*summed_cloud_cluster,
								true,
								maximum_trace_length_);
				//detections_assignments.push_back(track_assignments[i]);
			}
			else				     // if not matched continue using predictions, and increase life
			{
				tracks_[i].Update(autoware_msgs::CloudCluster(), //empty cluster
									false, //not matched,
									maximum_trace_length_
								);
				tracks_[i].skipped_frames++;
			}
			tracks_[i].life_span++;
		}

		// If track life is long, remove it.
		for (size_t i = 0; i < tracks_.size(); i++)
		{
			if (tracks_[i].skipped_frames > maximum_allowed_skipped_frames_)
			{
				tracks_.erase(tracks_.begin() + i);
				i--;
			}
		}

		// Search for unassigned detections and start new trackers.
		int una = 0;
		for (size_t i = 0; i < num_detections; ++i)
		{
			std::vector<size_t>::iterator it = find(detections_assignments.begin(), detections_assignments.end(), i);
			if (it == detections_assignments.end())//if detection not found in the already assigned ones, add new tracker
			{
				tracks_.push_back(CTrack(in_cloud_cluster_array.clusters[i],
										time_delta_,
										acceleration_noise_magnitude_,
										next_track_id_++)
								);
				if (next_track_id_ > maximum_track_id_)
					next_track_id_ = 0;
				una++;
			}
		}
		//finally check trackers among them. Remove previously merged objects, causing ID duplication
		for(size_t i=0; i< tracks_.size(); i++)
		{
			for (size_t j=0; j< tracks_.size(); j++)
			{
				if (i != j &&
					 (tracks_[i].GetCluster().centroid_point.point.x == tracks_[j].GetCluster().centroid_point.point.x &&
					  tracks_[i].GetCluster().centroid_point.point.y == tracks_[j].GetCluster().centroid_point.point.y &&
					  tracks_[i].GetCluster().centroid_point.point.z == tracks_[j].GetCluster().centroid_point.point.z
					 )
					)
				{
					tracks_.erase(tracks_.begin() + j);
					j--;
				}
			}
		}
		CheckAllTrackersForMerge(final_tracks);

		tracks_ = final_tracks;

		//std::cout << "Final Trackers " << tracks_.size() << std::endl;
	}//endof matching

}

void KfLidarTracker::CheckTrackerMerge(size_t in_tracker_id, std::vector<CTrack>& in_trackers, std::vector<bool>& in_out_visited_trackers, std::vector<size_t>& out_merge_indices, double in_merge_threshold)
{
	for(size_t i=0; i< in_trackers.size(); i++)
	{
		if (i != in_tracker_id && !in_out_visited_trackers[i])
		{
			double distance =  sqrt( pow(in_trackers[in_tracker_id].GetCluster().centroid_point.point.x - in_trackers[i].GetCluster().centroid_point.point.x,2) +
										pow(in_trackers[in_tracker_id].GetCluster().centroid_point.point.y - in_trackers[i].GetCluster().centroid_point.point.y,2)
								);
			boost_polygon in_tracker_poly;
			CreatePolygonFromPoints(in_trackers[in_tracker_id].GetCluster().convex_hull.polygon, in_tracker_poly);
			in_trackers[in_tracker_id].area = boost::geometry::area(in_tracker_poly);

			boost_polygon current_tracker_poly;
			CreatePolygonFromPoints(in_trackers[i].GetCluster().convex_hull.polygon, current_tracker_poly);
			in_trackers[i].area = boost::geometry::area(current_tracker_poly);

			if (!boost::geometry::disjoint(in_tracker_poly, current_tracker_poly)
				|| distance <= in_merge_threshold)
			{
				in_out_visited_trackers[i] = true;
				out_merge_indices.push_back(i);
				CheckTrackerMerge(i, in_trackers, in_out_visited_trackers, out_merge_indices, in_merge_threshold);
			}
		}
	}
}

void KfLidarTracker::MergeTrackers(std::vector<CTrack>& in_trackers, std::vector<CTrack>& out_trackers, std::vector<size_t> in_merge_indices, const size_t& current_index, std::vector<bool>& in_out_merged_trackers)
{
	size_t oldest_life =0;
	size_t oldest_index = 0;
	double largest_area = 0.0f;
	size_t largest_index = 0;
	for (size_t i=0; i<in_merge_indices.size(); i++)
	{
		if (in_trackers[in_merge_indices[i]].life_span> oldest_life)
		{
			oldest_life = in_trackers[in_merge_indices[i]].life_span;
			oldest_index = in_merge_indices[i];
		}
		if (in_trackers[in_merge_indices[i]].area> largest_area)
		{
			largest_index = in_merge_indices[i];
		}
		in_out_merged_trackers[in_merge_indices[i]] = true;
	}
	bool found=false;
	for(size_t i=0; i< out_trackers.size(); i++){
		found = out_trackers[i].track_id == in_trackers[oldest_index].track_id;
	}
	if (!found)
	{
		out_trackers.push_back(in_trackers[oldest_index]);
		in_out_merged_trackers[oldest_index] = true;
	}
	//out_trackers.back().cluster = in_trackers[largest_index].GetCluster();
}

void KfLidarTracker::CheckAllTrackersForMerge(std::vector<CTrack>& out_trackers)
{
	//std::cout << "checkAllForMerge" << std::endl;
	std::vector<bool> visited_trackers(tracks_.size(), false);
	std::vector<bool> merged_trackers(tracks_.size(), false);
	size_t current_index=0;
	for (size_t i = 0; i< tracks_.size(); i++)
	{
		if (!visited_trackers[i])
		{
			visited_trackers[i] = true;
			std::vector<size_t> merge_indices;
			CheckTrackerMerge(i, tracks_, visited_trackers, merge_indices, tracker_merging_threshold_);
			MergeTrackers(tracks_, out_trackers, merge_indices, current_index++, merged_trackers);
		}
	}
	for(size_t i =0; i< tracks_.size(); i++)
	{
		//check for clusters not merged, add them to the output
		if (!merged_trackers[i])
		{
			out_trackers.push_back(tracks_[i]);
		}
	}
	//ClusterPtr cluster(new Cluster());
}
// ---------------------------------------------------------------------------
//
// ---------------------------------------------------------------------------
/*void KfLidarTracker::Update(
	const autoware_msgs::CloudClusterArray& in_cloud_cluster_array,
	DistType distType
	)
{
	size_t detections_num = in_cloud_cluster_array.clusters.size();
	// -----------------------------------
	// If there is no tracks yet, then every cv::Point begins its own track.
	// -----------------------------------
	if (tracks_.size() == 0)
	{
		std::cout << "New track" << std::endl;
		// If no tracks yet
		for (size_t i = 0; i < detections_num; ++i)
		{
			tracks_.push_back(CTrack(in_cloud_cluster_array.clusters[i],
									time_delta_,
									acceleration_noise_magnitude_,
									next_track_id_++)
							);
		}
	}

	size_t N = tracks_.size();
	size_t M = detections_num;

	std::vector<int> assignment;

	if (!tracks_.empty())
	{
		std::cout << "Try to match" << std::endl;
		std::vector<float> cost_matrix(N * M);

		switch (distType)
		{
		case CentersDist:
			for (size_t i = 0; i < tracks_.size(); i++)
			{
				for (size_t j = 0; j < detections_num; j++)
				{
					cost_matrix[i + j * N] = tracks_[i].CalculateDistance(cv::Point2f(in_cloud_cluster_array.clusters[j].centroid_point.point.x, in_cloud_cluster_array.clusters[j].centroid_point.point.y));
				}
			}
			break;

		case RectsDist:
			for (size_t i = 0; i < tracks_.size(); i++)
			{
				for (size_t j = 0; j < detections_num; j++)
				{
					cost_matrix[i + j * N] = tracks_[i].CalculateDistance( cv::Rect_<float>(in_cloud_cluster_array.clusters[i].centroid_point.point.x - in_cloud_cluster_array.clusters[i].bounding_box.dimensions.x/2,
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
					tracks_[i].skipped_frames = 1;
					std::cout << "Existing track Not matched " << i << " Distance:" << cost_matrix[i + assignment[i] * N] << std::endl;
				}
			}
			else
			{
				// If track have no assigned detect, then increment skipped frames counter.
				tracks_[i].skipped_frames++;
				std::cout << "Existing track Not matched " << i <<", dying " << tracks_[i].skipped_frames << std::endl;
			}
		}

		// -----------------------------------
		// If track didn't get detects long time, remove it.
		// -----------------------------------
		for (size_t i = 0; i < tracks_.size(); i++)
		{
			if (tracks_[i].skipped_frames > maximum_allowed_skipped_frames_)
			{
				tracks_.erase(tracks_.begin() + i);
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
			tracks_.push_back(CTrack(in_cloud_cluster_array.clusters[i],
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
			tracks_[i].skipped_frames = 0;
			std::cout << "Matched. Kalman Update on " << i << std::endl;
			tracks_[i].Update(	in_cloud_cluster_array.clusters[i],
								true,
								maximum_trace_length_);
		}
		else				     // if not continue using predictions
		{
			std::cout << "Not Matched. Kalman Update on " << i << std::endl;
			tracks_[i].Update(autoware_msgs::CloudCluster(), false, maximum_trace_length_);
		}
	}

}*/
// ---------------------------------------------------------------------------
//
// ---------------------------------------------------------------------------
KfLidarTracker::~KfLidarTracker(void)
{
}

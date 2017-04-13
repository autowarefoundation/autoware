#pragma once
#include "Kalman.h"
#include "HungarianAlg.h"
#include <iostream>
#include <vector>
#include <memory>
#include <array>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <lidar_tracker/CloudCluster.h>
#include <lidar_tracker/CloudClusterArray.h>
#include <tf/tf.h>

#include <boost/assert.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/algorithms/disjoint.hpp>
#include <boost/assign/std/vector.hpp>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include "Cluster.h"

// --------------------------------------------------------------------------
class CTrack
{

	cv::Point2f prediction_point_;
	TKalmanFilter kf_;
public:
	lidar_tracker::CloudCluster cluster;

	std::vector<cv::Point2f> trace;
	size_t track_id;
	size_t skipped_frames;
	size_t life_span;
	double area;

	CTrack(const lidar_tracker::CloudCluster& in_cluster, float in_time_delta, float in_acceleration_noise_magnitude, size_t in_track_id)
		: kf_(cv::Point2f(in_cluster.centroid_point.point.x, in_cluster.centroid_point.point.y), in_time_delta, in_acceleration_noise_magnitude)
	{
		track_id = in_track_id;
		skipped_frames = 0;
		prediction_point_ = cv::Point2f(in_cluster.centroid_point.point.x, in_cluster.centroid_point.point.y);
		cluster = in_cluster;
		life_span = 0;
		area = 0;
	}

	float CalculateDistance(const cv::Point2f& in_point)
	{
		cv::Point2f diff = prediction_point_ - in_point;
		return sqrt(pow(diff.x, 2) + pow(diff.y, 2));
	}

	float CalculateDistance(const cv::Rect_<float>& in_rect)
	{
		return 0.0f;
	}

	void Update(const lidar_tracker::CloudCluster& in_cluster, bool in_data_correct, size_t in_max_trace_length)
	{
		kf_.GetPrediction();
		prediction_point_ = kf_.Update(cv::Point2f(in_cluster.centroid_point.point.x, in_cluster.centroid_point.point.y), in_data_correct);

		if (in_data_correct
			)
		{
			cluster = in_cluster;
		}

		if (trace.size() > in_max_trace_length)
		{
			trace.erase(trace.begin(), trace.end() - in_max_trace_length);
		}

		trace.push_back(prediction_point_);
	}

	lidar_tracker::CloudCluster GetCluster()
	{
		return cluster;
	}

};

// --------------------------------------------------------------------------
class KfLidarTracker
{
	typedef boost::geometry::model::d2::point_xy<double> boost_point_xy;
	typedef boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double> > boost_polygon;

	float time_delta_;
	float acceleration_noise_magnitude_;
	float distance_threshold_;
	float tracker_merging_threshold_;

	size_t maximum_allowed_skipped_frames_;
	size_t maximum_trace_length_;
	size_t next_track_id_;

	bool pose_estimation_;
	void CheckTrackerMerge(size_t in_tracker_id, std::vector<CTrack>& in_trackers, std::vector<bool>& in_out_visited_trackers, std::vector<size_t>& out_merge_indices, double in_merge_threshold);
	void CheckAllTrackersForMerge(std::vector<CTrack>& out_trackers);
	void MergeTrackers(std::vector<CTrack>& in_trackers, std::vector<CTrack>& out_trackers, std::vector<size_t> in_merge_indices, const size_t& current_index, std::vector<bool>& in_out_merged_trackers);
	void CreatePolygonFromPoints(const geometry_msgs::Polygon& in_points, boost_polygon& out_polygon);
public:
	KfLidarTracker(float in_time_delta, float accel_noise_mag, float dist_thres = 3, float tracker_merging_threshold=2, size_t maximum_allowed_skipped_frames = 10, size_t max_trace_length = 10, bool in_pose_estimation = false);
	~KfLidarTracker(void);

	enum DistType
	{
		CentersDist = 0,
		RectsDist = 1
	};

	std::vector< CTrack > tracks;
	void Update(const lidar_tracker::CloudClusterArray& in_cloud_cluster_array, DistType in_disttype);


};

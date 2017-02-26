#pragma once
#include "Kalman.h"
#include "HungarianAlg.h"
#include <iostream>
#include <vector>
#include <memory>
#include <array>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <lidar_tracker/CloudCluster.h>
#include <lidar_tracker/CloudClusterArray.h>
#include <tf/tf.h>

// --------------------------------------------------------------------------
class CTrack
{

	cv::Point2f prediction_point_;
	TKalmanFilter kf_;

	lidar_tracker::CloudCluster cluster_;
public:

	std::vector<cv::Point2f> trace;
	size_t track_id;
	size_t skipped_frames;

	CTrack(const lidar_tracker::CloudCluster& in_cluster, float in_time_delta, float in_acceleration_noise_magnitude, size_t in_track_id)
		: kf_(cv::Point2f(in_cluster.centroid_point.point.x, in_cluster.centroid_point.point.y), in_time_delta, in_acceleration_noise_magnitude)
	{
		track_id = in_track_id;
		skipped_frames = 0;
		prediction_point_ = cv::Point2f(in_cluster.centroid_point.point.x, in_cluster.centroid_point.point.y);
		cluster_ = in_cluster;
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

		if (in_data_correct)
		{
			cluster_ = in_cluster;
		}

		if (trace.size() > in_max_trace_length)
		{
			trace.erase(trace.begin(), trace.end() - in_max_trace_length);
		}

		trace.push_back(prediction_point_);
	}

	lidar_tracker::CloudCluster GetCluster()
	{
		return cluster_;
	}

};

// --------------------------------------------------------------------------
class KfLidarTracker
{
	float time_delta_;
	float acceleration_noise_magnitude_;
	float distance_threshold_;

	size_t maximum_allowed_skipped_frames_;
	size_t maximum_trace_length_;
	size_t next_track_id_;

	bool pose_estimation_;
public:
	KfLidarTracker(float in_time_delta, float accel_noise_mag, float dist_thres = 60, size_t maximum_allowed_skipped_frames = 10, size_t max_trace_length = 10, bool in_pose_estimation = false);
	~KfLidarTracker(void);

	enum DistType
	{
		CentersDist = 0,
		RectsDist = 1
	};

	std::vector< CTrack > tracks;
	void Update(const lidar_tracker::CloudClusterArray& in_cloud_cluster_array, DistType in_disttype);


};

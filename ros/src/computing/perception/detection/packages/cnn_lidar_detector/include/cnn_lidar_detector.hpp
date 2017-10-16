/*
 * cnn_lidar_detector.hpp
 *
 *  Created on: Jun 20, 2017
 *      Author: ne0
 */

#ifndef CNN_LIDAR_DETECTOR_HPP_
#define CNN_LIDAR_DETECTOR_HPP_

#include <string>
#include <vector>
#include <iostream>

#include <caffe/caffe.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/geometry.h>

#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "nms.hpp"


class CnnLidarDetector
{

public:
	CnnLidarDetector(const std::string& in_network_definition_file,
			const std::string& in_pre_trained_model_file,
			bool in_use_gpu,
			unsigned int in_gpu_id,
			float in_score_threshold);

	void Detect(const cv::Mat& in_image_intensity,
	            const cv::Mat& in_image_range,
	            const cv::Mat& in_image_x,
	            const cv::Mat& in_image_y,
	            const cv::Mat& in_image_z,
	            cv::Mat& out_objectness_image,
	            jsk_recognition_msgs::BoundingBoxArray& out_boxes);

private:
	struct BoundingBoxCorners
	{
		pcl::PointXYZ top_front_left, top_front_right, top_back_left, top_back_right;
		pcl::PointXYZ bottom_front_left, bottom_front_right, bottom_back_left, bottom_back_right;
		std::vector<float> distance_candidates;
		std::vector<size_t> neighbor_indices;
		size_t original_index;
	};
	boost::shared_ptr<caffe::Net<float> > net_;
	cv::Size 	input_geometry_;
	int 		num_channels_;
	float		score_threshold_;

	typedef boost::geometry::model::d2::point_xy<double> boost_point_xy;
	typedef boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double> > boost_polygon;

	//cheap functs
	/*!
	 * Extracts 4 values (x1, y1, x2,y2) forming the base of the 2D Bounding Box
	 * @param in_row row index
	 * @param in_col column index
	 * @param in_boxes_channels image channels forming the 3D bounding box
	 * @param out_bounding_box vector containing the 4 values
	 */
	void get_box_points_from_matrices(size_t in_row,
	                                  size_t in_col,
	                                  const std::vector<cv::Mat>& in_boxes_channels,
	                                  std::vector<float>& out_bounding_box);
	template<typename PointT> float get_points_distance(const PointT& in_p1, const PointT& in_p2);

	cv::Mat resize_image(cv::Mat in_image, cv::Size in_geometry);

	//regular functs
	void WrapInputLayer(std::vector<cv::Mat>* in_out_channels);//Point mat vector to the network input layer
	void PreProcess(const cv::Mat& in_image_intensity,
	                const cv::Mat& in_image_range,
	                const cv::Mat& in_image_x,
	                const cv::Mat& in_image_y,
	                const cv::Mat& in_image_z,
	                std::vector<cv::Mat>* in_out_channels);//match input images to input layer geometry
	void GetNetworkResults(cv::Mat& out_objectness_image,
	                       jsk_recognition_msgs::BoundingBoxArray& out_boxes);//get objectness image and bounding boxes

	void BoundingBoxCornersToJskBoundingBox(const CnnLidarDetector::BoundingBoxCorners& in_box_corners,
	                                        unsigned int in_class,
	                                        std_msgs::Header& in_header,
	                                        jsk_recognition_msgs::BoundingBox& out_jsk_box);

	/*void ApplyNms(std::vector<CnnLidarDetector::BoundingBoxCorners>& in_out_box_corners,
	              size_t in_min_num_neighbors,
	              float in_min_neighbor_distance,
	              float in_min_box_distance,
	              std::vector<CnnLidarDetector::BoundingBoxCorners>& out_nms_box_corners);*/
};

#endif /* CNN_LIDAR_DETECTOR_HPP_ */

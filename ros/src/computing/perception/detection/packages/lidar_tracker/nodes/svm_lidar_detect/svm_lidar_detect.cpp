/*
 * svm_detect.cpp
 *
 *  Created on: Nov 3, 2016
 *      Author: ne0
 */


#include "ros/ros.h"
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <lidar_tracker/CloudCluster.h>
#include <lidar_tracker/CloudClusterArray.h>

#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <jsk_rviz_plugins/Pictogram.h>
#include <jsk_rviz_plugins/PictogramArray.h>

#include <tf/tf.h>

#include "libsvm/svm.h"
#include <stdio.h>
#include <string>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/contrib/contrib.hpp>

class SvmDetect
{
public:
	SvmDetect();
	~SvmDetect();
	void Run();

private:

	ros::NodeHandle node_handle_;
	ros::Subscriber cloud_clusters_sub_;
	ros::Publisher cloud_clusters_pub_;
	ros::Publisher text_pictogram_pub_;

	std::string model_file_path_;

	FILE *model_file_handle_;
	svm_model* model_ptr_;

	void CloudClustersCallback(const lidar_tracker::CloudClusterArray::Ptr& in_cloud_cluster_array_ptr);
	void ClassifyFpfhDescriptor(const std::vector<float>& in_fpfh_descriptor, double& out_label, std::vector<double>& out_scores, double& out_sum_scores);
	svm_model* LoadSvmModel(const std::string& in_model_path);

	void CloseModel();
};

void SvmDetect::CloseModel()
{
	svm_free_and_destroy_model(&model_ptr_);
	fclose(model_file_handle_);
}

SvmDetect::~SvmDetect()
{
	CloseModel();
}

SvmDetect::SvmDetect() :
		node_handle_("~")
{

}

svm_model* SvmDetect::LoadSvmModel(const std::string& in_model_path)
{

	model_file_handle_ = fopen(in_model_path.c_str(),"r");
	if(model_file_handle_ == NULL)
	{
		ROS_INFO("SvmDetect. can't open model file %s", in_model_path.c_str());
		return NULL;
	}

	svm_model* model = svm_load_model(in_model_path.c_str());

	if(svm_check_probability_model(model) == 0)
	{
		CloseModel();
		ROS_INFO("SvmDetect. Model does not support probability estimates");
		return NULL;
	}

	return model;
}

void SvmDetect::Run()
{
	ros::NodeHandle private_node_handle("~");
	std::string clusters_node_name,
			out_clusters_topic_name = "/cloud_clusters_class",
			out_pictograms_topic_name="/pictogram_clusters_class";

	private_node_handle.param<std::string>("svm_model_file_path", model_file_path_, "models/svm.model");	ROS_INFO("svm_model_file_path: %s", model_file_path_.c_str());
	private_node_handle.param<std::string>("clusters_node_name", clusters_node_name, "/cloud_clusters");	ROS_INFO("clusters_node_name: %s", clusters_node_name.c_str());

	cloud_clusters_sub_ = node_handle_.subscribe(clusters_node_name, 10, &SvmDetect::CloudClustersCallback, this);
	cloud_clusters_pub_ = node_handle_.advertise<lidar_tracker::CloudClusterArray>(out_clusters_topic_name, 10); ROS_INFO("output clusters topic: %s", out_clusters_topic_name.c_str());

	text_pictogram_pub_ = node_handle_.advertise<jsk_rviz_plugins::PictogramArray>(out_pictograms_topic_name, 10); ROS_INFO("output pictograms topic: %s", out_pictograms_topic_name.c_str());

	model_ptr_ = LoadSvmModel(model_file_path_);

	if(model_ptr_ == NULL)
	{
		ROS_INFO("SvmDetect. Cannot perform classification. Invalid model file.");
	}
	else
	{
		ROS_INFO("SvmDetect. Ready, waiting for clusters...");
	}
	ros::spin();

}

void SvmDetect::ClassifyFpfhDescriptor(const std::vector<float>& in_fpfh_descriptor, double& out_label, std::vector<double>& out_scores, double& out_sum_scores)
{
	//int svm_type =  svm_get_svm_type(model_ptr_);
	unsigned int num_class = svm_get_nr_class(model_ptr_);
	const unsigned int feature_number = 33;//FPFH descriptor histogram bin count
	cv::TickMeter timer;
	timer.reset(); timer.start();
	svm_node* features;

	features = (struct svm_node*)malloc((feature_number+1)*sizeof(struct svm_node) ); //libsvm needs an extra node to mark the end of the linked list

	for(unsigned i=0;i<feature_number;i++)
	{
		features[i].index = i+1;
		features[i].value = in_fpfh_descriptor[i];
	}
	features[feature_number].index = -1;//end of the linked list

	double *dec_values = (double *) malloc(sizeof(double) * num_class*(num_class-1)/2);
	out_label = svm_predict_probability(model_ptr_, features, dec_values);
	out_scores.clear();
	out_sum_scores = 0.0;
	std::cout << " predicted value: " << out_label << ": ";
	for(unsigned int i=0; i<num_class; i++)
	{
		out_scores.push_back(dec_values[i]);
		out_sum_scores+=dec_values[i];
		std::cout << (i+1) << " score " << dec_values[i] << " | ";
	}
	free(dec_values);
	free(features);
	timer.stop(); std::cout << timer.getTimeMicro() << "us " ;
}

void SvmDetect::CloudClustersCallback(const lidar_tracker::CloudClusterArray::Ptr& in_cloud_cluster_array_ptr)
{
	if(model_ptr_ == NULL)
	{
		ROS_INFO("SvmDetect. Cannot perform classification. Invalid model file. Passing-through the clustered data.");
		cloud_clusters_pub_.publish(*in_cloud_cluster_array_ptr);
	}
	else
	{
		lidar_tracker::CloudClusterArray classified_clusters;
		jsk_rviz_plugins::PictogramArray pictograms_clusters;
		classified_clusters.header = in_cloud_cluster_array_ptr->header;
		pictograms_clusters.header = in_cloud_cluster_array_ptr->header;
		for(unsigned int i=0; i<in_cloud_cluster_array_ptr->clusters.size(); i++)
		{
			lidar_tracker::CloudCluster current_cluster = in_cloud_cluster_array_ptr->clusters[i];
			double svm_label_float = -1.0, sum_scores=0.0;
			std::vector<double> label_scores;
			std::vector<float> descriptor =current_cluster.fpfh_descriptor.data;
			std::string svm_class_label_str;
			ClassifyFpfhDescriptor(descriptor, svm_label_float, label_scores, sum_scores);
			int svm_label_int = (int) svm_label_float;
			switch(svm_label_int)
			{
				case 0:
					svm_class_label_str = "o ";
					break;
				case 1:
					svm_class_label_str = "c ";
					break;
				case 2:
					svm_class_label_str = "b ";
					break;
				case 3:
					svm_class_label_str = "p ";
					break;
				default:
					svm_class_label_str = "u ";
					break;
			}
			float class_score = label_scores[svm_label_int-1]/sum_scores;
			current_cluster.label = svm_class_label_str;
			current_cluster.score = label_scores[svm_label_int-1];

			std::cout << "Object detected as:" << svm_label_float << " str " << svm_class_label_str << " score: " << class_score << std::endl;
			classified_clusters.clusters.push_back(current_cluster);
			jsk_rviz_plugins::Pictogram pictogram_cluster;
			pictogram_cluster.header = classified_clusters.header;

			pictogram_cluster.mode = pictogram_cluster.STRING_MODE;
			pictogram_cluster.pose.position.x = current_cluster.max_point.point.x;
			pictogram_cluster.pose.position.y = current_cluster.max_point.point.y;
			pictogram_cluster.pose.position.z = current_cluster.max_point.point.z;
			tf::Quaternion quat(0.0, -0.7, 0.0, 0.7);
			tf::quaternionTFToMsg(quat, pictogram_cluster.pose.orientation);
			pictogram_cluster.size = 4;
			std_msgs::ColorRGBA color;
			color.a = 1; color.r = 1; color.g = 1; color.b = 1;
			pictogram_cluster.color = color;

			std::stringstream float_str;
			float_str << std::fixed << std::setprecision(2) << class_score;

			//if (label_scores[svm_label_int-1]>0.9)
				pictogram_cluster.character = svm_class_label_str + float_str.str();
			//else
			//	pictogram_cluster.character = "";


			pictograms_clusters.pictograms.push_back(pictogram_cluster);
		}
		cloud_clusters_pub_.publish(classified_clusters);
		text_pictogram_pub_.publish(pictograms_clusters);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "svm_lidar_detect");

	SvmDetect node;

	node.Run();

	return 0;
}

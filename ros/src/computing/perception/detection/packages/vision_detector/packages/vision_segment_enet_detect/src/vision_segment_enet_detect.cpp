/*
 * image_segmenter_enet.cpp
 *
 *  Created on: Aug 23, 2017
 *      Author: amc
 */

#include "vision_segment_enet_detect.h"

ENetSegmenter::ENetSegmenter(const std::string& in_model_file,
		const std::string& in_trained_file,
		const std::string& in_lookuptable_file)
{

	caffe::Caffe::set_mode(caffe::Caffe::GPU);

	/* Load the network. */
	net_.reset(new caffe::Net<float>(in_model_file, caffe::TEST));
	net_->CopyTrainedLayersFrom(in_trained_file);

	CHECK_EQ(net_->num_inputs(), 1)<< "Network should have exactly one input.";
	CHECK_EQ(net_->num_outputs(), 1)<< "Network should have exactly one output.";

	caffe::Blob<float>* input_layer = net_->input_blobs()[0];
	num_channels_ = input_layer->channels();
	CHECK(num_channels_ == 3 || num_channels_ == 1) << "Input layer should have 1 or 3 channels.";
	input_geometry_ = cv::Size(input_layer->width(), input_layer->height());

	lookuptable_file_ = in_lookuptable_file;

	pixel_mean_		= cv::Scalar(102.9801, 115.9465, 122.7717);
}

void ENetSegmenter::Predict(const cv::Mat& in_image_mat, cv::Mat& out_segmented)
{
	caffe::Blob<float>* input_layer = net_->input_blobs()[0];
	input_layer->Reshape(1, num_channels_, input_geometry_.height,
			input_geometry_.width);
	/* Forward dimension change to all layers. */
	net_->Reshape();

	std::vector<cv::Mat> input_channels;
	WrapInputLayer(&input_channels);

	Preprocess(in_image_mat, &input_channels);

	net_->Forward();

	/* Copy the output layer to a std::vector */
	caffe::Blob<float>* output_layer = net_->output_blobs()[0];

	int width = output_layer->width();
	int height = output_layer->height();
	int channels = output_layer->channels();

	// compute argmax
	cv::Mat class_each_row(channels, width * height, CV_32FC1,
			const_cast<float *>(output_layer->cpu_data()));
	class_each_row = class_each_row.t(); // transpose to make each row with all probabilities
	cv::Point maxId;    // point [x,y] values for index of max
	double maxValue;    // the holy max value itself
	cv::Mat prediction_map(height, width, CV_8UC1);
	for (int i = 0; i < class_each_row.rows; i++)
	{
		minMaxLoc(class_each_row.row(i), 0, &maxValue, 0, &maxId);
		prediction_map.at<uchar>(i) = maxId.x;
	}

	out_segmented = Visualization(prediction_map, lookuptable_file_);

	cv::resize(out_segmented, out_segmented, cv::Size(in_image_mat.cols, in_image_mat.rows));


}

cv::Mat ENetSegmenter::Visualization(cv::Mat in_prediction_map, std::string in_lookuptable_file)
{

	cv::cvtColor(in_prediction_map.clone(), in_prediction_map, CV_GRAY2BGR);
	cv::Mat label_colours = cv::imread(in_lookuptable_file, 1);
	cv::cvtColor(label_colours, label_colours, CV_RGB2BGR);
	cv::Mat output_image;
	LUT(in_prediction_map, label_colours, output_image);

	return output_image;
}

void ENetSegmenter::WrapInputLayer(std::vector<cv::Mat>* in_input_channels)
{
	caffe::Blob<float>* input_layer = net_->input_blobs()[0];

	int width = input_layer->width();
	int height = input_layer->height();
	float* input_data = input_layer->mutable_cpu_data();
	for (int i = 0; i < input_layer->channels(); ++i)
	{
		cv::Mat channel(height, width, CV_32FC1, input_data);
		in_input_channels->push_back(channel);
		input_data += width * height;
	}
}

void ENetSegmenter::Preprocess(const cv::Mat& in_image_mat,
		std::vector<cv::Mat>* in_input_channels)
{
	/* Convert the input image to the input image format of the network. */
	cv::Mat sample;
	if (in_image_mat.channels() == 3 && num_channels_ == 1){
		cv::cvtColor(in_image_mat, sample, cv::COLOR_BGR2GRAY);}
	else if (in_image_mat.channels() == 4 && num_channels_ == 1){
		cv::cvtColor(in_image_mat, sample, cv::COLOR_BGRA2GRAY);}
	else if (in_image_mat.channels() == 4 && num_channels_ == 3){
		cv::cvtColor(in_image_mat, sample, cv::COLOR_BGRA2BGR);}
	else if (in_image_mat.channels() == 1 && num_channels_ == 3){
		cv::cvtColor(in_image_mat, sample, cv::COLOR_GRAY2BGR);}
	else{
		sample = in_image_mat;
	}

	cv::Mat sample_resized;
	if (sample.size() != input_geometry_){
		cv::resize(sample, sample_resized, input_geometry_);}
	else{
		sample_resized = sample;
	}

	cv::Mat sample_float;
	if (num_channels_ == 3)
	{
		sample_resized.convertTo(sample_float, CV_32FC3);
	}
	else
	{
		sample_resized.convertTo(sample_float, CV_32FC1);
	}

	cv::split(sample_float, *in_input_channels);

	CHECK(reinterpret_cast<float*>(in_input_channels->at(0).data)
			== net_->input_blobs()[0]->cpu_data())
													<< "Input channels are not wrapping the input layer of the network.";
}

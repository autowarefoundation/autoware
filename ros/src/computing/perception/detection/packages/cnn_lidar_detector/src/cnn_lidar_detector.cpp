/*
 *  Copyright (c) 2017, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "cnn_lidar_detector.hpp"

void CnnLidarDetector::get_box_points_from_matrices(size_t in_row,
                                                    size_t in_col,
                                                    const std::vector<cv::Mat>& in_boxes_channels,
                                                    std::vector<float>& out_box)
{
	CHECK_EQ(in_boxes_channels.size(), 24) << "Incorrect Number of points to form a bounding box, expecting 24, got: " << in_boxes_channels.size();

	//bottom layer
	out_box.clear();
	out_box.resize(8);//8 points representing a rectangle in the XY plane
	out_box[0] = in_boxes_channels[0].at<float>(in_row, in_col); //bottom_front_left X
	out_box[1] = in_boxes_channels[1].at<float>(in_row, in_col); //bottom_front_left Y

	out_box[2] = in_boxes_channels[3].at<float>(in_row, in_col); //bottom_back_right X
	out_box[3] = in_boxes_channels[4].at<float>(in_row, in_col); //bottom_back_right Y

	out_box[4] = in_boxes_channels[6].at<float>(in_row, in_col); //bottom_back_right X
	out_box[5] = in_boxes_channels[7].at<float>(in_row, in_col); //bottom_back_right Y

	out_box[6] = in_boxes_channels[9].at<float>(in_row, in_col); //bottom_back_right X
	out_box[7] = in_boxes_channels[10].at<float>(in_row, in_col); //bottom_back_right Y

	/*out_box.bottom_front_left.x = in_boxes_channels[0].at<float>(row,col);
	out_box.bottom_front_left.y = in_boxes_channels[1].at<float>(row,col);
	out_box.bottom_front_left.z = in_boxes_channels[2].at<float>(row,col);

	out_box.bottom_back_left.x = in_boxes_channels[3].at<float>(row,col);
	out_box.bottom_back_left.y = in_boxes_channels[4].at<float>(row,col);
	out_box.bottom_back_left.z = in_boxes_channels[5].at<float>(row,col);

	out_box.bottom_back_right.x = in_boxes_channels[6].at<float>(row,col);
	out_box.bottom_back_right.y = in_boxes_channels[7].at<float>(row,col);
	out_box.bottom_back_right.z = in_boxes_channels[8].at<float>(row,col);

	out_box.bottom_front_right.x = in_boxes_channels[9].at<float>(row,col);
	out_box.bottom_front_right.y = in_boxes_channels[10].at<float>(row,col);
	out_box.bottom_front_right.z = in_boxes_channels[11].at<float>(row,col);

	//top layer
	out_box.top_front_left = out_box.bottom_front_left;
	out_box.top_back_left = out_box.bottom_back_left;
	out_box.top_back_right = out_box.bottom_back_right;
	out_box.top_front_right = out_box.bottom_front_right;

	//update only height for the top layer
	out_box.top_front_left.z = in_boxes_channels[14].at<float>(row,col);
	out_box.top_back_left.z = in_boxes_channels[17].at<float>(row,col);
	out_box.top_back_right.z = in_boxes_channels[20].at<float>(row,col);
	out_box.top_front_right.z = in_boxes_channels[23].at<float>(row,col);*/
}


template<typename PointT>
float CnnLidarDetector::get_points_distance(const PointT& in_p1, const PointT& in_p2)
{

	Eigen::Vector3f p1 = Eigen::Vector3f(in_p1.getArray3fMap());
	Eigen::Vector3f p2 = Eigen::Vector3f(in_p2.getArray3fMap());

	return pcl::geometry::distance(p1, p2);
}

cv::Mat CnnLidarDetector::resize_image(cv::Mat in_image, cv::Size in_geometry)
{
	cv::Mat resized;
	if (in_image.size() != in_geometry)
		{cv::resize(in_image, resized, in_geometry);}
	else
		{resized = in_image;}

	return resized;
}

void CnnLidarDetector::Detect(const cv::Mat& in_image_intensity,
                              const cv::Mat& in_image_range,
                              const cv::Mat& in_image_x,
                              const cv::Mat& in_image_y,
                              const cv::Mat& in_image_z,
                              cv::Mat& out_objectness_image,
                              jsk_recognition_msgs::BoundingBoxArray& out_boxes)
{
	caffe::Blob<float>* input_layer = net_->input_blobs()[0];
	input_layer->Reshape(1,
	                     num_channels_,
	                     input_geometry_.height,
	                     input_geometry_.width);
	/* Forward dimension change to all layers. */
	net_->Reshape();

	std::vector<cv::Mat> input_channels;
	WrapInputLayer(&input_channels);//create pointers for input layers

	PreProcess(in_image_intensity,
	           in_image_range,
	           in_image_x,
	           in_image_y,
	           in_image_z,
	           &input_channels);

	net_->Forward();

	GetNetworkResults(out_objectness_image, out_boxes);

}

void CnnLidarDetector::BoundingBoxCornersToJskBoundingBox(const CnnLidarDetector::BoundingBoxCorners& in_box_corners,
                                                          unsigned int in_class,
                                                          std_msgs::Header& in_header,
                                                          jsk_recognition_msgs::BoundingBox& out_jsk_box)
{
	//          top_front_left      top_front_right
	//              ----------------
	//             /               /
	//            /               /
	//           /               /
	//          /               /
	//         /               /
	//        /               /
	//       -----------------
	//   top_back_left     top_back_right
	out_jsk_box.header = in_header;

	out_jsk_box.dimensions.x = sqrt(in_box_corners.top_front_left.x * in_box_corners.top_front_left.x -
			                        in_box_corners.top_back_left.x * in_box_corners.top_back_left.x);
	out_jsk_box.dimensions.y = sqrt(in_box_corners.top_front_left.y * in_box_corners.top_front_left.y -
	                                in_box_corners.top_front_right.y * in_box_corners.top_front_right.y);
	out_jsk_box.dimensions.z = in_box_corners.top_front_right.z - in_box_corners.bottom_front_right.z;//any z would do

	//centroid
	out_jsk_box.pose.position.x = (in_box_corners.top_front_left.x + in_box_corners.top_back_right.x) / 2;
	out_jsk_box.pose.position.y = (in_box_corners.top_front_left.y + in_box_corners.top_back_right.y) / 2;
	out_jsk_box.pose.position.z = (in_box_corners.top_front_left.z + in_box_corners.top_front_left.z) / 2;

	//rotation angle
	float x_diff = in_box_corners.top_front_left.x - in_box_corners.top_back_right.x;
	float y_diff = in_box_corners.top_front_left.y - in_box_corners.top_back_right.y;
	float rotation_angle = atan2(y_diff, x_diff);

	tf::Quaternion quat = tf::createQuaternionFromRPY(0.0, 0.0, rotation_angle);
	tf::quaternionTFToMsg(quat, out_jsk_box.pose.orientation);

	out_jsk_box.label = in_class;

}


/*void CnnLidarDetector::ApplyNms(std::vector<CnnLidarDetector::BoundingBoxCorners>& in_out_box_corners,
              size_t in_min_num_neighbors,
              float in_min_neighbor_distance,
              float in_min_box_distance,
              std::vector<CnnLidarDetector::BoundingBoxCorners>& out_nms_box_corners)
{
	//calculate distance between each box in the input vector
	//the distance is calculated using the euclidean distance between the points forming the longer diagonal in each box
	size_t total_boxes = in_out_box_corners.size();
	out_nms_box_corners.clear();
	for (size_t i=0; i<total_boxes; i++)
	{
		in_out_box_corners[i].distance_candidates.resize(total_boxes);
		for (size_t j=0; j<total_boxes; j++)
		{
			in_out_box_corners[i].distance_candidates[j] =  get_points_distance(in_out_box_corners[i].top_front_left,
			                                                                    in_out_box_corners[j].top_front_left) +
			                                                get_points_distance(in_out_box_corners[i].bottom_back_right,
			                                                                    in_out_box_corners[j].bottom_back_right);
			//if its not the same box and is closer than the minimum threshold for neighbors, add it as one
			if (i != j && in_out_box_corners[i].distance_candidates[j] < in_min_neighbor_distance)
				{in_out_box_corners[i].neighbor_indices.push_back(j);}
		}
	}
	//keep only those who have only the minimum neighbors
	for (size_t i=0; i<total_boxes; i++)
	{
		if (in_out_box_corners[i].neighbor_indices.size() >= in_min_num_neighbors)
		{
			out_nms_box_corners.push_back(in_out_box_corners[i]);
		}
	}

	//for each group select one
}*/

void CnnLidarDetector::GetNetworkResults(cv::Mat& out_objectness_image,
                                         jsk_recognition_msgs::BoundingBoxArray& out_boxes)
{
	caffe::Blob<float>* boxes_blob = net_->output_blobs().at(0);//0 boxes
	caffe::Blob<float>* objectness_blob = net_->output_blobs().at(1);//1 objectness

	//output layer     0  1    2     3
	//prob 		shape  1 04 height width
	//bb_score 	shape  1 24 height width
	CHECK_EQ(boxes_blob->shape(1), 24) << "The output bb_score layer should be 24 channel image, but instead is " << boxes_blob->shape(1);
	CHECK_EQ(objectness_blob->shape(1), 4) << "The output prob layer should be 4 channel image, but instead is " << objectness_blob->shape(1) ;

	CHECK_EQ(boxes_blob->shape(3), objectness_blob->shape(3)) << "Boxes and Objectness should have the same shape, " << boxes_blob->shape(3);
	CHECK_EQ(boxes_blob->shape(2), objectness_blob->shape(2)) << "Boxes and Objectness should have the same shape, " << boxes_blob->shape(2);

	std::vector<cv::Mat> objectness_channels;
	int width = objectness_blob->shape(3);
	int height = objectness_blob->shape(2);

	//convert objectness (classes) channels to Mat
	float* objectness_ptr = objectness_blob->mutable_cpu_data();//pointer to the prob layer
	//copy each channel(class) from the output layer to a Mat
	for (int i = 0; i < objectness_blob->shape(1); ++i)
	{
		cv::Mat channel(height, width, CV_32FC1, objectness_ptr);
		cv::normalize(channel, channel, 1, 0, cv::NORM_MINMAX);
		objectness_channels.push_back(channel);
		objectness_ptr += width * height;
	}

	//convert boxes (24 floats representing each of the 8 3D points forming the bbox)
	float* boxes_ptr = boxes_blob->mutable_cpu_data();//pointer to the bbox layer
	std::vector<cv::Mat> boxes_channels;
	for (int i = 0; i < boxes_blob->shape(1); ++i)
	{
		cv::Mat channel(height, width, CV_32FC1, boxes_ptr);
		boxes_channels.push_back(channel);
		boxes_ptr += width * height;
	}
	//check each pixel of each channel and assign color depending threshold
	cv::Mat bgr_channels(height, width, CV_8UC3, cv::Scalar(0,0,0));

	std::vector< std::vector<float> > cars_boxes, person_boxes, bike_boxes;
	for(unsigned int row = 0; row < height; row++)
	{
		for(unsigned int col = 0; col < width; col++)
		{
			//0 nothing
			//1 car, red
			//2 person, green
			//3 bike, blue
			//BGR Image

			//8 item vector (x1, y1), (x2,y2), (x3, y3), (x4,y4) forming the bottom rectangle of the box
			std::vector<float> current_box;

			if (objectness_channels[1].at<float>(row,col) > score_threshold_)
			{
				get_box_points_from_matrices(row, col, boxes_channels, current_box);
				bgr_channels.at<cv::Vec3b>(row,col) = cv::Vec3b(0, 0, 255);
				cars_boxes.push_back(current_box);
			}
			if (objectness_channels[2].at<float>(row,col) > score_threshold_)
			{
				get_box_points_from_matrices(row, col, boxes_channels, current_box);
				bgr_channels.at<cv::Vec3b>(row,col) = cv::Vec3b(0, 255, 0);
				person_boxes.push_back(current_box);
			}
			if (objectness_channels[3].at<float>(row,col) > score_threshold_)
			{
				get_box_points_from_matrices(row, col, boxes_channels, current_box);
				bgr_channels.at<cv::Vec3b>(row,col) = cv::Vec3b(255, 0, 0);
				bike_boxes.push_back(current_box);
			}
		}
	}
	//apply NMS to boxes
	std::vector< std::vector<float> > final_cars_boxes, final_person_boxes, final_bike_boxes;

	//ApplyNms(cars_boxes, 5, 0.3, 7.0, final_cars_boxes);
	//ApplyNms(person_boxes, 5, 0.3, 7.0, final_person_boxes);
	//ApplyNms(bike_boxes, 5, 0.3, 7.0, final_bike_boxes);
	//copy resulting boxes to output message
	out_boxes.boxes.clear();

	cv::flip(bgr_channels, out_objectness_image, -1);
}

void CnnLidarDetector::PreProcess(const cv::Mat& in_image_intensity,
                                  const cv::Mat& in_image_range,
                                  const cv::Mat& in_image_x,
                                  const cv::Mat& in_image_y,
                                  const cv::Mat& in_image_z,
                                  std::vector<cv::Mat>* in_out_channels)
{
	//resize image if required
	cv::Mat intensity_resized, range_resized;
	cv::Mat x_resized, y_resized, z_resized;

	intensity_resized = resize_image(in_image_intensity, input_geometry_);
	range_resized = resize_image(in_image_range, input_geometry_);
	x_resized = resize_image(in_image_x, input_geometry_);
	y_resized = resize_image(in_image_y, input_geometry_);
	z_resized = resize_image(in_image_z, input_geometry_);

	//put each corrected mat geometry onto the correct input layer type pointers
	intensity_resized.copyTo(in_out_channels->at(0));
	range_resized.copyTo(in_out_channels->at(1));
	x_resized.copyTo(in_out_channels->at(2));
	y_resized.copyTo(in_out_channels->at(3));
	z_resized.copyTo(in_out_channels->at(4));

	//check that the pre processed and resized mat pointers correspond to the pointers of the input layers
	CHECK(reinterpret_cast<float*>(in_out_channels->at(0).data) == net_->input_blobs()[0]->cpu_data())	<< "Input channels are not wrapping the input layer of the network.";

}

void CnnLidarDetector::WrapInputLayer(std::vector<cv::Mat>* in_out_channels)
{
	caffe::Blob<float>* input_layer = net_->input_blobs()[0];

	int width = input_layer->width();
	int height = input_layer->height();
	float* input_data = input_layer->mutable_cpu_data();
	for (int i = 0; i < input_layer->shape(1); ++i)
	{
		cv::Mat channel(height, width, CV_32FC1, input_data);
		in_out_channels->push_back(channel);
		input_data += width * height;
	}
}

CnnLidarDetector::CnnLidarDetector(const std::string& in_network_definition_file,
		const std::string& in_pre_trained_model_file,
		bool in_use_gpu,
		unsigned int in_gpu_id,
		float in_score_threshold)
{
	if(in_use_gpu)
	{
		caffe::Caffe::set_mode(caffe::Caffe::GPU);
		caffe::Caffe::SetDevice(in_gpu_id);
	}
	else
		caffe::Caffe::set_mode(caffe::Caffe::CPU);

	/* Load the network. */
	net_.reset(new caffe::Net<float>(in_network_definition_file, caffe::TEST));
	net_->CopyTrainedLayersFrom(in_pre_trained_model_file);

	caffe::Blob<float>* input_layer = net_->input_blobs()[0];

	num_channels_ = input_layer->channels();

	input_geometry_ = cv::Size(input_layer->width(), input_layer->height());

	score_threshold_ = in_score_threshold;

}

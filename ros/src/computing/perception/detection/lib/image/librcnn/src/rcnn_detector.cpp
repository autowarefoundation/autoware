/*
 *  Copyright (c) 2015, Nagoya University
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

#include "rcnn_detector.h"

//#define _TIMEPROCESS

RcnnDetector::RcnnDetector(std::string& in_network_definition_file, std::string& in_pre_trained_model_file,
							bool in_use_gpu, unsigned int in_gpu_id)
{
	//Create Net based on RCNN model
	net_ = new caffe::Net<float>(in_network_definition_file, caffe::TEST);

	//If GPU mode is selected, set the GPU device ID if provided
	if (in_use_gpu)
	{
		caffe::Caffe::set_mode(caffe::Caffe::GPU);
		caffe::Caffe::SetDevice(in_gpu_id);
	}
	pixel_mean_ = cv::Scalar(102.9801, 115.9465, 122.7717);
	image_slices_ = 16;
	rois_blob_proto_ = NULL;

	//Load Pre trained model
	net_->CopyTrainedLayersFrom(in_pre_trained_model_file);

	counter_ = 0;
}

std::vector< cv::Scalar > RcnnDetector::GenerateProposals(unsigned int in_width, unsigned int in_height, unsigned int in_slices, float in_box_overlap)
{
	std::vector< cv::Scalar > proposals;
	unsigned int step_y = int(in_height/in_slices) + 1;
	unsigned int step_x = int(in_width*in_box_overlap/in_slices) + 1;

	for (unsigned int y = step_y*2; y<=(in_height - step_y*2); y+=step_y*2)//create proposals starting after the first slice in Y until penultimate
	{
		for (unsigned int x = 0; x<=in_width; x+=step_x)
		{
			proposals.push_back(cv::Scalar(x, y, step_x+x, step_y+y));//smallest subwindow

			//tall objects
			proposals.push_back(cv::Scalar(x, y, step_x+x, step_y*2+y));

			//wider objects
			//proposals.push_back(cv::Scalar(x, y, step_x*2+x, step_y+y));
			//proposals.push_back(cv::Scalar(x, y, step_x*4+x, step_y+y));

			//squared
			for (unsigned int z = 2; z <=in_slices; z+=2)
			{
				unsigned int w = step_x*z+x,
							h = step_y*z+y;
				if ((x+w) <= (in_width+step_x) || (y+h) <= (in_height+ step_y))
				{
					proposals.push_back(cv::Scalar(x, y, w, h));
				}
			}
			/*proposals.push_back(cv::Scalar(x, y, step_x*2+x, step_y*2+y));
			proposals.push_back(cv::Scalar(x, y, step_x*4+x, step_y*4+y));
			proposals.push_back(cv::Scalar(x, y, step_x*8+x, step_y*8+y));
			proposals.push_back(cv::Scalar(x, y, step_x*16+x, step_y*16+y));*/
		}
	}

	return proposals;
}

void RcnnDetector::PreProcessImage(cv::Mat& in_image, cv::Mat& out_image)
{
	//convert image to float
	in_image.convertTo(out_image,CV_32FC3);
	//create a mat full of means
	cv::Mat mean(in_image.rows,in_image.cols,CV_32FC3, pixel_mean_);
	//subtract images to get subtracted
	out_image -= mean;
}

void RcnnDetector::ConvertImageToBlob(cv::Mat& in_image, caffe::BlobProto& out_blob)
{
	out_blob.set_num(1);							//only 1 image
	out_blob.set_channels(in_image.channels());		//channels
	out_blob.set_height(in_image.rows);				//height
	out_blob.set_width(in_image.cols);				//width

	unsigned int img_channels = in_image.channels();
	unsigned int img_total = in_image.total();
	//OpenCV Data is stored in BGR
	//RE ORDER PIXEL DATA FROM [(BGR)(BGR)(BGR)...(BGR)]   => [(BBB)..(BBB) (GGG)..(GGG) (RRR)..(RRR)]
	for(unsigned int j = 0; j < img_channels; j++)						//do as manychannels as we have (in our tests 3)
	{
		for (unsigned int i = j; i < img_total; i++)
		{
			out_blob.add_data(((float*)(in_image.data))[i*img_channels]);// slide through each channel
		}
	}
}

void RcnnDetector::ConvertRoisToBlob(std::vector< cv::Scalar >& in_rois, caffe::BlobProto& out_blob)
{
	out_blob.set_num((int)in_rois.size());	//number of ROIS
	out_blob.set_channels(5);				//depth, always 5 (0, x1, y1, x2,y2)
	out_blob.set_height(1);				//one vector vector
	out_blob.set_width(1);					//one column vector

	for (unsigned int i = 0; i < in_rois.size(); ++i)
	{
		for (unsigned int j = 0; j < 4; ++j)
		{
			if (j == 0)
				out_blob.add_data(0.0f);	//append a 0 as the first column
			out_blob.add_data( (float) ((cv::Scalar)(in_rois)[i]).val[j] );
		}
	}
}

bool RcnnDetector::CheckClasses(unsigned int in_class, const std::vector<unsigned int>& in_classes)
{
	for (unsigned int i = 0; i< in_classes.size(); i++)
	{
		if(in_classes[i] == in_class)
			return true;
	}
	return false;
}

std::vector< RectClassScore<float> > RcnnDetector::GetRectClassesScored(std::vector< cv::Scalar >& in_proposals,
										const std::vector<unsigned int>& in_classes,
										float in_score_threshold,
										const float* in_boxes, const float* in_probabilities,
										unsigned int in_width, unsigned int in_height)
{
	std::vector< RectClassScore<float> > out_boxes;
	for (unsigned int j=0, k=0;j<in_proposals.size();j++)//repeat for each proposal
	{
		float x,y,w,h;
		x = ((cv::Scalar)(in_proposals)[j]).val[0];
		y = ((cv::Scalar)(in_proposals)[j]).val[1];
		w = ((cv::Scalar)(in_proposals)[j]).val[2] - ((cv::Scalar)(in_proposals)[j]).val[0];
		h = ((cv::Scalar)(in_proposals)[j]).val[3] - ((cv::Scalar)(in_proposals)[j]).val[1];
		for (int i = 0, m=0; i < Rcnn::NUM_CLASSES*4; i+=4, k++, m++)//repeat for all 21 classes			// four points on each rect
		{
			if ( (in_probabilities[k] >=in_score_threshold) && CheckClasses(m, in_classes))//check if the class is being searched, otherwise do not add the detection
			{
				int index = j*Rcnn::NUM_CLASSES*4 + i;
				//[0] = X1; [1] = Y1; [2] = W; [3] = H
				float dx,dy,dw,dh;
				dx = in_boxes[index];	//cout << dx << ", ";
				dy = in_boxes[index+1]; //cout << dy << ", ";
				dw = in_boxes[index+2]; //cout << dw << ", ";
				dh = in_boxes[index+3]; //cout << dh << endl;

				float pred_w = exp(dw)*w;
				float pred_h = exp(dh)*h;
				float new_x = dx*w + (x+0.5*w) - 0.5*pred_w;
				float new_y = dy*h + (y+0.5*h) - 0.5*pred_h;

				//clip boxes
				/*new_x = (new_x >= 0 ? new_x: 0);
				new_x = (new_x <= in_width ? new_x: in_width-1);

				new_y = (new_y >= 0 ? new_y : 0);
				new_y = (new_y <= in_height ? new_y: in_height-1);*/

				RectClassScore<float> tmp_rect;
				tmp_rect.x = new_x;
				tmp_rect.y = new_y;
				tmp_rect.w = pred_w;//(new_x + pred_w > in_width) ? in_width-1: pred_w;
				tmp_rect.h = pred_h;//(new_y + pred_h > in_height) ? in_height-1: pred_h;
				tmp_rect.score = in_probabilities[k];
				tmp_rect.class_type = m;
				tmp_rect.enabled = true;

				out_boxes.push_back(tmp_rect);

			}
		}
	}
	return out_boxes;
}

std::vector< RectClassScore<float> >
	RcnnDetector::Detect(cv::Mat& in_image,
						const std::vector<unsigned int>& in_classes,
						float in_score_threshold,
						unsigned int in_slices,
						float in_box_overlap,
						float in_nms_threshold)
{
	/////IMAGE INPUT 'data'
	//pre process image
	cv::Mat float_image;
#ifdef _TIMEPROCESS
	cv::TickMeter timer;
	timer.start();
#endif

	PreProcessImage(in_image, float_image);

#ifdef _TIMEPROCESS
	timer.stop();
	float t_preprocess = timer.getTimeMilli();
	timer.reset(); timer.start();
	timer.start();
#endif

	std::vector< cv::Scalar > proposals = GenerateProposals(float_image.cols, float_image.rows, in_slices, in_box_overlap);

#ifdef _TIMEPROCESS
	timer.stop();
	float t_proposal = timer.getTimeMilli();
	timer.reset(); timer.start();
#endif

	caffe::Blob<float>* input_image_blob = net_->input_blobs()[0];	//Get pointer to image input blob [0] image input (defined by net)
	input_image_blob->Reshape(	1, 									//sending only 1 image
								float_image.channels(),				//blob's number of channels
								float_image.rows, 					//blob's width
								float_image.cols					//blob's height
								);									//reshape input layer so it to matches the image

	caffe::BlobProto image_blob_proto;

	ConvertImageToBlob(float_image, image_blob_proto);				//convert float image to blob
	input_image_blob->FromProto(image_blob_proto);					//set data into 'data' layer

	/////PROPOSALS INPUT 'rois'
	caffe::Blob<float>* input_rois_blob = net_->input_blobs()[1];	//[0]data (image), [1]rois (proposals)


	caffe::BlobProto rois_blob_proto;
	ConvertRoisToBlob(proposals, rois_blob_proto);
	//GenerateBlobProposals(float_image.cols, float_image.rows, in_slices);	//generate proposals 1st time
	input_rois_blob->FromProto(rois_blob_proto);							//set data into 'data' layer

	//FORWARD the network
	std::vector<caffe::Blob<float>*> input;
	input.push_back( input_image_blob );
	input.push_back( input_rois_blob );

#ifdef _TIMEPROCESS
	timer.stop();
	float t_datapreparation = timer.getTimeMilli();
	timer.reset(); timer.start();
#endif

	net_->Forward( input ); //Forward the Network////////////////

#ifdef _TIMEPROCESS
	timer.stop();
	float t_fwd = timer.getTimeMilli();
	timer.reset(); timer.start();
#endif
	//check output
	const boost::shared_ptr<caffe::Blob<float> >& class_probability_layer = net_->blob_by_name("cls_prob");
	const boost::shared_ptr<caffe::Blob<float> >& bounding_box_predicted_layer = net_->blob_by_name("bbox_pred");

	//convert to rects
	const float* class_probability_data = class_probability_layer->cpu_data();
	const float* bounding_box_predicted_data = bounding_box_predicted_layer->cpu_data();

	std::vector< RectClassScore<float> > detections;
	if ((unsigned int)bounding_box_predicted_layer->count() == proposals.size()*Rcnn::NUM_CLASSES*4)//4 points for each of the 21 classes
	{
		detections = GetRectClassesScored(proposals, in_classes, in_score_threshold, bounding_box_predicted_data, class_probability_data, float_image.cols, float_image.rows);

		detections = ApplyNonMaximumSuppresion(detections, in_nms_threshold);
	}
	else
		std::cout << "Wrong output size:" << class_probability_layer->count() << ", expecting " << proposals.size()*Rcnn::NUM_CLASSES*4; std::cout << std::endl;

#ifdef _TIMEPROCESS
	timer.stop();
	float t_out = timer.getTimeMilli();
	std::cout << t_preprocess << "," << t_proposal << ","<<  t_datapreparation << "," << t_fwd << "," << t_out << std::endl;
#endif


	return detections;
}

std::vector< RectClassScore<float> > RcnnDetector::ApplyNonMaximumSuppresion(std::vector< RectClassScore<float> > in_source, float in_nms_threshold)
{
	std::vector< RectClassScore<float> > tmp_source = in_source;

	if (tmp_source.empty())
		return std::vector<RectClassScore<float> >();

	unsigned int size = in_source.size();

	std::vector<float> area(size);
	std::vector<float> scores(size);
	std::vector<int> x1(size);
	std::vector<int> y1(size);
	std::vector<int> x2(size);
	std::vector<int> y2(size);
	std::vector<unsigned int> indices(size);
	std::vector<bool> is_suppresed(size);

	for(unsigned int i = 0; i< in_source.size(); i++)
	{
		RectClassScore<float> tmp = in_source[i];
		area[i] = tmp.w * tmp.h;
		indices[i] = i;
		is_suppresed[i] = false;
		scores[i] = tmp.score;
		x1[i] = tmp.x;
		y1[i] = tmp.y;
		x2[i] = tmp.w + tmp.x;
		y2[i] = tmp.h + tmp.y;
	}

	Sort(scores, indices);//returns indices ordered based on scores

	for(unsigned int i=0; i< size; i++)
	{
		if(!is_suppresed[indices[i]])
		{
			for(unsigned int j= i+1; j< size; j++)
			{
				int x1_max = std::max(x1[indices[i]], x1[indices[j]]);
				int x2_min = std::min(x2[indices[i]], x2[indices[j]]);
				int y1_max = std::max(y1[indices[i]], y1[indices[j]]);
				int y2_min = std::min(y2[indices[i]], y2[indices[j]]);
				int overlap_width = x2_min - x1_max + 1;
				int overlap_height = y2_min - y1_max + 1;
				if(overlap_width > 0 && overlap_height>0)
				{
					float overlap_part = (overlap_width*overlap_height)/area[indices[j]];
					if(overlap_part > in_nms_threshold)
					{
						is_suppresed[indices[j]] = true;
					}
				}
			}
		}
	}

	unsigned int size_out = 0;
	for (unsigned int i = 0; i < size; i++)
	{
		if (!is_suppresed[i])
			size_out++;
	}

	std::vector< RectClassScore<float> > filtered_detections(size_out);

	unsigned int index = 0;
	for(unsigned int i = 0 ; i < size_out; i++)
	{
		if(!is_suppresed[indices[i]])
		{
			filtered_detections[index].x = in_source[indices[i]].x;//x1[indices[i]];
			filtered_detections[index].y = in_source[indices[i]].y;//y1[indices[i]];
			filtered_detections[index].w = in_source[indices[i]].w;//x2[indices[i]] - x1[indices[i]];
			filtered_detections[index].h = in_source[indices[i]].h;//y2[indices[i]] - y1[indices[i]];
			filtered_detections[index].class_type = in_source[indices[i]].class_type;
			filtered_detections[index].score = in_source[indices[i]].score;
			index++;
		}
	}
	return filtered_detections;
}

void RcnnDetector::Sort(const std::vector<float> in_scores, std::vector<unsigned int>& in_out_indices)
{
	for (unsigned int i = 0; i < in_scores.size(); i++)
		for (unsigned int j = i + 1; j < in_scores.size(); j++)
		{
			if (in_scores[in_out_indices[j]] > in_scores[in_out_indices[i]])
			{
				std::swap(in_out_indices[i], in_out_indices[j]);
				/*int index_tmp = in_out_indices[i];
				in_out_indices[i] = in_out_indices[j];
				in_out_indices[j] = index_tmp;*/
			}
		}
}

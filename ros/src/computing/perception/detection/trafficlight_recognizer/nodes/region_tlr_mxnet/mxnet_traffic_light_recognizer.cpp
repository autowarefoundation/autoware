#include "mxnet_traffic_light_recognizer.h"

MxNetTrafficLightRecognizer::MxNetTrafficLightRecognizer()
{
}

MxNetTrafficLightRecognizer::~MxNetTrafficLightRecognizer()
{
	MXPredFree(prediction_handle_);
}

void MxNetTrafficLightRecognizer::Init(const char* in_network_definition_buffer,
                                       const char* in_pretrained_model_buffer,
                                       int in_pretrained_model_length,
                                       const bool in_use_gpu,
                                       const unsigned int in_gpu_id)
{

	// Parameters
	int dev_type = (in_use_gpu) ? 2 : 1;  // 1: cpu, 2: gpu
	int dev_id = in_gpu_id;
	mx_uint num_input_nodes = 1;  // 1 for feedforward
	const char* input_key[1] = {"data"}; //input  layer name
	const char** input_keys = input_key;

	//pre defined model input size
	width_ = 32;
	height_ = 32;
	num_channels_ = 3;

	const mx_uint input_shape_indptr[2] = { 0, 4 };
	const mx_uint input_shape_data[4] = { 1,
	                                      static_cast<mx_uint>(num_channels_),
	                                      static_cast<mx_uint>(height_),
	                                      static_cast<mx_uint>(width_)};
	prediction_handle_ = 0;

	MXPredCreate(in_network_definition_buffer,
	             in_pretrained_model_buffer,
	             in_pretrained_model_length,
	             dev_type,
	             dev_id,
	             num_input_nodes,
	             input_keys,
	             input_shape_indptr,
	             input_shape_data,
	             &prediction_handle_);
	assert(prediction_handle_);

}

void MxNetTrafficLightRecognizer::PreProcessImage(const cv::Mat& in_image,
                                                  mx_float* out_image_data,
                                                  const int in_channels,
                                                  const cv::Size in_resize_size)
{
	if (in_image.empty()) {
		std::cerr << "Image is empty";
		assert(false);
	}

	cv::Mat im;

	cv::resize(in_image, im, in_resize_size);

	int size = im.rows * im.cols * in_channels;

	mx_float* ptr_image_r = out_image_data;
	mx_float* ptr_image_g = out_image_data + size / 3;
	mx_float* ptr_image_b = out_image_data + size / 3 * 2;

	for (int i = 0; i < im.rows; i++)
	{
		uchar* data = im.ptr<uchar>(i);

		for (int j = 0; j < im.cols; j++)
		{
			if (in_channels > 1)
			{
				*ptr_image_b++ = static_cast<mx_float>(*data++);
				*ptr_image_g++ = static_cast<mx_float>(*data++);
			}

			*ptr_image_r++ = static_cast<mx_float>(*data++);
		}
	}
}

LightState MxNetTrafficLightRecognizer::RecognizeLightState(const cv::Mat &in_image, double in_score_threshold)
{
	LightState result = LightState::UNDEFINED;

	if (in_image.empty())
	{
		return result;
	}

	int image_size = width_ * height_ * num_channels_;
	std::vector<mx_float> mx_image_data = std::vector<mx_float>(image_size);

	PreProcessImage(in_image, mx_image_data.data(), num_channels_, cv::Size(width_, height_));

	MXPredSetInput(prediction_handle_, "data", mx_image_data.data(), image_size);

	MXPredForward(prediction_handle_);

	mx_uint output_index = 0;

	mx_uint *shape = 0;
	mx_uint shape_len;

	// Get Output Result
	MXPredGetOutputShape(prediction_handle_, output_index, &shape, &shape_len);

	size_t size = 1;
	for (mx_uint i = 0; i < shape_len; ++i) size *= shape[i];

	std::vector<float> output_data(size);

	MXPredGetOutput(prediction_handle_, output_index, &(output_data[0]), size);

	int highest_score_index = std::distance(output_data.begin(), std::max_element(output_data.begin(), output_data.end()));

	if (output_data[highest_score_index] < in_score_threshold)
	{
		return LightState::UNDEFINED;
	}

	switch(highest_score_index)
	{
		case NetworkResults::Green:
			result = LightState::GREEN;
			break;
		case NetworkResults::Yellow:
			result = LightState::YELLOW;
			break;
		case NetworkResults::Red:
			result = LightState::RED;
			break;
		default:
			result = LightState::UNDEFINED;
			break;
	}

	return result;
}

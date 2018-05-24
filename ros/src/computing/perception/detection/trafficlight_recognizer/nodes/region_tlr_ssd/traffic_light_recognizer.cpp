#include "traffic_light_recognizer.h"

// ===========================================
// Constructor of TrafficLightRecognizer class
// ===========================================
TrafficLightRecognizer::TrafficLightRecognizer():
  num_channels_(0),
  kPixelMean_(cv::Scalar(102.9801, 115.9465, 122.7717)) {
} // TrafficLightRecognizer::TrafficLightRecognizer()

// ===========================================
// Destructor of TrafficLightRecognizer class
// ===========================================
TrafficLightRecognizer::~TrafficLightRecognizer() {
} // TrafficLightRecognizer::~TrafficLightRecognizer()


// ========
// Init SSD
// ========
void TrafficLightRecognizer::Init(const std::string &network_definition_file_name,
                                  const std::string &pretrained_model_file_name,
                                  const bool use_gpu,
                                  const unsigned int gpu_id) {
  // If user attempt to use GPU, set mode and specify the GPU ID
  if (use_gpu) {
    caffe::Caffe::set_mode(caffe::Caffe::GPU);
    caffe::Caffe::SetDevice(gpu_id);
  } else {
    caffe::Caffe::set_mode(caffe::Caffe::CPU);
  }

  // Load the network
  network_.reset(new caffe::Net<float>(network_definition_file_name, caffe::TEST));
  network_->CopyTrainedLayersFrom(pretrained_model_file_name);

  CHECK_EQ(network_->num_inputs(), 1) << "Network should have exactly one input.";
  CHECK_EQ(network_->num_outputs(), 1) << "Network should have exactly one output.";

  caffe::Blob<float>* input_layer = network_->input_blobs()[0];
  num_channels_ = input_layer->channels();
  CHECK(num_channels_ == 3 || num_channels_ == 1);

  input_geometry_ = cv::Size(input_layer->width(), input_layer->height());

  //SetMean(kPixelMean_);

} // void TrafficLightRecognizer::Init()


// ================================================
// Run SSD process to recognize traffic light state
// ================================================
LightState TrafficLightRecognizer::RecognizeLightState(const cv::Mat& image) {
  caffe::Blob<float>* input_layer = network_->input_blobs()[0];
  input_layer->Reshape(1, num_channels_, input_geometry_.height, input_geometry_.width);

  // Forward dimension change to all layers
  network_->Reshape();

  std::vector<cv::Mat> input_channels;
  WrapInputLayer(&input_channels);

  Preprocess(image, &input_channels);

  // Run SSD recognition
  network_->Forward();

  // Get the most probable state from candidates in output layer
  caffe::Blob<float>* candidate_blob = network_->output_blobs()[0];
  const float* candidate = candidate_blob->cpu_data(); // format: [image_id(0), label(1), score(2), xmin(3), ymin(4), xmax(5), ymax()6)]
  const int num_candidate = candidate_blob->height();
  float max_score = 0;
  LightState result = LightState::UNDEFINED;
  for (int k = 0; k < num_candidate; ++k) {
    if (candidate[0] == -1) {
      // Skip invalid detection
      candidate += 7;
      continue;
    }

    // Search the state candidate which has highest score
    if (max_score < candidate[2]) {
      result = static_cast<LightState>(candidate[1]);
    }

    // Go next candidate
    candidate += 7;
  }

  return result;
} // void TrafficLightRecognizer::RecognizeLightState()


// ================================================================
// Wrap the input layer of the network in separate cv::Mat objectes
// (one per channels). This way we save one memcpy operation and we
// don't need to rely on cudaMemcpy2D. the last preprocessing
// operation will write the separate channels directly to the input
// layer.
// ================================================================
void TrafficLightRecognizer::WrapInputLayer(std::vector<cv::Mat>* input_channels) {
  caffe::Blob<float>* input_layer = network_->input_blobs()[0];

  int width = input_layer->width();
  int height = input_layer->height();
  float* input_data = input_layer->mutable_cpu_data();
  for (int i = 0; i < input_layer->channels(); ++i) {
    cv::Mat channels(height, width, CV_32FC1, input_data);
    input_channels->push_back(channels);
    input_data += width * height;
  }
} // void TrafficLightRecognizer::WrapInputLayer()


// ================================================
// Preprocess of SSD
// ================================================
void TrafficLightRecognizer::Preprocess(const cv::Mat& image, std::vector<cv::Mat>* input_channells) {
  // Convert the input image to the input image format of the network.
  cv::Mat sample;
  if (image.channels() == 3 && num_channels_ == 1) {
    cv::cvtColor(image, sample, cv::COLOR_BGR2GRAY);
  } else if (image.channels() == 4 && num_channels_ == 1) {
    cv::cvtColor(image, sample, cv::COLOR_BGRA2GRAY);
  } else if (image.channels() == 4 && num_channels_ == 3) {
    cv::cvtColor(image, sample, cv::COLOR_BGRA2BGR);
  } else if (image.channels() == 1 && num_channels_ == 3) {
    cv::cvtColor(image, sample, cv::COLOR_GRAY2BGR);
  } else {
    sample = image;
  }

  cv::Mat sample_resized;
  if (sample.size() != input_geometry_) {
    cv::resize(sample, sample_resized, input_geometry_);
  } else {
    sample_resized = sample;
 }

  cv::Mat sample_float;
  cv::Mat mean_image;
  if (num_channels_ == 3) {
    sample_resized.convertTo(sample_float, CV_32FC3);
    mean_image = cv::Mat(input_geometry_, CV_32FC3, kPixelMean_);
  } else {
    sample_resized.convertTo(sample_float, CV_32FC1);
    mean_image = cv::Mat(input_geometry_, CV_32FC1, kPixelMean_);
  }

  cv::Mat sample_normalized;
  cv::subtract(sample_float, mean_image, sample_normalized);

  // This operation will write the separate BGR planes directly to the
  // input layer of the network because it is wrapped by the cv::Mat
  // objects in input_channels.
  cv::split(sample_normalized, *input_channells);
  CHECK(reinterpret_cast<float*>(input_channells->at(0).data)
        == network_->input_blobs()[0]->cpu_data());

} // void TrafficLightRecognizer::Preprocess()

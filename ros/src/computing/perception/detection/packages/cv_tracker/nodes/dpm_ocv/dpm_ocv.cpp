#include <iostream>
#include <string>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/contrib/contrib.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_tracker/image_obj.h>

#include <dpm_ocv.hpp>

#define XSTR(x) #x
#define STR(x) XSTR(x)

#if defined(HAS_GPU)
static bool use_gpu = true;
#endif

static constexpr float SCORE_THRESHOLD = -0.5;
static constexpr int NUM_CELLS = 8;
static constexpr int NUM_BINS = 9;

enum class DetectType {
	CAR,
	PEDESTRIAN
};

class objectDetect
{
public:
	objectDetect(DetectType type);
	~objectDetect();
	void run();

private:
	void imageCallback(const sensor_msgs::ImageConstPtr& img);

	ros::NodeHandle nh_;
	ros::Subscriber img_sub_;
	ros::Publisher detect_pub_;
	DPMOCVCPULatentSvmDetector *cpu_detector_;
#if defined(HAS_GPU)
	DPMOCVGPULatentSvmDetector *gpu_detector_;
#endif
	double score_threshold_;
	double overlap_threshold_;
	double val_of_truncate_;
	int num_threads_;
	int lambda_;
	int side_length_;
	std::string model_file_;
	DetectType type_;
};

// Constructor
objectDetect::objectDetect(DetectType type) :
	cpu_detector_(NULL),
#if defined(HAS_GPU)
	gpu_detector_(NULL),
#endif
	type_(type)
{
	ros::NodeHandle private_nh_("~");

	private_nh_.param("overlap_threashold", overlap_threshold_, 0.5);
	private_nh_.param("num_threads", num_threads_, 8);
	private_nh_.param("lambda", lambda_, 10);
	private_nh_.param("side_length", side_length_, 8);
	private_nh_.param("val_of_tuncate", val_of_truncate_, 0.2);

	std::string default_model;
	switch (type) {
	case DetectType::CAR:
		default_model =  std::string(STR(MODEL_DIR) "car_2008.xml");
		break;
	case DetectType::PEDESTRIAN:
		default_model =  std::string(STR(MODEL_DIR) "person.xml");
		break;
	default:
		break;
	}

	private_nh_.param("model_file", model_file_, default_model);

	std::vector<std::string> model_filenames;
	model_filenames.clear();
	model_filenames.push_back(model_file_);

	cpu_detector_ = new DPMOCVCPULatentSvmDetector(model_filenames);
	if (!private_nh_.getParam("score_threshold", score_threshold_))
	{
		score_threshold_ = SCORE_THRESHOLD;
	}

#if defined(HAS_GPU)
	gpu_detector_ = new DPMOCVGPULatentSvmDetector(model_filenames, (float)score_threshold_);
#endif
}

// Destructor
objectDetect::~objectDetect()
{
	delete cpu_detector_;
#if defined(HAS_GPU)
	if(gpu_detector_ != NULL)
	{
		delete(gpu_detector_);
		gpu_detector_ = NULL;
	}
#endif
}

void objectDetect::run()
{
	img_sub_ = nh_.subscribe<sensor_msgs::Image>("image_raw", 1, &objectDetect::imageCallback, this);
	detect_pub_ = nh_.advertise<cv_tracker::image_obj>("image_obj", 1);
}

// Callback
void objectDetect::imageCallback(const sensor_msgs::ImageConstPtr& img)
{
	// transform image
	cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
	cv::Mat image = cv_image->image;

	std::vector<cv::LatentSvmDetector::ObjectDetection> detections;

	// detection main
#if defined(HAS_GPU)
	if (use_gpu) {
		gpu_detector_->detect(image, detections, (float)overlap_threshold_, num_threads_,
			lambda_, side_length_, (float)val_of_truncate_);
	} else {
#endif
		cpu_detector_->detect(image, detections, (float)overlap_threshold_, num_threads_,
			score_threshold_, lambda_, NUM_CELLS, NUM_BINS);
#if defined(HAS_GPU)
	}
#endif

	int num = detections.size();
	std::vector<int> corner_point_array(num * 4.0);
	std::vector<int> type_array(num, 0);

	cv_tracker::image_obj msg;
	msg.header = img->header;
	msg.type = type_ == DetectType::CAR ? "car" : "pedestrian";

	for(size_t i = 0; i < detections.size(); i++)
	{
		const cv::LatentSvmDetector::ObjectDetection& od = detections[i];
		cv_tracker::image_rect rect;

		type_array[i] = od.classID;
		rect.x = od.rect.x;
		rect.y = od.rect.y;
		rect.width = od.rect.width;
		rect.height = od.rect.height;
		rect.score = od.score;
		msg.obj.push_back(rect);
	}

	detect_pub_.publish(msg);
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "dpm_node");

	objectDetect car_detector(DetectType::CAR);
	objectDetect pedestrian_detector(DetectType::PEDESTRIAN);

	car_detector.run();
	pedestrian_detector.run();

	ros::spin();
	return 0;
}

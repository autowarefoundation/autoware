#include <string>

#include <autoware_msgs/traffic_light.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "Context.h"
#include "region_tlr_mxnet.h"


static bool show_superimpose_result = false;
static const std::string window_name = "superimpose result";

RegionTlrMxNetRosNode::RegionTlrMxNetRosNode() :
		image_topic_name_("/image_raw"),
		network_definition_file_name_(""),
		pretrained_model_file_name_(""),
		use_gpu_(false),
		gpu_id_(0),
		kAdvertiseInLatch_(true),
		kTrafficLightRed(0),
		kTrafficLightGreen(1),
		kTrafficLightUnknown(2),
		kStringRed("red signal"),
		kStringGreen("green signal"),
		kStringUnknown("")
{

}


RegionTlrMxNetRosNode::~RegionTlrMxNetRosNode()
{
}


void RegionTlrMxNetRosNode::RunRecognition()
{
	// Get execution parameters from ROS parameter server
	GetRosParam();

	BufferFile json_data(network_definition_file_name_);
	BufferFile param_data(pretrained_model_file_name_);

	if(json_data.GetLength() <= 0 ||
			param_data.GetLength() <= 0)
	{
		ROS_FATAL("Network definition JSON and/or Pre-trained model are empty.");
		return;
	}
	// Initialize recognizer
	recognizer.Init((const char*)json_data.GetBuffer(),
	                (const char*)param_data.GetBuffer(),
	                param_data.GetLength(),
	                use_gpu_,
	                gpu_id_);

	StartSubscribersAndPublishers();
	ROS_INFO("Node initialized, waiting for signals from feat_proj...");
	ros::spin();
}


void RegionTlrMxNetRosNode::ImageRawCallback(const sensor_msgs::Image &image)
{
	cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
	frame_ = cv_image->image.clone();

	frame_header_ = image.header;

}

void RegionTlrMxNetRosNode::RoiSignalCallback(const autoware_msgs::Signals::ConstPtr &extracted_pos)
{
	static ros::Time previous_timestamp;
	// If frame has not been prepared, abort this callback
	if (frame_.empty() ||
	    frame_header_.stamp == previous_timestamp)
	{
		std::cout << "No Image" << std::endl;
		return;
	}
	// Acquire signal posotion on the image
	Context::SetContexts(contexts_, extracted_pos, frame_.rows, frame_.cols);

	// Recognize the color of the traffic light
	for (Context &context: contexts_)
	{
		// for (unsigned int i = 0; i < contexts_.size(); i++) {
		//   Context& context = contexts_.at(i);
		if (context.topLeft.x > context.botRight.x)
		{
			continue;
		}

		cv::Mat roi = frame_(cv::Rect(context.topLeft, context.botRight)).clone();

		// Get current state of traffic light from current frame
		LightState current_state = recognizer.RecognizeLightState(roi, score_threshold_);

		// The state of the traffic light WON'T be changed
		// unless the new state is found at least change_state_threshold_ times
		DetermineState(current_state, context);
	}

	// Publish recognition result as some topic format
	PublishTrafficLight(contexts_);
	PublishString(contexts_);
	PublishMarkerArray(contexts_);
	PublishImage(contexts_);

	// Save timestamp of this frame so that same frame has never been process again
	previous_timestamp = frame_header_.stamp;
}

void RegionTlrMxNetRosNode::GetRosParam()
{
	ros::NodeHandle private_node_handle("~");

	private_node_handle.param<std::string>("image_raw_topic", image_topic_name_, "/image_raw");
	ROS_INFO("image_raw_topic: %s", image_topic_name_.c_str());
	private_node_handle.param<std::string>("network_definition_file", network_definition_file_name_, "");
	ROS_INFO("network_definition_file: %s", network_definition_file_name_.c_str());
	private_node_handle.param<std::string>("pretrained_model_file", pretrained_model_file_name_, "");
	ROS_INFO("pretrained_model_file: %s", pretrained_model_file_name_.c_str());
	private_node_handle.param<bool>("use_gpu", use_gpu_, false);
	ROS_INFO("use_gpu: %d", use_gpu_);
	private_node_handle.param<int>("gpu_id", gpu_id_, 0);
	ROS_INFO("gpu_id: %d", gpu_id_);
	private_node_handle.param<double>("score_threshold", score_threshold_, 0.9);
	ROS_INFO("score_threshold: %f", score_threshold_);
	private_node_handle.param<int>("change_state_threshold", change_state_threshold_, 2);
	ROS_INFO("change_state_threshold: %d", change_state_threshold_);

	// If network-definition-file or pretrained-model-file are not specified,
	// terminate program with error status
	if (network_definition_file_name_.empty())
	{
		ROS_FATAL("No Network Definition File was specified. Terminate program... ");
		exit(EXIT_FAILURE);
	}

	if (pretrained_model_file_name_.empty())
	{
		ROS_FATAL("No Pretrained Model File was specified. Terminate program... ");
		exit(EXIT_FAILURE);
	}
} // RegionTlrMxNetRosNode::ProcessRosParam()


void RegionTlrMxNetRosNode::StartSubscribersAndPublishers()
{
	ros::NodeHandle node_handle;

	// Register subscribers
	image_subscriber = node_handle.subscribe(image_topic_name_,
	                                         1,
	                                         &RegionTlrMxNetRosNode::ImageRawCallback,
	                                         this);
	roi_signal_subscriber = node_handle.subscribe("/roi_signal",
	                                              1,
	                                              &RegionTlrMxNetRosNode::RoiSignalCallback,
	                                              this);
	superimpose_sub = node_handle.subscribe("/config/superimpose",
                                          1,
                                          &RegionTlrMxNetRosNode::SuperimposeCb,
                                          this);

	// Register publishers
	signal_state_publisher = node_handle.advertise<autoware_msgs::traffic_light>("light_color", 1);
	signal_state_string_publisher = node_handle.advertise<std_msgs::String>("/sound_player", 1);
	marker_publisher = node_handle.advertise<visualization_msgs::MarkerArray>("tlr_result", 1, kAdvertiseInLatch_);
	superimpose_image_publisher = node_handle.advertise<sensor_msgs::Image>("tlr_superimpose_image", 1);

} // RegionTlrMxNetRosNode::StartSubscribersAndPublishers()

/*!
 * DetermineState works as a latch to reduce the chance of sudden changes in the state of the traffic light, caused by
 * mis classifications in the detector. To change the traffic light state, the new candidate should be found at
 * least kChangeStateThreshold times.
 * @param current_state the current state of the traffic light as reported by the classifier.
 * @param in_out_signal_context the object containing the data of the current Traffic Light instance.
 */
void RegionTlrMxNetRosNode::DetermineState(LightState in_current_state,
                                           Context& in_out_signal_context)
{
	//if reported state by classifier is different than the previously stored
	if (in_current_state != in_out_signal_context.lightState)
	{
		//and also different from the previous difference
		if (in_current_state != in_out_signal_context.newCandidateLightState)
		{
			//set classifier result as a candidate
			in_out_signal_context.newCandidateLightState = in_current_state;
			in_out_signal_context.stateJudgeCount = 0;
		}
		else
		{
			//if classifier returned the same result previously increase its confidence
			in_out_signal_context.stateJudgeCount++;
		}
	}
	//if new candidate has been found enough times, change state to the new candidate
	if (in_out_signal_context.stateJudgeCount >= change_state_threshold_)
	{
		in_out_signal_context.lightState = in_current_state;
	}

} // LightState RegionTlrMxNetRosNode::DetermineState()


void RegionTlrMxNetRosNode::PublishTrafficLight(std::vector<Context> contexts)
{
	autoware_msgs::traffic_light topic;
	static int32_t previous_state = kTrafficLightUnknown;
	topic.traffic_light = kTrafficLightUnknown;
	for (const auto ctx: contexts)
	{
		switch (ctx.lightState)
		{
			case GREEN:
				topic.traffic_light = kTrafficLightGreen;
				break;
			case YELLOW:
			case RED:
				topic.traffic_light = kTrafficLightRed;
				break;
			case UNDEFINED:
				topic.traffic_light = kTrafficLightUnknown;
				break;
		}

		// Publish the first state in contexts,
		// which has largest estimated radius of signal.
		// This program assume that the signal which has the largest estimated radius
		// equal the nearest one from camera.
		if (topic.traffic_light != kTrafficLightUnknown)
		{
			break;
		}
	}

	// If state changes from previous one, publish it
	if (topic.traffic_light != previous_state)
	{
		signal_state_publisher.publish(topic);
		previous_state = topic.traffic_light;
	}
} // void RegionTlrMxNetRosNode::PublishTrafficLight()


void RegionTlrMxNetRosNode::PublishString(std::vector<Context> contexts)
{
	std_msgs::String topic;
	static std::string previous_state = kStringUnknown;
	topic.data = kStringUnknown;
	for (const auto ctx: contexts)
	{
		switch (ctx.lightState)
		{
			case GREEN:
				topic.data = kStringGreen;
				break;
			case YELLOW:
			case RED:
				topic.data = kStringRed;
				break;
			case UNDEFINED:
				topic.data = kStringUnknown;
				break;
		}

		// Publish the first state in contexts,
		// which has largest estimated radius of signal.
		// This program assume that the signal which has the largest estimated radius
		// equal the nearest one from camera.
		if (topic.data != kStringUnknown)
		{
			break;
		}
	}

	// If state changes from previous one, publish it
	if (topic.data != previous_state)
	{
		signal_state_string_publisher.publish(topic);
		previous_state = topic.data;
	}
} // void RegionTlrMxNetRosNode::PublishString()


void RegionTlrMxNetRosNode::PublishMarkerArray(std::vector<Context> contexts)
{
	// Define color constants
	std_msgs::ColorRGBA color_black;
	color_black.r = 0.0f;
	color_black.g = 0.0f;
	color_black.b = 0.0f;
	color_black.a = 1.0f;

	std_msgs::ColorRGBA color_red;
	color_red.r = 1.0f;
	color_red.g = 0.0f;
	color_red.b = 0.0f;
	color_red.a = 1.0f;

	std_msgs::ColorRGBA color_yellow;
	color_yellow.r = 1.0f;
	color_yellow.g = 1.0f;
	color_yellow.b = 0.0f;
	color_yellow.a = 1.0f;

	std_msgs::ColorRGBA color_green;
	color_green.r = 0.0f;
	color_green.g = 1.0f;
	color_green.b = 0.0f;
	color_green.a = 1.0f;

	// publish all result as ROS MarkerArray
	for (const auto ctx: contexts)
	{
		visualization_msgs::MarkerArray signal_set;
		visualization_msgs::Marker red_light, yellow_light, green_light;

		// Set the frame ID
		red_light.header.frame_id = "map";
		yellow_light.header.frame_id = "map";
		green_light.header.frame_id = "map";

		// Set the namespace and ID for this markers
		red_light.ns = "tlr_result_red";
		red_light.id = ctx.signalID;

		yellow_light.ns = "tlr_result_yellow";
		yellow_light.id = ctx.signalID;

		green_light.ns = "tlr_result_green";
		green_light.id = ctx.signalID;

		// Set the markers type
		red_light.type = visualization_msgs::Marker::SPHERE;
		yellow_light.type = visualization_msgs::Marker::SPHERE;
		green_light.type = visualization_msgs::Marker::SPHERE;

		// Set the pose of the markers
		red_light.pose.position.x = ctx.redCenter3d.x;
		red_light.pose.position.y = ctx.redCenter3d.y;
		red_light.pose.position.z = ctx.redCenter3d.z;
		red_light.pose.orientation.x = 0.0;
		red_light.pose.orientation.y = 0.0;
		red_light.pose.orientation.z = 0.0;
		red_light.pose.orientation.w = 0.0;

		yellow_light.pose.position.x = ctx.yellowCenter3d.x;
		yellow_light.pose.position.y = ctx.yellowCenter3d.y;
		yellow_light.pose.position.z = ctx.yellowCenter3d.z;
		yellow_light.pose.orientation.x = 0.0;
		yellow_light.pose.orientation.y = 0.0;
		yellow_light.pose.orientation.z = 0.0;
		yellow_light.pose.orientation.w = 0.0;

		green_light.pose.position.x = ctx.greenCenter3d.x;
		green_light.pose.position.y = ctx.greenCenter3d.y;
		green_light.pose.position.z = ctx.greenCenter3d.z;
		green_light.pose.orientation.x = 0.0;
		green_light.pose.orientation.y = 0.0;
		green_light.pose.orientation.z = 0.0;
		green_light.pose.orientation.w = 0.0;

		// Set the scale of the markers. We assume lamp radius is 30cm in real world
		red_light.scale.x = 0.3;
		red_light.scale.y = 0.3;
		red_light.scale.z = 0.3;

		yellow_light.scale.x = 0.3;
		yellow_light.scale.y = 0.3;
		yellow_light.scale.z = 0.3;

		green_light.scale.x = 0.3;
		green_light.scale.y = 0.3;
		green_light.scale.z = 0.3;

		// Set the color for each marker
		switch (ctx.lightState)
		{
			case GREEN:
				red_light.color = color_black;
				yellow_light.color = color_black;
				green_light.color = color_green;
				break;
			case YELLOW:
				red_light.color = color_black;
				yellow_light.color = color_yellow;
				green_light.color = color_black;
				break;
			case RED:
				red_light.color = color_red;
				yellow_light.color = color_black;
				green_light.color = color_black;
				break;
			case UNDEFINED:
				red_light.color = color_black;
				yellow_light.color = color_black;
				green_light.color = color_black;
				break;
		}

		red_light.lifetime = ros::Duration(0.1);
		yellow_light.lifetime = ros::Duration(0.1);
		green_light.lifetime = ros::Duration(0.1);

		// Pack each light marker into one
		signal_set.markers.push_back(red_light);
		signal_set.markers.push_back(yellow_light);
		signal_set.markers.push_back(green_light);

		// Publish
		marker_publisher.publish(signal_set);
	}

} // void RegionTlrMxNetRosNode::PublishMarkerArray()


void RegionTlrMxNetRosNode::PublishImage(std::vector<Context> contexts)
{
	// Copy the frame image for output
	cv::Mat result_image = frame_.clone();

	// Define information for written label
	std::string label;
	const int kFontFace = cv::FONT_HERSHEY_COMPLEX_SMALL;
	const double kFontScale = 0.8;
	int font_baseline = 0;
	CvScalar label_color;

	for (const auto ctx: contexts_)
	{
		// Draw superimpose result on image
		circle(result_image, ctx.redCenter, ctx.lampRadius, CV_RGB(255, 0, 0), 1, 0);
		circle(result_image, ctx.yellowCenter, ctx.lampRadius, CV_RGB(255, 255, 0), 1, 0);
		circle(result_image, ctx.greenCenter, ctx.lampRadius, CV_RGB(0, 255, 0), 1, 0);

		// Draw recognition result on image
		switch (ctx.lightState)
		{
			case GREEN:
				label = "GREEN";
				label_color = CV_RGB(0, 255, 0);
				break;
			case YELLOW:
				label = "YELLOW";
				label_color = CV_RGB(255, 255, 0);
				break;
			case RED:
				label = "RED";
				label_color = CV_RGB(255, 0, 0);
				break;
			case UNDEFINED:
				label = "UNKNOWN";
				label_color = CV_RGB(0, 0, 0);
		}

		if (ctx.leftTurnSignal)
		{
			label += " LEFT";
		}
		if (ctx.rightTurnSignal)
		{
			label += " RIGHT";
		}
		//add lane # text
		label += " " + std::to_string(ctx.closestLaneId);

		cv::Point label_origin = cv::Point(ctx.topLeft.x, ctx.botRight.y + font_baseline);

		cv::putText(result_image, label, label_origin, kFontFace, kFontScale, label_color);
	}

	// Publish superimpose result image
	cv_bridge::CvImage converter;
	converter.header = frame_header_;
	converter.encoding = sensor_msgs::image_encodings::BGR8;
	converter.image = result_image;
	superimpose_image_publisher.publish(converter.toImageMsg());

} // void RegionTlrMxNetRosNode::PublishImage()

void RegionTlrMxNetRosNode::SuperimposeCb(const std_msgs::Bool::ConstPtr &config_msg)
{
	show_superimpose_result = config_msg->data;

	if (show_superimpose_result)
	{
		cv::namedWindow(window_name, cv::WINDOW_NORMAL);
		cv::startWindowThread();
	}

	if (!show_superimpose_result)
	{
		if (cvGetWindowHandle(window_name.c_str()) != NULL)
		{
			cv::destroyWindow(window_name);
			cv::waitKey(1);
		}
	}

} // void RegionTlrMxNetRosNode::SuperimposeCb()

int main(int argc, char *argv[])
{
	// Initialize ros node
	ros::init(argc, argv, "region_tlr_mxnet");

	// Create RegionTlrRosNode class object and do initialization
	RegionTlrMxNetRosNode region_tlr_mxnet_ros_node;

	// Start recognition process
	region_tlr_mxnet_ros_node.RunRecognition();

	return 0;
} // main()

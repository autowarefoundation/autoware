#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include "autoware_msgs/projection_matrix.h"

static cv::Mat CameraExtrinsicMat;
static cv::Mat CameraMat;
static cv::Mat DistCoeff;
static cv::Size ImageSize;
static std::string DistModel;

static ros::Publisher camera_info_pub;
static ros::Publisher projection_matrix_pub;

static bool isRegister_tf;
static bool isPublish_extrinsic;
static bool isPublish_cameraInfo;

static std::string camera_id_str;
static std::string target_frame_;

static bool instrinsics_parsed_;
static bool extrinsics_parsed_;

static sensor_msgs::CameraInfo camera_info_msg_;
static autoware_msgs::projection_matrix extrinsic_matrix_msg_;

void tfRegistration(const cv::Mat &camExtMat, const ros::Time &timeStamp)
{
	tf::Matrix3x3 rotation_mat;
	double roll = 0, pitch = 0, yaw = 0;
	tf::Quaternion quaternion;
	tf::Transform transform;
	static tf::TransformBroadcaster broadcaster;

	rotation_mat.setValue(camExtMat.at<double>(0, 0), camExtMat.at<double>(0, 1), camExtMat.at<double>(0, 2),
	                      camExtMat.at<double>(1, 0), camExtMat.at<double>(1, 1), camExtMat.at<double>(1, 2),
	                      camExtMat.at<double>(2, 0), camExtMat.at<double>(2, 1), camExtMat.at<double>(2, 2));

	rotation_mat.getRPY(roll, pitch, yaw, 1);

	quaternion.setRPY(roll, pitch, yaw);

	transform.setOrigin(
			tf::Vector3(camExtMat.at<double>(0, 3), camExtMat.at<double>(1, 3), camExtMat.at<double>(2, 3)));

	transform.setRotation(quaternion);

	broadcaster.sendTransform(tf::StampedTransform(transform, timeStamp, target_frame_, camera_id_str));
}

void projectionMatrix_sender(const cv::Mat &projMat, const ros::Time &timeStamp)
{
	if (!extrinsics_parsed_)
	{
		for (int row = 0; row < 4; row++)
		{
			for (int col = 0; col < 4; col++)
			{
				extrinsic_matrix_msg_.projection_matrix[row * 4 + col] = projMat.at<double>(row, col);
			}
		}
		extrinsics_parsed_ = true;
	}
	extrinsic_matrix_msg_.header.stamp = timeStamp;
	extrinsic_matrix_msg_.header.frame_id = camera_id_str;
	projection_matrix_pub.publish(extrinsic_matrix_msg_);
}

void cameraInfo_sender(const cv::Mat &camMat, const cv::Mat &distCoeff, const cv::Size &imgSize,
                       const std::string &distModel, const ros::Time &timeStamp)
{
	if (!instrinsics_parsed_)
	{
		for (int row = 0; row < 3; row++)
		{
			for (int col = 0; col < 3; col++)
			{
				camera_info_msg_.K[row * 3 + col] = camMat.at<double>(row, col);
			}
		}

		for (int row = 0; row < 3; row++)
		{
			for (int col = 0; col < 4; col++)
			{
				if (col == 3)
				{
					camera_info_msg_.P[row * 4 + col] = 0.0f;
				} else
				{
					camera_info_msg_.P[row * 4 + col] = camMat.at<double>(row, col);
				}
			}
		}

		for (int row = 0; row < distCoeff.rows; row++)
		{
			for (int col = 0; col < distCoeff.cols; col++)
			{
				camera_info_msg_.D.push_back(distCoeff.at<double>(row, col));
			}
		}
		camera_info_msg_.distortion_model = distModel;
		camera_info_msg_.height = imgSize.height;
		camera_info_msg_.width = imgSize.width;

		instrinsics_parsed_ = true;
	}
	camera_info_msg_.header.stamp = timeStamp;
	camera_info_msg_.header.frame_id = camera_id_str;


	camera_info_pub.publish(camera_info_msg_);
}

static void image_raw_cb(const sensor_msgs::Image &image_msg)
{
	// ros::Time timeStampOfImage = image_msg.header.stamp;

	ros::Time timeStampOfImage;
	timeStampOfImage.sec = image_msg.header.stamp.sec;
	timeStampOfImage.nsec = image_msg.header.stamp.nsec;

	/* create TF between velodyne and camera with time stamp of /image_raw */
	if (isRegister_tf)
	{
		tfRegistration(CameraExtrinsicMat, timeStampOfImage);
	}

	if (isPublish_cameraInfo)
	{
		cameraInfo_sender(CameraMat, DistCoeff, ImageSize, DistModel, timeStampOfImage);
	}
	if (isPublish_extrinsic)
	{
		projectionMatrix_sender(CameraExtrinsicMat, timeStampOfImage);
	}
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "calibration_publisher");
	ros::NodeHandle n;
	ros::NodeHandle private_nh("~");

	if (!private_nh.getParam("register_lidar2camera_tf", isRegister_tf))
	{
		isRegister_tf = true;
	}

	if (!private_nh.getParam("publish_extrinsic_mat", isPublish_extrinsic))
	{
		isPublish_extrinsic = true; /* publish extrinsic_mat in default */
	}

	if (!private_nh.getParam("publish_camera_info", isPublish_cameraInfo))
	{
		isPublish_cameraInfo = true; /* doesn't publish camera_info in default */
	}

	private_nh.param<std::string>("target_frame", target_frame_, "velodyne");
	ROS_INFO("target_frame: %s", target_frame_.c_str());

	if (argc < 2)
	{
		std::cout << "Usage: calibration_publisher <calibration-file>." << std::endl;
		return -1;
	}

	cv::FileStorage fs(argv[1], cv::FileStorage::READ);
	if (!fs.isOpened())
	{
		std::cout << "Cannot open " << argv[1] << std::endl;
		return -1;
	}

	fs["CameraExtrinsicMat"] >> CameraExtrinsicMat;
	fs["CameraMat"] >> CameraMat;
	fs["DistCoeff"] >> DistCoeff;
	fs["ImageSize"] >> ImageSize;
	fs["DistModel"] >> DistModel;

	std::string image_topic_name("/image_raw");
	std::string camera_info_name("/camera_info");
	std::string projection_matrix_name("/projection_matrix");

	instrinsics_parsed_ = false;
	extrinsics_parsed_ = false;

	std::string name_space_str = ros::this_node::getNamespace();
	camera_id_str = "camera";
	if (name_space_str != "/")
	{
		image_topic_name = name_space_str + image_topic_name;
		camera_info_name = name_space_str + camera_info_name;
		projection_matrix_name = name_space_str + projection_matrix_name;
		if (name_space_str.substr(0, 2) == "//")
		{
			/* if name space obtained by ros::this::node::getNamespace()
			   starts with "//", delete one of them */
			name_space_str.erase(name_space_str.begin());
		}
		camera_id_str = name_space_str;
	}

	ros::Subscriber image_sub;

	image_sub = n.subscribe(image_topic_name, 10, image_raw_cb);

	camera_info_pub = n.advertise<sensor_msgs::CameraInfo>(camera_info_name, 10, true);

	projection_matrix_pub = n.advertise<autoware_msgs::projection_matrix>(projection_matrix_name, 10, true);

	ros::spin();

	return 0;
}

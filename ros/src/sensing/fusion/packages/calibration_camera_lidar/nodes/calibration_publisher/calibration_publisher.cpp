#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <calibration_camera_lidar/projection_matrix.h>

static cv::Mat  CameraExtrinsicMat;
static cv::Mat  CameraMat;
static cv::Mat  DistCoeff;
static cv::Size ImageSize;

static ros::Publisher camera_info_pub;
static ros::Publisher projection_matrix_pub;

static bool isRegister_tf;
static bool isPublish_extrinsic;
static bool isPublish_cameraInfo;


void tfRegistration (const cv::Mat &camExtMat, const ros::Time& timeStamp)
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

  transform.setOrigin(tf::Vector3(camExtMat.at<double>(0, 3),
                                  camExtMat.at<double>(1, 3),
                                  camExtMat.at<double>(2, 3)));

  transform.setRotation(quaternion);


  broadcaster.sendTransform(tf::StampedTransform(transform, timeStamp, "velodyne", "camera"));
  //broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "velodyne", "camera"));
}


static void image_raw_cb(const sensor_msgs::Image& image_msg)
{
  //ros::Time timeStampOfImage = image_msg.header.stamp;

  ros::Time timeStampOfImage;
  timeStampOfImage.sec = image_msg.header.stamp.sec;
  timeStampOfImage.nsec = image_msg.header.stamp.nsec;


  /* create TF between velodyne and camera with time stamp of /image_raw */
  tfRegistration(CameraExtrinsicMat, timeStampOfImage);

}

void projectionMatrix_sender(const cv::Mat  &projMat)
{
	calibration_camera_lidar::projection_matrix projMsg;

	projMsg.header.frame_id="camera";

	for (int row=0; row<4; row++) {
	      for (int col=0; col<4; col++) {
	    	  	  projMsg.projection_matrix[row * 4 + col] = projMat.at<double>(row, col);

	      }
	}
	projection_matrix_pub.publish(projMsg);
}

void cameraInfo_sender(const cv::Mat  &camMat,
                       const cv::Mat  &distCoeff,
                       const cv::Size &imgSize)
{
  sensor_msgs::CameraInfo msg;

  msg.header.frame_id = "camera";

  msg.height = imgSize.height;
  msg.width  = imgSize.width;

  for (int row=0; row<3; row++) {
      for (int col=0; col<3; col++) {
        msg.K[row * 3 + col] = camMat.at<double>(row, col);
        }
    }

  for (int row=0; row<3; row++) {
    for (int col=0; col<4; col++) {
      if (col == 3) {
        msg.P[row * 4 + col] = 0.0f;
      } else {
        msg.P[row * 4 + col] = camMat.at<double>(row, col);
      }
    }
  }

  for (int row=0; row<distCoeff.rows; row++) {
    for (int col=0; col<distCoeff.cols; col++) {
      msg.D.push_back(distCoeff.at<double>(row, col));
    }
  }

  camera_info_pub.publish(msg);
}


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "calibration_publisher");
  ros::NodeHandle n;
  ros::NodeHandle private_nh("~");

  if (!private_nh.getParam("register_lidar2camera_tf", isRegister_tf)) {
    isRegister_tf = true;
  }

  if (!private_nh.getParam("publish_extrinsic_mat", isPublish_extrinsic)) {
    isPublish_extrinsic = true; /* publish extrinsic_mat in default */
  }

  if (!private_nh.getParam("publish_camera_info", isPublish_cameraInfo)) {
    isPublish_extrinsic = false; /* doesn't publish camera_info in default */
  }

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

  ros::Subscriber image_sub;
  if (isRegister_tf) {
    image_sub = n.subscribe("/image_raw", 10, image_raw_cb);
  }

  if (isPublish_cameraInfo) {
    camera_info_pub = n.advertise<sensor_msgs::CameraInfo>("/camera/camera_info", 10, true);
    cameraInfo_sender(CameraMat, DistCoeff, ImageSize);
  }

  if (isPublish_extrinsic) {
    projection_matrix_pub = n.advertise<calibration_camera_lidar::projection_matrix>("/projection_matrix", 10, true);
    projectionMatrix_sender(CameraExtrinsicMat);
  }

  ros::spin();

  return 0;

}

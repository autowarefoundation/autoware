#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

static ros::Publisher camera_info_pub;

void tfRegistration (const cv::Mat &camExtMat)
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

  broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "velodyne", "camera"));
}


void cameraInfo_sender(const cv::Mat  &camMat,
                       const cv::Mat  &disCoeff,
                       const cv::Size &imgSize)
{
  sensor_msgs::CameraInfo msg;

  msg.header.frame_id = "camera";
  //  msg.header.stamp    = ros::Time::now();

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

  camera_info_pub.publish(msg);
}


int main(int argc, char* argv[])
{
  cv::Mat  cameraExtrinsicMat;
  cv::Mat  cameraMat;
  cv::Mat  distCoeff;
  cv::Size imageSize;

  ros::init(argc, argv, "camera_info_publisher");
  ros::NodeHandle n;

  camera_info_pub = n.advertise<sensor_msgs::CameraInfo>("/camera/camera_info", 10);

  if (argc < 2)
    {
      std::cout << "Usage: camera_info_publisher <calibration-file>." << std::endl;
      return -1;
    }

  cv::FileStorage fs(argv[1], cv::FileStorage::READ);
  if (!fs.isOpened())
    {
      std::cout << "Cannot open " << argv[1] << std::endl;
      return -1;
    }

  fs["CameraExtrinsicMat"] >> cameraExtrinsicMat;
  fs["CameraMat"] >> cameraMat;
  fs["DistCoeff"] >> distCoeff;
  fs["ImageSize"] >> imageSize;

  while(ros::ok())
    {
      /* create tf between velodyne and camera */
      tfRegistration(cameraExtrinsicMat);

      /* publish camera_info */
      cameraInfo_sender(cameraMat, distCoeff, imageSize);
    }

}

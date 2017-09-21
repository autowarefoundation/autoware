#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <sensor_msgs/LaserScan.h>
#include <algorithm>
#include "scan2image.h"

#include "autoware_msgs/ScanImage.h"
#include "autoware_msgs/projection_matrix.h"
#include <sensor_msgs/CameraInfo.h>

#if 1 // AXE
#define XSTR(x) #x
#define STR(x) XSTR(x)
#endif

static cv::Mat cameraExtrinsicMat;
static cv::Mat cameraMat;
static cv::Mat distCoeff;
static cv::Size imageSize;

ros::Publisher transformed_point_data;
static bool isProjection;
static bool isIntrinsic;
Scan_image scan_image;

void trans_depth_points_to_image_points(Scan_points_dataset* scan_points_dataset, Image_points_dataset* image_points_dataset)
{
    float camera_x;
    float camera_y;
    float camera_z;
    int i;

    for(i = 0; i < (int)scan_points_dataset->scan_points.x.size(); i++) {

        /*
         * Coordinate transformation. Change from laser range finder coordinate to camera coordinate
         */
        camera_x = (cameraExtrinsicMat.at<double>(0,0) * scan_points_dataset->scan_points.x.at(i)*1000
                    + cameraExtrinsicMat.at<double>(0,1) * scan_points_dataset->scan_points.y.at(i)*1000
                    + cameraExtrinsicMat.at<double>(0,2) * scan_points_dataset->scan_points.z.at(i)*1000)
            + (cameraExtrinsicMat.at<double>(0,3));
        camera_y = (cameraExtrinsicMat.at<double>(1,0) * scan_points_dataset->scan_points.x.at(i)*1000
                    + cameraExtrinsicMat.at<double>(1,1) * scan_points_dataset->scan_points.y.at(i)*1000
                    + cameraExtrinsicMat.at<double>(1,2) * scan_points_dataset->scan_points.z.at(i)*1000)
            + (cameraExtrinsicMat.at<double>(1,3));
        camera_z = (cameraExtrinsicMat.at<double>(2,0) * scan_points_dataset->scan_points.x.at(i)*1000
                    + cameraExtrinsicMat.at<double>(2,1) * scan_points_dataset->scan_points.y.at(i)*1000
                    + cameraExtrinsicMat.at<double>(2,2) * scan_points_dataset->scan_points.z.at(i)*1000)
            + (cameraExtrinsicMat.at<double>(2,3));
        if (camera_z > 0.0) {
            /*
             * Projection transformation. Change from camera coordinate to image coordinate
             */
            image_points_dataset->image_points.x.push_back((camera_x * cameraMat.at<double>(0,0) / camera_z) + cameraMat.at<double>(0,2));
            image_points_dataset->image_points.y.push_back((camera_y * cameraMat.at<double>(1,1) / camera_z) + cameraMat.at<double>(1,2));
            /*
             * Calculate euclidean distance from the camera to objects
             */
            image_points_dataset->distance.push_back(sqrt(camera_x * camera_x + camera_y * camera_y + camera_z * camera_z) * 100); //unit of length is centimeter

            /*
             * Copy to intensity
             */
            if(!(scan_points_dataset->intensity.empty())){
                image_points_dataset->intensity.push_back(scan_points_dataset->intensity.at(i));
            }
        }
    }
}

static void projection_callback(const autoware_msgs::projection_matrix& msg)
{
    printf("projection\n");

	cameraExtrinsicMat = cv::Mat(4,4,CV_64F);
	for (int row=0; row<4; row++) {
		for (int col=0; col<4; col++) {
			cameraExtrinsicMat.at<double>(row, col) = msg.projection_matrix[row * 4 + col];
            printf("%f\t", cameraExtrinsicMat.at<double>(row, col));
		}
        printf("\n");
	}
    isProjection = true;
}

static void intrinsic_callback(const sensor_msgs::CameraInfo& msg)
{
    printf("intrinsic\n");

    if (!isIntrinsic || imageSize.height != msg.height || imageSize.width != msg.width) {
        if (isIntrinsic) {
            free(scan_image.distance);
            free(scan_image.intensity);
        }
        scan_image.distance = (float *)calloc(msg.height * msg.width, sizeof(float));
        scan_image.intensity = (float *)calloc(msg.height * msg.width, sizeof(float));
        scan_image.max_y = NO_DATA;
        scan_image.min_y = NO_DATA;

    }

	imageSize.height = msg.height;
	imageSize.width = msg.width;

	cameraMat = cv::Mat(3,3, CV_64F);
	for (int row=0; row<3; row++) {
		for (int col=0; col<3; col++) {
			cameraMat.at<double>(row, col) = msg.K[row * 3 + col];
            printf("%f\t", cameraMat.at<double>(row, col));
		}
        printf("\n");
	}

	distCoeff = cv::Mat(1,5,CV_64F);
	for (int col=0; col<5; col++) {
		distCoeff.at<double>(col) = msg.D[col];
	}
    isIntrinsic = true;
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    if (!(isIntrinsic && isProjection)){
        return;
    }

    static Scan_points_dataset scan_points_dataset;
    static Image_points_dataset image_points_dataset;
    int i;

//    ROS_INFO("angle_min[%f]\nangle_max:[%f]\nangle_increment:[%f]\ntime_increment:[%f]\nscan_time:[%f]\nrange_min:[%f]\nrange_max:[%f]\n", msg->angle_min * 180 / 3.141592, msg->angle_max * 180 / 3.141592, msg->angle_increment * 180 / 3.141592, msg->time_increment, msg->scan_time, msg->range_min, msg->range_max);

    /*
     * Initialize
     */
    scan_points_dataset.scan_points.x.resize(msg->ranges.size());
    scan_points_dataset.scan_points.y.resize(msg->ranges.size());
    scan_points_dataset.scan_points.z.resize(msg->ranges.size());
    scan_points_dataset.intensity.resize(msg->intensities.size());
    image_points_dataset.image_points.x.clear();
    image_points_dataset.image_points.y.clear();
    image_points_dataset.distance.clear();
    image_points_dataset.intensity.clear();

    /*
     * Change to three dimentional coordinate. And copy intensity
     */
    for(i = 0; i < (int)msg->ranges.size(); i++) {
        scan_points_dataset.scan_points.x.at(i) = msg->ranges.at(i) * sin(msg->angle_min + msg->angle_increment * i); //unit of length is meter
        scan_points_dataset.scan_points.y.at(i) = 0; //unit of length is meter
        scan_points_dataset.scan_points.z.at(i) = msg->ranges.at(i) * cos(msg->angle_min + msg->angle_increment * i); //unit of length is meter
        if(!(msg->intensities.empty())){
            scan_points_dataset.intensity.at(i) = msg->intensities.at(i);
        }
    }

    /*
     * Change from laser range finder coordinate to image coordinate
     */
    trans_depth_points_to_image_points(&scan_points_dataset, &image_points_dataset);

    /*
     * Judge out of image frame. And Determine max_y and min_y
     */
    for (i = 0; i < (int)image_points_dataset.image_points.x.size(); i++) {
        /* Judge NaN */
        if(isnan(image_points_dataset.image_points.x.at(i)) == 1 || isnan(image_points_dataset.image_points.y.at(i)) == 1) {
            std::cout <<"Not a Number is i:" << i << std::endl;
            continue;
        }
        /* Judge out of X-axis image */
        if(0 > (int)image_points_dataset.image_points.x.at(i) || (int)image_points_dataset.image_points.x.at(i) > imageSize.width - 1) {
            continue;
        }
        /* Judge out of Y-axis image */
        if(0 > (int)image_points_dataset.image_points.y.at(i) || (int)image_points_dataset.image_points.y.at(i) > imageSize.height - 1) {
            continue;
        }

        scan_image.distance[(int)image_points_dataset.image_points.x.at(i) * imageSize.height + (int)image_points_dataset.image_points.y.at(i)] = image_points_dataset.distance.at(i);

        if(!msg->intensities.empty()){
            scan_image.intensity[(int)image_points_dataset.image_points.x.at(i) * imageSize.height + (int)image_points_dataset.image_points.y.at(i)] = image_points_dataset.intensity.at(i);
        }

        if ((scan_image.max_y < (int)image_points_dataset.image_points.y.at(i)) || (scan_image.max_y == NO_DATA)) {
            scan_image.max_y = (int)image_points_dataset.image_points.y.at(i);
        } else if ((scan_image.min_y > (int)image_points_dataset.image_points.y.at(i)) || (scan_image.min_y == NO_DATA)) {
            scan_image.min_y = (int)image_points_dataset.image_points.y.at(i);
        }
    }

    /*
     * Create message(Topic)
     */
    autoware_msgs::ScanImage scan_image_msg;
    scan_image_msg.header = msg->header;
    scan_image_msg.distance.assign(scan_image.distance, scan_image.distance + imageSize.width * imageSize.height);
    scan_image_msg.intensity.assign(scan_image.intensity, scan_image.intensity + imageSize.width * imageSize.height);
    scan_image_msg.max_y = scan_image.max_y;
    scan_image_msg.min_y = scan_image.min_y;

    /*
     * Publish message(Topic)
     */
    transformed_point_data.publish(scan_image_msg);

    /*
     * Init zero
     */
    std::fill_n(scan_image.distance, imageSize.width * imageSize.height,0);
    std::fill_n(scan_image.intensity, imageSize.width * imageSize.height,0);
    scan_image.max_y = NO_DATA;
    scan_image.min_y = NO_DATA;
}

int main(int argc, char **argv)
{
    isProjection = false;
    isIntrinsic = false;
    ros::init(argc, argv, "scan2image");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("scan", 1, scanCallback);
    ros::Subscriber projection_sub = n.subscribe("projection_matrix", 1, projection_callback);
    ros::Subscriber intrinsic_sub = n.subscribe("camera/camera_info", 1, intrinsic_callback);
    transformed_point_data = n.advertise<autoware_msgs::ScanImage>("scan_image", 1);

    ros::spin();

    free(scan_image.distance);
    free(scan_image.intensity);
    return 0;
}

/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <fusion/fusion_func.h>
#include "autoware_msgs/ImageObjRanged.h"
#include "autoware_config_msgs/ConfigCarFusion.h"

static void publishTopic();
static ros::Publisher fused_objects;
static std_msgs::Header sensor_header;

bool ready_ = false;

static void DetectedObjectsCallback(const autoware_msgs::ImageObj& image_object)
{
    sensor_header = image_object.header;
    setDetectedObjects(image_object);
    if (ready_) {
        fuse();
        publishTopic();
        ready_ = false;
        return;
    }
    ready_ = true;
}

/*static void ScanImageCallback(const scan2image::ScanImage& scan_image)
{
	setScanImage(scan_image);
	sensor_header = scan_image.header;

	calcDistance();
	publishTopic();
}*/

static void PointsImageCallback(const autoware_msgs::PointsImage& points_image)
{
    sensor_header = points_image.header;
    setPointsImage(points_image);
    if (ready_) {
		fuse();
		publishTopic();
        ready_ = false;
        return;
    }
    ready_ = true;
}

static void publishTopic()
{
	/*
	 * Publish topic(obj position ranged).
	 */
	autoware_msgs::ImageObjRanged fused_objects_msg;
	fused_objects_msg.header = sensor_header;

	fused_objects_msg.type = getObjectsType();
	fused_objects_msg.obj = getObjectsRectRanged();
	fused_objects.publish(fused_objects_msg);
}

static void config_cb(const autoware_config_msgs::ConfigCarFusion::ConstPtr& param)
{
	setParams(param->min_low_height,
			param->max_low_height,
			param->max_height,
			param->min_points,
			param->dispersion);
}

int main(int argc, char **argv)
{
	init();
	ros::init(argc, argv, "range_fusion");

	ros::NodeHandle n;
	ros::NodeHandle private_nh("~");

	std::string image_topic;
	std::string points_topic;
	if (private_nh.getParam("image_node", image_topic))
	{
		ROS_INFO("Setting image node to %s", image_topic.c_str());
	}
	else
	{
		ROS_INFO("No image node received, defaulting to image_obj, you can use _image_node:=YOUR_TOPIC");
		image_topic = "image_obj";
	}
	if (private_nh.getParam("points_node", points_topic))
	{
		ROS_INFO("Setting points node to %s", points_topic.c_str());
	}
	else
	{
		ROS_INFO("No points node received, defaulting to vscan_image, you can use _points_node:=YOUR_TOPIC");
		points_topic = "/vscan_image";
	}

//	ros::Subscriber image_obj_sub = n.subscribe("/obj_car/image_obj", 1, DetectedObjectsCallback);
	ros::Subscriber image_obj_sub = n.subscribe(image_topic, 1, DetectedObjectsCallback);
	//ros::Subscriber scan_image_sub = n.subscribe("scan_image", 1, ScanImageCallback);
	ros::Subscriber points_image_sub =n.subscribe(points_topic, 1, PointsImageCallback);
#if _DEBUG
	ros::Subscriber image_sub = n.subscribe(IMAGE_TOPIC, 1, IMAGE_CALLBACK);
#endif
	fused_objects = n.advertise<autoware_msgs::ImageObjRanged>("image_obj_ranged", 1);

	ros::Subscriber config_subscriber;
	std::string config_topic("/config");
	config_topic += ros::this_node::getNamespace() + "/fusion";
	config_subscriber = n.subscribe(config_topic, 1, config_cb);

	ros::spin();
	destroy();

	return 0;
}

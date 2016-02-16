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

#include <ros/ros.h>
#include <cv_tracker/image_obj_ranged.h>
#include <std_msgs/Header.h>
#include <fusion_func.h>
#include <runtime_manager/ConfigCarFusion.h>

static void publishTopic();
static ros::Publisher fused_objects;
static std_msgs::Header sensor_header;

bool ready_ = false;

static void DetectedObjectsCallback(const cv_tracker::image_obj& image_object)
{
    if (ready_) {
		sensor_header = image_object.header;
        setDetectedObjects(image_object);
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

static void PointsImageCallback(const points2image::PointsImage& points_image)
{
    if (ready_) {
		sensor_header = points_image.header;
		setPointsImage(points_image);
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
	cv_tracker::image_obj_ranged fused_objects_msg;
	fused_objects_msg.header = sensor_header;

	fused_objects_msg.type = getObjectsType();
	fused_objects_msg.obj = getObjectsRectRanged();
	fused_objects.publish(fused_objects_msg);
}

static void config_cb(const runtime_manager::ConfigCarFusion::ConstPtr& param)
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
	fused_objects = n.advertise<cv_tracker::image_obj_ranged>("image_obj_ranged", 1);

	ros::Subscriber config_subscriber;
	std::string config_topic("/config");
	config_topic += ros::this_node::getNamespace() + "/fusion";
	config_subscriber = n.subscribe(config_topic, 1, config_cb);

	ros::spin();
	destroy();

	return 0;
}

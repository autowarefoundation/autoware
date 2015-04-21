/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <car_detector/FusedObjects.h>
#include <std_msgs/Header.h>
#include <fusion_func.h>

static void publishTopic();
static ros::Publisher fused_objects;
static std_msgs::Header sensor_header;

static void ImageObjectsCallback(const dpm::ImageObjects& image_object)
{
    setImageObjects(image_object);

    calcDistance();
    publishTopic();
}

static void ScanImageCallback(const scan2image::ScanImage& scan_image)
{
    setScanImage(scan_image);
    sensor_header = scan_image.header;

    calcDistance();
    publishTopic();
}

static void PointsImageCallback(const points2image::PointsImage& points_image)
{
    setPointsImage(points_image);
    sensor_header = points_image.header;

    calcDistance();
    publishTopic();
}

static void publishTopic()
{
    /*
     * Publish topic(Car position xyz).
     */
    car_detector::FusedObjects fused_objects_msg;
    fused_objects_msg.header = sensor_header;
    fused_objects_msg.car_num = getObjectNum();
    fused_objects_msg.corner_point = getCornerPoint();
    fused_objects_msg.distance = getDistance();
    fused_objects.publish(fused_objects_msg);
}

int main(int argc, char **argv)
{
    init();
    ros::init(argc, argv, "car_fusion");

    ros::NodeHandle n;

    ros::Subscriber car_pixel_xy_sub = n.subscribe("car_pixel_xy_tracked", 1, ImageObjectsCallback);
    ros::Subscriber scan_image_sub = n.subscribe("scan_image", 1, ScanImageCallback);
    ros::Subscriber points_image_sub =n.subscribe("vscan_image", 1, PointsImageCallback);
#if _DEBUG
    ros::Subscriber image_sub = n.subscribe(IMAGE_TOPIC, 1, IMAGE_CALLBACK);
#endif
    fused_objects = n.advertise<car_detector::FusedObjects>("car_pixel_xyz", 1);

    ros::spin();
    destroy();

    return 0;
}

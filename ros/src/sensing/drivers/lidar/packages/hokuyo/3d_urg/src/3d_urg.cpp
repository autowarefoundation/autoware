/*
 * Copyright (c) 2015, <ATR> Atsushi Watanabe,  <Ritsumeikan University> Yukihiro Saito
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
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

#include <boost/bind.hpp>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Imu.h>
#include "vssp.hpp"

#ifndef _INVALID_RANGE
#define _INVALID_RANGE 1
#endif

class hokuyo3d_node
{
public:
    void cbPoint_field(
        const vssp::header &header,
        const vssp::range_header &range_header,
        const vssp::range_index &range_index,
        const boost::shared_array<vssp::xyzi> &points,
        const std::chrono::microseconds &delayRead,
        const vssp::data_range_size &data_range_size)
        {
            if(timestampBase == ros::Time(0)) return;
            if(cloud2.data.size() == 0)
            {
                cloud2.header.frame_id = frame_id;
                cloud2.header.stamp = timestampBase + ros::Duration(range_header.line_head_timestamp_ms * 0.001);
                ping();
            }

            cloud2.data.resize(((data_range_size.necho + points_size) * cloud2.point_step));

#if _INVALID_RANGE
            // Check invalid rang
            int i, invalid;
            for(i = 0, invalid = 0; i < data_range_size.necho; i++)
            {
                double distance= sqrt(points[i].x * points[i].x + points[i].y * points[i].y);
                if (distance < invalid_range){
                    invalid++;
                    std::vector<uint8_t>::iterator it = cloud2.data.end();
                    cloud2.data.erase(it - cloud2.point_step, it);
                    continue;
                }
                // Pack scan data
                memcpy (&cloud2.data[(i + points_size) * cloud2.point_step + cloud2.fields[0].offset], &points[i], sizeof (float) * cloud2.fields.size());
            }
            points_size += data_range_size.necho - invalid;
#else
            // Pack scan data
            memcpy (&cloud2.data[points_size * cloud2.point_step], &points[0], sizeof(float) * cloud2.fields.size() * data_range_size.necho);
            points_size += data_range_size.necho;
#endif
            // Publish frame
            if(range_header.frame != frame)
            {
                cloud2.width = points_size;
                cloud2.row_step = cloud2.point_step * cloud2.width;
                pubPc2.publish(cloud2);

                field = range_header.field;
                frame = range_header.frame;
                points_size = 0;
                cloud2.data.clear();
            }

        };

    void cbPoint_frame(
        const vssp::header &header,
        const vssp::range_header &range_header,
        const vssp::range_index &range_index,
        const boost::shared_array<vssp::xyzi> &points,
        const std::chrono::microseconds &delayRead,
        const vssp::data_range_size &data_range_size)
        {
            if(timestampBase == ros::Time(0)) return;
            if(cloud2.data.size() == 0)
            {
                cloud2.header.frame_id = frame_id;
                cloud2.header.stamp = timestampBase + ros::Duration(range_header.line_head_timestamp_ms * 0.001);
                ping();
            }

            cloud2.data.resize(((data_range_size.necho + points_size) * cloud2.point_step));

#if _INVALID_RANGE
            // Check invalid rang
            int i, invalid;
            for(i = 0, invalid = 0; i < data_range_size.necho; i++)
            {
                double distance= sqrt(points[i].x * points[i].x + points[i].y * points[i].y);
                if (distance < invalid_range){
                    invalid++;
                    std::vector<uint8_t>::iterator it = cloud2.data.end();
                    cloud2.data.erase(it - cloud2.point_step, it);
                    continue;
                }
                // Pack scan data
                memcpy (&cloud2.data[(i + points_size) * cloud2.point_step + cloud2.fields[0].offset], &points[i], sizeof (float) * cloud2.fields.size());
            }
            points_size += data_range_size.necho - invalid;
#else
            // Pack scan data
            memcpy (&cloud2.data[points_size * cloud2.point_step], &points[0], sizeof(float) * cloud2.fields.size() * data_range_size.necho);
            points_size += data_range_size.necho;
#endif
            // Publish frame
            if(range_header.field != field ||
               range_header.frame != frame)
            {
                cloud2.width = points_size;
                cloud2.row_step = cloud2.point_step * cloud2.width;
                pubPc2.publish(cloud2);

                field = range_header.field;
                frame = range_header.frame;
                points_size = 0;
                cloud2.data.clear();
            }

        };

    void cbPing(const vssp::header &header, const std::chrono::microseconds &delayRead)
        {
            ros::Time now = ros::Time::now() - ros::Duration(delayRead.count() * 0.001 * 0.001);
            ros::Duration delay = ((now - timePing)
                                   - ros::Duration(header.send_time_ms * 0.001 - header.received_time_ms * 0.001)) * 0.5;
            ros::Time base = timePing + delay - ros::Duration(header.received_time_ms * 0.001);
            if(timestampBase == ros::Time(0)) timestampBase = base;
            else timestampBase += (base - timestampBase) * 0.01;
        }
    void cbAux(
        const vssp::header &header,
        const vssp::aux_header &aux_header,
        const boost::shared_array<vssp::aux> &auxs,
        const std::chrono::microseconds &delayRead)
        {
            if(timestampBase == ros::Time(0)) return;
            ros::Time stamp = timestampBase + ros::Duration(aux_header.timestamp_ms * 0.001);

            if((aux_header.data_bitfield & (vssp::AX_MASK_ANGVEL | vssp::AX_MASK_LINACC))
               == (vssp::AX_MASK_ANGVEL | vssp::AX_MASK_LINACC))
            {
                imu.header.frame_id = frame_id;
                imu.header.stamp = stamp;
                for(int i = 0; i < aux_header.data_count; i ++)
                {
                    imu.orientation_covariance[0] = -1.0;
                    imu.angular_velocity.x = auxs[i].ang_vel.x;
                    imu.angular_velocity.y = auxs[i].ang_vel.y;
                    imu.angular_velocity.z = auxs[i].ang_vel.z;
                    imu.linear_acceleration.x = auxs[i].lin_acc.x;
                    imu.linear_acceleration.y = auxs[i].lin_acc.y;
                    imu.linear_acceleration.z = auxs[i].lin_acc.z;
                    pubImu.publish(imu);
                    imu.header.stamp += ros::Duration(aux_header.data_ms * 0.001);
                }
            }
            if((aux_header.data_bitfield & vssp::AX_MASK_MAG) == vssp::AX_MASK_MAG )
            {
                mag.header.frame_id = frame_id;
                mag.header.stamp = stamp;
                for(int i = 0; i < aux_header.data_count; i ++)
                {
                    mag.magnetic_field.x = auxs[i].mag.x;
                    mag.magnetic_field.y = auxs[i].mag.y;
                    mag.magnetic_field.z = auxs[i].mag.z;
                    pubMag.publish(mag);
                    mag.header.stamp += ros::Duration(aux_header.data_ms * 0.001);
                }
            }
        };
    void cbConnect(bool success)
        {
            if(success)
            {
                ROS_INFO("Connection established");
                ping();
                driver.setInterlace(interlace);
                driver.requestHorizontalTable();
                driver.requestVerticalTable();
                driver.requestData(true, true);
                driver.requestAuxData();
                driver.receivePackets();
                ROS_INFO("Communication started");
            }
            else
            {
                ROS_ERROR("Connection failed");
            }
        };
    hokuyo3d_node() :
        nh("~"),
        timestampBase(0)
        {
            nh.param("interlace", interlace, 4);
            nh.param("ip", ip, std::string("192.168.0.10"));
            nh.param("port", port, 10940);
            nh.param("frame_id", frame_id, std::string("3d_urg"));
#if _INVALID_RANGE
            nh.param("invalid_range", invalid_range, 0.0);
#endif
            nh.param("interlaced_cycle", interlaced_cycle, true);
            pubPc2 = nh.advertise<sensor_msgs::PointCloud2>("hokuyo_cloud2", 5);
            pubImu = nh.advertise<sensor_msgs::Imu>("imu", 5);
            pubMag = nh.advertise<sensor_msgs::MagneticField>("mag", 5);

            driver.setTimeout(2.0);
            ROS_INFO("Connecting to %s", ip.c_str());
            driver.connect(ip.c_str(), port,
                           boost::bind(&hokuyo3d_node::cbConnect, this, _1));
            if(interlaced_cycle) {
                driver.registerCallback(
                    boost::bind(&hokuyo3d_node::cbPoint_field, this, _1, _2, _3, _4, _5, _6));
            } else {
                driver.registerCallback(
                    boost::bind(&hokuyo3d_node::cbPoint_frame, this, _1, _2, _3, _4, _5, _6));
            }
            driver.registerAuxCallback(
                boost::bind(&hokuyo3d_node::cbAux, this, _1, _2, _3, _4));
            driver.registerPingCallback(
                boost::bind(&hokuyo3d_node::cbPing, this, _1, _2));
            field = 0;
            frame = 0;
            points_size = 0;

            cloud2.height = 1;
            cloud2.fields.resize(4);
            cloud2.fields[0].name = "x";
            cloud2.fields[1].name = "y";
            cloud2.fields[2].name = "z";
            int offset = 0;
            // All offsets are *4, as all field data types are float32
            for (size_t d = 0; d < cloud2.fields.size (); ++d, offset += 4)
            {
                cloud2.fields[d].offset = offset;
                cloud2.fields[d].datatype = sensor_msgs::PointField::FLOAT32;
                cloud2.fields[d].count  = 1;
            }
            cloud2.point_step = offset;
            cloud2.fields[3].name = "intensity";
            cloud2.is_bigendian = false;
            cloud2.is_dense     = false;
        };
    ~hokuyo3d_node()
        {
            driver.requestAuxData(false);
            driver.requestData(true, false);
            driver.requestData(false, false);
            driver.poll();
            ROS_INFO("Communication stoped");
        };
    bool poll()
        {
            if(driver.poll())
            {
                return true;
            }
            ROS_ERROR("Connection closed");
            return false;
        };
    void ping()
        {
            driver.requestPing();
            timePing = ros::Time::now();
        };
private:
    ros::NodeHandle nh;
    ros::Publisher pubPc;
    ros::Publisher pubPc2;
    ros::Publisher pubImu;
    ros::Publisher pubMag;
    vssp::vsspDriver driver;
    sensor_msgs::PointCloud2 cloud2;
    sensor_msgs::Imu imu;
    sensor_msgs::MagneticField mag;

    ros::Time timePing;
    ros::Time timestampBase;

    int field;
    int frame;
    int points_size;
#if _INVALID_RANGE
    double invalid_range;
#endif

    std::string ip;
    int port;
    int interlace;
    std::string frame_id;
    bool interlaced_cycle;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "3d_urg");
    hokuyo3d_node node;

    ros::Rate wait(200);

    while (ros::ok())
    {
        if(!node.poll()) break;
        ros::spinOnce();
        wait.sleep();
    }

    return 1;
}

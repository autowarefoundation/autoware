/*
 *  Copyright (c) 2017, Tier IV, Inc.
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

#ifndef NDT_SLAM_CORE_H
#define NDT_SLAM_CORE_H

#include <string>
#include <memory>

#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <autoware_msgs/ConfigNdtSlam.h>

#include <points_localizer/ndt/ndt_slam_pcl_omp.h>
#include <points_localizer/ndt/ndt_slam_pcl_anh.h>
#include <points_localizer/ndt/ndt_slam_pcl_anh_gpu.h>
#include <points_localizer/ndt/ndt_slam_pcl.h>
#include <points_localizer/reliability/slam_reliability.h>

class NdtSlam
{
    using PointSource = pcl::PointXYZI;
    using PointTarget = pcl::PointXYZI;
    using SyncPolicyPoints = message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2>;

    enum class MethodType
    {
      PCL_GENERIC = 0,
      PCL_ANH = 1,
      PCL_ANH_GPU = 2,
      PCL_OPENMP = 3,
    };

    public:
        NdtSlam(ros::NodeHandle nh, ros::NodeHandle private_nh);

    private:
        void configCallback(const autoware_msgs::ConfigNdtSlam::ConstPtr& config_msg_ptr);
        void pointsMapUpdatedCallback(const sensor_msgs::PointCloud2::ConstPtr& pointcloud2_msg_ptr);
        void manualPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_cov_msg_ptr);
        void staticPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg_ptr);
        void gnssPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg_ptr);
        void odometryCallback(const nav_msgs::Odometry::ConstPtr& odom_msg_ptr);
        void pointsRawAndFilterdCallback(const sensor_msgs::PointCloud2::ConstPtr& points_raw_msg_ptr, const sensor_msgs::PointCloud2::ConstPtr& points_filtered_msg_ptr);

        void publishPosition(ros::Time time_stamp);
        void publishPointsMap(ros::Time time_stamp);
        void publishNDVoxelMap(ros::Time time_stamp);
        void publishLocalizerStatus(ros::Time time_stamp);

        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;

        ros::Publisher points_map_region_pub_;
        ros::Publisher localizer_pose_pub_;
        ros::Publisher localizer_twist_pub_;
        ros::Publisher localizer_score_pub_;
        ros::Publisher localizer_score_ave_pub_;
        ros::Publisher localizer_score_var_pub_;
        ros::Publisher ndt_marker_pub_;

        ros::Subscriber config_sub_;
        ros::Subscriber points_map_updated_sub_;
        ros::Subscriber manual_pose_sub_;
        ros::Subscriber static_pose_sub_;
        ros::Subscriber gnss_pose_sub_;
        ros::Subscriber odom_sub_;

        std::unique_ptr< message_filters::Subscriber<sensor_msgs::PointCloud2> > points_raw_sub_;
        std::unique_ptr< message_filters::Subscriber<sensor_msgs::PointCloud2> > points_filtered_sub_;
        std::unique_ptr< message_filters::Synchronizer<SyncPolicyPoints> > points_synchronizer_;

        tf::TransformBroadcaster tf_broadcaster_;
        tf::TransformListener tf_listener_;

        std::unique_ptr< LibNdtSlamBase<PointSource, PointTarget> > localizer_ptr_;
        LibSlamReliability reliability_;

        MethodType method_type_;
        Eigen::Matrix4f tf_btol_;
        bool with_mapping_;
        bool init_pos_gnss_;
        bool use_odometry_;
        std::string sensor_frame_;
        std::string base_link_frame_;
        std::string map_frame_;

};

#endif

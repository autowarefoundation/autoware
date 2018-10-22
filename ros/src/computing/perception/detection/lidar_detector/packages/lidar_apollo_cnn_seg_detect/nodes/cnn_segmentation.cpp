#include "cnn_segmentation.h"

#include <chrono>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

CNNSegmentation::CNNSegmentation() : nh_()
{
    // points_sub_ = nh_.subscribe("/points_raw", 1, &CNNSegmentation::pointsCallback, this);
    // points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/detection/colored_cloud", 1);
}

bool CNNSegmentation::init()
{
    // std::string config_file;
    std::string proto_file;
    std::string weight_file;

    ros::NodeHandle private_node_handle("~");//to receive args

    if (private_node_handle.getParam("network_definition_file", proto_file))
    {
        ROS_INFO("[%s] network_definition_file: %s", __APP_NAME__, proto_file.c_str());
    } else
    {
        ROS_INFO("[%s] No Network Definition File was received. Finishing execution.", __APP_NAME__);
        return false;
    }
    if (private_node_handle.getParam("pretrained_model_file", weight_file))
    {
        ROS_INFO("[%s] Pretrained Model File: %s", __APP_NAME__, weight_file.c_str());
    } else
    {
        ROS_INFO("[%s] No Pretrained Model File was received. Finishing execution.", __APP_NAME__);
        return false;
    }


    private_node_handle.param<std::string>("points_src", topic_src_, "points_raw");
    ROS_INFO("points_src: %s", topic_src_.c_str());

    private_node_handle.param<double>("range", range_, 60.);
    ROS_INFO("[%s] Pretrained Model File: %.2f", __APP_NAME__, range_);

    private_node_handle.param<double>("score_threshold", score_threshold_, 0.6);
    ROS_INFO("[%s] score_threshold: %.2f", __APP_NAME__, score_threshold_);

    private_node_handle.param<int>("width", width_, 512);
    ROS_INFO("[%s] width: %d", __APP_NAME__, width_);

    private_node_handle.param<int>("height", height_, 512);
    ROS_INFO("[%s] height: %d", __APP_NAME__, height_);

/// Instantiate Caffe net
#ifndef USE_CAFFE_GPU
    caffe::Caffe::set_mode(caffe::Caffe::CPU);
#else
    int gpu_id = 0;
    caffe::Caffe::SetDevice(gpu_id);
    caffe::Caffe::set_mode(caffe::Caffe::GPU);
    caffe::Caffe::DeviceQuery();
#endif

    caffe_net_.reset(new caffe::Net<float>(proto_file, caffe::TEST));
    caffe_net_->CopyTrainedLayersFrom(weight_file);


    std::string instance_pt_blob_name = "instance_pt";
    instance_pt_blob_ = caffe_net_->blob_by_name(instance_pt_blob_name);
    CHECK(instance_pt_blob_ != nullptr) << "`" << instance_pt_blob_name
                                        << "` not exists!";

    std::string category_pt_blob_name = "category_score";
    category_pt_blob_ = caffe_net_->blob_by_name(category_pt_blob_name);
    CHECK(category_pt_blob_ != nullptr) << "`" << category_pt_blob_name
                                        << "` not exists!";

    std::string confidence_pt_blob_name = "confidence_score";
    confidence_pt_blob_ = caffe_net_->blob_by_name(confidence_pt_blob_name);
    CHECK(confidence_pt_blob_ != nullptr) << "`" << confidence_pt_blob_name
                                          << "` not exists!";

    std::string height_pt_blob_name = "height_pt";
    height_pt_blob_ = caffe_net_->blob_by_name(height_pt_blob_name);
    CHECK(height_pt_blob_ != nullptr) << "`" << height_pt_blob_name
                                      << "` not exists!";

    std::string feature_blob_name = "data";
    feature_blob_ = caffe_net_->blob_by_name(feature_blob_name);
    CHECK(feature_blob_ != nullptr) << "`" << feature_blob_name
                                    << "` not exists!";

    std::string class_pt_blob_name = "class_score";
    class_pt_blob_ = caffe_net_->blob_by_name(class_pt_blob_name);
    CHECK(class_pt_blob_ != nullptr) << "`" << class_pt_blob_name
                                     << "` not exists!";

    cluster2d_.reset(new Cluster2D());
    if (!cluster2d_->init(height_, width_, range_))
    {
        std::cout << "Fail to Init cluster2d for CNNSegmentation" << std::endl;
    }

    feature_generator_.reset(new FeatureGenerator());
    if (!feature_generator_->init(feature_blob_.get()))
    {
        std::cout << "Fail to Init feature generator for CNNSegmentation" << std::endl;
        return false;
    }

    return true;
}

bool CNNSegmentation::segment(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_ptr,
                              const pcl::PointIndices &valid_idx,
                              autoware_msgs::DetectedObjectArray *objects)
{
    int num_pts = static_cast<int>(pc_ptr->points.size());
    if (num_pts == 0)
    {
        std::cout << "None of input points, return directly." << std::endl;
        return true;
    }

    feature_generator_->generate(pc_ptr);

// network forward process
    caffe::Caffe::set_mode(caffe::Caffe::GPU);
    caffe_net_->Forward();

    // clutser points and construct segments/objects
    float objectness_thresh = 0.5;
    bool use_all_grids_for_clustering = true;
    cluster2d_->cluster(*category_pt_blob_, *instance_pt_blob_, pc_ptr,
                        valid_idx, objectness_thresh,
                        use_all_grids_for_clustering);
    cluster2d_->filter(*confidence_pt_blob_, *height_pt_blob_);
    cluster2d_->classify(*class_pt_blob_);
    float confidence_thresh = score_threshold_;
    float height_thresh = 0.5;
    int min_pts_num = 3;
    cluster2d_->getObjects(confidence_thresh, height_thresh, min_pts_num,
                           objects, message_header_);
    return true;
}

void CNNSegmentation::test_run()
{
    std::string in_pcd_file = "/home/kosuke/apollo/modules/perception/data/cnnseg_test/uscar_12_1470770225_1470770492_1349.pcd";
    // apollo::PointCloudPtr in_pc;
    pcl::PointCloud<pcl::PointXYZI>::Ptr in_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile(in_pcd_file, *in_pc_ptr);


    pcl::PointIndices valid_idx;
    auto &indices = valid_idx.indices;
    indices.resize(in_pc_ptr->size());
    std::iota(indices.begin(), indices.end(), 0);

    autoware_msgs::DetectedObjectArray objects;
    init();
    segment(in_pc_ptr, valid_idx, &objects);


}

void CNNSegmentation::run()
{
    init();

    points_sub_ = nh_.subscribe(topic_src_, 1, &CNNSegmentation::pointsCallback, this);
    points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/detection/lidar_detector/points_cluster", 1);
    markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/detection/lidar_detector/objects_markers", 1);
    objects_pub_ = nh_.advertise<autoware_msgs::DetectedObjectArray>("/detection/lidar_detector/objects", 1);
    boxes_pub_ = nh_.advertise<jsk_recognition_msgs::BoundingBoxArray>("/detection/lidar_detector/objects_boxes", 1);
}

void CNNSegmentation::pointsCallback(const sensor_msgs::PointCloud2 &msg)
{
    std::chrono::system_clock::time_point start, end;
    start = std::chrono::system_clock::now();

    pcl::PointCloud<pcl::PointXYZI>::Ptr in_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(msg, *in_pc_ptr);
    pcl::PointIndices valid_idx;
    auto &indices = valid_idx.indices;
    indices.resize(in_pc_ptr->size());
    std::iota(indices.begin(), indices.end(), 0);
    message_header_  = msg.header;

    autoware_msgs::DetectedObjectArray objects;
    objects.header = message_header_;
    segment(in_pc_ptr, valid_idx, &objects);

    pubColoredPoints(objects);

    visualization_msgs::MarkerArray detection_markers = ObjectsToMarkers(objects);
    jsk_recognition_msgs::BoundingBoxArray objects_boxes = ObjectsToBoxes(objects);

    markers_pub_.publish(detection_markers);
    objects_pub_.publish(objects);
    boxes_pub_.publish(objects_boxes);

    end = std::chrono::system_clock::now();
    double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
}

jsk_recognition_msgs::BoundingBoxArray
CNNSegmentation::ObjectsToBoxes(const autoware_msgs::DetectedObjectArray &in_objects)
{
    jsk_recognition_msgs::BoundingBoxArray final_boxes;

    final_boxes.header = message_header_;

    int i = 0;
    for(const autoware_msgs::DetectedObject& object : in_objects.objects)
    {
        jsk_recognition_msgs::BoundingBox box;
        box.header = message_header_;
        box.pose = object.pose;
        box.dimensions = object.dimensions;

        final_boxes.boxes.push_back(box);
    }
    return final_boxes;
}

visualization_msgs::MarkerArray
CNNSegmentation::ObjectsToMarkers(const autoware_msgs::DetectedObjectArray &in_objects)
{
    visualization_msgs::MarkerArray final_markers;

    int i = 0;
    for(const autoware_msgs::DetectedObject& object : in_objects.objects)
    {
        /*if (object.dimensions.x != 0
            && object.dimensions.y != 0
            && object.dimensions.z != 0)*/
        {
            visualization_msgs::Marker marker;
            marker.header = in_objects.header;
            marker.ns = __APP_NAME__;
            marker.id = i++;
            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker.scale.z = 1.0;
            marker.text = object.label;
            if (object.id != 0)
                marker.text += " " + std::to_string(object.id);
            marker.pose.position = object.pose.position;
            marker.pose.position.z += 2.5;
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
            marker.color.a = 1.0;
            marker.scale.x = 1.5;
            marker.scale.y = 1.5;
            marker.scale.z = 1.5;

            marker.lifetime = ros::Duration(0.2);
            final_markers.markers.push_back(marker);
        }
    }
    return final_markers;
}

void CNNSegmentation::pubColoredPoints(const autoware_msgs::DetectedObjectArray &objects_array)
{
    pcl::PointCloud<pcl::PointXYZRGB> colored_cloud;
    for (size_t object_i = 0; object_i < objects_array.objects.size(); object_i++)
    {
        // std::cout << "objct i" << object_i << std::endl;
        pcl::PointCloud<pcl::PointXYZI> object_cloud;
        pcl::fromROSMsg(objects_array.objects[object_i].pointcloud, object_cloud);
        int red = (object_i * 40) % 254;
        int green = (object_i * 36) % 254;
        int blue = (object_i * 20) % 254;
        // std::cout << "red "<< red <<std::endl;
        // std::cout << "green "<< green <<std::endl;
        // std::cout << "blue "<< blue <<std::endl;

        for (size_t i = 0; i < object_cloud.size(); i++)
        {
            // std::cout << "point i" << i << "/ size: "<<object_cloud.size()  << std::endl;
            pcl::PointXYZRGB colored_point;
            colored_point.x = object_cloud[i].x;
            colored_point.y = object_cloud[i].y;
            colored_point.z = object_cloud[i].z;
            colored_point.r = red;
            colored_point.g = green;
            colored_point.b = blue;
            colored_cloud.push_back(colored_point);
        }
    }
    sensor_msgs::PointCloud2 output_colored_cloud;
    pcl::toROSMsg(colored_cloud, output_colored_cloud);
    output_colored_cloud.header = message_header_;
    points_pub_.publish(output_colored_cloud);
}

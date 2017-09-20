#!/usr/bin/env python
# coding: utf-8

from xml.etree.ElementTree import Element, SubElement, Comment, tostring
from xml.dom import minidom
from os import listdir
from os.path import realpath


def prettify(elem):
    """
    Return a pretty-printed XML string for the Element.
    """
    rough_string = tostring(elem, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ")


def get_file_paths(dir_path, extension=None):
    file_names = listdir(dir_path)
    file_paths = list(map(lambda x: "{}/{}".format(realpath(dir_path), x), file_names))
    if extension is not None:
        file_paths = list(filter(lambda x: x[-(len(extension)):] == extension, file_paths))
    return file_paths


def create_initialization_launch_file(rosbag=True):
    launch = Element("launch")

    launch.append(Comment("set parameters"))
    if rosbag:
        SubElement(launch, "param", {"name": "use_sim_time", "value": "true"})
    SubElement(launch, "param", {"name": "localizer", "value": "velodyne"})
    SubElement(launch, "param", {"name": "tf_x", "value": "1.2"})
    SubElement(launch, "param", {"name": "tf_y", "value": "0.0"})
    SubElement(launch, "param", {"name": "tf_z", "value": "2.0"})
    SubElement(launch, "param", {"name": "tf_yaw", "value": "0.0"})
    SubElement(launch, "param", {"name": "tf_pitch", "value": "0.0"})
    SubElement(launch, "param", {"name": "tf_roll", "value": "0.0"})


    launch.append(Comment("TF: base_link to velodyne"))
    SubElement(launch, "arg", {"name": "x", "value": "1.2"})
    SubElement(launch, "arg", {"name": "y", "value": "0.0"})
    SubElement(launch, "arg", {"name": "z", "value": "2.0"})
    SubElement(launch, "arg", {"name": "yaw", "value": "0.0"})
    SubElement(launch, "arg", {"name": "pitch", "value": "0.0"})
    SubElement(launch, "arg", {"name": "roll", "value": "0.0"})
    SubElement(launch, "arg", {"name": "frame_id", "value": "/base_link"})
    SubElement(launch, "arg", {"name": "child_frame_id", "value": "/velodyne"})
    SubElement(launch, "arg", {"name": "period_in_ms", "value": "10"})
    SubElement(
        launch,
        "node",
        {
            "pkg": "tf",
            "type": "static_transform_publisher",
            "name": "base_link_to_localizer",
            "args": "$(arg x) $(arg y) $(arg z) $(arg yaw) $(arg pitch) $(arg roll) $(arg frame_id) $(arg child_frame_id) $(arg period_in_ms)"
        }
    )


    launch.append(Comment("vehicle model"))
    vehicle_model = SubElement(
        launch,
        "include",
        {
            "file": "$(find model_publisher)/launch/vehicle_model.launch",
        }
    )
    SubElement(vehicle_model, "arg", {"name": "model_path", "value": realpath("../../../../ros/src/.config/model/default.urdf")})


    launch.append(Comment("tf2 web republisher"))
    SubElement(
        launch,
        "node",
        {
            "pkg": "tf2_web_republisher",
            "type": "tf2_web_republisher",
            "name": "tf2_web_republisher"
        }
    )


    launch.append(Comment("rosbridge server (websocket)"))
    SubElement(
        launch,
        "include",
        {
            "file": "$(find rosbridge_server)/launch/rosbridge_websocket.launch",
        }
    )


    launch.append(Comment("web_vider_server"))
    SubElement(
        launch, "node", {"pkg": "web_video_server", "type": "web_video_server", "name": "web_vide_server"}
    )


    launch.append(Comment("pc2 downsampler"))
    SubElement(
        launch,
        "node",
        {
            "pkg": "pc2_downsampler",
            "type": "app.py",
            "name": "pc2_downsampler",
        }
    )

    with open("./res/initialization/initialization.launch", "w") as f:
        f.write(prettify(launch))


def create_map_launch_file():
    launch = Element("launch")

    launch.append(Comment("TF: world to map"))
    SubElement(
        launch,
        "node",
        {
            "pkg": "tf",
            "type": "static_transform_publisher",
            "name": "world_to_map",
            "args": "14771 84757 -39 0 0 0 /world /map 10"
        }
    )

    launch.append(Comment("Point Cloud"))
    SubElement(
        launch,
        "node",
        {
            "pkg": "map_file",
            "type": "points_map_loader",
            "name": "points_map_loader",
            "args": " ".join(["noupdate"] + get_file_paths("./res/map/points", "pcd"))
        }
    )

    launch.append(Comment("Vector Map"))
    SubElement(
        launch,
        "node",
        {
            "pkg": "map_file",
            "type": "vector_map_loader",
            "name": "vector_map_loader",
            "args": " ".join(get_file_paths("./res/map/vectors", "csv")),
        }
    )

    with open("./res/map/map.launch", "w") as f:
        f.write(prettify(launch))

    return True


def create_localization_launch_file():
    launch = Element("launch")

    launch.append(Comment("voxel_grid_filter"))
    voxelGridFilter = SubElement(
        launch,
        "include",
        {
            "file": "$(find points_downsampler)/launch/points_downsample.launch"
        }
    )
    SubElement(voxelGridFilter, "arg", {"name": "node_name", "value": "voxel_grid_filter"})

    launch.append(Comment("nmea2tfpose"))
    SubElement(
        launch,
        "include",
        {
            "file": "$(find gnss_localizer)/launch/nmea2tfpose.launch"
        }
    )

    launch.append(Comment("ndt matching"))
    ndt_matching = SubElement(
        launch,
        "include",
        {
            "file": "$(find ndt_localizer)/launch/ndt_matching.launch"
        }
    )
    SubElement(ndt_matching, "arg", {"name": "use_openmp", "value": "false"})
    SubElement(ndt_matching, "arg", {"name": "get_height", "value": "true"})

    with open("./res/localization/localization.launch", "w") as f:
        f.write(prettify(launch))

    return True


def create_sensing_launch_file(rosbag=True):
    launch = Element("launch")

    launch.append(Comment("calibration file path"))
    SubElement(launch, "arg", {"name": "velodyne_calib", "default": "$(find velodyne_pointcloud)/params/32db.yaml"})
    if not rosbag:
        SubElement(launch, "arg", {"name": "camera_calib", "default": realpath("./res/detection/calibration_camera_lidar_3d_prius_nic-150407.yml")})


    launch.append(Comment("HDL-32e"))
    hdl32e = SubElement(launch, "include", {"file": "$(find velodyne_pointcloud)/launch/velodyne_hdl32e.launch"})
    SubElement(hdl32e, "arg", {"name": "calibration", "value": "$(arg velodyne_calib)"})


    if not rosbag:
        # launch.append(Comment("Javad Delta 3"))
        # SubElement(launch, "node", {"pkg": "javad_navsat_driver", "type": "gnss.sh", "name": "javad_driver"})


        launch.append(Comment("PointGrey Grasshopper3"))
        pointGrayGrashopper3 = SubElement(launch, "include", {"file": "$(find pointgrey)/scripts/grasshopper3.launch"})
        SubElement(pointGrayGrashopper3, "arg", {"name": "CalibrationFile", "value": "$(arg camera_calib)"})


    with open("./res/sensing/sensing.launch", "w") as f:
        f.write(prettify(launch))


def create_detection_launch_file():
    launch = Element("launch")


    launch.append(Comment("setting of this launch file"))
    SubElement(launch, "arg", {"name": "car_detection", "default": "true"})
    SubElement(launch, "arg", {"name": "pedestrian_detection", "default": "false"})
    SubElement(launch, "arg", {"name": "is_use_gpu", "default": "true"})
    SubElement(launch, "arg", {"name": "is_register_lidar2camera_tf", "default": "true"})
    SubElement(launch, "arg", {"name": "is_publish_projection_matrix", "default": "true"})
    SubElement(launch, "arg", {"name": "is_publish_camera_info", "default": "true"})
    SubElement(launch, "arg", {"name": "camera_calib", "default": realpath("./res/detection/calibration_camera_lidar_3d_prius_nic-150407.yml")})


    launch.append(Comment("calibration_publisher"))
    calibration_publisher = SubElement(
        launch,
        "include",
        {
            "file": "$(find runtime_manager)/scripts/calibration_publisher.launch",
        }
    )
    SubElement(calibration_publisher, "arg", {"name": "file", "value": "$(arg camera_calib)"})
    SubElement(calibration_publisher, "arg", {"name": "register_lidar2camera_tf", "value": "$(arg is_register_lidar2camera_tf)"})
    SubElement(calibration_publisher, "arg", {"name": "publish_extrinsic_mat", "value": "$(arg is_publish_projection_matrix)"})
    SubElement(calibration_publisher, "arg", {"name": "publish_camera_info", "value": "$(arg is_publish_camera_info)"})


    launch.append(Comment("points2image"))
    SubElement(
        launch,
        "node",
        {
            "pkg": "points2image",
            "type": "points2image",
            "name": "points2image"
        }
    )


    launch.append(Comment("car and pedestrian detection"))
    launch.append(Comment("dpm_XXX"))
    dpm_XXX = SubElement(
        launch,
        "include",
        {
            "file": "$(find cv_tracker)/launch/dpm_ttic.launch",
        }
    )
    SubElement(dpm_XXX, "arg", {"name": "car", "value": "$(arg car_detection)"})
    SubElement(dpm_XXX, "pedestrian", {"name": "car", "value": "$(arg pedestrian_detection)"})
    SubElement(dpm_XXX, "use_gpu", {"name": "car", "value": "$(arg is_use_gpu)"})


    launch.append(Comment("range_fusion"))
    range_fusion = SubElement(
        launch,
        "include",
        {
            "file": "$(find cv_tracker)/launch/ranging.launch",
        }
    )
    SubElement(range_fusion, "arg", {"name": "car", "value": "$(arg car_detection)"})
    SubElement(range_fusion, "pedestrian", {"name": "car", "value": "$(arg pedestrian_detection)"})


    launch.append(Comment("XXX_track"))
    xxx_track = SubElement(
        launch,
        "include",
        {
            "file": "$(find cv_tracker)/launch/klt_tracking.launch",
        }
    )
    SubElement(xxx_track, "arg", {"name": "car", "value": "$(arg car_detection)"})
    SubElement(xxx_track, "pedestrian", {"name": "car", "value": "$(arg pedestrian_detection)"})


    launch.append(Comment("obj_reproj"))
    obj_reproj = SubElement(
        launch,
        "include",
        {
            "file": "$(find cv_tracker)/launch/reprojection.launch",
        }
    )
    SubElement(obj_reproj, "arg", {"name": "car", "value": "$(arg car_detection)"})
    SubElement(obj_reproj, "pedestrian", {"name": "car", "value": "$(arg pedestrian_detection)"})


    launch.append(Comment("euclidean_cluster"))
    SubElement(
        launch,
        "include",
        {
            "file": "$(find lidar_tracker)/launch/euclidean_clustering.launch",
        }
    )


    launch.append(Comment("obj_fusion"))
    obj_fusion = SubElement(
        launch,
        "include",
        {
            "file": "$(find lidar_tracker)/launch/obj_fusion.launch",
        }
    )
    SubElement(obj_fusion, "arg", {"name": "car", "value": "$(arg car_detection)"})
    SubElement(obj_fusion, "pedestrian", {"name": "car", "value": "$(arg pedestrian_detection)"})


    # launch.append(Comment("traffic light recognition"))
    # launch.append(Comment("feat_proj"))
    # SubElement(
    #     launch,
    #     "node",
    #     {
    #         "pkg": "road_wizard",
    #         "type": "feat_proj",
    #         "name": "feat_proj"
    #     }
    # )


    # launch.append(Comment("region_tlr"))
    # SubElement(
    #     launch,
    #     "include",
    #     {
    #         "file": "$(find road_wizard)/launch/traffic_light_recognition.launch",
    #     }
    # )

    with open("./res/detection/detection.launch", "w") as f:
        f.write(prettify(launch))
    return True


def create_mission_launch_file():
    launch = Element("launch")


    launch.append(Comment("setting path parameter"))
    SubElement(launch, "arg", {"name": "multi_lane_csv", "default": realpath("./res/mission/waypoints.csv")})
    SubElement(launch, "arg", {"name": "topic_pose_stamped", "default": "/ndt_pose"})
    SubElement(launch, "arg", {"name": "topic_twist_stamped", "default": "/estimate_twist"})

    """    
    launch.append(Comment("Tablet UI"))
    <!--
    <include file="$(find runtime_manager)/scripts/tablet_socket.launch"/>
    -->
    """

    launch.append(Comment("vel_pose_mux"))
    vel_pose_mux = SubElement(
        launch,
        "include",
        {
            "file": "$(find autoware_connector)/launch/vel_pose_connect.launch",
        }
    )
    SubElement(vel_pose_mux, "arg", {"name": "topic_pose_stamped", "value": "$(arg topic_pose_stamped)"})
    SubElement(vel_pose_mux, "arg", {"name": "topic_twist_stamped", "value": "$(arg topic_twist_stamped)"})


    launch.append(Comment("waypoint_loader"))
    waypoint_loader = SubElement(
        launch,
        "include",
        {
            "file": "$(find waypoint_maker)/launch/waypoint_loader.launch",
        }
    )
    SubElement(waypoint_loader, "arg", {"name": "multi_lane_csv", "value": "$(arg multi_lane_csv)"})


    """
    launch.append(Comment("lane_navi"))
    <!--
    <node pkg="lane_planner" type="lane_navi" name="lane_navi" />
    -->
    """

    launch.append(Comment("lane_rule"))
    SubElement(
        launch,
        "node",
        {
            "pkg": "lane_planner",
            "type": "lane_rule",
            "name": "lane_rule"
        }
    )


    launch.append(Comment("lane_stop"))
    SubElement(
        launch,
        "node",
        {
            "pkg": "lane_planner",
            "type": "lane_stop",
            "name": "lane_stop"
        }
    )


    launch.append(Comment("lane_select"))
    SubElement(
        launch,
        "node",
        {
            "pkg": "lane_planner",
            "type": "lane_select",
            "name": "lane_select"
        }
    )


    launch.append(Comment("velocity_set"))
    SubElement(
        launch,
        "include",
        {
            "file": "$(find astar_planner)/launch/velocity_set.launch",
        }
    )


    launch.append(Comment("obstacle_avoid"))
    obstacleAvoid = SubElement(launch, "include", {"file": "$(find astar_planner)/launch/obstacle_avoid.launch"})
    SubElement(obstacleAvoid, "arg", {"name": "avoidance", "value": "false"})
    SubElement(obstacleAvoid, "arg", {"name": "avoid_distance", "value": "13"})
    SubElement(obstacleAvoid, "arg", {"name": "avoid_velocity_limit_mps", "value": "4"})


    with open("./res/mission/mission.launch", "w") as f:
        f.write(prettify(launch))
    return True


def create_motion_launch_file():
    launch = Element("launch")


    launch.append(Comment("Vehicle Contorl"))
    SubElement(
        launch,
        "include",
        {
            "file": "$(find runtime_manager)/scripts/vehicle_socket.launch",
        }
    )


    launch.append(Comment("path_select"))
    SubElement(
        launch,
        "node",
        {
            "pkg": "lattice_planner",
            "type": "path_select",
            "name": "path_select"
        }
    )


    launch.append(Comment("pure_pursuit"))
    params = "{header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: ''}, param_flag: 0, velocity: 5.0, lookahead_distance: 4.0, lookahead_ratio: 2.0, minimum_lookahead_distance: 6.0, displacement_threshold: 0.0, relative_angle_threshold: 0}"
    SubElement(
        launch, "node", {
            "pkg": "rostopic", "type": "rostopic", "name": "rostopic",
            "args": "pub /config/waypoint_follower autoware_msgs/ConfigWaypointFollower '" + params + "'"
        }
    )
    purePersuit = SubElement(
        launch,
        "include",
        {
            "file": "$(find waypoint_follower)/launch/pure_pursuit.launch",
        }
    )
    SubElement(purePersuit, "arg", {"name": "is_linear_interpolation", "value": "true"})
    SubElement(purePersuit, "arg", {"name": "publishes_for_steering_robot", "value": "true"})


    launch.append(Comment("twist_filter"))
    SubElement(
        launch,
        "include",
        {
            "file": "$(find waypoint_follower)/launch/twist_filter.launch",
        }
    )

    launch.append(Comment("marker downsampler"))
    SubElement(launch, "node", {"pkg": "marker_downsampler", "type": "app.py", "name": "marker_downsampler"})

    with open("./res/motion/motion.launch", "w") as f:
        f.write(prettify(launch))
    return True


def create_rosbag_launch_file():
    launch = Element("launch")

    SubElement(launch, "node", {
        "pkg": "rosbag", "type": "play", "name": "rosbag_play",
        "args": "{} --clock --pause".format(
            get_file_paths("./res/rosbag/bagfile/", "bag")[0])})

    with open("./res/rosbag/rosbag.launch", "w") as f:
        f.write(prettify(launch))
    return True


def create_rosbag_pause_launch_files():
    launch = Element("launch")

    SubElement(launch, "service", {
        "pkg": "rosbag", "type": "play", "name": "rosbag_play",
        "args": "{} --clock --pause".format(
            get_file_paths("./res/rosbag/bagfile/")[0])})

    with open("./res/rosbag/rosbag.launch", "w") as f:
        f.write(prettify(launch))
    return True


if __name__ == '__main__':
    create_initialization_launch_file()
    create_map_launch_file()
    create_localization_launch_file()
    create_detection_launch_file()
    create_sensing_launch_file()
    create_mission_launch_file()
    create_motion_launch_file()
    create_rosbag_launch_file()

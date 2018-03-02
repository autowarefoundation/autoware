# How to use KITTI Player in Autoware

1. Download any of the KITTI dataset RAW rectified sets and its tracklets (http://www.cvlibs.net/datasets/kitti/raw_data.php).
You should have a directory structure that looks like this:
```
kitti_download_dir/2011_09_26/2011_09_26_drive_0001_sync
├── image_00
│   └── data
├── image_01
│   └── data
├── image_02
│   └── data
├── image_03
│   └── data
├── oxts
│   └── data
└── velodyne_points
    └── data
```
2. Create a symlink under `src/util/packages/kitti_pkg/kitti_player` with the name `dataset` that points to your `kitti_download_dir`.

2a. Instead of creating a symlink you can also use the `directory` parameter in the launch file, i.e.
`roslaunch kitti_launch kittiplayer.launch directory:=/PATH_TO_KITTI_DATASET/2011_09_26/2011_09_26_drive_0093_sync/`.

3. Modify the `src/util/packages/kitti_pkg/kitti_launch/launch/kittiplayer.launch` file to choose the set you wish to reproduce (lines 5 and 16).

4. Select the frame rate you wish to reproduce by changing the fps argument from the launch file:
```
roslaunch kitti_launch kittiplayer.launch fps:=10
```
In the above example, the player will reproduce the set at 10 fps.

5. The launch file will run `calibration_publisher` and `kitti_box_publisher` nodes and publish:
```
/projection_matrix
/camera/camera_info (sensor_msgs/CameraInfo)
/image_raw (sensor_msgs/Image)
/points_raw (sensor_msgs/PointCloud2)
/kitti_3d_labels (jsk_rviz_plugins/PictogramArray)
/kitti_box (jsk_recognition_msgs/BoundingBoxArray)
/kitti_player/oxts/gps (sensor_msgs/NavSatFix)
/kitti_player/oxts/imu (sensor_msgs/Imu)

```

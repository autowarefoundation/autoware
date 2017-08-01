oxford_ros is a ROS node to publish Oxford datasets (http://robotcar-dataset.robots.ox.ac.uk/) to ROS just like rosbag player.
Currently it supports camera center, GPS/INS poses, 3D LiDAR and front 2D LiDAR.

How to use:

1) First, download and extract the datasets.
2) Launch oxford_ros.launch with parameter dir set to the directory in which you extract the dataset.
3) To visualize, launch RViz and use oxford.rviz configuration file. 

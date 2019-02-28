Modification of a few bag_tools scripts for Autoware.

Original repository: https://github.com/srv/srv_tools
bag_tools ROS wiki: http://wiki.ros.org/bag_tools

This repository includes:
- Tool to change frame_id from topics: change_frame_id.py
- Tool to replace timestamp in a message with the header message (unmodified): replace_msg_time_with_hdr.py

#### change_frame_id

Using this script, you can use change_frame_id.py as before (see bag_tools wiki) and all the specified topics topics will be have their frame_id changed to the same frame_id:
```
rosrun autoware_bag_tools change_frame_id.py -o out.bag -i in.bag -t /camera2/image_raw /camera3/image_raw /camera4/image_raw /camera5/image_raw /camera6/image_raw -f camera_frame
```
Alternatively, you specify a 1-to-1 mapping of and topics and frame_id. Below, the frame_id of topic /camera2/image_raw is changed to camera2, the frame_id of the topic /camera3/image_raw is changed to camera3 and so on:
```
rosrun autoware_bag_tools change_frame_id.py -o out.bag -i in.bag -t /camera2/image_raw /camera3/image_raw /camera4/image_raw /camera5/image_raw /camera6/image_raw -f camera2 camera3 camera4 camera5 camera6
```

## nmea2kml tool 
Extract GPS data from rosbag file(s) into .kml and .csv files. 
.kml file could be viewed by Google Earth. color information indicate the quality of GPS satellite coverage (dark to light red) - (bad - good coverage) 

How to Run: 

```
rosrun autoware_bag_tools nmea2kml bag_file_name.bag
rosrun autoware_bag_tools nmea2kml bag_files_folder/
```

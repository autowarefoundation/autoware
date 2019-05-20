# Rosbag Demo

## Demo Data
1. Download the following files from https://drive.google.com/open?id=1ZwNQIJolJ2ogzpNprh89jCj6NocLi78f.<br>
  * sample_moriyama_data.tar.gz
  * sample_moriyama_150324.tar.gz
1. Create the `.autoware` directory in your home directory.<br>
1. Inside this, extract the previously downloaded files.<br>
```
~/.autoware/data/tf/tf.launch
~/.autoware/data/map/pointcloud_map/*.pcd
~/.autoware/data/map/vector_map/*.csv
~/.autoware/sample_moriyama_150324.bag
```

## Demo Run
1. Start the Autoware Launcher.<br>
```
$ cd Autoware/ros
$ ./run-experimental
```
![rosbag01](./images/rosbag01.png)
1. If you have a previously created profile, you can load it selecting *Load Profile* from the *File* menu.<br>
1. Show simulation panel by selecting *Simulation* from the *View* menu.<br>
1. Play rosbag.<br>
Switch on "Simulation Mode" check box, then, push "Play" button.<br>
1. Select launch buttons: "Map", "Vehicle", "Sensing", and "Visualization".<br>
![rosbag02](./images/rosbag02.png)
1. Select "Localization" button on rviz plugin and check the estimated vehicle pose.<br>
![rosbag03](./images/rosbag03.png)
1. Select "Detection" and "Prediction" buttons on rviz plugin.<br>
![rosbag04](./images/rosbag04.png)
1. Select "Decision", "Mission", and "Motion" buttons on rviz plugin.<br>
![rosbag05](./images/rosbag05.png)

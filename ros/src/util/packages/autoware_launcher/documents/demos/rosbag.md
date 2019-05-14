# Rosbag Demo

## Demo Data
1. Download the following files from https://www.autoware.ai/ ([Mirror](https://drive.google.com/open?id=1ZwNQIJolJ2ogzpNprh89jCj6NocLi78f)).<br>
  * https://www.autoware.ai/sample/sample_moriyama_data.tar.gz
  * https://www.autoware.ai/sample/sample_moriyama_150324.tar.gz

1. Make `~/.autoware` directory and extract the demo data.<br>
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
1. Load a profile if needed.<br>
Window Menu -> File -> Load Profile<br>
1. Show simulation panel.<br>
Window Menu -> View -> Simulation<br>
1. Play rosbag.<br>
Switch on "Simulation Mode" check box, then, push "Play" button.<br>
1. Push launch buttons: "Map", "Vehicle", "Sensing", and "Visualization".<br>
![rosbag02](./images/rosbag02.png)
1. Push "Localization" button on rviz plugin and check the estimated vehicle pose.<br>
![rosbag03](./images/rosbag03.png)
1. Push "Detection" and "Prediction" buttons on rviz plugin.<br>
![rosbag04](./images/rosbag04.png)
1. Push "Decision", "Mission", and "Motion" buttons on rviz plugin.<br>
![rosbag05](./images/rosbag05.png)

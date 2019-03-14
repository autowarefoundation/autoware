# Autoware Launcher

## Quick Start
1. Put map and rosbag file
```
~/.autoware/data/tf/tf.launch
~/.autoware/data/pointcloud_map/*.pcd
~/.autoware/data/vector_map/*.csv
~/.autoware/log/20150324.bag
```
1. Start the Autoware Launcher<br>
```
$ cd Autoware/ros
$ ./run-experimental`
```
![quickstart01](./documents/images/quickstart01.png)
1. Load a profile if needed<br>
Window Menu -> File -> Load Profile
1. Show simulation panel<br>
Window Menu -> View -> Simulation
1. Play rosbag<br>
Check "Simulation Mode", then, push "Play" button.
1. Push launch buttons<br>
Map, Vehicle, Sensing, and Visualization
![quickstart02](./documents/images/quickstart02.png)
1. Push localization button on rviz plugin
Check the estimated vehicle pose
![quickstart03](./documents/images/quickstart03.png)
1. Push buttons on rviz plugin<br>
detection, prediction, decision, mission, motion
![quickstart04](./documents/images/quickstart04.png)
![quickstart05](./documents/images/quickstart05.png)


## Develop
Develop mode is under development and very unstable (Window Menu -> View -> Develop Mode).<br>
* [Plugin File Format](./documents/plugin/format.md)

![develop](./documents/images/develop.png)

NOTICE
* On treeview widget, R key is run, T key is terminate.
* Use "save as" instead of "save".
* Choose .launch file when you save/load profile.

# Overview
Autonomous driving car simulator based on gazebo.
![screenshot from 2019-01-11 00-42-10-min](https://user-images.githubusercontent.com/8327598/50984661-bed3f780-1545-11e9-9af6-071ddd1cec76.png)

## Sensors
- velodyne (VLP-16, HDL-32E)
- camera
- imu

## Input, Output
![arch](https://user-images.githubusercontent.com/8327598/52900689-1861cd00-323c-11e9-84cc-ecf7bf15ee6a.png)
- Input :
  -  Autoware
     - vehicle_cmd : select field twist_cmd (default) or ctrl_cmd
  - Debug
    - cmd_vel : for teleop
- Output : 
  - Autoware
    - /vehicle_status : CAN data
    - /image_raw : Image data
    - /camera_info : Camera infomation
    - /points_raw : Lidar data
    - /imu : Imu data
  - Debug (Ground Truth)
    - /gazebo_vehicle/pose : base_link pose
    - /gazebo_vehicle/twist : base_link twist
    - /gazebo_vehicle/velocity : vechicle velocity[m/s]
    - /gazebo_vehicle/steering_angle : vehicle steering angle[rad]
    - ground truth pose tf : default disable

# **Requirements**
- ROS (higher kinetic)
- gazebo (higher version 7)
- CUDA (Optional but requires GPU by default)

# Setup
1. upgrade gazebo version and download gazebo models(**only once**)
   -  reason : https://bitbucket.org/DataspeedInc/velodyne_simulator/src/56d11e899ce0a198e7206298b3aaaf8004f3a2c6/gazebo_upgrade.md?fileviewer=file-view-default
```
$ rosrun vehicle_gazebo_simulation_launcher setup.sh
```

# Some example
## **Citysim** : http://gazebosim.org/blog/car_sim
```
$ roslaunch vehicle_gazebo_simulation_launcher gazebo_launcher.launch world_name:=citysim_gazebo7 gpu:=true
```

If you use gazebo9, simulate traffic lights and moving objects.
Build according to the [readme](https://github.com/CPFL/osrf_citysim) and execute the following command
```
$ roslaunch vehicle_gazebo_simulation_launcher gazebo_launcher.launch world_name:=citysim_gazebo9 gpu:=true
```

![screenshot from 2019-01-11 00-40-35-min](https://user-images.githubusercontent.com/8327598/50985197-19ba1e80-1547-11e9-98d1-284b3172c064.png)
## **mcity(car_demo)** : https://github.com/osrf/car_demo
```
$ roslaunch vehicle_gazebo_simulation_launcher gazebo_launcher.launch world_name:=mcity gpu:=true
```
![screenshot from 2019-01-11 00-38-49-min](https://user-images.githubusercontent.com/8327598/50985258-3e15fb00-1547-11e9-91d4-3b826b82136e.png)

## **simple** :
```
$ roslaunch vehicle_gazebo_simulation_launcher gazebo_launcher.launch world_name:=simple gpu:=true
```
[![](https://img.youtube.com/vi/wIzZ25XJI2M/0.jpg)](https://www.youtube.com/watch?v=wIzZ25XJI2M)

##  Download pointcloud map
If you need pointcloud map and path files, you can [download](https://drive.google.com/open?id=1yu8s885HDkJp3IbMV06KWim2ZdUxIoIF).  

# How to change vehicle info
You can customize sensor position and vehicle info.
- sensor position: vehicle/vehicle_model/config/caibration.yaml
- vehicle info: vehicle/vehicle_model/config/vehicle_info.yaml

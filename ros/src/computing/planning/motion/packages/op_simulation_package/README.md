# OpenPlanner - Simulator

Collection of nodes developed to help testing planning algorithm, it could be used with any planner. 
it consists of three main modules (perception simulator, traffic lights simulator, vehicle simulator) 

[Demo Movie](https://youtu.be/BS5nLtBsXPE)

## op_car_simulator_i

This node simulate a vehicle and its motion, it uses global planning and local planning libraries exactly as OpenPlanner with minor differences.
user can launch any number of simulated vehicles with simple customization. currently there are 5 simulated vehicles as an example in autoware.

_i represents the id of the simulated vehicle.

### Outputs
simulated vehicle position, TF and its dimentions. 

### Options
* plan and move from start position to goal position. 
* start and goal position are recorder so it starts automatically every time use launches the node. 
* user can set auto replay , which start the vehicle again when it arrives to the goal.
* user can log all simulated vehicles internal state 
* could be controlled by game wheel manually 
* motion could be controled frame by frame for testing with time intervals (0.1) second.

### Requirements

1. Vector map 
1. Start/Goal points 

### How to launch

* From a sourced terminal:

`roslaunch op_simulation_package op_car_simulator_i.launch`

* From Runtime Manager:

Computing Tab -> Motion Planning -> OpenPlanner - Simulator  -> op_car_simulator_i


### Parameters 
 * similar to Local planner parameters



## op_signs_simulator

This node simulates traffic lights for only one intersection with interchangable traffic lights. user can specify two sets of traffic lights, and the node will switch between them (green, red), yellow is considered as red.

### Outputs
* /roi_signal [autoware_msgs::Signals]

### Requirements

1. vector map with signal information.

### How to launch

* From a sourced terminal:

`roslaunch op_simulation_package op_signs_simulator.launch`

* From Runtime Manager:

Computing Tab -> Motion Planning -> OpenPlanner - Simulator  -> op_signs_simulator

### Parameters 
 * Ids and time for traffic signs first set 
 * Ids and time for traffic signs second set



## op_perception_simulator

This node emulate the object detection using LIDAR data similar to (lidar_euclidean_cluster_detect). 
The node receives position and dimention from op_car_simulator_i then generate noisy point cloud for each vehicle, then send all data as one cluster_cloud to lidar_kf_contour_track

### Outputs
* /cloud_clusters [autoware_msgs::CloudClusterArray]

### How to launch

* From a sourced terminal:

`roslaunch op_simulation_package op_perception_simulator.launch`

* From Runtime Manager:

Computing Tab -> Motion Planning -> OpenPlanner - Simulator  -> op_perception_simulator

### Parameters 
 * Maximum number of vehicles that will be simulated 

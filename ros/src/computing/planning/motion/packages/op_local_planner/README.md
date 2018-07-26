# OpenPlanner - Local Planner

previously known as dp_planner, now it is a collection of node with each task of local planning is separated.
for more details about OpenPlanner check our paper [Open Source Integrated Planner for Autonomous Navigation in Highly Dynamic Environments](https://www.fujipress.jp/jrm/rb/robot002900040668/)

to run the local planner user needs to run all nodes in "OpenPlanner - Local planning"
for more details about how to run [check tutorial video](https://youtu.be/BS5nLtBsXPE)

## op_common_params

This node loads the common parameters for the local planner, these parameters are using by op_trajectory_generator, op_motion_predictor, op_trajectory_evaluator, op_behavior_selector and lidar_kf_contour_track. 

### Outputs
Loads the common parameters for the local planning 

### Options
 * loads params included in the launch file. 
 * Creates folders for logging and saving important data for all nodes of OpenPlanner 
folder structure: 

- /home/user/autoware_openplanner_logs
  - /BehaviorsLogs
  - /ControlLogs
  - /GlobalPathLogs
  - /PredictionResults
  - /SimulatedCar1
  - /SimulatedCar2
  - /SimulatedCar3
  - /SimulatedCar4
  - /SimulatedCar5
  - /SimulationData
  - /TrackingLogs
  - /TrajectoriesLogs

### How to launch

* From a sourced terminal:

`roslaunch op_local_planner op_common_params.launch `

* From Runtime Manager:

Computing Tab -> Motion Planning -> OpenPlanner - Local planning  -> op_common_params

### Parameters 
 * [check paper](https://www.fujipress.jp/jrm/rb/robot002900040668/)



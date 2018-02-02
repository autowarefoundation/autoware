# Lidar Sick Package

## Steps to execute the package

### Using Runtime Manager

1. Go to `Sensing` Tab
1. Click on `config` button next to the SICK LMS511 label.
1. Write the current IP address of the SICK LMS511 you wish to connect.
1. Launch the node, clicking on the checkbox.


### Using Commandline

In a sourced terminal execute `roslaunch sick_driver lms511.launch ip:=XXX.YYY.ZZZ.WWW`


## Confirm data is coming from the LiDAR
1. Open RViz
1. Change the fixed frame to `sick`
1. Add topic type Scan, with the name `/scan`
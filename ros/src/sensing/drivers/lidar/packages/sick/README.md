# Lidar Sick Package

## Steps to Execute the Package

### Using Runtime Manager

1. Go to the `Sensing` Tab.
1. Click on the `config` button next to the SICK LMS511 label.
1. Write the current IP address of the SICK LMS511 that you wish to connect.
1. Launch the node, clicking on the checkbox.


### Using the command line

In a sourced terminal, execute `roslaunch sick_driver lms511.launch ip:=XXX.YYY.ZZZ.WWW`.


### To confirm that data is coming from the LiDAR
1. Open RViz.
1. Change the fixed frame to `sick`.
1. Add topic type Scan, with the name `/scan`.

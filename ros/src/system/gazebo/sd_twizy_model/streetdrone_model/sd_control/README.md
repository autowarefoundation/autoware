# sd_control
This package contains the nodes responsible for controlling the vehicle along with the configuration file `config/sd_twizy_control.yaml` for the PID Controllers used on the wheel and steering joints. This package is only built for the Gazebo vehicle model.

### Controlling the model
The vehicle model accepts as input the ROS topics `/cmd_vel` and `/twist_cmd` from the default ROS messages `geometry_msgs/Twist` and `geometry_msgs/TwistStamped` respectively. These are being converted into `ackermann_msgs` at `Twist_to_Ackermann.py`.  
This package also includes the robot_control.cpp written by Robotnik Automation and is used to transform the commands received from the controller in motor position/velocity commands.

### Launch
This package requires all the joints of the model to be loaded first, else it will brake.
`roslaunch sd_control sd_twizy_control.launch`
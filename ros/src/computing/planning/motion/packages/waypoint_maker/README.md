# waypoint_maker

## Overview

- This package has following nodes.

    - waypoint_clicker
    - waypoint_saver
    - waypoint_marker_publisher
    - waypoint_loader
    - waypoint_velocity_visualizer

- 3-formats of waypoints.csv handled by waypoint_maker

    - ver1： consist of x, y, z, velocity（no velocity in the first line）

    ex)

    > 3699.6206,-99426.6719,85.8506 <br>
    > 3700.6453,-99426.6562,85.8224,3.1646 <br>
    > 3701.7373,-99426.6250,85.8017,4.4036 <br>
    > 3702.7729,-99426.6094,85.7969,4.7972 <br>
    > 3703.9048,-99426.6094,85.7766,4.7954 <br>
    > 3704.9192,-99426.5938,85.7504,4.5168 <br>
    > 3705.9497,-99426.6094,85.7181,3.6313 <br>
    > 3706.9897,-99426.5859,85.6877,4.0757 <br>
    > 3708.0266,-99426.4453,85.6608,4.9097 <br>
    > ... <br>

    - ver2： consist of x, y, z, yaw, velocity（no velocity in the first line）

    ex)

    > 3804.5657,-99443.0156,85.6206,3.1251 <br>
    > 3803.5195,-99443.0078,85.6004,3.1258,4.8800 <br>
    > 3802.3425,-99442.9766,85.5950,3.1279,7.2200 <br>
    > 3801.2092,-99442.9844,85.5920,3.1293,8.8600 <br>
    > 3800.1633,-99442.9688,85.5619,3.1308,10.6000 <br>
    > 3798.9702,-99442.9609,85.5814,3.1326,10.5200 <br>
    > 3796.5706,-99442.9375,85.6056,3.1359,10.2200 <br>
    > 3795.3232,-99442.9453,85.6082,3.1357,11.0900 <br>
    > 3794.0771,-99442.9375,85.6148,3.1367,11.2300 <br>
    > ... <br>

    - ver3： category names are on the first line

    ex） consist of x,y,z,yaw,velocity,change_flag

    > x,y,z,yaw,velocity,change_flag <br>
    > 3742.216,-99411.311,85.728,3.141593,0,0 <br>
    > 3741.725,-99411.311,85.728,3.141593,10,0 <br>
    > 3740.725,-99411.311,85.733,3.141593,10,0 <br>
    > 3739.725,-99411.311,85.723,3.141593,10,0 <br>
    > 3738.725,-99411.311,85.719,3.141593,10,0 <br>
    > 3737.725,-99411.311,85.695,3.141593,10,0 <br>
    > 3736.725,-99411.311,85.667,3.141593,10,0 <br>
    > 3735.725,-99411.311,85.654,3.141593,10,0 <br>
    > ... <br>

## Nodes

### waypoint_loader

1. Overview

    - Convert waypoints.csv to ROS message type.
    - Correspond to the above 3 types of csv.
    - Adjust waypoints offline (resample and replan velocity)
    - Save waypoints.csv as ver3 format.

1. How to use

  * How to start
    - At `Computing`->`waypoint_loader`:
      - Check app->`disable_decision_maker`.
        If you want to use `decision_maker`, switch to false.
        Otherwise switch to true.
      - Check `waypoint_loader` and start.
  * Idea of velocity replanning
    - The velocity plan is based on the following idea.
      - On a straight line, it accelerates to the maximum speed.
      - At one curve:
        * Finish decelerating before entering the curve.
          If the curve is sharper, the more deceleration is required.
        * Maintain a constant speed in the curve.
        * Start to accelerate after getting out of the curve.
  * Detail of app tab
    - On `multi_lane`, please select multiple input files. If you want lane change with `lane_select`, prepare ver3 type.
    - Check `replanning_mode` if you want to replan velocity.
      - On replanning mode:
        * Check `resample_mode` if you want to resample waypoints.
          On resample mode, please set `resample_interval`.
        * Velocity replanning parameter
          - `Vmax` is max velocity.
          - `Rth`  is radius threshold for extracting curve in waypoints.
            Increasing this, you can extract curves more sensitively.
          - `Rmin` and `Vmin` are pairs of values used for velocity replanning.
            Designed velocity plan that minimizes velocity in the assumed sharpest curve.
            In the _i_-th curve, the minimum radius _r<sub>i</sub>_ and the velocity _v<sub>i</sub>_ are expressed by the following expressions.
            _v<sub>i</sub>_ = _Vmax_ - _(Vmax - Vmin)/(Rth - Rmin)_ * _(Rth - r<sub>i</sub>)_
          - `Accel limit` is acceleration value for limitting velocity.
          - `Decel limit` is deceleration value for limitting velocity.
          - `Velocity Offset` is offset amount preceding the velocity plan.
          - `End Point Offset` is the number of 0 velocity points at the end of waypoints.

1. Subscribed Topics

    - /config/waypoint_loader (autoware_msgs/ConfigWaypointLoadre)
    - /config/waypoint_loader_output (std_msgs/Bool)

1. Published Topics

    - /lane_waypoints_array (autoware_msgs/LaneArray)

1. Parameters

    - ~disable_decision_maker


### waypoint_saver

1. Overview

    - When activated, subscribe `/current_pose`, `/current_velocity`(option) and save waypoint in the file at specified intervals.
    - `change_flag` is basically stored as 0 (straight ahead),
      so if you want to change the lane, edit by yourself. (1 turn right, 2 turn left)
    - This node corresponds to preservation of ver3 format.

1. How to use

    On app:
    - Ref on the `Save File` and specify the save file name.
    - Check `Save/current_velocity` if you want to save velocity.
      In otherwise, saved as 0 velocity.
    - Using `Interval`, it is set how many meters to store waypoint.

1. Subscribed Topics

    - /current_pose (geometry_msgs/PoseStamped) : default
    - /current_velocity (geometry_msgs/TwistStamped) : default

1. Published Topics

    - nothing

1. Parameters

    - ~save_filename
    - ~interval
    - ~velocity_topic
    - ~pose_topic
    - ~save_velocity

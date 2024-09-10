# Control Evaluator

## Purpose

This package provides nodes that generate metrics to evaluate the quality of control.

It publishes diagnostic information about control modules' outputs as well as the ego vehicle's current kinematics and position.

## Evaluated metrics

The control evaluator uses the metrics defined in `include/autoware/control_evaluator/metrics/deviation_metrics.hpp`to calculate deviations in yaw and lateral distance from the ego's set-point. The control_evaluator can also be customized to offer metrics/evaluation about other control modules. Currently, the control_evaluator offers a simple diagnostic output based on the autonomous_emergency_braking node's output, but this functionality can be extended to evaluate other control modules' performance.

## Kinematics output

The control evaluator module also constantly publishes information regarding the ego vehicle's kinematics and position. It publishes the current ego lane id with the longitudinal `s` and lateral `t` arc coordinates. It also publishes the current ego speed, acceleration and jerk in its diagnostic messages.

This information can be used by other nodes to establish automated evaluation using rosbags: by crosschecking the ego position and kinematics with the evaluated control module's output, it is possible to judge if the evaluated control modules reacted in a satisfactory way at certain interesting points of the rosbag reproduction.

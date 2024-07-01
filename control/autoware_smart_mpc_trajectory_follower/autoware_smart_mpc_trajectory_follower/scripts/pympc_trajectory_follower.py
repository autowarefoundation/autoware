#!/usr/bin/env python3

# Copyright 2024 Proxima Technology Inc, TIER IV
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# cspell: ignore interp

from enum import Enum
import time

from autoware_adapi_v1_msgs.msg import OperationModeState
from autoware_control_msgs.msg import Control
from autoware_planning_msgs.msg import Trajectory
from autoware_planning_msgs.msg import TrajectoryPoint
from autoware_smart_mpc_trajectory_follower.scripts import drive_controller
from autoware_smart_mpc_trajectory_follower.scripts import drive_functions
from autoware_vehicle_msgs.msg import SteeringReport
from builtin_interfaces.msg import Duration
from diagnostic_msgs.msg import DiagnosticStatus
import diagnostic_updater
from diagnostic_updater import DiagnosticStatusWrapper
from geometry_msgs.msg import AccelWithCovarianceStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import numpy as np
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.node import Node
import scipy
import scipy.interpolate
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
from std_msgs.msg import String
from tier4_debug_msgs.msg import BoolStamped
from tier4_debug_msgs.msg import Float32MultiArrayStamped
from tier4_debug_msgs.msg import Float32Stamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


class ControlStatus(Enum):
    DRIVE = 0
    STOPPING = 1
    STOPPED = 2
    EMERGENCY = 3


def getYaw(orientation_xyzw):
    return R.from_quat(orientation_xyzw.reshape(-1, 4)).as_euler("xyz")[:, 2]


class PyMPCTrajectoryFollower(Node):
    def __init__(self):
        # MPC logic
        self.controller = drive_controller.drive_controller(
            model_file_name=(drive_functions.load_dir + "/model_for_test_drive.pth"),
        )

        # ROS 2
        super().__init__("pympc_trajectory_follower")

        self.declare_parameter(
            "plot_sampling_paths",
            False,
            ParameterDescriptor(description="Publish MPPI sampling paths as MarkerArray."),
        )

        self.sub_trajectory_ = self.create_subscription(
            Trajectory,
            "/planning/scenario_planning/trajectory",
            self.onTrajectory,
            1,
        )
        self.sub_trajectory_  # prevent unused variable warning

        self.sub_odometry_ = self.create_subscription(
            Odometry,
            "/localization/kinematic_state",
            self.onOdometry,
            1,
        )
        self.sub_odometry_

        self.sub_accel_ = self.create_subscription(
            AccelWithCovarianceStamped,
            "/localization/acceleration",
            self.onAccel,
            1,
        )
        self.sub_accel_

        self.sub_steering_ = self.create_subscription(
            SteeringReport,
            "/vehicle/status/steering_status",
            self.onSteering,
            1,
        )
        self.sub_steering_

        self.sub_operation_mode_ = self.create_subscription(
            OperationModeState,
            "/system/operation_mode/state",
            self.onOperationMode,
            1,
        )
        self.sub_operation_mode_

        self.sub_stop_mode_reset_request_ = self.create_subscription(
            String,
            "/pympc_stop_mode_reset_request",
            self.onStopModeResetRequest,
            1,
        )
        self.sub_stop_mode_reset_request_

        self.sub_goal_pose_ = self.create_subscription(
            PoseStamped,
            "/planning/mission_planning/echo_back_goal_pose",
            self.onGoalPose,
            1,
        )
        self.sub_goal_pose_

        self.sub_reload_mpc_param_trigger_ = self.create_subscription(
            String,
            "/pympc_reload_mpc_param_trigger",
            self.onReloadMpcParamTrigger,
            1,
        )
        self.sub_reload_mpc_param_trigger_

        self.sub_control_command_control_cmd_ = self.create_subscription(
            Control,
            "/control/command/control_cmd",
            self.onControlCommandControlCmd,
            3,
        )
        self.sub_control_command_control_cmd_

        self.control_cmd_pub_ = self.create_publisher(
            Control,
            "/control/trajectory_follower/control_cmd",
            1,
        )

        self.predicted_trajectory_pub_ = self.create_publisher(
            Trajectory,
            "/control/trajectory_follower/lateral/predicted_trajectory",
            1,
        )

        self.timer_period_callback = 0.03  # 30ms

        self.timer = self.create_timer(self.timer_period_callback, self.timer_callback)

        self.debug_mpc_x_current_pub_ = self.create_publisher(
            Float32MultiArrayStamped,
            "/debug_mpc_x_current",
            1,
        )
        self.debug_mpc_x_des_pub_ = self.create_publisher(
            Float32MultiArrayStamped,
            "/debug_mpc_x_des",
            1,
        )
        self.debug_mpc_y_des_pub_ = self.create_publisher(
            Float32MultiArrayStamped,
            "/debug_mpc_y_des",
            1,
        )
        self.debug_mpc_v_des_pub_ = self.create_publisher(
            Float32MultiArrayStamped,
            "/debug_mpc_v_des",
            1,
        )
        self.debug_mpc_yaw_des_pub_ = self.create_publisher(
            Float32MultiArrayStamped,
            "/debug_mpc_yaw_des",
            1,
        )
        self.debug_mpc_acc_des_pub_ = self.create_publisher(
            Float32MultiArrayStamped,
            "/debug_mpc_acc_des",
            1,
        )
        self.debug_mpc_steer_des_pub_ = self.create_publisher(
            Float32MultiArrayStamped,
            "/debug_mpc_steer_des",
            1,
        )

        self.debug_mpc_X_des_converted_pub_ = self.create_publisher(
            Float32MultiArrayStamped,
            "/debug_mpc_X_des_converted",
            1,
        )

        self.debug_mpc_nominal_traj_pub_ = self.create_publisher(
            Float32MultiArrayStamped,
            "/debug_mpc_nominal_traj",
            1,
        )

        self.debug_mpc_nominal_inputs_pub_ = self.create_publisher(
            Float32MultiArrayStamped,
            "/debug_mpc_nominal_inputs",
            1,
        )

        self.debug_mpc_X_input_list_pub_ = self.create_publisher(
            Float32MultiArrayStamped,
            "/debug_mpc_X_input_list",
            1,
        )

        self.debug_mpc_Y_output_list_pub_ = self.create_publisher(
            Float32MultiArrayStamped,
            "/debug_mpc_Y_output_list",
            1,
        )

        self.debug_mpc_error_prediction_pub_ = self.create_publisher(
            Float32MultiArrayStamped,
            "/debug_mpc_error_prediction",
            1,
        )

        self.debug_mpc_emergency_stop_mode_pub_ = self.create_publisher(
            BoolStamped,
            "/debug_mpc_emergency_stop_mode",
            1,
        )

        self.debug_mpc_goal_stop_mode_pub_ = self.create_publisher(
            BoolStamped,
            "/debug_mpc_goal_stop_mode",
            1,
        )

        self.debug_mpc_max_trajectory_err_pub_ = self.create_publisher(
            Float32Stamped,
            "/debug_mpc_max_trajectory_err",
            1,
        )

        self.debug_mpc_calc_u_opt_time_pub_ = self.create_publisher(
            Float32Stamped,
            "/debug_mpc_calc_u_opt_time",
            1,
        )

        self.debug_mpc_total_ctrl_time_pub_ = self.create_publisher(
            Float32Stamped,
            "/debug_mpc_total_ctrl_time",
            1,
        )

        self.debug_mpc_sampling_paths_marker_array_ = self.create_publisher(
            MarkerArray,
            "/debug_mpc_sampling_paths_marker_array",
            1,
        )

        self._present_trajectory = None
        self._present_kinematic_state = None
        self._present_acceleration = None
        self._present_steering_status = None
        self._present_operation_mode = None
        self._goal_pose = None
        self.control_cmd_time_stamp_list = []
        self.control_cmd_steer_list = []
        self.control_cmd_acc_list = []

        self.emergency_stop_mode_flag = False
        self.stop_mode_reset_request = False
        self.last_acc_cmd = 0.0
        self.last_steer_cmd = 0.0
        self.past_control_trajectory_mode = 1

        self.diagnostic_updater = diagnostic_updater.Updater(self)
        self.setup_diagnostic_updater()
        self.control_state = ControlStatus.STOPPED

    def onTrajectory(self, msg):
        self._present_trajectory = msg

    def onOdometry(self, msg):
        self._present_kinematic_state = msg

    def onAccel(self, msg):
        self._present_acceleration = msg

    def onSteering(self, msg):
        self._present_steering_status = msg

    def onOperationMode(self, msg):
        self._present_operation_mode = msg

    def onStopModeResetRequest(self, msg):
        self.stop_mode_reset_request = True
        self.get_logger().info("receive stop mode reset request")

    def onGoalPose(self, msg):
        self._goal_pose = msg

    def onReloadMpcParamTrigger(self, msg):
        self.get_logger().info("receive reload mpc param trigger")

        # Re-starting the controller
        # Reloading parameters etc. internally.
        self.controller = drive_controller.drive_controller(
            model_file_name=(drive_functions.load_dir + "/model_for_test_drive.pth"),
        )

    def onControlCommandControlCmd(self, msg):
        present_cmd_msg = msg
        if self.past_control_trajectory_mode == 0:
            self.control_cmd_time_stamp_list.append(
                present_cmd_msg.stamp.sec + 1e-9 * present_cmd_msg.stamp.nanosec
            )
            self.control_cmd_steer_list.append(present_cmd_msg.lateral.steering_tire_angle)
            self.control_cmd_acc_list.append(present_cmd_msg.longitudinal.acceleration)
            if self.control_cmd_time_stamp_list[-1] - self.control_cmd_time_stamp_list[0] > 3.0:
                self.control_cmd_time_stamp_list.pop(0)
                self.control_cmd_steer_list.pop(0)
                self.control_cmd_acc_list.pop(0)

    def timer_callback(self):
        if (self._present_trajectory is not None) and (self._present_kinematic_state is not None):
            self.control()

    def control(self):
        start_ctrl_time = time.time()

        # [1] Processing of information received from topics.
        trajectory_position = []
        trajectory_orientation = []
        trajectory_longitudinal_velocity = []
        trajectory_lateral_velocity = []
        trajectory_acceleration = []
        trajectory_heading_rate = []
        points = self._present_trajectory.points
        for i in range(len(points)):
            trajectory_position.append(
                [points[i].pose.position.x, points[i].pose.position.y, points[i].pose.position.z]
            )
            trajectory_orientation.append(
                [
                    points[i].pose.orientation.x,
                    points[i].pose.orientation.y,
                    points[i].pose.orientation.z,
                    points[i].pose.orientation.w,
                ]
            )
            trajectory_longitudinal_velocity.append(points[i].longitudinal_velocity_mps)
            trajectory_lateral_velocity.append(points[i].lateral_velocity_mps)
            trajectory_acceleration.append(points[i].acceleration_mps2)
            trajectory_heading_rate.append(points[i].heading_rate_rps)
        trajectory_position = np.array(trajectory_position)
        trajectory_orientation = np.array(trajectory_orientation)
        trajectory_longitudinal_velocity = np.array(trajectory_longitudinal_velocity)
        trajectory_lateral_velocity = np.array(trajectory_lateral_velocity)
        trajectory_acceleration = np.array(trajectory_acceleration)
        trajectory_heading_rate = np.array(trajectory_heading_rate)

        present_position = np.array(
            [
                self._present_kinematic_state.pose.pose.position.x,
                self._present_kinematic_state.pose.pose.position.y,
                self._present_kinematic_state.pose.pose.position.z,
            ]
        )
        present_orientation = np.array(
            [
                self._present_kinematic_state.pose.pose.orientation.x,
                self._present_kinematic_state.pose.pose.orientation.y,
                self._present_kinematic_state.pose.pose.orientation.z,
                self._present_kinematic_state.pose.pose.orientation.w,
            ]
        )
        present_linear_velocity = np.array(
            [
                self._present_kinematic_state.twist.twist.linear.x,
                self._present_kinematic_state.twist.twist.linear.y,
                self._present_kinematic_state.twist.twist.linear.z,
            ]
        )
        present_linear_acceleration = np.array(
            [
                self._present_acceleration.accel.accel.linear.x,
                self._present_acceleration.accel.accel.linear.y,
                self._present_acceleration.accel.accel.linear.z,
            ]
        )

        present_steering_tire_angle = np.array([self._present_steering_status.steering_tire_angle])[
            0
        ]

        nearestIndex = ((trajectory_position - present_position) ** 2).sum(axis=1).argmin()

        is_applying_control = False
        if self._present_operation_mode is not None:
            if (
                self._present_operation_mode.mode == 2
                and self._present_operation_mode.is_autoware_control_enabled
            ):
                is_applying_control = True

        # [2] deviation check
        position_deviation = np.sqrt(
            ((trajectory_position[nearestIndex] - present_position) ** 2).sum()
        )
        orientation_deviation = getYaw(present_orientation) - getYaw(
            trajectory_orientation[nearestIndex]
        )
        while True:
            if orientation_deviation > np.pi:
                orientation_deviation -= 2.0 * np.pi
            if orientation_deviation < (-np.pi):
                orientation_deviation += 2.0 * np.pi
            if np.abs(orientation_deviation) < np.pi:
                break
        position_deviation_threshold = 2.0
        orientation_deviation_threshold = 60 * np.pi / 180.0

        # The moment the threshold is exceeded, it enters emergency stop mode.
        control_state = self.control_state
        if is_applying_control:
            if (
                position_deviation > position_deviation_threshold
                or orientation_deviation > orientation_deviation_threshold
            ):
                self.emergency_stop_mode_flag = True
                control_state = ControlStatus.EMERGENCY

            # Normal return from emergency stop mode when within the threshold value and a request to cancel the stop mode has been received.
            if (
                not (
                    position_deviation > position_deviation_threshold
                    or orientation_deviation > orientation_deviation_threshold
                )
                and self.stop_mode_reset_request
            ):
                self.emergency_stop_mode_flag = False

        # Processes to ensure that requests do not continue to remain after the stop mode is cancelled.
        if not self.emergency_stop_mode_flag:
            self.stop_mode_reset_request = False

        # [3] Control logic calculation
        # [3-1] State variables in the MPC
        present_mpc_x = np.array(
            [
                present_position[0],
                present_position[1],
                present_linear_velocity[0],
                getYaw(present_orientation)[0],
                present_linear_acceleration[0],
                present_steering_tire_angle,
            ]
        )

        # [3-2] resampling trajectory by time (except steer angle).
        X_des = np.zeros((drive_functions.N + 1, 8))
        dist = np.sqrt(((trajectory_position[1:] - trajectory_position[:-1]) ** 2).sum(axis=1))
        timestamp_from_start = [0.0]
        tmp_t = 0.0
        for i in range(len(dist)):
            tmp_t += dist[i] / max(np.abs(trajectory_longitudinal_velocity[i]), 0.1)
            timestamp_from_start.append(1 * tmp_t)
        timestamp_from_start = np.array(timestamp_from_start)

        mpc_trajectory_time_step = 0.1
        timestamp_mpc = (
            np.arange(drive_functions.N + 1) * mpc_trajectory_time_step
            + timestamp_from_start[nearestIndex]
        )
        for i in range(drive_functions.N + 1):
            if timestamp_mpc[drive_functions.N - i] > timestamp_from_start[-1]:
                timestamp_mpc[drive_functions.N - i] = timestamp_from_start[-1]
            else:
                break
        pos_x_interpolate = scipy.interpolate.interp1d(
            timestamp_from_start, trajectory_position[:, 0]
        )
        pos_y_interpolate = scipy.interpolate.interp1d(
            timestamp_from_start, trajectory_position[:, 1]
        )
        longitudinal_velocity_interpolate = scipy.interpolate.interp1d(
            timestamp_from_start, trajectory_longitudinal_velocity
        )
        key_rots = R.from_quat(trajectory_orientation.reshape(-1, 4))
        orientation_interpolate = Slerp(timestamp_from_start, key_rots)
        acceleration_interpolate = scipy.interpolate.interp1d(
            timestamp_from_start, trajectory_acceleration
        )

        X_des[:, 0] = pos_x_interpolate(timestamp_mpc)
        X_des[:, 1] = pos_y_interpolate(timestamp_mpc)
        X_des[:, 2] = longitudinal_velocity_interpolate(timestamp_mpc)
        X_des[:, 3] = orientation_interpolate(timestamp_mpc).as_euler("xyz")[:, 2]
        X_des[:, 4] = acceleration_interpolate(timestamp_mpc)
        X_des[:, 6] = 1 * X_des[:, 4]

        # [3-3] resampling trajectory by time (steer angle)
        previous_des_steer = 0.0
        steer_trajectory2 = np.zeros(len(timestamp_mpc))
        downsampling = 5
        for ii in range(2 + (int(len(timestamp_mpc) // downsampling))):
            i = ii * downsampling
            if i >= len(timestamp_mpc):
                i = len(timestamp_mpc) - 1
                if (i % downsampling) == 0:
                    continue

            tmp_des_pos = 1 * X_des[i, :2]
            distance_squared = ((trajectory_position[:, :2] - tmp_des_pos[:2]) ** 2).sum(axis=1)
            tmp_nearest_index = distance_squared.argmin()
            curvature_calculation_distance = 3.0
            nearestIndex_in_front = (
                np.abs(
                    distance_squared[tmp_nearest_index:]
                    - curvature_calculation_distance * curvature_calculation_distance
                )
            ).argmin() + tmp_nearest_index
            distance_in_front = np.sqrt(distance_squared[nearestIndex_in_front])
            if distance_in_front < curvature_calculation_distance:
                curvature_calculation_distance = distance_in_front
            if tmp_nearest_index > 0:
                nearestIndex_behind = (
                    np.abs(
                        distance_squared[:tmp_nearest_index]
                        - curvature_calculation_distance * curvature_calculation_distance
                    )
                ).argmin()
                curvature_calculation_distance_threshold = 0.5
                if distance_in_front < curvature_calculation_distance_threshold:
                    # If there is no point in front, fill in with the value before it.
                    tmp_des_steer = 1 * previous_des_steer
                else:
                    # Normal line passing through a point 3 m behind.：b1*x + b2*y + b3 = 0
                    b1 = tmp_des_pos[0] - trajectory_position[nearestIndex_behind][0]
                    b2 = tmp_des_pos[1] - trajectory_position[nearestIndex_behind][1]
                    b3 = (
                        -b1 * trajectory_position[nearestIndex_behind][0]
                        - b2 * trajectory_position[nearestIndex_behind][1]
                    )
                    # Normal line through a point 3 m in front：f1*x + f2*y + f3 = 0
                    f1 = tmp_des_pos[0] - trajectory_position[nearestIndex_in_front][0]
                    f2 = tmp_des_pos[1] - trajectory_position[nearestIndex_in_front][1]
                    f3 = (
                        -f1 * trajectory_position[nearestIndex_in_front][0]
                        - f2 * trajectory_position[nearestIndex_in_front][1]
                    )
                    det = b1 * f2 - b2 * f1
                    if np.abs(det) < 1e-6:
                        # The two normals have the same slope, so steer is 0
                        tmp_des_steer = 0.0
                    else:
                        # solve Ax+b=0
                        invA = np.array([[f2, -b2], [-f1, b1]]) / det
                        sol = -invA @ np.array([b3, f3])  # center of the circle
                        curvature = np.sqrt(
                            ((sol - trajectory_position[nearestIndex_behind][:2]) ** 2).sum()
                        )
                        tmp_des_steer = np.arctan(drive_functions.L / curvature)

                        tmp_vec_behind = sol - trajectory_position[nearestIndex_behind][:2]
                        # Line segment connecting backward point - centre of circle
                        cross_product = b1 * tmp_vec_behind[1] - b2 * tmp_vec_behind[0]
                        if cross_product < 0.0:
                            tmp_des_steer = -tmp_des_steer

                steer_trajectory2[i] = 1.0 * tmp_des_steer
                previous_des_steer = 1.0 * tmp_des_steer
            else:
                steer_trajectory2[i] = 0.0
                previous_des_steer = 0.0

        for i in range(len(timestamp_mpc) - 1):
            reminder = i % downsampling
            i0 = i - reminder
            i1 = i0 + downsampling
            if reminder == 0:
                continue
            if i1 >= len(timestamp_mpc):
                i1 = len(timestamp_mpc) - 1
            w = (1.0 * reminder) / (i1 - i0)
            steer_trajectory2[i] = (1 - w) * steer_trajectory2[i0] + w * steer_trajectory2[i1]

        X_des[:, 5] = steer_trajectory2
        X_des[:, 7] = 1 * X_des[:, 5]

        # [3-4] (optimal) control computation
        U_des = np.zeros((drive_functions.N, 2))
        start_calc_u_opt = time.time()
        acc_time_stamp = (
            self._present_acceleration.header.stamp.sec
            + 1e-9 * self._present_acceleration.header.stamp.nanosec
        )
        steer_time_stamp = (
            self._present_steering_status.stamp.sec
            + 1e-9 * self._present_steering_status.stamp.nanosec
        )
        u_opt = self.controller.update_input_queue_and_get_optimal_control(
            self.control_cmd_time_stamp_list,
            self.control_cmd_acc_list,
            self.control_cmd_steer_list,
            present_mpc_x,
            X_des,
            U_des,
            acc_time_stamp,
            steer_time_stamp,
        )
        end_calc_u_opt = time.time()
        mpc_acc_cmd = u_opt[0]
        mpc_steer_cmd = u_opt[1]

        # [3-5] Enter goal stop mode (override command value) to maintain stop at goal
        self.goal_stop_mode_flag = False
        if self._goal_pose is not None:
            goal_position = np.array(
                [
                    self._goal_pose.pose.position.x,
                    self._goal_pose.pose.position.y,
                    self._goal_pose.pose.position.z,
                ]
            )

            distance_from_goal = np.sqrt(((goal_position[:2] - present_position[:2]) ** 2).sum())
            goal_distance_threshold = 0.8
            if distance_from_goal < goal_distance_threshold:
                self.goal_stop_mode_flag = True

        # [3-6] Determine the control logic to be finally applied and publish it
        acc_cmd = 0.0
        steer_cmd = 0.0
        if (not self.emergency_stop_mode_flag) and (not self.goal_stop_mode_flag):
            # in normal mode
            acc_cmd = mpc_acc_cmd
            steer_cmd = mpc_steer_cmd
        else:
            # in stop mode
            acc_cmd_decrease_limit = 5.0 * self.timer_period_callback  # lon_jerk_lim
            steer_cmd_decrease_limit = 0.5 * self.timer_period_callback  # steer_rate_lim
            acc_cmd = max(
                self.last_acc_cmd - acc_cmd_decrease_limit, -drive_functions.acc_lim_points[0]
            )  # lon_acc_lim
            if self.last_steer_cmd > (steer_cmd_decrease_limit + 1e-6):
                steer_cmd = self.last_steer_cmd - steer_cmd_decrease_limit
            elif self.last_steer_cmd < -(steer_cmd_decrease_limit + 1e-6):
                steer_cmd = self.last_steer_cmd + steer_cmd_decrease_limit
            else:
                steer_cmd = 0.0
            control_state = ControlStatus.STOPPING

        cmd_msg = Control()
        cmd_msg.stamp = cmd_msg.lateral.stamp = cmd_msg.longitudinal.stamp = (
            self.get_clock().now().to_msg()
        )
        cmd_msg.longitudinal.velocity = trajectory_longitudinal_velocity[nearestIndex]
        cmd_msg.longitudinal.acceleration = acc_cmd
        cmd_msg.lateral.steering_tire_angle = steer_cmd

        self.control_cmd_pub_.publish(cmd_msg)
        self.last_acc_cmd = 1 * acc_cmd
        self.last_steer_cmd = 1 * steer_cmd

        if self.past_control_trajectory_mode == 1:
            self.control_cmd_time_stamp_list.append(
                cmd_msg.stamp.sec + 1e-9 * cmd_msg.stamp.nanosec
            )
            self.control_cmd_steer_list.append(steer_cmd)
            self.control_cmd_acc_list.append(acc_cmd)
            if self.control_cmd_time_stamp_list[-1] - self.control_cmd_time_stamp_list[0] > 3.0:
                self.control_cmd_time_stamp_list.pop(0)
                self.control_cmd_steer_list.pop(0)
                self.control_cmd_acc_list.pop(0)

        # [3-7] Update control state
        if control_state != ControlStatus.EMERGENCY:
            stopped_velocity_threshold = 0.1
            if present_linear_velocity[0] <= stopped_velocity_threshold:
                control_state = ControlStatus.STOPPED
            elif control_state != ControlStatus.STOPPING:
                control_state = ControlStatus.DRIVE
        self.control_state = control_state

        # [4] Update MPC internal variables
        if not is_applying_control:
            self.controller.send_initialize_input_queue()
            self.controller.stop_model_update()
            self.control_cmd_time_stamp_list.clear()
            self.control_cmd_steer_list.clear()
            self.control_cmd_acc_list.clear()

        # [5] MPC prediction trajectory publish
        traj_msg = Trajectory()
        traj_msg.header.stamp = cmd_msg.stamp
        traj_msg.header.frame_id = "map"
        downsampling = 3
        for ii in range(int(self.controller.nominal_traj.shape[0] // downsampling)):
            i = ii * downsampling
            traj_msg.points += [TrajectoryPoint()]
            int_sec = int(i * mpc_trajectory_time_step)
            traj_msg.points[ii].time_from_start = Duration()
            traj_msg.points[ii].time_from_start.sec = int_sec
            traj_msg.points[ii].time_from_start.nanosec = int(
                (i * mpc_trajectory_time_step - int_sec) * 1e9
            )
            traj_msg.points[ii].pose.position.x = self.controller.nominal_traj[i, 0]
            traj_msg.points[ii].pose.position.y = self.controller.nominal_traj[i, 1]
            traj_msg.points[ii].pose.position.z = present_position[2]
            tmp_orientation_xyzw = R.from_euler("z", self.controller.nominal_traj[i, 3]).as_quat()
            traj_msg.points[ii].pose.orientation.x = tmp_orientation_xyzw[0]
            traj_msg.points[ii].pose.orientation.y = tmp_orientation_xyzw[1]
            traj_msg.points[ii].pose.orientation.z = tmp_orientation_xyzw[2]
            traj_msg.points[ii].pose.orientation.w = tmp_orientation_xyzw[3]
            traj_msg.points[ii].longitudinal_velocity_mps = self.controller.nominal_traj[i, 2]
            traj_msg.points[ii].acceleration_mps2 = self.controller.nominal_traj[i, 4]

        self.predicted_trajectory_pub_.publish(traj_msg)

        # [-1] Publish internal information for debugging
        enable_debug_pub = True
        if enable_debug_pub:
            self.debug_mpc_emergency_stop_mode_pub_.publish(
                BoolStamped(stamp=cmd_msg.stamp, data=self.emergency_stop_mode_flag)
            )
            self.debug_mpc_goal_stop_mode_pub_.publish(
                BoolStamped(stamp=cmd_msg.stamp, data=self.goal_stop_mode_flag)
            )
            self.debug_mpc_x_des_pub_.publish(
                Float32MultiArrayStamped(stamp=cmd_msg.stamp, data=X_des[:, 0])
            )
            self.debug_mpc_y_des_pub_.publish(
                Float32MultiArrayStamped(stamp=cmd_msg.stamp, data=X_des[:, 1])
            )
            self.debug_mpc_v_des_pub_.publish(
                Float32MultiArrayStamped(stamp=cmd_msg.stamp, data=X_des[:, 2])
            )
            self.debug_mpc_yaw_des_pub_.publish(
                Float32MultiArrayStamped(stamp=cmd_msg.stamp, data=X_des[:, 3])
            )
            self.debug_mpc_acc_des_pub_.publish(
                Float32MultiArrayStamped(stamp=cmd_msg.stamp, data=X_des[:, 4])
            )
            self.debug_mpc_steer_des_pub_.publish(
                Float32MultiArrayStamped(stamp=cmd_msg.stamp, data=X_des[:, 5])
            )
            self.debug_mpc_x_current_pub_.publish(
                Float32MultiArrayStamped(stamp=cmd_msg.stamp, data=present_mpc_x.tolist())
            )
            self.debug_mpc_X_des_converted_pub_.publish(
                Float32MultiArrayStamped(
                    stamp=cmd_msg.stamp, data=self.controller.X_des.flatten().tolist()
                )
            )
            self.debug_mpc_nominal_traj_pub_.publish(
                Float32MultiArrayStamped(
                    stamp=cmd_msg.stamp, data=self.controller.nominal_traj.flatten().tolist()
                )
            )
            self.debug_mpc_nominal_inputs_pub_.publish(
                Float32MultiArrayStamped(
                    stamp=cmd_msg.stamp, data=self.controller.nominal_inputs.flatten().tolist()
                )
            )
            self.debug_mpc_X_input_list_pub_.publish(
                Float32MultiArrayStamped(
                    stamp=cmd_msg.stamp,
                    data=np.array(self.controller.X_input_list[-1:]).flatten().tolist(),
                )
            )
            self.debug_mpc_Y_output_list_pub_.publish(
                Float32MultiArrayStamped(
                    stamp=cmd_msg.stamp,
                    data=np.array(self.controller.Y_output_list[-1:]).flatten().tolist(),
                )
            )
            self.debug_mpc_max_trajectory_err_pub_.publish(
                Float32Stamped(
                    stamp=cmd_msg.stamp,
                    data=self.controller.err,
                )
            )
            if self.controller.use_trained_model:
                self.debug_mpc_error_prediction_pub_.publish(
                    Float32MultiArrayStamped(
                        stamp=cmd_msg.stamp, data=self.controller.previous_error[:6]
                    )
                )

            end_ctrl_time = time.time()

            self.debug_mpc_calc_u_opt_time_pub_.publish(
                Float32Stamped(
                    stamp=cmd_msg.stamp,
                    data=(end_calc_u_opt - start_calc_u_opt),
                )
            )

            self.debug_mpc_total_ctrl_time_pub_.publish(
                Float32Stamped(
                    stamp=cmd_msg.stamp,
                    data=(end_ctrl_time - start_ctrl_time),
                )
            )
            if self.controller.mode == "mppi" or self.controller.mode == "mppi_ilqr":
                if self.get_parameter("plot_sampling_paths").get_parameter_value().bool_value:
                    marker_array = MarkerArray()
                    for i in range(self.controller.mppi_candidates.shape[0]):
                        marker = Marker()
                        marker.type = marker.LINE_STRIP
                        marker.header.stamp = cmd_msg.stamp
                        marker.header.frame_id = "map"
                        marker.id = i
                        marker.action = marker.ADD

                        marker.scale.x = 0.05
                        marker.scale.y = 0.0
                        marker.scale.z = 0.0

                        marker.color.a = 1.0
                        marker.color.r = np.random.uniform()
                        marker.color.g = np.random.uniform()
                        marker.color.b = np.random.uniform()

                        marker.lifetime.nanosec = 500000000
                        marker.frame_locked = True

                        marker.points = []

                        for j in range(self.controller.mppi_candidates.shape[1]):
                            tmp_point = Point()
                            tmp_point.x = self.controller.mppi_candidates[i, j, 0]
                            tmp_point.y = self.controller.mppi_candidates[i, j, 1]
                            tmp_point.z = self._present_kinematic_state.pose.pose.position.z
                            marker.points.append(tmp_point)
                        marker_array.markers.append(marker)
                    self.debug_mpc_sampling_paths_marker_array_.publish(marker_array)

            self.diagnostic_updater.force_update()

    def setup_diagnostic_updater(self):
        self.diagnostic_updater.setHardwareID("pympc_trajectory_follower")
        self.diagnostic_updater.add("control_state", self.check_control_state)

    def check_control_state(self, stat: DiagnosticStatusWrapper):
        msg = "emergency occurred" if self.control_state == ControlStatus.EMERGENCY else "OK"
        level = (
            DiagnosticStatus.ERROR
            if self.control_state == ControlStatus.EMERGENCY
            else DiagnosticStatus.OK
        )
        stat.summary(level, msg)
        stat.add("control_state", str(self.control_state))
        return stat


def main(args=None):
    rclpy.init(args=args)

    pympc_trajectory_follower = PyMPCTrajectoryFollower()

    rclpy.spin(pympc_trajectory_follower)

    pympc_trajectory_follower.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

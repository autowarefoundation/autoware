#!/usr/bin/env python3

from autoware_adapi_v1_msgs.msg import SteeringFactor
from autoware_adapi_v1_msgs.msg import SteeringFactorArray
from autoware_adapi_v1_msgs.msg import VelocityFactor
from autoware_adapi_v1_msgs.msg import VelocityFactorArray
from geometry_msgs.msg import Pose
import numpy
import rclpy
import rclpy.duration
import rclpy.node
import tf_transformations
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

velocity_type_text = {
    VelocityFactor.UNKNOWN: "unknown",
    VelocityFactor.SURROUNDING_OBSTACLE: "surrounding obstacle",
    VelocityFactor.ROUTE_OBSTACLE: "route obstacle",
    VelocityFactor.INTERSECTION: "intersection",
    VelocityFactor.CROSSWALK: "crosswalk",
    VelocityFactor.REAR_CHECK: "rear check",
    VelocityFactor.USER_DEFINED_DETECTION_AREA: "detection area",
    VelocityFactor.NO_STOPPING_AREA: "no stopping area",
    VelocityFactor.STOP_SIGN: "stop sign",
    VelocityFactor.TRAFFIC_SIGNAL: "traffic signal",
    VelocityFactor.V2I_GATE_CONTROL_ENTER: "v2i enter",
    VelocityFactor.V2I_GATE_CONTROL_LEAVE: "v2i leave",
    VelocityFactor.MERGE: "merge",
    VelocityFactor.SIDEWALK: "sidewalk",
    VelocityFactor.LANE_CHANGE: "lane change",
    VelocityFactor.AVOIDANCE: "avoidance",
    VelocityFactor.EMERGENCY_STOP_OPERATION: "emergency stop operation",
}

steering_type_text = {
    SteeringFactor.INTERSECTION: "intersection",
    SteeringFactor.LANE_CHANGE: "lane change",
    SteeringFactor.AVOIDANCE_PATH_CHANGE: "avoidance change",
    SteeringFactor.AVOIDANCE_PATH_RETURN: "avoidance return",
    SteeringFactor.STATION: "station",
    SteeringFactor.START_PLANNER: "start planner",
    SteeringFactor.GOAL_PLANNER: "goal planner",
    SteeringFactor.EMERGENCY_OPERATION: "emergency operation",
}

velocity_status_color = {
    VelocityFactor.UNKNOWN: (1.0, 1.0, 1.0),
    VelocityFactor.APPROACHING: (1.0, 0.0, 0.0),
    VelocityFactor.STOPPED: (1.0, 1.0, 0.0),
}

steering_status_color = {
    SteeringFactor.UNKNOWN: (1.0, 1.0, 1.0),
    SteeringFactor.APPROACHING: (1.0, 0.0, 0.0),
    SteeringFactor.TRYING: (1.0, 1.0, 0.0),
    SteeringFactor.TURNING: (1.0, 1.0, 1.0),
}


class PlanningFactorVisualizer(rclpy.node.Node):
    def __init__(self):
        super().__init__("planning_factor_visualizer")
        self.front_offset = self.declare_parameter("front_offset", 0.0).value
        self.pub_velocity = self.create_publisher(MarkerArray, "/visualizer/velocity_factors", 1)
        self.pub_steering = self.create_publisher(MarkerArray, "/visualizer/steering_factors", 1)
        self.sub_velocity = self.create_subscription(VelocityFactorArray, "/api/planning/velocity_factors", self.on_velocity_factor, 1)  # fmt: skip
        self.sub_steering = self.create_subscription(SteeringFactorArray, "/api/planning/steering_factors", self.on_steering_factor, 1)  # fmt: skip

    def on_velocity_factor(self, msg):
        markers = MarkerArray()
        for index, factor in enumerate(msg.factors):
            color = velocity_status_color[factor.status]
            text = velocity_type_text[factor.type]
            markers.markers.append(self.create_wall(index, msg.header, factor.pose, color))
            markers.markers.append(self.create_text(index, msg.header, factor.pose, text))
        self.pub_velocity.publish(markers)

    def on_steering_factor(self, msg):
        markers = MarkerArray()
        for index, factor in enumerate(msg.factors):
            index1 = index * 2 + 0
            index2 = index * 2 + 1
            color = steering_status_color[factor.status]
            text = steering_type_text[factor.type]
            markers.markers.append(self.create_wall(index1, msg.header, factor.pose[0], color))
            markers.markers.append(self.create_wall(index2, msg.header, factor.pose[1], color))
            markers.markers.append(self.create_text(index1, msg.header, factor.pose[0], text))
            markers.markers.append(self.create_text(index2, msg.header, factor.pose[1], text))
        self.pub_steering.publish(markers)

    def offset_pose(self, pose):
        q = pose.orientation
        q = numpy.array([q.x, q.y, q.z, q.w])
        p = numpy.array([self.front_offset, 0, 0, 1])
        r = tf_transformations.quaternion_matrix(q)
        x = numpy.dot(r, p)
        result = Pose()
        result.position.x = pose.position.x + x[0]
        result.position.y = pose.position.y + x[1]
        result.position.z = pose.position.z + x[2]
        result.orientation = pose.orientation
        return result

    def create_wall(self, index, header, pose, color):
        pose = self.offset_pose(pose)
        marker = Marker()
        marker.ns = "wall"
        marker.id = index
        marker.lifetime = rclpy.duration.Duration(nanoseconds=0.3 * 1e9).to_msg()
        marker.header = header
        marker.action = Marker.ADD
        marker.type = Marker.CUBE
        marker.pose.position.x = pose.position.x
        marker.pose.position.y = pose.position.y
        marker.pose.position.z = pose.position.z + 1.0
        marker.pose.orientation = pose.orientation
        marker.scale.x = 0.1
        marker.scale.y = 5.0
        marker.scale.z = 2.0
        marker.color.a = 0.7
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        return marker

    def create_text(self, index, header, pose, text):
        pose = self.offset_pose(pose)
        marker = Marker()
        marker.ns = "type"
        marker.id = index
        marker.lifetime = rclpy.duration.Duration(nanoseconds=0.3 * 1e9).to_msg()
        marker.header = header
        marker.action = Marker.ADD
        marker.type = Marker.TEXT_VIEW_FACING
        marker.text = text
        marker.pose.position.x = pose.position.x
        marker.pose.position.y = pose.position.y
        marker.pose.position.z = pose.position.z + 2.0
        marker.pose.orientation = pose.orientation
        marker.scale.z = 1.0
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        return marker


def main(args=None):
    try:
        rclpy.init(args=args)
        rclpy.spin(PlanningFactorVisualizer())
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

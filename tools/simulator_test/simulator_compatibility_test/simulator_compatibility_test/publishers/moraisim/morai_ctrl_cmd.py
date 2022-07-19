from enum import Enum

from morai_msgs.msg import CtrlCmd
import rclpy
from rclpy.node import Node


class LongCmdType(Enum):
    NONE = 0
    THROTTLE = 1
    VELOCITY = 2
    ACCELERATION = 3


class PublisherMoraiCtrlCmd(Node):
    def __init__(self):
        super().__init__("CtrlCmd")
        self.topic = "/ctrl_cmd"
        self.publisher_ = self.create_publisher(CtrlCmd, self.topic, 10)

    def publish_msg(self, cmd):
        msg = CtrlCmd()
        msg.longl_cmd_type = cmd["longCmdType"]
        msg.accel = cmd["accel"]
        msg.brake = cmd["brake"]
        msg.steering = cmd["steering"]
        msg.velocity = cmd["velocity"]
        msg.acceleration = cmd["acceleration"]
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    publisher = PublisherMoraiCtrlCmd()
    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        publisher.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

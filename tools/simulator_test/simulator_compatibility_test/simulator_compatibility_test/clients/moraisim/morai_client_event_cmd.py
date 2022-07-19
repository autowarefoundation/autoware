from morai_msgs.srv import MoraiEventCmdSrv
import rclpy
from rclpy.node import Node


class ClientEventCmdAsync(Node):
    def __init__(self):
        super().__init__("MoraiEventCmdSrv")
        self.client_ = self.create_client(MoraiEventCmdSrv, "morai_msgs/MoraiEventCmdSrv")
        while not self.client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting ...")
        self.msg_ = MoraiEventCmdSrv.Request()

    def send_request(self):
        self.msg_.request.option = 1
        self.msg_.request.ctrl_mode = 3
        self.msg_.request.gear = 0
        self.msg_.request.lamps.turn_signal = 0
        self.msg_.request.lamps.emergency_signal = 0
        self.msg_.request.set_pause = False
        self.future = self.client_.call_async(self.msg_)


def main(args=None):
    rclpy.init(args=args)

    event_cmd_client = ClientEventCmdAsync()
    event_cmd_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(event_cmd_client)
        if event_cmd_client.future.done():
            result_msg = event_cmd_client.future.result()
            event_cmd_client.get_logger().info(
                f"Change Control Mode : \
                {result_msg.response.gear}"
            )
            break

    event_cmd_client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

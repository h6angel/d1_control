import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ddt_msgs.msg import UserCommand


class D1UserCommandBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("d1_user_command_bridge_node")

        self.declare_parameter("robot_ns", "d15019979")
        self.declare_parameter("input_twist_topic", "/d1_demo/cmd_vel_raw")
        self.declare_parameter("output_user_command_topic", "")
        self.declare_parameter("fsm_mode", "loco")
        self.declare_parameter("frame_id", "base_link")

        self.robot_ns = str(self.get_parameter("robot_ns").value).strip("/")
        self.input_twist_topic = self.get_parameter("input_twist_topic").value
        configured_out = self.get_parameter("output_user_command_topic").value
        self.fsm_mode = self.get_parameter("fsm_mode").value
        self.frame_id = self.get_parameter("frame_id").value

        if configured_out:
            self.output_topic = configured_out
        else:
            self.output_topic = f"/{self.robot_ns}/command/user_command"

        self.user_pub = self.create_publisher(UserCommand, self.output_topic, 20)
        self.twist_sub = self.create_subscription(Twist, self.input_twist_topic, self.twist_cb, 20)

        self.get_logger().info(
            f"Bridge {self.input_twist_topic} -> {self.output_topic}, fsm_mode={self.fsm_mode}"
        )

    def twist_cb(self, msg: Twist) -> None:
        out = UserCommand()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id
        out.fsm_mode = self.fsm_mode

        # D1 only needs linear.x and angular.z for this demo.
        out.twist.linear.x = float(msg.linear.x)
        out.twist.linear.y = 0.0
        out.twist.linear.z = 0.0
        out.twist.angular.x = 0.0
        out.twist.angular.y = 0.0
        out.twist.angular.z = float(msg.angular.z)

        # Keep pose defaults; only twist command is used in this demo.
        out.pose.orientation.w = 1.0

        self.user_pub.publish(out)


def main() -> None:
    rclpy.init()
    node = D1UserCommandBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

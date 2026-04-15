import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class FakeBsplineToPathNode(Node):
    """
    Temporary adapter for demo.
    TODO(user): replace this node with real EgoPlanner bspline parser.
    """

    def __init__(self) -> None:
        super().__init__("fake_bspline_to_path_node")

        self.declare_parameter("input_path_topic", "/d1_demo/fake_bspline_path")
        self.declare_parameter("output_path_topic", "/d1_demo/path")
        self.declare_parameter("output_frame_id", "global")

        self.input_topic = self.get_parameter("input_path_topic").value
        self.output_topic = self.get_parameter("output_path_topic").value
        self.output_frame_id = self.get_parameter("output_frame_id").value

        self.path_pub = self.create_publisher(Path, self.output_topic, 10)
        self.path_sub = self.create_subscription(Path, self.input_topic, self.path_cb, 10)

        self.get_logger().warn(
            "[FAKE] Using Path as placeholder for bspline input. "
            "Replace with real bspline topic/msg later."
        )

    def path_cb(self, msg: Path) -> None:
        out = Path()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.output_frame_id

        for pose_stamped in msg.poses:
            ps = PoseStamped()
            ps.header = out.header
            ps.pose = pose_stamped.pose
            out.poses.append(ps)

        self.path_pub.publish(out)


def main() -> None:
    rclpy.init()
    node = FakeBsplineToPathNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

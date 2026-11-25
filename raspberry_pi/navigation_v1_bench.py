import json
import time
import rclpy

from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class NavigationV1Bench(Node):

    def __init__(self):
        super().__init__("navigation_v1_bench")

        self.cmd_pub = self.create_publisher(
            Twist, "/agrobot/bench/cmd_vel", 10
        )

        self.result_sub = self.create_subscription(
            String,
            "/plant_health/result",
            self.result_callback,
            10
        )

        self.state = "waiting"
        self.move_start = 0.0
        self.move_duration = 2.0
        self.forward_speed = 0.045

        self.timer = self.create_timer(
            0.05, self.control_loop
        )

        self.get_logger().info(
            "Navigation V1 Bench ready. Waiting for AI result..."
        )

    def result_callback(self, msg):
        if self.state != "waiting":
            return

        try:
            result = json.loads(msg.data)
        except ValueError:
            return

        if not isinstance(result, dict):
            return

        if result.get("status") != "inspection_complete":
            return

        self.get_logger().info(
            f"Inspection completed: {result.get('prediction')}"
        )

        self.move_start = time.monotonic()
        self.state = "moving"

        self.get_logger().info("Starting forward movement")

    def control_loop(self):
        if self.state == "moving":
            if time.monotonic() - self.move_start < self.move_duration:
                msg = Twist()
                msg.linear.x = self.forward_speed
                self.cmd_pub.publish(msg)
            else:
                self.cmd_pub.publish(Twist())
                self.state = "done"
                self.get_logger().info("Movement complete. Motor stopped.")

        elif self.state == "done":
            self.cmd_pub.publish(Twist())


def main():
    rclpy.init()
    node = NavigationV1Bench()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

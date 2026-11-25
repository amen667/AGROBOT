import json
import rclpy

from rclpy.node import Node
from std_msgs.msg import String


class InspectionController(Node):

    def __init__(self):
        super().__init__("inspection_controller")

        self.subscription = self.create_subscription(
            String,
            "/plant_health/result",
            self.result_callback,
            10
        )

        self.get_logger().info("Waiting for AI inspection results...")

    def result_callback(self, msg):
        try:
            result = json.loads(msg.data)

            if result.get("status") != "inspection_complete":
                return

            self.get_logger().info(
                f"AI result received: {result['prediction']}"
            )

            self.get_logger().info(
                "SIMULATION: Move motor to next inspection position"
            )

        except (ValueError, KeyError, TypeError) as e:
            self.get_logger().error(str(e))


def main():
    rclpy.init()
    node = InspectionController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

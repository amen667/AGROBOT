import json
import time
import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int64
from std_srvs.srv import Trigger


class NavigationV1Auto(Node):

    def __init__(self):
        super().__init__("navigation_v1_auto")

        self.cmd_pub = self.create_publisher(
            Twist, "/agrobot/bench/cmd_vel", 10
        )

        self.encoder_sub = self.create_subscription(
            Int64,
            "/agrobot/bench/encoder_ticks",
            self.encoder_callback,
            10
        )

        self.ai_client = self.create_client(
            Trigger, "/plant_health/inspect"
        )

        self.state = "settling"
        self.state_start = time.monotonic()

        self.encoder = None
        self.last_feedback = 0.0
        self.last_encoder_change = time.monotonic()

        self.future = None

        self.cycle = 0
        self.max_cycles = 3
        self.move_duration = 2.0
        self.forward_speed = 0.045
        self.settle_duration = 0.7
        self.inspection_timeout = 10.0

        self.timer = self.create_timer(
            0.05, self.control_loop
        )

        self.get_logger().info(
            "AGROBOT Automatic Navigation V1 ready"
        )

    def encoder_callback(self, msg):
        now = time.monotonic()
        self.last_feedback = now

        if msg.data != self.encoder:
            self.last_encoder_change = now

        self.encoder = msg.data

    def stop_motor(self):
        self.cmd_pub.publish(Twist())

    def move_forward(self):
        msg = Twist()
        msg.linear.x = self.forward_speed
        self.cmd_pub.publish(msg)

    def fault(self, reason):
        self.stop_motor()
        self.state = "fault"
        self.get_logger().error(reason)

    def control_loop(self):
        now = time.monotonic()

        if self.state in ("done", "fault"):
            self.stop_motor()
            return

        if now - self.last_feedback > 0.5:
            self.stop_motor()

            if self.state != "settling" or \
               now - self.state_start > 5.0:
                self.fault("Encoder feedback unavailable")

            return

        if self.state == "settling":
            self.stop_motor()

            if now - self.state_start > 5.0:
                self.fault("Stop settling or service timeout")
                return

            if now - self.state_start < self.settle_duration:
                return

            if now - self.last_encoder_change < self.settle_duration:
                return

            if self.cycle >= self.max_cycles:
                self.state = "done"
                self.get_logger().info("ALL CYCLES COMPLETED")
                return

            if not self.ai_client.service_is_ready():
                return

            self.get_logger().info(
                f"CYCLE {self.cycle + 1}: Requesting inspection"
            )

            self.future = self.ai_client.call_async(
                Trigger.Request()
            )

            self.state = "inspecting"
            self.state_start = now

        elif self.state == "inspecting":
            self.stop_motor()

            if now - self.state_start > self.inspection_timeout:
                self.fault("AI inspection timeout")
                return

            if now - self.last_encoder_change < self.settle_duration:
                self.fault("Unexpected encoder movement")
                return

            if not self.future.done():
                return

            try:
                response = self.future.result()

                if not response.success:
                    self.fault(response.message)
                    return

                result = json.loads(response.message)

                if result.get("status") != "inspection_complete":
                    self.fault("Invalid inspection result")
                    return

                self.get_logger().info(
                    f"AI RESULT: {result['prediction']}"
                )

            except Exception as e:
                self.fault(str(e))
                return

            self.get_logger().info("MOVING FORWARD")

            self.state = "moving"
            self.state_start = now

        elif self.state == "moving":

            if now - self.state_start < self.move_duration:
                self.move_forward()

            else:
                self.stop_motor()

                self.cycle += 1

                self.get_logger().info(
                    f"Motor stop commanded. Cycle {self.cycle} complete"
                )

                self.state = "settling"
                self.state_start = now


def main():
    rclpy.init()
    node = NavigationV1Auto()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop_motor()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

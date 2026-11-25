import time
import re
import serial
import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int64


class SingleMotorBridge(Node):

    def __init__(self):
        super().__init__("single_motor_bridge")

        self.ser = serial.Serial(
            "/dev/ttyAMA0", 115200,
            timeout=0, write_timeout=0.1
        )

        self.command = b"s"
        self.last_command = 0.0
        self.last_feedback = 0.0
        self.buffer = b""

        self.ser.write(b"s")

        self.subscription = self.create_subscription(
            Twist,
            "/agrobot/bench/cmd_vel",
            self.command_callback,
            10
        )

        self.encoder_pub = self.create_publisher(
            Int64,
            "/agrobot/bench/encoder_ticks",
            10
        )

        self.timer = self.create_timer(0.05, self.update)

        self.get_logger().info("Single motor bridge ready")

    def command_callback(self, msg):
        self.last_command = time.monotonic()

        if msg.linear.x > 0.02:
            self.command = b"f"
        elif msg.linear.x < -0.02:
            self.command = b"b"
        else:
            self.command = b"s"

    def update(self):
        now = time.monotonic()

        self.buffer += self.ser.read(4096)

        while b"\n" in self.buffer:
            line, self.buffer = self.buffer.split(b"\n", 1)

            match = re.search(
                rb"ENC:(-?\d+) ACTIVE:([01])", line
            )

            if match:
                self.last_feedback = now

                msg = Int64()
                msg.data = int(match.group(1))
                self.encoder_pub.publish(msg)

        if len(self.buffer) > 4096:
            self.buffer = b""

        if now - self.last_command > 0.25:
            self.command = b"s"

        if now - self.last_feedback > 0.5:
            self.command = b"s"

        self.ser.write(self.command)


def main():
    rclpy.init()
    node = SingleMotorBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.ser.write(b"s")
        finally:
            node.ser.close()
            node.destroy_node()

            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()

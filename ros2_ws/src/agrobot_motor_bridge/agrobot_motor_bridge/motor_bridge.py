import glob
import os
import time
import serial

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


BAUD = 115200
BOOST_STEPS = 4   # ESP32 speed: 130 + 4*20 = 210


def find_esp32_port():
    by_id_ports = glob.glob("/dev/serial/by-id/*")

    for port in by_id_ports:
        name = os.path.basename(port).lower()
        if any(key in name for key in ["cp210", "ch340", "wch", "silicon", "uart", "usb"]):
            return port

    ports = glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
    if ports:
        return ports[0]

    return None


class MotorBridge(Node):
    def __init__(self):
        super().__init__("agrobot_motor_bridge")

        self.port = find_esp32_port()
        self.ser = None
        self.last_cmd = None
        self.last_msg_time = time.time()

        if self.port is None:
            self.get_logger().error("No ESP32 serial port found.")
            return

        try:
            self.ser = serial.Serial(self.port, BAUD, timeout=1)
            time.sleep(2)
            self.ser.reset_input_buffer()
            self.get_logger().info(f"Connected to ESP32 on {self.port}")
        except Exception as e:
            self.get_logger().error(f"Failed to open ESP32 serial port: {e}")
            self.ser = None
            return

        self.initialize_motor_speed()

        self.sub = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cmd_vel_callback,
            10
        )

        self.watchdog_timer = self.create_timer(0.2, self.watchdog)

        self.get_logger().info("Motor bridge ready. Listening on /cmd_vel")

    def initialize_motor_speed(self):
        if self.ser is None:
            return

        self.ser.write(b"s")
        time.sleep(0.2)

        for _ in range(BOOST_STEPS):
            self.ser.write(b"+")
            time.sleep(0.25)

        self.ser.write(b"s")
        time.sleep(0.2)

        self.get_logger().info(f"Motor speed boosted with {BOOST_STEPS} '+' commands")

    def send_cmd(self, cmd):
        if self.ser is None:
            return

        self.ser.write(cmd.encode())

        if cmd != self.last_cmd:
            self.get_logger().info(f"Sent command: {cmd}")
            self.last_cmd = cmd

    def cmd_vel_callback(self, msg):
        self.last_msg_time = time.time()

        linear = msg.linear.x
        angular = msg.angular.z

        if linear > 0.05:
            self.send_cmd("f")
        elif linear < -0.05:
            self.send_cmd("b")
        elif angular > 0.05:
            self.send_cmd("l")
        elif angular < -0.05:
            self.send_cmd("r")
        else:
            self.send_cmd("s")

    def watchdog(self):
        if time.time() - self.last_msg_time > 0.8:
            self.send_cmd("s")


def main(args=None):
    rclpy.init(args=args)
    node = MotorBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    if node.ser is not None:
        node.ser.write(b"s")
        node.ser.close()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

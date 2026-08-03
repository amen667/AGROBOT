import os
import re
import math
import time
import serial

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


ESP32_BY_ID = "/dev/ttyUSB1"

WHEEL_DIAMETER = 0.065
WHEEL_SEPARATION = 0.161

LEFT_TICKS_PER_REV = 231.0
RIGHT_TICKS_PER_REV = 206.0

LEFT_ENCODER_SIGN = -1.0
RIGHT_ENCODER_SIGN = 1.0

PULSE_SECONDS = 0.5


def find_esp32_port():
    if os.path.exists(ESP32_BY_ID):
        return ESP32_BY_ID

    for port in ["/dev/ttyUSB1", "/dev/ttyUSB0", "/dev/ttyACM0"]:
        if os.path.exists(port):
            return port

    return None


class AgrobotMotorBridge(Node):
    def __init__(self):
        super().__init__("agrobot_motor_bridge")

        self.ser = None
        self.last_cmd = "s"
        self.active_pulse_cmd = None
        self.pulse_stop_time = 0.0

        self.last_left_raw = None
        self.last_right_raw = None

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.ticks_pub = self.create_publisher(String, "/wheel_ticks", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.cmd_sub = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cmd_vel_callback,
            10,
        )

        port = find_esp32_port()
        if port is None:
            self.get_logger().error("ESP32 serial port not found")
            return

        self.ser = serial.Serial(port, 115200, timeout=0.05)
        time.sleep(2.0)

        self.get_logger().info(f"Connected to ESP32 on {port}")
        self.get_logger().info("Pulse bridge ready: strong speed, 0.5s movement, auto-stop")
        self.get_logger().info("Listening on /cmd_vel")
        self.get_logger().info("Publishing /wheel_ticks, /odom, and TF odom -> base_link")

        self.send_stop()

        self.serial_timer = self.create_timer(0.05, self.read_serial)
        self.pulse_timer = self.create_timer(0.02, self.check_pulse_timeout)

    def write_byte(self, b):
        if self.ser is not None:
            self.ser.write(b)

    def send_stop(self):
        for _ in range(3):
            self.write_byte(b"s")
            time.sleep(0.02)

        if self.last_cmd != "s":
            self.get_logger().info("Sent command: s")

        self.last_cmd = "s"
        self.active_pulse_cmd = None
        self.pulse_stop_time = 0.0

    def send_pulse(self, cmd):
        now = time.time()

        if self.active_pulse_cmd == cmd and now < self.pulse_stop_time:
            return

        for _ in range(10):
            self.write_byte(b"+")
            time.sleep(0.01)

        for _ in range(3):
            self.write_byte(cmd.encode())
            time.sleep(0.02)

        self.active_pulse_cmd = cmd
        self.pulse_stop_time = time.time() + PULSE_SECONDS
        self.last_cmd = cmd

        self.get_logger().info(f"Sent pulse command: {cmd} for {PULSE_SECONDS}s")

    def check_pulse_timeout(self):
        if self.active_pulse_cmd is not None and time.time() >= self.pulse_stop_time:
            self.send_stop()
            self.get_logger().info("Auto-stop after pulse")

    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        if abs(linear_x) < 0.01 and abs(angular_z) < 0.01:
            self.send_stop()
            return

        if linear_x > 0.01:
            self.send_pulse("f")
        elif linear_x < -0.01:
            self.send_pulse("b")
        elif angular_z > 0.01:
            self.send_pulse("l")
        elif angular_z < -0.01:
            self.send_pulse("r")

    def read_serial(self):
        if self.ser is None:
            return

        try:
            while self.ser.in_waiting:
                line = self.ser.readline().decode(errors="ignore").strip()
                if not line:
                    continue

                self.ticks_pub.publish(String(data=line))

                match = re.search(r"L:\s*(-?\d+)\s+R:\s*(-?\d+)", line)
                if match:
                    left_raw = int(match.group(1))
                    right_raw = int(match.group(2))
                    self.update_odom(left_raw, right_raw)

        except serial.SerialException as e:
            self.get_logger().error(f"Serial error: {e}")

    def update_odom(self, left_raw, right_raw):
        if self.last_left_raw is None:
            self.last_left_raw = left_raw
            self.last_right_raw = right_raw
            return

        delta_left_ticks = (left_raw - self.last_left_raw) * LEFT_ENCODER_SIGN
        delta_right_ticks = (right_raw - self.last_right_raw) * RIGHT_ENCODER_SIGN

        self.last_left_raw = left_raw
        self.last_right_raw = right_raw

        left_dist = (delta_left_ticks / LEFT_TICKS_PER_REV) * math.pi * WHEEL_DIAMETER
        right_dist = (delta_right_ticks / RIGHT_TICKS_PER_REV) * math.pi * WHEEL_DIAMETER

        distance = (left_dist + right_dist) / 2.0
        delta_theta = (right_dist - left_dist) / WHEEL_SEPARATION

        self.x += distance * math.cos(self.theta + delta_theta / 2.0)
        self.y += distance * math.sin(self.theta + delta_theta / 2.0)
        self.theta += delta_theta

        now = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_link"
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.z = math.sin(self.theta / 2.0)
        tf.transform.rotation.w = math.cos(self.theta / 2.0)

        self.tf_broadcaster.sendTransform(tf)

    def destroy_node(self):
        try:
            self.send_stop()
            if self.ser is not None:
                self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AgrobotMotorBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

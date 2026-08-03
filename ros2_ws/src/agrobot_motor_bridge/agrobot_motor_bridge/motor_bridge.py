import glob
import os
import math
import re
import time
import serial

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


BAUD = 115200
BOOST_STEPS = 4

WHEEL_DIAMETER = 0.065      # meters
WHEEL_SEPARATION = 0.161    # meters, center-to-center
LEFT_TICKS_PER_REV = 231.0
RIGHT_TICKS_PER_REV = 206.0

LEFT_ENCODER_SIGN = -1.0
RIGHT_ENCODER_SIGN = 1.0

WHEEL_CIRCUMFERENCE = math.pi * WHEEL_DIAMETER
LEFT_METERS_PER_TICK = WHEEL_CIRCUMFERENCE / LEFT_TICKS_PER_REV
RIGHT_METERS_PER_TICK = WHEEL_CIRCUMFERENCE / RIGHT_TICKS_PER_REV


def find_esp32_port():
    # In combined test:
    # /dev/ttyUSB0 = LiDAR
    # /dev/ttyUSB1 = ESP32
    if os.path.exists("/dev/ttyUSB1"):
        return "/dev/ttyUSB1"

    if os.path.exists("/dev/ttyACM0"):
        return "/dev/ttyACM0"

    if os.path.exists("/dev/ttyUSB0"):
        return "/dev/ttyUSB0"

    return None


class MotorBridge(Node):
    def __init__(self):
        super().__init__("agrobot_motor_bridge")

        self.port = find_esp32_port()
        self.ser = None
        self.last_cmd = None
        self.last_msg_time = time.time()

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.last_left_ticks = None
        self.last_right_ticks = None
        self.last_odom_time = self.get_clock().now()

        self.encoder_pub = self.create_publisher(String, "/wheel_ticks", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        if self.port is None:
            self.get_logger().error("No ESP32 serial port found.")
            return

        try:
            self.ser = serial.Serial(self.port, BAUD, timeout=0.05)
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
        self.serial_timer = self.create_timer(0.05, self.read_serial)

        self.get_logger().info("Motor bridge ready. Listening on /cmd_vel")
        self.get_logger().info("Publishing encoder ticks on /wheel_ticks")
        self.get_logger().info("Publishing odometry on /odom and TF odom -> base_link")

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

    def read_serial(self):
        if self.ser is None:
            return

        try:
            while self.ser.in_waiting:
                line = self.ser.readline().decode(errors="ignore").strip()

                if not line:
                    continue

                if line.startswith("L:") and "R:" in line:
                    msg = String()
                    msg.data = line
                    self.encoder_pub.publish(msg)

                    match = re.search(r"L:([-0-9]+)\s+R:([-0-9]+)", line)
                    if match:
                        left_ticks = int(match.group(1))
                        right_ticks = int(match.group(2))
                        self.update_odometry(left_ticks, right_ticks)

        except Exception as e:
            self.get_logger().warn(f"Serial read error: {e}")

    def update_odometry(self, left_ticks, right_ticks):
        now = self.get_clock().now()

        if self.last_left_ticks is None or self.last_right_ticks is None:
            self.last_left_ticks = left_ticks
            self.last_right_ticks = right_ticks
            self.last_odom_time = now
            return

        dt = (now - self.last_odom_time).nanoseconds / 1e9
        if dt <= 0.0:
            dt = 0.001

        delta_left_ticks = (left_ticks - self.last_left_ticks) * LEFT_ENCODER_SIGN
        delta_right_ticks = (right_ticks - self.last_right_ticks) * RIGHT_ENCODER_SIGN

        self.last_left_ticks = left_ticks
        self.last_right_ticks = right_ticks
        self.last_odom_time = now

        dl = delta_left_ticks * LEFT_METERS_PER_TICK
        dr = delta_right_ticks * RIGHT_METERS_PER_TICK

        dc = (dl + dr) / 2.0
        dtheta = (dr - dl) / WHEEL_SEPARATION

        self.x += dc * math.cos(self.theta + dtheta / 2.0)
        self.y += dc * math.sin(self.theta + dtheta / 2.0)
        self.theta += dtheta

        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = dc / dt
        odom.twist.twist.angular.z = dtheta / dt

        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = now.to_msg()
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_link"
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(tf)


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

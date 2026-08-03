import glob
import math
import re
import serial
import threading
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32MultiArray
from tf2_ros import TransformBroadcaster


WHEEL_DIAMETER = 0.065
WHEEL_SEPARATION = 0.161

LEFT_TICKS_PER_REV = 231.0
RIGHT_TICKS_PER_REV = 206.0

LEFT_ENCODER_SIGN = -1.0
RIGHT_ENCODER_SIGN = 1.0

DEFAULT_SPEED = 120
CMD_TIMEOUT = 0.7


def find_esp32_port():
    ports = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
    print("PORTS:", ports)

    if not ports:
        raise RuntimeError("NO USB SERIAL PORT FOUND")

    pattern = re.compile(r"^L:\s*-?\d+\s+R:\s*-?\d+\s+Speed:\s*\d+")

    for port in ports:
        print("Testing port:", port)

        try:
            ser = serial.Serial()
            ser.port = port
            ser.baudrate = 115200
            ser.timeout = 0.2
            ser.dtr = False
            ser.rts = False
            ser.open()
            time.sleep(4)

            start = time.time()
            while time.time() - start < 8:
                raw = ser.readline()
                line = raw.decode(errors="ignore").strip()

                if line:
                    print(port, "=>", line[:80])

                if "AGROBOT" in line or "Commands:" in line or pattern.match(line):
                    ser.close()
                    print("ESP32 FOUND:", port)
                    return port

                ser.write(b"s")
                ser.flush()
                time.sleep(0.1)

            ser.close()

        except Exception as e:
            print("Failed testing", port, e)

    raise RuntimeError("ESP32 NOT FOUND. Press EN/RST on ESP32 and run again.")


class AgrobotMotorBridge(Node):
    def __init__(self):
        super().__init__("agrobot_motor_bridge")

        self.port = find_esp32_port()
        self.get_logger().info(f"Opening ESP32 on {self.port}")

        self.ser = serial.Serial()
        self.ser.port = self.port
        self.ser.baudrate = 115200
        self.ser.timeout = 0.1
        self.ser.dtr = False
        self.ser.rts = False
        self.ser.open()
        time.sleep(4)

        self.serial_lock = threading.Lock()
        self.data_lock = threading.Lock()

        self.left_count = None
        self.right_count = None
        self.prev_left_count = None
        self.prev_right_count = None
        self.prev_odom_time = None

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.current_cmd = "s"
        self.last_cmd_time = time.time()

        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.ticks_pub = self.create_publisher(Int32MultiArray, "/wheel_ticks", 10)
        self.cmd_sub = self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.reader_thread = threading.Thread(target=self.serial_reader, daemon=True)
        self.reader_thread.start()

        self.wait_for_first_ticks()
        self.set_speed(DEFAULT_SPEED)

        self.odom_timer = self.create_timer(0.05, self.publish_odom)
        self.safety_timer = self.create_timer(0.1, self.safety_check)

        self.get_logger().info("AGROBOT motor bridge ready")
        self.get_logger().info("Subscribing: /cmd_vel")
        self.get_logger().info("Publishing: /odom, /wheel_ticks, TF odom -> base_link")

    def wait_for_first_ticks(self):
        self.get_logger().info("Waiting for ESP32 encoder lines...")
        start = time.time()

        while time.time() - start < 15:
            with self.data_lock:
                if self.left_count is not None and self.right_count is not None:
                    self.get_logger().info("Encoder data received")
                    return
            time.sleep(0.1)

        raise RuntimeError("No encoder lines from ESP32. Press EN/RST and run again.")

    def serial_reader(self):
        pattern = re.compile(r"^L:\s*(-?\d+)\s+R:\s*(-?\d+)\s+Speed:\s*(\d+)")

        while rclpy.ok():
            try:
                line = self.ser.readline().decode(errors="ignore").strip()

                if not line:
                    continue

                match = pattern.match(line)
                if match:
                    left = int(match.group(1))
                    right = int(match.group(2))

                    with self.data_lock:
                        self.left_count = left
                        self.right_count = right

            except Exception as e:
                self.get_logger().error(f"Serial read error: {e}")
                time.sleep(0.2)

    def send_char(self, ch, repeat=1, delay=0.02):
        with self.serial_lock:
            for _ in range(repeat):
                self.ser.write(ch.encode())
                self.ser.flush()
                time.sleep(delay)

    def stop_robot(self):
        self.send_char("s", repeat=5, delay=0.02)
        self.current_cmd = "s"

    def set_speed(self, target_speed):
        target_speed = max(0, min(255, int(target_speed)))

        self.get_logger().info(f"Setting ESP32 speed to about {target_speed}")

        self.send_char("-", repeat=20, delay=0.01)

        steps = int(target_speed / 20)
        self.send_char("+", repeat=steps, delay=0.01)

    def cmd_vel_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z

        self.last_cmd_time = time.time()

        if abs(linear) < 0.02 and abs(angular) < 0.05:
            desired = "s"
        elif abs(angular) > abs(linear):
            desired = "l" if angular > 0 else "r"
        else:
            desired = "f" if linear > 0 else "b"

        if desired != self.current_cmd:
            self.send_char(desired, repeat=3, delay=0.02)
            self.current_cmd = desired
            self.get_logger().info(f"Sent command: {desired}")

    def safety_check(self):
        if self.current_cmd != "s" and time.time() - self.last_cmd_time > CMD_TIMEOUT:
            self.get_logger().info("CMD timeout -> stop")
            self.stop_robot()

    def publish_odom(self):
        now = self.get_clock().now()
        now_sec = time.time()

        with self.data_lock:
            if self.left_count is None or self.right_count is None:
                return

            left = self.left_count
            right = self.right_count

        ticks_msg = Int32MultiArray()
        ticks_msg.data = [left, right]
        self.ticks_pub.publish(ticks_msg)

        if self.prev_left_count is None:
            self.prev_left_count = left
            self.prev_right_count = right
            self.prev_odom_time = now_sec
            return

        dt = now_sec - self.prev_odom_time
        if dt <= 0.0:
            return

        delta_left_ticks = left - self.prev_left_count
        delta_right_ticks = right - self.prev_right_count

        self.prev_left_count = left
        self.prev_right_count = right
        self.prev_odom_time = now_sec

        wheel_circumference = math.pi * WHEEL_DIAMETER

        left_distance = (
            delta_left_ticks
            * LEFT_ENCODER_SIGN
            / LEFT_TICKS_PER_REV
            * wheel_circumference
        )

        right_distance = (
            delta_right_ticks
            * RIGHT_ENCODER_SIGN
            / RIGHT_TICKS_PER_REV
            * wheel_circumference
        )

        distance = (left_distance + right_distance) / 2.0
        delta_theta = (right_distance - left_distance) / WHEEL_SEPARATION

        theta_mid = self.theta + delta_theta / 2.0

        self.x += distance * math.cos(theta_mid)
        self.y += distance * math.sin(theta_mid)
        self.theta += delta_theta

        linear_velocity = distance / dt
        angular_velocity = delta_theta / dt

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

        odom.twist.twist.linear.x = linear_velocity
        odom.twist.twist.angular.z = angular_velocity

        odom.pose.covariance[0] = 0.05
        odom.pose.covariance[7] = 0.05
        odom.pose.covariance[35] = 0.1

        odom.twist.covariance[0] = 0.1
        odom.twist.covariance[35] = 0.2

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

    node = AgrobotMotorBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

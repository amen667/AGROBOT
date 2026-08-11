import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class SafeStepNavigator(Node):
    def __init__(self):
        super().__init__("agrobot_safe_step_navigation")

        self.pose_received = False
        self.scan_received = False

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.scan = None
        self.last_scan_time = 0.0

        self.active = False
        self.mode = "idle"
        self.phase = "idle"
        self.phase_start = time.time()
        self.pulse_count = 0

        self.start_x = 0.0
        self.start_y = 0.0
        self.start_yaw = 0.0

        self.target_distance = 0.0
        self.target_angle = 0.0
        self.turn_direction = 1

        # Forward movement settings
        self.forward_speed = 0.045
        self.forward_pulse_seconds = 0.12
        self.forward_wait_seconds = 0.25
        self.distance_tolerance = 0.015

        # Turning settings
        self.turn_speed = 0.14
        self.turn_pulse_seconds = 0.08
        self.turn_wait_seconds = 0.25
        self.angle_tolerance = math.radians(2.0)

        # Safety
        self.safety_front_stop = 0.35
        self.max_pulses = 8

        self.last_log = 0.0

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10,
        )

        scan_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            scan_qos,
        )

        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("AGROBOT SAFE STEP NAVIGATION ready")
        self.get_logger().info("Commands: status / pose / forward D / left DEG / right DEG / stop / q")

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.pose_received = True

    def scan_callback(self, msg):
        self.scan = msg
        self.last_scan_time = time.time()
        self.scan_received = True

    def publish_stop(self):
        self.cmd_pub.publish(Twist())

    def publish_forward(self):
        msg = Twist()
        msg.linear.x = self.forward_speed
        msg.angular.z = 0.0
        self.cmd_pub.publish(msg)

    def publish_turn(self, direction):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = self.turn_speed * direction
        self.cmd_pub.publish(msg)

    def sector_min(self, center_deg, width_deg):
        if self.scan is None:
            return None

        center = math.radians(center_deg)
        half = math.radians(width_deg / 2.0)
        values = []

        for i, r in enumerate(self.scan.ranges):
            if not math.isfinite(r):
                continue

            if r < self.scan.range_min or r > self.scan.range_max:
                continue

            angle = self.scan.angle_min + i * self.scan.angle_increment
            diff = normalize_angle(angle - center)

            if abs(diff) <= half:
                values.append(r)

        if not values:
            return None

        return min(values)

    def traveled_distance(self):
        return math.hypot(self.x - self.start_x, self.y - self.start_y)

    def turned_angle(self):
        return abs(normalize_angle(self.yaw - self.start_yaw))

    def log_limited(self, text):
        now = time.time()
        if now - self.last_log > 0.5:
            self.get_logger().info(text)
            self.last_log = now

    def start_forward(self, distance):
        if not self.pose_received:
            self.get_logger().warn("Cannot start: no /odom yet")
            return

        if not self.scan_received:
            self.get_logger().warn("Cannot start: no /scan yet")
            return

        front = self.sector_min(0, 45)

        if front is not None and front < self.safety_front_stop:
            self.publish_stop()
            self.get_logger().warn(f"Refused to move: obstacle/wall ahead at {front:.2f} m")
            return

        self.start_x = self.x
        self.start_y = self.y
        self.start_yaw = self.yaw
        self.target_distance = abs(distance)

        self.max_pulses = max(12, min(120, int(self.target_distance / 0.010) + 20))

        self.active = True
        self.mode = "forward"
        self.phase = "pulse"
        self.phase_start = time.time()
        self.pulse_count = 0

        self.get_logger().info("")
        self.get_logger().info("NEW FORWARD GOAL")
        self.get_logger().info(f"Goal: move straight forward {self.target_distance:.2f} m")
        self.get_logger().info("No turning. /odom distance stop. LiDAR safety enabled.")
        self.get_logger().info("")

    def start_turn(self, degrees):
        if not self.pose_received:
            self.get_logger().warn("Cannot start: no /odom yet")
            return

        if abs(degrees) < 0.1:
            self.get_logger().warn("Turn angle too small")
            return

        self.start_x = self.x
        self.start_y = self.y
        self.start_yaw = self.yaw

        self.target_angle = abs(math.radians(degrees))
        self.turn_direction = 1 if degrees > 0 else -1

        self.max_pulses = max(6, min(40, int(abs(degrees) / 1.5) + 4))

        self.active = True
        self.mode = "turn"
        self.phase = "pulse"
        self.phase_start = time.time()
        self.pulse_count = 0

        direction_name = "left" if self.turn_direction > 0 else "right"

        self.get_logger().info("")
        self.get_logger().info("NEW TURN GOAL")
        self.get_logger().info(f"Goal: turn {direction_name} about {abs(degrees):.1f} degrees")
        self.get_logger().info("No forward movement. /odom yaw stop.")
        self.get_logger().info("")

    def stop_navigation(self):
        self.active = False
        self.mode = "idle"
        self.phase = "idle"

        for _ in range(5):
            self.publish_stop()
            time.sleep(0.03)

        self.get_logger().info("STOPPED")

    def control_loop(self):
        if not self.active:
            self.publish_stop()
            return

        if not self.pose_received:
            self.publish_stop()
            self.log_limited("Waiting for /odom...")
            return

        now = time.time()

        if self.mode == "forward":
            if not self.scan_received or time.time() - self.last_scan_time > 1.0:
                self.active = False
                self.publish_stop()
                self.get_logger().warn("Safety stop: /scan missing or timeout")
                return

            front = self.sector_min(0, 45)
            moved = self.traveled_distance()

            if front is not None and front < self.safety_front_stop:
                self.active = False
                self.publish_stop()
                self.get_logger().warn(
                    f"Safety stop: obstacle/wall ahead at {front:.2f} m. Navigation cancelled."
                )
                return

            if moved >= max(0.0, self.target_distance - self.distance_tolerance):
                self.active = False
                self.publish_stop()
                self.get_logger().info(
                    f"Forward reached ✅ moved={moved:.2f} m target={self.target_distance:.2f} m"
                )
                return

            if self.pulse_count >= self.max_pulses:
                self.active = False
                self.publish_stop()
                self.get_logger().warn(
                    f"Stopped: max pulses reached. moved={moved:.2f} m target={self.target_distance:.2f} m"
                )
                return

            if self.phase == "pulse":
                self.publish_forward()

                if now - self.phase_start >= self.forward_pulse_seconds:
                    self.phase = "wait"
                    self.phase_start = now
                    self.pulse_count += 1
                    self.publish_stop()

            elif self.phase == "wait":
                self.publish_stop()

                if now - self.phase_start >= self.forward_wait_seconds:
                    self.phase = "pulse"
                    self.phase_start = now

            self.log_limited(
                f"FORWARD | moved={moved:.2f}/{self.target_distance:.2f} m "
                f"front={front if front is not None else -1:.2f} pulses={self.pulse_count}"
            )

        elif self.mode == "turn":
            turned = self.turned_angle()

            if turned >= max(0.0, self.target_angle - self.angle_tolerance):
                self.active = False
                self.publish_stop()
                self.get_logger().info(
                    f"Turn reached ✅ turned={math.degrees(turned):.1f} deg "
                    f"target={math.degrees(self.target_angle):.1f} deg"
                )
                return

            if self.pulse_count >= self.max_pulses:
                self.active = False
                self.publish_stop()
                self.get_logger().warn(
                    f"Stopped: max turn pulses reached. turned={math.degrees(turned):.1f} deg "
                    f"target={math.degrees(self.target_angle):.1f} deg"
                )
                return

            if self.phase == "pulse":
                self.publish_turn(self.turn_direction)

                if now - self.phase_start >= self.turn_pulse_seconds:
                    self.phase = "wait"
                    self.phase_start = now
                    self.pulse_count += 1
                    self.publish_stop()

            elif self.phase == "wait":
                self.publish_stop()

                if now - self.phase_start >= self.turn_wait_seconds:
                    self.phase = "pulse"
                    self.phase_start = now

            direction_name = "left" if self.turn_direction > 0 else "right"
            self.log_limited(
                f"TURN {direction_name.upper()} | turned={math.degrees(turned):.1f}/"
                f"{math.degrees(self.target_angle):.1f} deg pulses={self.pulse_count}"
            )


def console_thread(node):
    print("")
    print("AGROBOT SAFE STEP NAVIGATION")
    print("")
    print("Commands:")
    print("  status")
    print("  pose")
    print("  forward D     -> move forward D meters, example: forward 0.10")
    print("  left DEG      -> turn left, example: left 5")
    print("  right DEG     -> turn right, example: right 5")
    print("  stop")
    print("  q")
    print("")
    print("First turn test: left 5")
    print("")

    while rclpy.ok():
        try:
            raw = input("step-nav> ").strip()
        except EOFError:
            break

        if not raw:
            continue

        parts = raw.split()
        cmd = parts[0].lower()

        try:
            if cmd == "status":
                print("pose received =", node.pose_received)
                print("scan received =", node.scan_received)
                print("active =", node.active)
                print("mode =", node.mode)

            elif cmd == "pose":
                print(f"x={node.x:.3f}, y={node.y:.3f}, yaw={node.yaw:.3f}")

            elif cmd == "forward":
                if len(parts) != 2:
                    print("Usage: forward 0.10")
                    continue
                node.start_forward(float(parts[1]))

            elif cmd == "left":
                if len(parts) != 2:
                    print("Usage: left 5")
                    continue
                node.start_turn(abs(float(parts[1])))

            elif cmd == "right":
                if len(parts) != 2:
                    print("Usage: right 5")
                    continue
                node.start_turn(-abs(float(parts[1])))

            elif cmd == "stop":
                node.stop_navigation()

            elif cmd == "q":
                node.stop_navigation()
                rclpy.shutdown()
                break

            else:
                print("Use: status / pose / forward D / left DEG / right DEG / stop / q")

        except ValueError:
            print("Bad number. Example: forward 0.10 or left 5")


def main(args=None):
    rclpy.init(args=args)
    node = SafeStepNavigator()

    thread = threading.Thread(target=console_thread, args=(node,), daemon=True)
    thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_navigation()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

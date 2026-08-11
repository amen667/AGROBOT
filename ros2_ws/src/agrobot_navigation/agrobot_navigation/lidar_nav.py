import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class NavigationV1(Node):
    def __init__(self):
        super().__init__("agrobot_navigation_v1")

        self.odom_received = False
        self.amcl_received = False
        self.scan_received = False

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0

        self.map_x = 0.0
        self.map_y = 0.0
        self.map_yaw = 0.0

        self.scan = None
        self.last_scan_time = 0.0

        self.active = False
        self.mode = "idle"
        self.phase = "idle"
        self.phase_start = time.time()
        self.pulse_count = 0

        self.start_odom_x = 0.0
        self.start_odom_y = 0.0
        self.start_odom_yaw = 0.0

        self.target_distance = 0.0
        self.target_angle = 0.0
        self.turn_direction = 1

        # Forward settings
        self.forward_speed = 0.045
        self.forward_pulse_seconds = 0.12
        self.forward_wait_seconds = 0.25
        self.distance_tolerance = 0.015

        # Turn settings
        self.turn_speed = 0.14
        self.turn_pulse_seconds = 0.08
        self.turn_wait_seconds = 0.25
        self.angle_tolerance = math.radians(2.0)

        # Safety
        self.safety_front_stop = 0.35
        self.max_pulses = 30

        # Practical calibration from real tests:
        # command 45 deg gives about real 90 deg on this robot.
        self.turn_command_factor = 0.50

        # Goto settings
        self.goto_tolerance = 0.18
        self.goto_max_step = 0.50
        self.goto_max_cycles = 4
        self.goto_busy = False

        self.last_log = 0.0

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10,
        )

        amcl_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.amcl_callback,
            amcl_qos,
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

        self.get_logger().info("AGROBOT Navigation V1 ready")
        self.get_logger().info("Commands: status / pose / forward D / left DEG / right DEG / goto X Y / stop / q")

    def odom_callback(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.odom_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.odom_received = True

    def amcl_callback(self, msg):
        self.map_x = msg.pose.pose.position.x
        self.map_y = msg.pose.pose.position.y
        self.map_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.amcl_received = True

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
        return math.hypot(
            self.odom_x - self.start_odom_x,
            self.odom_y - self.start_odom_y,
        )

    def turned_angle(self):
        return abs(normalize_angle(self.odom_yaw - self.start_odom_yaw))

    def log_limited(self, text):
        now = time.time()
        if now - self.last_log > 0.5:
            self.get_logger().info(text)
            self.last_log = now

    def start_forward(self, distance):
        if not self.odom_received:
            self.get_logger().warn("Cannot start forward: no /odom yet")
            return False

        if not self.scan_received:
            self.get_logger().warn("Cannot start forward: no /scan yet")
            return False

        front = self.sector_min(0, 45)

        if front is not None and front < self.safety_front_stop:
            self.publish_stop()
            self.get_logger().warn(f"Refused forward: obstacle/wall ahead at {front:.2f} m")
            return False

        self.start_odom_x = self.odom_x
        self.start_odom_y = self.odom_y
        self.start_odom_yaw = self.odom_yaw

        self.target_distance = abs(distance)
        self.max_pulses = max(12, min(140, int(self.target_distance / 0.010) + 25))

        self.active = True
        self.mode = "forward"
        self.phase = "pulse"
        self.phase_start = time.time()
        self.pulse_count = 0

        self.get_logger().info("")
        self.get_logger().info("NEW FORWARD GOAL")
        self.get_logger().info(f"Goal: move straight forward {self.target_distance:.2f} m")
        self.get_logger().info("Uses /odom distance. LiDAR safety enabled.")
        self.get_logger().info("")

        return True

    def start_turn(self, degrees):
        if not self.odom_received:
            self.get_logger().warn("Cannot start turn: no /odom yet")
            return False

        if abs(degrees) < 0.1:
            self.get_logger().warn("Turn angle too small")
            return False

        self.start_odom_x = self.odom_x
        self.start_odom_y = self.odom_y
        self.start_odom_yaw = self.odom_yaw

        self.target_angle = abs(math.radians(degrees))
        self.turn_direction = 1 if degrees > 0 else -1
        self.max_pulses = max(8, min(80, int(abs(degrees) / 1.2) + 8))

        self.active = True
        self.mode = "turn"
        self.phase = "pulse"
        self.phase_start = time.time()
        self.pulse_count = 0

        direction_name = "left" if self.turn_direction > 0 else "right"

        self.get_logger().info("")
        self.get_logger().info("NEW TURN GOAL")
        self.get_logger().info(f"Goal: turn {direction_name} command {abs(degrees):.1f} deg")
        self.get_logger().info("Practical calibration: command 45 deg is about real 90 deg.")
        self.get_logger().info("")

        return True

    def stop_navigation(self):
        self.active = False
        self.mode = "idle"
        self.phase = "idle"

        for _ in range(5):
            self.publish_stop()
            time.sleep(0.03)

        self.get_logger().info("STOPPED")

    def wait_until_idle(self, timeout=30.0):
        start = time.time()
        while rclpy.ok() and self.active:
            if time.time() - start > timeout:
                self.stop_navigation()
                self.get_logger().warn("Timeout while waiting for movement to finish")
                return False
            time.sleep(0.05)
        return True

    def execute_goto(self, target_x, target_y):
        if self.goto_busy:
            self.get_logger().warn("Goto already running")
            return

        if not self.amcl_received:
            self.get_logger().warn("Cannot goto: no /amcl_pose. Set initial pose first.")
            return

        if not self.odom_received:
            self.get_logger().warn("Cannot goto: no /odom")
            return

        if not self.scan_received:
            self.get_logger().warn("Cannot goto: no /scan")
            return

        self.goto_busy = True

        self.get_logger().info("")
        self.get_logger().info("NEW MAP GOTO GOAL")
        self.get_logger().info(f"Target map coordinate: x={target_x:.2f}, y={target_y:.2f}")
        self.get_logger().info("Method: AMCL map pose -> calibrated turn -> odom forward -> repeat")
        self.get_logger().info("")

        try:
            for cycle in range(1, self.goto_max_cycles + 1):
                dx = target_x - self.map_x
                dy = target_y - self.map_y
                distance = math.hypot(dx, dy)

                self.get_logger().info(
                    f"GOTO cycle {cycle}: current=({self.map_x:.2f},{self.map_y:.2f}) "
                    f"target=({target_x:.2f},{target_y:.2f}) distance={distance:.2f} m"
                )

                if distance <= self.goto_tolerance:
                    self.publish_stop()
                    self.get_logger().info(
                        f"Goto reached enough ✅ distance={distance:.2f} m tolerance={self.goto_tolerance:.2f} m"
                    )
                    return

                desired_yaw = math.atan2(dy, dx)
                yaw_error = normalize_angle(desired_yaw - self.map_yaw)
                yaw_error_deg = math.degrees(yaw_error)

                self.get_logger().info(
                    f"Map heading: current_yaw={math.degrees(self.map_yaw):.1f} deg "
                    f"desired={math.degrees(desired_yaw):.1f} deg "
                    f"error={yaw_error_deg:.1f} deg"
                )

                if abs(yaw_error_deg) > 10.0:
                    command_turn = yaw_error_deg * self.turn_command_factor
                    command_turn = max(-55.0, min(55.0, command_turn))

                    self.get_logger().info(
                        f"Turning first: real error={yaw_error_deg:.1f} deg "
                        f"-> command turn={command_turn:.1f} deg"
                    )

                    if not self.start_turn(command_turn):
                        return

                    self.wait_until_idle(timeout=25.0)
                    time.sleep(0.5)

                step_distance = min(distance, self.goto_max_step)

                front = self.sector_min(0, 45)
                if front is not None and front < self.safety_front_stop:
                    self.publish_stop()
                    self.get_logger().warn(
                        f"Goto cancelled: obstacle ahead at {front:.2f} m"
                    )
                    return

                self.get_logger().info(f"Moving forward step: {step_distance:.2f} m")

                if not self.start_forward(step_distance):
                    return

                self.wait_until_idle(timeout=35.0)
                time.sleep(0.7)

            dx = target_x - self.map_x
            dy = target_y - self.map_y
            distance = math.hypot(dx, dy)

            if distance <= self.goto_tolerance:
                self.get_logger().info(
                    f"Goto reached enough ✅ final distance={distance:.2f} m"
                )
            else:
                self.get_logger().warn(
                    f"Goto stopped after max cycles. final distance={distance:.2f} m"
                )

        finally:
            self.goto_busy = False
            self.publish_stop()

    def control_loop(self):
        if not self.active:
            self.publish_stop()
            return

        if not self.odom_received:
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
                    f"Safety stop: obstacle/wall ahead at {front:.2f} m. Movement cancelled."
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
                    f"Stopped: max forward pulses reached. moved={moved:.2f} m target={self.target_distance:.2f} m"
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
    print("AGROBOT NAVIGATION V1")
    print("")
    print("Commands:")
    print("  status")
    print("  pose")
    print("  forward D     -> move forward D meters, example: forward 0.50")
    print("  left DEG      -> calibrated left turn, example: left 45")
    print("  right DEG     -> calibrated right turn, example: right 45")
    print("  goto X Y      -> go to map coordinate, example: goto 0.50 0.20")
    print("  stop")
    print("  q")
    print("")
    print("Manual demo rule: forward 1.00 approx 0.85-1.00 m real, left/right 45 approx real 90 deg.")
    print("")

    while rclpy.ok():
        try:
            raw = input("nav-v1> ").strip()
        except EOFError:
            break

        if not raw:
            continue

        parts = raw.split()
        cmd = parts[0].lower()

        try:
            if cmd == "status":
                print("odom received =", node.odom_received)
                print("amcl received =", node.amcl_received)
                print("scan received =", node.scan_received)
                print("active =", node.active)
                print("mode =", node.mode)
                print("goto busy =", node.goto_busy)

            elif cmd == "pose":
                print(
                    f"odom: x={node.odom_x:.3f}, y={node.odom_y:.3f}, yaw={math.degrees(node.odom_yaw):.1f} deg"
                )
                print(
                    f"map : x={node.map_x:.3f}, y={node.map_y:.3f}, yaw={math.degrees(node.map_yaw):.1f} deg"
                )

            elif cmd == "forward":
                if len(parts) != 2:
                    print("Usage: forward 0.50")
                    continue
                node.start_forward(float(parts[1]))

            elif cmd == "left":
                if len(parts) != 2:
                    print("Usage: left 45")
                    continue
                node.start_turn(abs(float(parts[1])))

            elif cmd == "right":
                if len(parts) != 2:
                    print("Usage: right 45")
                    continue
                node.start_turn(-abs(float(parts[1])))

            elif cmd == "goto":
                if len(parts) != 3:
                    print("Usage: goto 0.50 0.20")
                    continue

                target_x = float(parts[1])
                target_y = float(parts[2])

                t = threading.Thread(
                    target=node.execute_goto,
                    args=(target_x, target_y),
                    daemon=True,
                )
                t.start()

            elif cmd == "stop":
                node.goto_busy = False
                node.stop_navigation()

            elif cmd == "q":
                node.goto_busy = False
                node.stop_navigation()
                rclpy.shutdown()
                break

            else:
                print("Use: status / pose / forward D / left DEG / right DEG / goto X Y / stop / q")

        except ValueError:
            print("Bad number. Examples: forward 0.50 / left 45 / goto 0.50 0.20")


def main(args=None):
    rclpy.init(args=args)
    node = NavigationV1()

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

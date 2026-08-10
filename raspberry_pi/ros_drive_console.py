import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class RosDriveConsole(Node):
    def __init__(self):
        super().__init__("ros_drive_console")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        time.sleep(1.0)

    def send(self, linear, angular, duration):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular

        stop = Twist()

        end = time.time() + duration
        while time.time() < end:
            self.pub.publish(msg)
            time.sleep(0.05)

        for _ in range(5):
            self.pub.publish(stop)
            time.sleep(0.05)

def main():
    rclpy.init()
    node = RosDriveConsole()

    print("")
    print("ROS DRIVE CONSOLE")
    print("Commands:")
    print("  f = forward short")
    print("  b = backward short")
    print("  l = left turn")
    print("  r = right turn")
    print("  s = stop")
    print("  q = quit")
    print("")
    print("Drive slowly for mapping.")
    print("")

    while True:
        cmd = input("ros-drive> ").strip().lower()

        if cmd == "q":
            node.send(0.0, 0.0, 0.1)
            break

        elif cmd == "s":
            node.send(0.0, 0.0, 0.1)
            print("STOP")

        elif cmd == "f":
            node.send(0.10, 0.0, 0.8)
            print("FORWARD")

        elif cmd == "b":
            node.send(-0.10, 0.0, 0.5)
            print("BACKWARD")

        elif cmd == "l":
            node.send(0.0, 0.8, 0.4)
            print("LEFT")

        elif cmd == "r":
            node.send(0.0, -0.8, 0.4)
            print("RIGHT")

        else:
            print("Use only: f b l r s q")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

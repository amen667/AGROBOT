import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class AgrobotStatusPublisher(Node):
    def __init__(self):
        super().__init__('agrobot_status_publisher')

        self.publisher_ = self.create_publisher(
            String,
            'agrobot/status',
            10
        )

        self.timer = self.create_timer(1.0, self.publish_status)
        self.counter = 0

        self.get_logger().info('AGROBOT status publisher started.')

    def publish_status(self):
        msg = String()
        msg.data = f'AGROBOT is online. Message number: {self.counter}'

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

        self.counter += 1


def main(args=None):
    rclpy.init(args=args)

    node = AgrobotStatusPublisher()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
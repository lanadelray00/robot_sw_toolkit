# This node is a test node for '/pick_and_place/start' topic to see if it can be received
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class PickAndPlaceListener(Node):
    def __init__(self):
        super().__init__('pick_and_place_listener')
        self.sub = self.create_subscription(
            Bool,
            '/pick_and_place/start',
            self.callback,
            10
        )
        self.get_logger().info('Waiting for /pick_and_place/start ...')

    def callback(self, msg: Bool):
        self.get_logger().info(
            f'✅ Received /pick_and_place/start = {msg.data}'
        )


def main():
    rclpy.init()
    node = PickAndPlaceListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

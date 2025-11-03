import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

class CoordListener(Node):
    def __init__(self):
        super().__init__('coord_listener')
        self.subscription = self.create_subscription(
            Point,
            '/ip_coords',
            self.listener_callback,
            10)
    
    def listener_callback(self, msg):
        self.get_logger().info(f"📍 좌표 수신: X={msg.x:.2f}, Y={msg.y:.2f}, Z={msg.z:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = CoordListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
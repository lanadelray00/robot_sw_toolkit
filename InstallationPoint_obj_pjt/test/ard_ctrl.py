import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import serial
import time


class DistanceSenderNode(Node):
    def __init__(self):
        super().__init__('arduino_node')
        self.subscription = self.create_subscription(
            Point,
            'ip_coords',  
            self.listener_callback,
            10)

        # 아두이노와의 시리얼 통신 설정 (포트명은 환경에 맞게 수정)
        try:
            self.arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
            time.sleep(2)        
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            exit(1)

    def listener_callback(self, msg):
        z = max(0, int(msg.z))
        if  40 < z <= 50:
            z_str = "45"
        elif 30 < z <= 40:
            z_str = "35"
        elif 20 < z <= 30:
            z_str = "25"
        elif z <= 20:
            z_str = "20"
        else :
            z_str = "off"

        self.arduino.write(f"{z_str}\n".encode())
        self.get_logger().info(f"x: {msg.x:.2f}, y: {msg.y:.2f}, z: {msg.z:.2f}, ard_cmd_z: {z_str}")


def main(args=None):
    rclpy.init(args=args)
    node = DistanceSenderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
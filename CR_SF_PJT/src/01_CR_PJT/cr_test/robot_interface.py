import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand


class RobotInterface(Node):
    def __init__(self):
        super().__init__('gripper_interface')

        # === Action Client 연결 ===
        self.client = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd')
        while not self.client.wait_for_server(timeout_sec=2.0):
            self.get_logger().info('⏳ Waiting for /gripper_controller/gripper_cmd action server...')

        self.get_logger().info("✅ Connected to GripperActionController")

    def send_gripper_command(self, position: float, effort: float = 0.0):
        """그리퍼 열기/닫기 명령 전송"""
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position  # 열림/닫힘 각도 (m 단위: finger distance)
        goal_msg.command.max_effort = effort  # 필요한 경우 토크 제한

        self.get_logger().info(f"🚀 Sending gripper command (pos={position})...")
        send_future = self.client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().warn("⚠️ Gripper goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.get_logger().info(f"✅ Gripper moved (reached={result.reached_goal})")
        return True

    def open_gripper(self):
        self.send_gripper_command(0.019)  # 열린 거리 (조정 가능)
        self.get_logger().info("✅ Gripper opened")

    def close_gripper(self):
        self.send_gripper_command(-0.01)   # 닫힌 위치
        self.get_logger().info("✅ Gripper closed")

    # === 단독 실행 테스트용 ===
    def test_sequence(self):
        # self.open_gripper()
        self.close_gripper()


def main(args=None):
    rclpy.init(args=args)
    node = RobotInterface()
    node.test_sequence()  # 단독 테스트 실행
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
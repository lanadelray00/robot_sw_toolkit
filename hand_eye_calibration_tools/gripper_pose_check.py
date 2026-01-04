# EE position, orientation checker
# execute equipment first

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.msg import RobotState

class FKClient(Node):
    def __init__(self):
        super().__init__('fk_client')
        self.fk_client = self.create_client(GetPositionFK, '/compute_fk')
        self.subscription = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        
        # 서비스가 준비될 때까지 대기
        while not self.fk_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /compute_fk service...')

    def joint_callback(self, msg):
        # FK 서비스 요청 생성
        request = GetPositionFK.Request()
        request.header.frame_id = 'world'  # 기준 좌표계
        request.fk_link_names = ['end_effector_link']    # FK를 계산할 링크 이름 (EE link 이름 확인 필요!)
        robot_state = RobotState()
        robot_state.joint_state = msg
        request.robot_state = robot_state

        # 비동기 서비스 호출
        future = self.fk_client.call_async(request)
        future.add_done_callback(self.fk_response_callback)

    def fk_response_callback(self, future):
        try:
            response = future.result()
            if len(response.pose_stamped) > 0:
                pose = response.pose_stamped[0].pose

                x, y, z = pose.position.x, pose.position.y, pose.position.z

                qx, qy, qz, qw = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
                self.get_logger().info(
                    f"🦾 EE Pose → x={x:.3f}, y={y:.3f}, z={z:.3f}, qx={qx:.3f}, qy={qy:.3f}, qz={qz:.3f}, qw={qw:.3f}"
                    )
            else:
                self.get_logger().warn("❌ FK failed or returned empty pose")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = FKClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

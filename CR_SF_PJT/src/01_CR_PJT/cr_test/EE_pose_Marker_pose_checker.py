# EE position, orientation

import cv2
import cv2.aruco as aruco
import numpy as np
import os
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.msg import RobotState
from transformations import euler_from_quaternion


class FKClient(Node):
    def __init__(self):
        super().__init__('fk_client')
        self.fk_client = self.create_client(GetPositionFK, '/compute_fk')
        self.subscription = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)

        # ✅ EE pose 저장용 변수 (초기값)
        self.current_position = None
        self.current_orientation = None
        
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
                quat = [qx, qy, qz, qw]
                roll, pitch, yaw = euler_from_quaternion(quat)

                # ✅ 여기서 실시간으로 변수 업데이트
                self.current_position = [x, y, z]
                self.current_orientation = [roll, pitch, yaw]

                print(f"EE position → x: {x*100:.3f}, y: {y*100:.3f}, z: {z*100:.3f}")
                # print(f"EE orientation (quaternion) → x: {qx:.3f}, y: {qy:.3f}, z: {qz:.3f}, w: {qw:.3f}")
                print(f"RPY → roll: {roll:.3f}, pitch: {pitch:.3f}, yaw: {yaw:.3f}")

            else:
                print("No FK result returned.")
        except Exception as e:
            print(f"FK call failed: {e}")


class ArucoDetector:
    def __init__(self, fk_node, camera_index=0, marker_length=0.08, calib_path=None):
        # 카메라 초기화
        self.cap = cv2.VideoCapture(camera_index)
        cv2.setLogLevel(0)

        self.fk_node = fk_node  # ✅ FKClient 인스턴스 저장

        # ArUco 설정
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters()

        # 카메라 보정 데이터 불러오기
        if calib_path is None:
            raise ValueError("⚠️ Calibration file path (calib_path) must be provided.")
        data = np.load(calib_path)
        self.camera_matrix = data['mtx']
        self.dist_coeffs = data['dist']

        # 마커 길이
        self.marker_length = marker_length

    def detect_markers(self, frame):
        """입력 프레임에서 ArUco 마커 탐지 및 pose 계산"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        detections = []

        if ids is not None:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, self.marker_length, self.camera_matrix, self.dist_coeffs
            )
            for i in range(len(ids)):
                detection = {
                    "id": int(ids[i][0]),
                    "rvec": rvecs[i][0],
                    "tvec": tvecs[i][0],
                    "corners": corners[i][0]
                }
                detections.append(detection)
        return detections

    def draw_markers(self, frame, detections):
        """탐지된 마커와 축을 영상에 표시"""
        for det in detections:
            aruco.drawDetectedMarkers(frame, [det["corners"].reshape(1, -1, 2)], np.array([[det["id"]]]))
            cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs,
                              det["rvec"], det["tvec"], self.marker_length)
            cX, cY = int(det["corners"][0][0]), int(det["corners"][0][1])
            id, m_x, m_y, m_z = det['id'], det['tvec'][0], det['tvec'][1], det['tvec'][2]
            cv2.putText(frame, f"ID:{id} M X={m_x*100:.3f} Y={m_y*100:.3f} Z={m_z*100:.3f}",
                        (cX, cY - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            # EE pose 텍스트 (두 번째 줄)
            if self.fk_node.current_position is not None:
                x, y, z = self.fk_node.current_position
                roll, pitch, yaw = self.fk_node.current_orientation
                cv2.putText(frame,
                    f"EE x={x*100:.3f} y={y*100:.3f} z={z*100:.3f}",
                    (cX, cY + 10),  # 👈 y좌표를 +로 해서 아래로 이동
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2
                )
        return frame

    def run(self):
        """메인 루프"""
        save_dir = "/home/choigh/practice_ws/CR_SF_PJT/src/01_CR_PJT/cr_test/EE_pose_Marker_pose_checker_screenshots"
        os.makedirs(save_dir, exist_ok=True)

        count = 1  # 저장 파일 번호
        while True:
            ret, frame = self.cap.read()
            if not ret:
                break

            detections = self.detect_markers(frame)
            if detections:
                for det in detections:
                    print(f"ID {det['id']} | X={det['tvec'][0]:.3f}  Y={det['tvec'][1]:.3f}  Z={det['tvec'][2]:.3f}")
                frame = self.draw_markers(frame, detections)
            cv2.imshow("Aruco Detection", frame)
            key = cv2.waitKey(1) & 0xFF

            # ✅ 스페이스바 → 현재 화면 저장
            if key == 32:  # Space
                filename = os.path.join(save_dir, f"EE_Marker_{count:03d}.png")
                cv2.imwrite(filename, frame)
                print(f"📸 Saved snapshot → {filename}")
                count += 1

            if key == 27:  # ESC
                break

        self.cap.release()
        cv2.destroyAllWindows()



# 실행
rclpy.init()
fk_node = FKClient()

ros_thread = threading.Thread(target=rclpy.spin, args=(fk_node,), daemon=True)
ros_thread.start()

detector = ArucoDetector(
        fk_node=fk_node,
        camera_index=2,
        marker_length=0.08,
        calib_path='/home/choigh/practice_ws/Test_Tools_code/calib_data.npz'
    )
detector.run()

print("🔻 Shutting down...")
fk_node.destroy_node()
rclpy.shutdown()
ros_thread.join(timeout=1.0)
print("✅ Clean exit.")
# gripper, Hand-eye Calibration rvecs, tvecs param checker SW
# 1. gripper_pose coordinate from shooting position에서의 
# 2. target coordinate from camera 

import cv2
import cv2.aruco as aruco
import numpy as np
import os
import threading
import csv

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
        self.current_joint_state = None
        
        # 서비스가 준비될 때까지 대기b
        while not self.fk_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /compute_fk service...')

    def joint_callback(self, msg):
        # FK 서비스 요청 생성
        self.current_joint_state = msg
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

                # print(f"EE position → x: {x*100:.3f}, y: {y*100:.3f}, z: {z*100:.3f}")
                # print(f"EE orientation (quaternion) → x: {qx:.3f}, y: {qy:.3f}, z: {qz:.3f}, w: {qw:.3f}")
                # print(f"RPY → roll: {roll:.3f}, pitch: {pitch:.3f}, yaw: {yaw:.3f}")

            else:
                print("No FK result returned.")
        except Exception as e:
            print(f"FK call failed: {e}")


class ArucoDetector:
    def __init__(self, fk_node, csv_path, camera_index=0, marker_length=0.08, calib_path=None):
        # 카메라 초기화
        self.cap = cv2.VideoCapture(camera_index)
        self.csv_path = csv_path
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
        BASE_DIR = os.path.dirname(os.path.abspath(__file__))
        save_dir = os.path.join(BASE_DIR, 'gripper_target_pose_checker_screenshots')   # 원하는 폴더명
        os.makedirs(save_dir, exist_ok=True)

        count = 1  # 저장 파일 번호
        while True:
            ret, frame = self.cap.read()
            if not ret:
                break

            detections = self.detect_markers(frame)
            if detections:
                # for det in detections:
                #     print(f"ID {det['id']} | X={det['tvec'][0]:.3f}  Y={det['tvec'][1]:.3f}  Z={det['tvec'][2]:.3f}")
                frame = self.draw_markers(frame, detections)
            cv2.imshow("Aruco Detection", frame)
            key = cv2.waitKey(1) & 0xFF

            # ✅ 스페이스바 → 현재 화면 저장
            if key == 32:  # Space
                if self.fk_node.current_position is None or self.fk_node.current_joint_state is None:
                    print("❌ EE pose or joint state not ready, skip saving")
                    continue
                print("========== [SPACE CAPTURE] ==========")

                # EE pose 출력
                if self.fk_node.current_position is not None:
                    x, y, z = self.fk_node.current_position
                    roll, pitch, yaw = self.fk_node.current_orientation

                    print(f"EE pose x={x*100:.4f}, y={y*100:.4f}, z={z*100:.4f}, roll={roll:.4f}, pitch={pitch:.4f}, yaw={yaw:.4f}")
                else:
                    print("[EE pose] ❌ Not available")

                # joint 출력
                if self.fk_node.current_joint_state is not None:
                    js = self.fk_node.current_joint_state
                    joint_map = dict(zip(js.name, js.position))
                    ordered_joints = ["joint1", "joint2", "joint3", "joint4"]
                    pairs = [
                        f"{j}={joint_map[j]:.5f} rad"
                        for j in ordered_joints
                        if j in joint_map
                    ]
                    print(", ".join(pairs))
                else:
                    print("[Joint angles] ❌ Not available")

                # CSV 저장
                with open(self.csv_path, "a", newline="") as f:
                    writer = csv.writer(f)
                    writer.writerow([
                        x * 100, y * 100, z * 100,
                        roll, pitch, yaw,
                        joint_map.get("joint1"),
                        joint_map.get("joint2"),
                        joint_map.get("joint3"),
                        joint_map.get("joint4"),
                    ])

                # 3️⃣ 스크린샷 저장
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
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
csv_path = os.path.join(BASE_DIR, "ee_joint_log.csv")
# 헤더가 없으면 생성
if not os.path.exists(csv_path):
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "x_cm", "y_cm", "z_cm",
            "roll_rad", "pitch_rad", "yaw_rad",
            "joint1_rad", "joint2_rad", "joint3_rad", "joint4_rad"
        ])
detector = ArucoDetector(
        fk_node=fk_node,
        csv_path=csv_path,
        camera_index=2,
        marker_length=0.08,
        calib_path = os.path.join(BASE_DIR, 'calib_data_logitech_c270.npz')

    )
detector.run()

print("🔻 Shutting down...")
fk_node.destroy_node()
rclpy.shutdown()
ros_thread.join(timeout=1.0)
print("✅ Clean exit.")
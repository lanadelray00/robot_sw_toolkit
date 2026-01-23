# ArUco Marker 좌표 추정

import cv2
import numpy as np
import cv2.aruco as aruco
from collections import deque
import os

# cv2.setLogLevel(0) 
cap = cv2.VideoCapture('/dev/video2')

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()
BASE_DIR = os.path.dirname(os.path.abspath(__file__))

calib_path = os.path.join(BASE_DIR, 'calib_data_logitech_c270.npz')
calib_data = np.load(calib_path)

camera_matrix = calib_data['mtx']
dist_coeffs = calib_data['dist']
marker_length = 0.08

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    if ids is not None:
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)

        print(ids, rvecs)

        for i in range(len(ids)):
            aruco.drawDetectedMarkers(frame, corners, ids)
            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], marker_length)

            # 정보 출력
            # print(f"ID {ids[i][0]} | X={tvecs[i][0][0]:.3f}  Y={tvecs[i][0][1]:.3f}  Z={tvecs[i][0][2]:.3f}")
            # print(f"ID {ids[i][0]}", f"rvec {rvecs[i][0][0]}")
            

            # 화면 표시용 텍스트
            cX, cY = int(corners[i][0][0][0]), int(corners[i][0][0][1])
            cv2.putText(frame, f"ID:{ids[i][0]} Z={tvecs[i][0][2]:.2f}m", (cX, cY - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    cv2.imshow("Aruco Detection", frame)
    if cv2.waitKey(1) & 0xFF == 27:  # ESC to exit
        break

cap.release()
cv2.destroyAllWindows()


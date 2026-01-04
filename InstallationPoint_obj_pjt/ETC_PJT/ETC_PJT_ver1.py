
# image_coord 에서 coords yield 반환
# yolo_thread 에서 coords 받아서 queue로 put
# coordinate_refinery 에서 put 값 get으로 받아서 데이터 정제 후 
# 정제된 x,y,z 값 yield 반환
# 정제된 coords get 받아서 아두이노 서보모터 제어하여 object detection 실시

# ver.1_25.06.16
# 단순 Object Tracking, YOLO로 탐지한 Object의 BBox의 좌표를 이용해서 
# 해당 BOX 중심의 x,y 좌표의 -2 ~ 2 범위 내에 카메라 중심이 위치하도록 제어


# import
import serial
import math
import cv2
import numpy as np
from ultralytics import YOLO
import pandas as pd
import threading, queue
import time


q_coordinate_refinery = queue.Queue()
q_output = queue.Queue() 

# YOLO I.P Object detecting def

# ✅ 모델 로딩
model = YOLO('/home/choigh/WS/IP_OB/Installation_point/yolo_data/runs/detect/train3/weights/best.pt')

# Arduino
ser = serial.Serial('/dev/arduinoUno', 9600, timeout=1)  # 포트 번호는 환경에 맞게 조정
time.sleep(2)  # 아두이노 초기화 대기


# YOLO I.P Object detection
def image_coord():
        # 카메라 파라미터 (미리 측정 또는 캘리브레이션)
    fx, fy = 724.79, 720.74  # 초점 거리
    cx, cy = 322.13, 235.08  # 중심 좌표 (frame size 640x480 기준)

    # 실제 객체 크기 (미리 측정한 값, 단위: m)
    real_width = {'I_P': 0.06}   # 6cm

    # 원하는 label 매핑
    label_map = {'I_P': 'I_P'}

    # 거리 보정 함수
    def corrected_distance(bbox_w):
        bbox_w_list = [234, 152, 125, 99, 91, 80, 59, 61]
        true_distance_list = [20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0]
        coeffs = np.polyfit(bbox_w_list, true_distance_list, deg=2)
        return np.polyval(coeffs, bbox_w)

    cap = cv2.VideoCapture('/dev/usb_cam')

    if not cap.isOpened():
        print("웹캠 열기 실패")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            print("프레임 수신 실패")
            break

        # red dot
        height, width = frame.shape[:2]
        red_cx, red_cy = int(width / 2), int(height / 2)
        cv2.circle(frame, (red_cx, red_cy), radius=5, color=(0, 0, 255), thickness=-1)

        results = model(frame)[0]
        coords = []

        for box in results.boxes:
            cls_id = int(box.cls[0])
            label = model.names[cls_id]
            if label in real_width:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                bbox_cx = (x1 + x2) / 2
                bbox_cy = (y1 + y2) / 2
                bbox_w = x2 - x1

                Z = corrected_distance(bbox_w)
                X = (bbox_cx - cx) * Z / fx
                Y = -(bbox_cy - cy) * Z / fy

                coords.append({"label": label, "x": X, "y": Y, "z": Z})  # 이 프레임의 탐지 결과 추가
                name = label_map[label]

                cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)
                cv2.putText(frame, f"{name} ({X:.2f},{Y:.2f},{Z:.2f})m", (x1, y1-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
        
        cv2.imshow("YOLOv8 + 3D", frame)
        if cv2.waitKey(1) == ord('q'):
            break

        yield coords # ✅ 이 프레임의 모든 탐지된 좌표 리스트 반환

    cap.release()
    cv2.destroyAllWindows()


# coord refining
def coordinate_refinery():
    
    frame_count = 0
    FRAME_RESET_INTERVAL = 60  # Setting FPS 마다 초기화
    data = pd.DataFrame({
    "frame": pd.Series(dtype='int'),
    "label": pd.Series(dtype='str'),
    "x": pd.Series(dtype='float'),
    "y": pd.Series(dtype='float'),
    "z": pd.Series(dtype='float')
    })
    while True:
        coords = q_coordinate_refinery.get()
        frame_count += 1

        # 새 프레임 좌표 추가
        new_df = pd.DataFrame(coords)
        new_df["frame"] = frame_count
               
        if not new_df.empty and not new_df.isna().all(axis=None):
            data = pd.concat([data, new_df], ignore_index=True)
            
        # 슬라이딩 윈도우 유지
        if frame_count > FRAME_RESET_INTERVAL:
            data = data[data["frame"] >= frame_count - FRAME_RESET_INTERVAL + 1]

        # label별 z 평균
        counts = data["label"].value_counts() # 다른 label이 나올 경우 대비
        valid_labels = counts[counts >= 10].index # 현재 처음 실행 후 counts >= 5 도달 전에는 좌표 반환X
        label_z_avg = data[data["label"].isin(valid_labels)].groupby("label")["z"].mean()
                

        if label_z_avg.empty:
            continue

        target_label = label_z_avg.idxmin()
        target_data = data[data["label"] == target_label]
        
        
        x_mean = target_data["x"].mean()
        y_mean = target_data["y"].mean()
        z_mean = target_data["z"].mean()
        
        q_output.put((x_mean, y_mean, z_mean))


# arduino servo motor control
def send_thread():
    last_sent_time = 0
    Min_interval = 0.03 # "n" ms 마다 
    f_angle = True # first angle setting
    ticks = 1
    gap = 2

    while True:
        x, y, z = q_output.get()
        if f_angle:
            t1, t2, t3 = 90, 90, 90
            f_angle = False

        now = time.time()
        if now - last_sent_time > Min_interval:
            if x <= -gap:
                t1 += ticks
            elif x >= gap:
                t1 -= ticks

            if y <= -gap:
                t3 += ticks
            elif y >= gap:
                t3 -= ticks

            # 각도제한
            t1 = max(0, min(180, t1))
            t2 = max(0, min(180, t2))
            t3 = max(0, min(180, t3))
            print(t1, t2, t3)

            # 전송 및 출력
            ard_data = f"{t1},{t2},{t3}\n"
            if ser.is_open:
                ser.write(ard_data.encode())
            else:
                print("⚠️ 시리얼 포트가 닫혀 있음. 전송 실패")

            print(f"[전송됨] t1={t1, t2, t3}")
            last_sent_time = now

        time.sleep(0.01)  # CPU 과점유 방지용


# yolo object detecting 포함된 함수에서 queue put 못하니 밖에서 coords data put하는 기능
def yolo_thread():
    for coords in image_coord():
        q_coordinate_refinery.put(coords)

def result_viewer():
    while True:
        x, y, z = q_output.get()
        print(f"📍 정제된 평균 좌표: X={x:.2f}, Y={y:.2f}, Z={z:.2f}")


# 3. 실행부
if __name__ == "__main__":
    threading.Thread(target=yolo_thread).start()
    threading.Thread(target=coordinate_refinery).start()
    threading.Thread(target=result_viewer).start()
    threading.Thread(target=send_thread).start()
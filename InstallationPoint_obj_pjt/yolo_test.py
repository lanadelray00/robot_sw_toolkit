import cv2
from ultralytics import YOLO

# 1. 모델 로드
# model = YOLO('/home/cgh/workspace/project1/Installation_point/yolo_data/runs/detect/train3/weights/best.pt')
model = YOLO('yolov8n.pt')   # Nano (기본)
# model = YOLO('yolov8s.pt')   # Small
# model = YOLO('yolov8m.pt')   # Medium
# model = YOLO('yolov8l.pt')   # Large
# model = YOLO('yolov8x.pt')   # XLarge

# 2. 웹캠 열기
cap = cv2.VideoCapture(0)  # 0번 카메라

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # 3. YOLOv8으로 프레임 탐지
    results = model.predict(source=frame, save=False, verbose=False, show=False)

    # 4. 탐지 결과 시각화
    annotated_frame = results[0].plot()

    # 5. 화면에 출력
    cv2.imshow('YOLOv8 Webcam Detection', annotated_frame)

    # 6. 'q' 누르면 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 7. 정리
cap.release()
cv2.destroyAllWindows()

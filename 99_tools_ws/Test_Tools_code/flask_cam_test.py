# Flask 띄우기
import cv2

cap = cv2.VideoCapture("http://192.168.0.26:5000/video_feed", cv2.CAP_FFMPEG)

if not cap.isOpened():
    print("❌ 카메라를 열 수 없습니다.")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("❌ 프레임을 읽을 수 없습니다.")
        break

    # 영상 출력
    cv2.imshow("Camera", frame)

    # 'q' 키 누르면 종료
    if cv2.waitKey(1) & 0xFF == 27:  # ESC to exit
        break

# 자원 해제
cap.release()
cv2.destroyAllWindows()

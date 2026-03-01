# Flask 띄우기
import cv2
import socket

# ======================================================
# ip finder
# ======================================================
def get_local_ip(self):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))  # 실제로 연결되지는 않음
        self.ip = s.getsockname()[0]
    finally:
        s.close()
    return self.ip

ip = get_local_ip()

url = f"http://{ip}:5000/video_feed" # 현재 pc에서 실행 시
# url = "http://192.168.0.105:5000/video_feed" # ip 192.168.0.105
cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)

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



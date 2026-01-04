import cv2

cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("웹캠 열기 실패")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("프레임 수신 실패")
        break

    cv2.imshow('Webcam', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
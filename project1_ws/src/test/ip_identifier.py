import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO


class YoloInferNode(Node):
    def __init__(self):
        super().__init__('yolo_infer_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10)
        self.pub_coords = self.create_publisher(Point, '/ip_coords', 10)
        self.bridge = CvBridge()
        self.model = YOLO('/home/cgh/workspace/project1/Installation_point/' \
        'yolo_data/runs/detect/train3/weights/best.pt')  # YOLO 커스텀 모델
                
        self.fx, self.fy = 724.79, 720.74  # 초점 거리
        self.cx, self.cy = 322.13, 235.08  # 중심 좌표 (frame size 640x480 기준)

        # 실제 객체 크기 (미리 측정한 값, 단위: m)
        self.real_width = {'I_P': 0.06, }

        # 원하는 label 매핑
        self.label_map = {'I_P': 'I_P', }

        # 2차 다항 회귀 함수를 이용한 거리 보정
        self.bbox_w_list = [234, 152, 125, 99, 91, 80, 59, 61]
        self.true_distance_list = [20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0]
        self.coeffs = np.polyfit(self.bbox_w_list, self.true_distance_list, deg=2)
    
    def corrected_distance(self, bbox_w):
        return np.polyval(self.coeffs, bbox_w)

    def listener_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        frame = cv2.resize(frame, (640, 480))
        self.get_logger().info(f"Frame size: {frame.shape}")
        results = self.model(frame)[0]
                
        for box in results.boxes:
            cls_id = int(box.cls[0])
            label = self.model.names[cls_id]
            conf = float(box.conf[0])

            # 객체가 우리가 추적하고 싶은 것인지 확인
            if label in self.real_width:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                bbox_cx = (x1 + x2) / 2
                bbox_cy = (y1 + y2) / 2
                bbox_w = x2 - x1

                # 거리(Z) 추정
                Z = self.corrected_distance(bbox_w)
                X = (bbox_cx - self.cx) * Z / self.fx
                Y = (bbox_cy - self.cy) * Z / self.fy

                name = self.label_map[label]
                print(f"[{name}] → X: {X:.2f}m, Y: {Y:.2f}m, Z: {Z:.2f}m")
                
                pt = Point()
                pt.x = X
                pt.y = Y
                pt.z = Z
                self.pub_coords.publish(pt)

                # 시각화
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)
                cv2.putText(frame, f"{name} ({X:.2f},{Y:.2f},{Z:.2f})m", (x1, y1-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

        cv2.imshow("YOLOv8 + 3D", frame)
        cv2.waitKey(1)
        

def main(args=None):
    rclpy.init(args=args)
    node = YoloInferNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

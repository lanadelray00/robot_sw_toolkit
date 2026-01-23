# Useful Robot SW Developement Tools Repository

## hand_eye_calibration_tools - File Descriptions
### calib_data_logitech_c270.npz
Camera calibration result for Logitech C270 (RMS reprojection error ≈ 0.3).

### gripper_pose_check.py
MoveIt-based end-effector position and orientation checker (execute equipment first).

### gripper_target_pose_checker.py
Tool for simultaneously checking gripper (EE) pose and camera-based target pose for hand-eye calibration (rvecs, tvecs).

### handeyecalibration_param_generator.py
Core tool that records target poses in the camera frame and corresponding EE poses for each robot configuration to generate hand-eye calibration parameters.

### handeyecalibration_param_refineray.ipynb
Core notebook for validating, filtering, and refining hand-eye calibration datasets generated from the parameter generator.

### Target_pose_check.py
Simple tool for estimating target pose in the camera coordinate frame.



## Test_Tools_code - File Descriptions
### calibration.ipynb
Camera calibration notebook using 34 checkerboard images to compute camera matrix, distortion coefficients, and RMS reprojection error (~0.316).

### gpu_test.ipynb
GPU environment verification notebook (PyTorch, TensorFlow, YOLO checks, GPU device listing, simple GPU computation, camera connection and FPS test).

### tool_1_camera_test.ipynb
Camera validation tool for USB camera streaming, OpenCV frame display, Flask-based video streaming check, and resolution verification.

### tool_2_capture_img.ipynb
Image capture tool for acquiring calibration images (checkerboard) from the camera.

### tool_3_ArUco_Marker_gen.py
Script for generating ArUco markers for vision-based pose estimation.

### calib_data.npz
Stored camera calibration parameters generated from checkerboard-based calibration.

### yolov8n.pt
Pretrained YOLOv8 nano model file for object detection experiments.

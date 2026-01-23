# Useful Robot SW Developement Tools Repository

# File Descriptions

## hand_eye_calibration_tools 
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

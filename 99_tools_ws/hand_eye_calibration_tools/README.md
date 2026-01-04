# Hand–Eye Calibration Tools

This folder contains tools and scripts for performing hand–eye calibration
between a robot gripper (end-effector) and a camera.

---

## Hardware Dependency Notes

- **Manipulator** (currently OpenManipulator)
  - The manipulator can be replaced as long as it provides:
    - `sensor_msgs/JointState`
    - `moveit_msgs/GetPositionFK`
    - `moveit_msgs/RobotState`
  - Any manipulator compatible with ROS 2 and MoveIt FK services can be used
    without modifying the core logic.

- **Camera** (currently Logitech C270)
  - If the camera is changed, **camera calibration must be performed again**.
  - Update the calibration file (`.npz`) with the new camera intrinsic
    parameters before running the calibration tools.

---

## 📁 Files Description

- **`calib_data_logitech_c270.npz`**  
  Camera calibration data for the Logitech C270 camera  
  (camera matrix and distortion coefficients).

- **`gripper_pose_check.py`**  
  Script for checking and monitoring the robot gripper (end-effector) pose only,
  using forward kinematics.

- **`Target_pose_check.py`**  
  Script for checking and monitoring the target pose only,
  obtained from ArUco marker detection (rvecs, tvecs).

- **`gripper_target_pose_checker.py`**  
  Script for simultaneously checking both:
  - Gripper pose (FK-based)
  - Target pose (ArUco rvecs, tvecs)

- **`handeyecalibration_param_generator.py`**  
  Data collection tool for hand–eye calibration.  
  It records synchronized gripper poses and target poses
  (rvecs, tvecs) at multiple robot configurations.

- **`handeyecalibration_param_refineray.ipynb`**  
  Jupyter Notebook for inspecting, validating, and refining
  the collected hand–eye calibration parameters, including
  outlier and missing data handling.

---

## 🎯 Purpose

The scripts in this folder are used to:
- Collect pose pairs for hand–eye calibration (AX = XB)
- Validate gripper and target pose measurements
- Refine and prepare calibration data before final computation


## Workflow

1. **Data Collection**  
   `handeyecalibration_param_generator.py`  
   → Collects synchronized gripper poses and target poses (rvecs, tvecs).

2. **Data Refinement**  
   `handeyecalibration_param_refineray.ipynb`  
   → Inspects and cleans the collected data (outliers / missing values).

3. **Calibration**  
   → Uses the refined pose pairs to compute the hand–eye calibration (AX = XB).
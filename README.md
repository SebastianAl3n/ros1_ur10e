# Vision based Pick-and-Place Simulation

![Vision Based Pick and Place](img/simulation.png)

## Overview
Prototype ROS/Gazebo simulation using HSV colour space.

## Features
- Vision-based object detection using HSV color space
- 2D to 3D pose estimation using RGB-D data
- Coordinate transformation from camera frame to world frame
- Basic motion planning (joint + Cartesian)
- Grasping via Gazebo link attacher
- Simplified industrial pick-and-place workflow

## Limitations / Next Steps
- Object detection relies on HSV color thresholding, which is sensitive to lighting changes and requires manual tuning.
- The system assumes a controlled environment with clearly distinguishable object colors.
- Depth-based Z estimation is affected by sensor noise and edge artifacts.
- No semantic understanding of objects (only color-based detection is used).
- Replace HSV-based segmentation with learning-based object detection (e.g., YOLO) for improved robustness.
# ROS1_UR10e Workspace
A ROS1 workspace for controlling the ur10e cobot with paired together with a RGBD sensor.
Ensure the UR description package is installed and sourced
```bash
sudo apt install ros-noetic-ur-description
```

# Starting the simulation
# source the workspace in each terminal
```bash

cd ros1_ur10e/
source devel/setup.bash
roslaunch ur10e_sim full_robot.launch
roslaunch ur10e_sim moveit.launch
LIBGL_ALWAYS_SOFTWARE=1 roslaunch moveit_config moveit_rviz.launch
rosrun ur10e_sim pick_and_place_node.py
```
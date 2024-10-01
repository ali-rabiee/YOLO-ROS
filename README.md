# YOLO-ROS: Object Detection and Gripper Control Integration

This ROS package integrates YOLO object detection with a robotic arm control system, utilizing Kinova gripper finger positions and head position feedback for automating grasp and release actions. The detected objects are processed and published along with segmentation masks and image annotations.

## Table of Contents
- [Overview](#overview)
- [Requirements](#requirements)
- [Installation](#installation)
- [Node Structure](#node-structure)
  - [Subscribed Topics](#subscribed-topics)
  - [Published Topics](#published-topics)
- [Usage](#usage)
- [Parameters](#parameters)
- [Customization](#customization)
- [Contributing](#contributing)
- [License](#license)

## Overview

This ROS package subscribes to the camera feed and finger position of the Kinova robotic arm and detects objects in real-time using a pre-trained YOLO model (`best.pt`). Based on object positions and head orientation feedback, the package can determine whether the gripper should automatically execute grasp or release actions. The package also publishes object detection results, bounding boxes, segmentation masks, and the positions of objects in the scene.

### Key Features
- Real-time object detection using YOLOv8.
- Automatic execution of grasp and release based on the proximity of the gripper to detected objects.
- Visualization of object bounding boxes, segmentation masks, and center dots of objects.
- Integration with the Kinova arm's finger positions and head position input.

## Requirements

The following dependencies are required to run this package:

- **ROS (Robot Operating System)**: Tested with ROS Noetic.
- **Python 3.x**
- **YOLOv8**: You can install YOLO from the [Ultralytics repository](https://github.com/ultralytics/ultralytics).
- **cv_bridge**: ROS package for OpenCV.
- **OpenCV**: Image processing library.
- **Kinova ROS Package**: For controlling and receiving data from the Kinova arm.

### Python Dependencies:
```bash
pip install opencv-python ultralytics numpy

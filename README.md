# Object Tracking with DepthAI and Arduino

This project integrates advanced object tracking capabilities using a DepthAI camera and an Arduino Uno. Utilizing ROS Noetic and Ultralytics YOLOv8, this system achieves real-time object detection and tracking. The object tracking system communicates with the Arduino Uno to control the movement of a mobile platform based on detected objects.

## Table of Contents

- Overview
- Features
- System Architecture
- Installation
- Usage
- Code Structure
- Dependencies
- License

## Overview

The project leverages a DepthAI camera for capturing real-time video streams and detecting objects using YOLOv8. The object detection results are processed to control the motion of a vehicle through commands sent to an Arduino Uno via ROS serial communication.

## Features

Real-time object detection and tracking using YOLOv8. Integration with DepthAI for high-performance video processing. Command-based vehicle control through Arduino Uno. ROS Noetic-based modular and scalable design. Optional GPU acceleration with NVIDIA CUDA.

## System Architecture

**Components**: DepthAI Camera for capturing video streams and running object detection, YOLOv8 as the real-time object detection model, Arduino Uno to control the vehicle's motion based on received commands, and ROS Noetic to provide the messaging framework and hardware abstraction.

**Data Flow**: The DepthAI camera captures video and runs the YOLOv8 model. Detected objects are processed to compute movement commands. Commands are sent to the Arduino Uno via ROS serial communication. The Arduino controls the vehicle’s motors to follow the detected objects.

## Installation

### Prerequisites

Ubuntu 20.04 and ROS Noetic.

### Steps

Clone the repository and navigate to the project directory. Run the installation script using `./install.sh`. This script installs all necessary dependencies, including ROS Noetic, DepthAI, YOLOv8, and CUDA (optional).

## Usage

### Running the Project

Source the ROS setup using `source ~/catkin_ws/devel/setup.bash`. Launch the main script with `rosrun object_tracking main.py`.

### Controlling the Vehicle

The system automatically detects objects and sends appropriate control commands to the Arduino for vehicle movement.

## Code Structure

Key files include `main.py` for object detection and command publishing, `install.sh` for setting up the environment, `CMakeLists.txt` and `package.xml` for ROS package configuration, and `run_project.py` & `uvc_rgb.py` as supporting scripts.

## Dependencies

The project relies on the following key dependencies: ROS Noetic (`ros-noetic-desktop-full`), DepthAI (`depthai`), Ultralytics YOLO (`ultralytics`), Additional Python Libraries (`torch`, `torchvision`, `opencv-python`, `numpy`), and ROS Packages (`rosserial`, `rosserial-arduino`, `rosserial-server`, `rosserial-python`).

## License

This project is licensed under the BSD License. See the `LICENSE` file for details.

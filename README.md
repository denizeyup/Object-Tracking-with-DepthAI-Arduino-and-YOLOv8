
# Object Tracking Project

## Overview

This project is designed to perform object tracking using ROS Noetic and YOLOv8. The core components include a ROS package named `object_tracking`, which communicates via standard ROS messages (`std_msgs`, `geometry_msgs`, `sensor_msgs`) to process and track objects detected by a camera. YOLOv8 is used for object detection, while ROS handles communication between different modules and manages data flow in the robotic environment.

## Table of Contents

1. [Requirements](#requirements)
2. [Installation](#installation)
3. [Project Structure](#project-structure)
4. [Usage](#usage)
5. [Files and Scripts](#files-and-scripts)
6. [ROS Package Details](#ros-package-details)
7. [Known Issues](#known-issues)
8. [Contribution Guidelines](#contribution-guidelines)
9. [License](#license)

## Requirements

- **Operating System**: Ubuntu 20.04 (or compatible ROS Noetic environment)
- **ROS Noetic**: Robot Operating System required for inter-process communication.
- **Python**: Python 3 with `pip` package manager.
- **Hardware (Optional)**: NVIDIA GPU for CUDA acceleration (optional but recommended for YOLOv8 performance).
- **Dependencies**: Required ROS packages and Python libraries as specified below.

## Installation

1. **Clone the Repository**:
   ```bash
   git clone https://github.com/denizeyup/Object-Tracking-with-DepthAI-Arduino-and-YOLOv8.git
   cd Object-Tracking-with-DepthAI-Arduino-and-YOLOv8
   ```

2. **Run the Installation Script**:
   The `install.sh` script installs ROS Noetic, sets up the environment, installs necessary Python and ROS packages, and configures a Catkin workspace.
   ```bash
   bash install.sh
   ```


3. **Install Additional Dependencies**:
   YOLOv8 and DepthAI libraries:
   ```bash
   pip3 install ultralytics torch torchvision numpy opencv-python depthai
   ```

## Project Structure

```
catkin_ws/                       # Project root
├── package.xml                  # ROS package metadata
├── CMakeLists.txt               # Build instructions for ROS
├── install.sh                   # Installation script for dependencies
├── src/
│   ├── main.py                  # Main script for object tracking
│   ├── run_project.py           # Entry point for running the project
│   ├── uvc_rgb.py               # UVC camera handling script
└── README.md                    # Project documentation
```

## Usage

1. **Source ROS Environment**:
   Before running the project, source the ROS and Catkin environment:
   ```bash
   source /opt/ros/noetic/setup.bash
   source ~/Object-Tracking-with-DepthAI-Arduino-and-YOLOv8/devel/setup.bash
   ```

2. **Launch the Project**:
   Use `run_project.py` as the main entry point for the project:
   ```bash
   python3 src/run_project.py
   ```

3. **Configure YOLO Model**:
   Ensure YOLOv8 is correctly configured in `main.py` to handle object detection. If the model path needs adjustment, update it within the script.

## Files and Scripts

### 1. `package.xml`
Defines the ROS package `object_tracking` and its dependencies:
- `rospy`, `std_msgs`, `geometry_msgs`, and `sensor_msgs` for ROS communication.

### 2. `CMakeLists.txt`
Contains the build instructions for the `object_tracking` package, specifying dependencies on ROS messages and including Python scripts for installation.

### 3. `install.sh`
A shell script that installs necessary ROS packages, Python dependencies, YOLO libraries, and NVIDIA CUDA Toolkit (optional). The script also initializes and configures a Catkin workspace.

   To run:
   ```bash
   bash install.sh
   ```

### 4. `src/main.py`
The main script for tracking objects:
   - Initializes the ROS node for object tracking.
   - Publishes detected objects' data using `geometry_msgs` for spatial information and `sensor_msgs` for camera data.

### 5. `src/run_project.py`
Acts as the main execution point. This script sets up the environment and initiates `main.py` for object detection and tracking.

   To run:
   ```bash
   python3 ~/Object-Tracking-with-DepthAI-Arduino-and-YOLOv8/catkin_ws/src/run_project.py
   ```

### 6. `src/uvc_rgb.py`
Handles the UVC camera RGB input:
   - Interfaces with connected cameras to capture RGB frames.
   - Prepares the frames for YOLO-based object detection.

   To run:
   ```bash
   python3 ~/Object-Tracking-with-DepthAI-Arduino-and-YOLOv8/catkin_ws/src/uvc_rgb.py
   ```

## ROS Package Details

The `object_tracking` package utilizes ROS for managing the communication between nodes and handling data from sensors, particularly camera data. Here are the key components:

- **Dependencies**: `rospy`, `std_msgs`, `geometry_msgs`, `sensor_msgs`.
- **Messaging**:
  - `std_msgs`: General messages for standard data types.
  - `geometry_msgs`: Used for transmitting object location data.
  - `sensor_msgs`: Manages image and sensor data for tracking purposes.

### Key Topics

- **`/object_tracking/detected_objects`**: Publishes detected objects' information, including their spatial location and size.
- **`/camera/rgb/image_raw`**: Handles raw RGB data from the camera to feed into YOLO.

## Known Issues

- **CUDA Compatibility**: The NVIDIA CUDA Toolkit is optional but enhances performance if running YOLOv8 on a compatible GPU.
- **Camera Compatibility**: Ensure your camera supports UVC; otherwise, modify `uvc_rgb.py` to handle different input formats.

## Contribution Guidelines

1. Fork the repository.
2. Create a new branch for your feature or bug fix.
3. Test your changes thoroughly.
4. Submit a pull request with a detailed description of your changes.

## License

This project is licensed under the BSD License. See the `LICENSE` file for details.

#!/bin/bash

set -e

echo "Updating package list and installing dependencies..."
sudo apt update
sudo apt install -y python3-pip python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential cmake

echo "Installing ROS Noetic..."
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install -y ros-noetic-desktop-full

echo "Setting up rosdep..."
sudo rosdep init
rosdep update

echo "Sourcing ROS setup script..."
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

echo "Installing ROS Python dependencies..."
pip3 install rospkg catkin_pkg

echo "Installing additional ROS packages..."
sudo apt install -y ros-noetic-rosserial ros-noetic-rosserial-arduino ros-noetic-rosserial-server ros-noetic-rosserial-python

echo "Setting up Catkin workspace..."
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
source devel/setup.bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

echo "Installing DepthAI dependencies and library..."
sudo wget -qO- https://docs.luxonis.com/install_dependencies.sh | bash
python3 -m pip install depthai

echo "Installing Python libraries for YOLOv8..."
pip3 install ultralytics torch torchvision numpy opencv-python

echo "Installing Ultralytics YOLO library..."
pip3 install ultralytics

echo "Installing NVIDIA CUDA Toolkit (optional)..."
sudo apt install -y nvidia-cuda-toolkit

echo "Installation complete. Please restart your terminal or run 'source ~/.bashrc' to apply changes."

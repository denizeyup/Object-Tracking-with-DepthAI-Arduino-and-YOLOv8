import subprocess
import time
import sys
import os

def start_roscore():
    terminal_command = "roscore"
    subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', terminal_command + '; exec bash'])
    
def start_ros_serial():
    terminal_command = "rosrun rosserial_python serial_node.py port:=/dev/serial/by-id/usb-1a86_USB2.0-Ser_-if00-port0 _baud:=115200" 
    subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', terminal_command + '; exec bash'])

def start_depthaiCam():
    depthai_path = os.path.join(os.path.dirname(__file__), "uvc_rgb.py")
    subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', f'python3 {python_file_path}; exec bash'])


def start_ros_node():
    terminal_command = "cd catkin_ws"
    terminal_command_0 = "rosrun kangal talker"
    terminal_command_1 = "source ./devel/setup.bash"
    subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', terminal_command + '; exec bash'])
    subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', terminal_command_0 + '; exec bash'])
    subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', terminal_command_1 + '; exec bash'])
    


if __name__ == "__main__":
    start_roscore()
    time.sleep(3)
    start_ros_serial()
    time.sleep(3)
    start_depthaiCam()
    time.sleep(5)
    start_ros_node()




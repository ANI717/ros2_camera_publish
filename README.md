<p align="center">
  <h1 align="center">ROS 2 Camera Image Publish Package (Python 3)</h1>
</p>

ROS 2 Package to Publish Camera Image as sensor_msgs/Image message. Compatible with Raspberry Pi 64 Bit OS. ROS cv_bridge package is not required.

## Colaborators
[Computer Fusion Laboratory (CFL) - Temple University College of Engineering](https://sites.temple.edu/cflab/people/)
* [Animesh Bala Ani](https://animeshani.com/)
* [Dr. Li Bai](https://engineering.temple.edu/about/faculty-staff/li-bai-lbai)

## Setup Raspberry Pi 64 Bit (RaspiOS arm64)
Follow instructions from https://github.com/ANI717/headless_raspberrypi_setup

## Setup ROS 2 Dashing from Source
Follow instructions from https://github.com/ANI717/ros2_raspberrypi64

## Dependency
GLobal Modules (Can be installed globally with PIP)
```
OpenCV
NumPy
Json
```
ROS Modules (Comes with ROS 2 installation)
```
rclpy
sensor_msgs
ament_index_python.packages
```

## Install OpenCV
```
sudo apt update
sudo apt install python3-pip
python3 -m pip install opencv-contrib-python
```

## Parameter Settings
Edit settings.json file to set the parameters.
```
device_index: 0 (Camera Device Index for OpenCV Videocapture. This is useful while working with multiple cameras.)
topic: "racecar/camera" (topic name)
queue_size: 1 (amount of queued messages)
period: 0.1 (executes every 0.1 seconds)
```

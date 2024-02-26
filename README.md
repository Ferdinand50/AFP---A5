# tof_detection_module
AFP Project 5


## Prerequisite
- Python 3.10
- ROS 2 Humble
- Python API Lucid Vision https://thinklucid.com/downloads-hub/


## Installation

1. Setup ROS ws
```shell
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2. Clone repository
```shell
git clone git@github.com:Ferdinand50/tof_detection_module.git
```

3. Build ws
```shell
colcon build
```

## RUN
```shell
ros2 run tof_detection_module detection
```

If you can not connect the camera follow these steps
Camera IP Setup

follow official docs
file:///home/ferdi/ArenaSDK_Linux_x64/docs/html/arena_sdk_linux.html

## ToF Camera
https://support.thinklucid.com/using-ros2-for-linux/


## Docs
The code first gets a point cloud from the tof camera and saves the ply file. The saved ply is the passed to the processing method. Here seggregation and detection takes place. Then the type and pose of the detected object is printed.

Possible Bug: It looks like that in Linux the point cloud camera coordinate system has an offset. Therefore the returned pose is wrong. The code works fine but the Linux implementation of the Python API is not working correctly. In windows this is not a problem. Therefore the transformation is needed for Linux (this needs detailed calibration).

An Point cloud publisher is also implement. However this node is not yet stable. It uses a saved point cloud. In the future it should use the point cloud from the camera directly.

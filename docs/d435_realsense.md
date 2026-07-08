# Intel RealSense D435 Bring-up

This document records the current D435 state and the minimum repeatable test
flow for the K1 Muse Pi Pro / Bianbu LXQT v2.3.3 robot.

## Current Status

- Device: Intel RealSense D435.
- RGB and depth capture have been verified on the K1 board.
- Point cloud/depth quality is not yet accepted as final.
- The previous short tests showed possible dark RGB exposure and questionable
  depth/point cloud quality, so this needs a controlled retest.
- The camera must be mounted with the lens cover removed.
- Mechanical mount thread: `1/4-20 UNC`.

## Cable Requirement

Use a real USB 3.x data cable.

Recommended cable:

- USB-C to USB-A or USB-C to USB-C, depending on the host port being used.
- Must explicitly support USB 3.0/3.1/3.2 data.
- Avoid charge-only phone cables for long-term deployment.

Symptoms of a bad or USB 2.0-only cable:

- Reduced frame rate.
- Depth stream drops or unstable point cloud.
- `rs-enumerate-devices` reports a low-speed connection.
- RGB works but depth is poor or intermittent.

## K1 Software Setup

Official SpaceMIT D4xx notes recommend librealsense `2.56.4` on K1-class
RISC-V boards.

Install SDK:

```bash
wget https://archive.spacemit.com/ros2/prebuilt_libs/install_scripts_common/install_librealsense.sh
bash install_librealsense.sh 2.56.4
```

Install ROS dependencies:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-diagnostic-updater \
  ros-humble-rqt-image-view \
  ros-dev-tools
```

Build `realsense-ros`:

```bash
mkdir -p ~/realsense_ws/src
cd ~/realsense_ws/src
git clone https://github.com/IntelRealSense/realsense-ros.git
cd realsense-ros
git checkout 5ef0858501a94d769381417aaafe6e0f56515292

cd ~/realsense_ws
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_PREFIX_PATH=/opt/ext/librealsense/librealsense-2.56.4
```

## Hardware Detection

```bash
lsusb
rs-enumerate-devices
rs-enumerate-devices -c
```

The board should detect the camera as a RealSense device. If the USB connection
is not SuperSpeed, change the cable before judging depth quality.

## ROS Runtime

```bash
source /opt/ros/humble/setup.bash
source ~/realsense_ws/install/setup.bash

ros2 launch realsense2_camera rs_launch.py \
  depth_module.depth_profile:=640,480,30 \
  depth_module.infra_profile:=640,480,30 \
  rgb_camera.color_profile:=640,480,30 \
  pointcloud.enable:=false
```

Useful checks:

```bash
ros2 topic list | grep camera
ros2 topic hz /camera/camera/depth/image_rect_raw
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/depth/color/points
```

Viewer:

```bash
ros2 run rqt_image_view rqt_image_view
```

## Quality Retest Scene

Use a static scene before using D435 data for perception or point cloud fusion:

- Camera on the robot, lens cover removed.
- Scene contains boxes, chair/table edges, and a flat wall or board.
- Person outside the frame.
- Room light on, then repeat with robot lamp at 0-5 percent. The current
  20 W lamp can overexpose D435 RGB at 10-15 percent depending on angle and
  distance, so do not use 20-40 percent without a diffuser or new lamp angle.
- Capture RGB, depth colorization, and point cloud separately.

Pass criteria:

- RGB exposure is usable without appearing much darker than the real scene.
- Depth edge ghosting is limited around object boundaries.
- Point cloud is optional on K1 and should be enabled only after RGB/depth are stable.
- No large holes on close matte objects in the expected range.
- Frame rate remains near the requested profile.

## Known Open Items

- Depth quality still needs a longer controlled video/rosbag capture.
- USB throughput should be checked with the final cable.
- D435 should not share an unstable power path with high-current light tests.

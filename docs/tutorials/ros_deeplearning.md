# Deep Learning in ROS

Note: This tutorial is applicable for Kerloud UAV products equipped with Jetson Nano only.
To develop robotics applications, we can follow the official repository in https://github.com/dusty-nv/ros_deep_learning
to integrate Nvidia deep learning capabilities with ROS. The code include several ros nodes to deploy networks based on installed jetson inference
libraries.

## Code Structure

Main directories are listed below for the ros deep learning repo:

- launch/: launch files to deploy ROS nodes for deep learning tasks.
- src/: source codes for ROS nodes:
  - node_detectnet: ROS node to deploy the detectnet network for object localization.
  - node_imagenet: ROS node to deploy the imagenet network for visual recognition.
  - node_segment: ROS node to deploy the segnet network for semantic segmentation.
  - node_video_source: ROS node to handle the video input and publish image messages.
  - node_video_output: ROS node to create video stream with overlayed images.
  - image_converter.cpp: class to convert images to various ros messages.

## How to Install

Users have to install jetson-inference libraries , ROS and build the ros_deep_learning workspace. The official guide is
https://github.com/dusty-nv/ros_deep_learning#installation. For jetson-inference installation, please refer to the previous page.
For ROS melodic, we have to install the prerequisites below:

```bash
sudo apt-get install ros-melodic-image-transport ros-melodic-vision-msgs
```

Then run ‘catkin_make’ for the workspace under the directory: ~/ros_workspace.

## How to Run

Before proceeding, if you’re using ROS Melodic make sure that roscore is running first.
Launch the video viewer to check whether the video stream is OK:

```bash
cd ~/ros_workspace
source devel/setup.bash
roslaunch ros_deep_learning video_viewer.ros1.launch input:=csi://0 output:=display://0
```

For input and output settings, refer to https://github.com/dusty-nv/jetson-inference/blob/master/docs/aux-streaming.md for details

Launch the imagenet node for video recognition:

```bash
cd ~/ros_workspace
source devel/setup.bash
roslaunch ros_deep_learning imagenet.ros1.launch input:=csi://0 output:=display://0
```

Launch the detectnode for object detection:

```bash
cd ~/ros_workspace
source devel/setup.bash
roslaunch ros_deep_learning detectnet.ros1.launch input:=csi://0 output:=display://0
```

Launch the segnet for semantic segmentation:

```bash
cd ~/ros_workspace
source devel/setup.bash
roslaunch ros_deep_learning segnet.ros1.launch input:=csi://0 output:=display://0
```

Make sure that you have downloaded necessary networks for jetson-inference. If not, you might consider downloading them manually
by following instructions in https://github.com/dusty-nv/jetson-inference/releases.

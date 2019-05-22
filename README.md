# ros_tensorflow

## Introduction

This repo introduces how to integrate **Tensorflow** framework into **ROS** with object detection API. 

And through this repo, you can realize **mnist**, **object recognition**, and **object detection** respectively.

## Requirements

- Ubuntu 16.04 with Python2.7
- [Install ROS(Kinetic)](http://wiki.ros.org/kinetic/Installation/Ubuntu) with [catkin build](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment). Create a catkin workspace.
- [Install Tensorflow](https://www.tensorflow.org/install/)(1.2.0-1.10.0 all be ok)
- [Install CUDA 6.5 for NVIDIA TK1](https://gist.github.com/jetsonhacks/6da905e0675dcb5cba6f) (Choosing according to your needs)
- Some dependencies
  ```sh
  sudo apt-get install protobuf-compiler python-pil python-lxml
  sudo pip install jupyter
  sudo pip install matplotlib
  ```

## Grab the source

```sh
cd [CATKIN_WS]/src
git clone https://github.com/cong/ros_tensorflow.git
```

## Build

Build ROS package by

```sh
cd [CATKIN_WS]
catkin_make
```

PS: Python doesn't seem to need to be compiled.

## Run

#### For mnist:

```sh
# First, open a terminal, execute
roscore

# Second, Open another terminal, then execute
# Please install "usb_cam" node before you execute blow
roslaunch usb_cam usb_cam-test.launch

# Third, open another terminal, then execute
roslaunch ros_tensorflow ros_tensorflow_mnist.launch
# You can echo a topic to receive the string message.
rostopic echo /result_ripe
```

#### For object recognition:

```sh
# Third, open another terminal, then execute
roslaunch ros_tensorflow ros_tensorflow_classify.launch
# You can echo a topic to receive the string message.
rostopic echo /result_ripe
```

#### For object detection:

```sh
# Third, open another terminal, then execute
roslaunch ros_tensorflow ros_tensorflow_detect.launch
# You can through "image_view" node to receive images detected.
rosrun image_view image_view image:=/result_ripe
```

## ROS Topics

Publish a topic : `/result_ripe`

Receive an image : `usb_cam/image_raw`

## Optional setting

- You can realize your project by replacing the files in "ros_tensorflow/include/" with your own files.
- If you feel that it is helpful to you, please give me a star. Thx!  :)
- For more information you can visit the [Blog](http://wangcong.net).

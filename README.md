# VINS-application: OAK-D
+ Build Process and explanation [here](https://github.com/engcang/VINS-application)
<br>

## Index
1. Prerequisites
+ [ROS packages](#ros-package)
+ [Kalibr (Calibration for Cam-IMU)](#kalibrcalibration-for-cameras-imu)
2. [Results](#results)

<br>

## ROS package
+ OAK-D ROS [here](https://github.com/engcang/oakd-ros-simple)
~~~shell
$ sudo apt install libusb-1.0-0-dev
$ cd ~/<your_workspace>/src
$ git clone --recursive git@github.com:engcang/oakd-ros-simple

$ rm -r ~/.hunter

$ cd oakd-ros-simple/depthai-core
$ mkdir build && cd build
$ cmake .. -DBUILD_SHARED_LIBS=ON

$ make install

$ cd ~/your_workspace (check directory)

Check directory (your_workspace)

$ catkin build -Ddepthai_DIR=<your_workspace>/src/oakd-ros-simple/depthai-core/build/install/lib/cmake/depthai

or

$ catkin config -Ddepthai_DIR=<your_workspace>/src/oakd-ros-simple/depthai-core/build/install/lib/cmake/depthai
$ catkin build

$ roslaunch oakd_ros main.launch
~~~

  
<br>

## Kalibr(calibration for cameras-IMU)

+ Used [Kalibr](https://github.com/ethz-asl/kalibr) as [here](https://support.stereolabs.com/hc/en-us/articles/360012749113-How-can-I-use-Kalibr-with-the-ZED-Mini-camera-in-ROS-) for ZED-mini camera
+ First, calibrate cameras
~~~shell
$ kalibr_calibrate_cameras --bag Kalibr_data.bag --topics /oakd/stereo_ir/left/image_raw /oakd/stereo_ir/right/image_raw --models pinhole-radtan pinhole-radtan --target april_grid.yaml
~~~
+ Then, calibrate IMU with cameras
~~~shell
$ kalibr_calibrate_imu_camera --bag Kalibr_data.bag --cam camchain-Kalibr_data.yaml --imu imu-params.yaml --target april_grid.yaml
~~~
+ for IMU, I tried `Pixhawk 4 mini`, and built-in IMU of `OAK-D-PRO`

<br>

## Results
+ VINS-Fusion Stereo version test with OAK-D+Pixhawk 4 mini: [here](https://youtu.be/Hjcjg9L4j9o)
+ VINS-FUsion Stereo version test with OAK-D PRO version: [here](https://youtu.be/Xw-HIPbn0wg)

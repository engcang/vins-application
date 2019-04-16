# VINS-application
+ VINS-Mono and Fusion application of different sets of cameras and imu on different board including desktop and jetson xavier
+ Branch : zed-m, pointgrey_myahrs, intel d435i
zed-m : calibration from co. pt,my : links, 

<br>

# Index
### 1. [Algorithm & Gpu, Cpu version](#1-algorithm--gpu-cpu-version-1)
### 2. [Parameters](#2-parameters-1)
### 3. Prerequisites
#### ● [OpenCV with CUDA](#-opencv-with-cuda-1)
#### ● [USB performance](#-usb-performance-1)
#### ● [IMU-Camera Calibration](#-calibration--kalibr---synchronization-time-offset-1)
#### ● [Installation](#-installation-1)
### 4. [Comparison & Application](#4-comparison--application-1)

<br><br><br>

# 1. Algorithm & GPU, CPU version
# 2. Parameters
Hz, feature numbers
# 3. Prerequisites
### ● OpenCV with CUDA
### ● USB performance
  + Link : [here](https://github.com/KumarRobotics/flea3#optimizing-usb-performance-under-linux)
  + For Xavier : [here](https://devtalk.nvidia.com/default/topic/1049581/jetson-agx-xavier/change-usbcore-usbfs_memory_mb/)
  ~~~shell
  $ sudo ./flash.sh -k kernel -C "usbcore.usbfs_memory_mb=1000" -k kernel-dtb jetson-xavier mmcblk0p1
  ~~~
### ● Calibration : Kalibr -> synchronization, time offset
[Kalibr](https://github.com/ethz-asl/kalibr) -> synchronization, time offset
### ● Installation
# 4. Comparison & Application

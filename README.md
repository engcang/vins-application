# VINS-application
+ VINS-Mono and Fusion application of different sets of cameras and imu on different board including desktop and jetson xavier
+ Branch : zed-m, pointgrey_myahrs, intel d435i
<br>
<br>

# Index
### 1. [Algorithm & Gpu, Cpu version](#1-algorithm--gpu-cpu-version-1)
### 2. [Parameters](#2-parameters-1)
### 3. Prerequisites
#### ● [ceres solver and eigen](#-ceres-solver-and-eigen--mandatory-for-vins-1) : Mandatory for VINS
#### ● [OpenCV with CUDA](#-opencv-with-cuda--necessary-for-gpu-version-1) : Necessary for GPU version
#### ● [USB performance](#-usb-performance--have-to-improve-performance-of-sensors-with-usb-1) : Have to improve performance of sensors with USB
#### ● [IMU-Camera Calibration](#-calibration--kalibr---synchronization-time-offset-extrinsic-parameter) : Synchronization, time offset, extrinsic parameter
#### ● [Installation](#-installation-1)
### 4. [Comparison & Application](#4-comparison--application-1)

<br><br><br>

# 1. Algorithm & GPU, CPU version
+ [CPU version](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)
+ [GPU version](https://github.com/pjrambo/VINS-Fusion-gpu)
<br>

# 2. Parameters
+ Camera frame rate
+ Max tracking Feature number
+ time offset
+ GPU acceleration
+ Thread numbers
<br>

# 3. Prerequisites
### ● ceres solver and eigen : Mandatory for VINS
### ● OpenCV with CUDA : Necessary for GPU version
+ Install CUDA : [here](https://askubuntu.com/questions/799184/how-can-i-install-cuda-on-ubuntu-16-04)
+ Build OpenCV with CUDA 
~~~shell
$ sudo apt-get purge libopencv* python-opencv
$ sudo apt-get update
$ sudo apt-get install -y build-essential pkg-config
$ sudo apt-get install -y \
    cmake \
    libavcodec-dev \
    libavformat-dev \
    libavutil-dev \
    libeigen3-dev \ # recommend to build from source : http://eigen.tuxfamily.org/index.php?title=Main_Page
    libglew-dev \
    libgtk2.0-dev \
    libgtk-3-dev \
    libjpeg-dev \
    libpng-dev \
    libpostproc-dev \
    libswscale-dev \
    libtbb-dev \
    libtiff5-dev \
    libv4l-dev \
    libxvidcore-dev \
    libx264-dev \
    qt5-default \
    zlib1g-dev \
    libgl1 \
    libglvnd-dev \
    pkg-config
$ mkdir <opencv_source_directory> && cd <opencv_source_directory>
$ wget -O opencv.zip https://github.com/opencv/opencv/archive/3.4.1.zip # check version
$ unzip opencv.zip
$ cd <opencv_source_directory>/opencv && mkdir build && cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D CMAKE_INSTALL_PREFIX=/usr/local \ # set Path you want, default is recommended
      -D WITH_CUDA=ON \
      -D CUDA_ARCH_BIN=7.2 \ # check your BIN version : http://arnon.dk/matching-sm-architectures-arch-and-gencode-for-various-nvidia-cards/
      -D CUDA_ARCH_PTX="" \
      -D ENABLE_FAST_MATH=ON \
      -D CUDA_FAST_MATH=ON \
      -D WITH_CUBLAS=ON \
      -D WITH_LIBV4L=ON \
      -D WITH_GSTREAMER=ON \
      -D WITH_GSTREAMER_0_10=OFF \
      -D WITH_QT=ON \
      -D WITH_OPENGL=ON \
      -D CUDA_NVCC_FLAGS="--expt-relaxed-constexpr" \
      -D WITH_TBB=ON \
      ../
$ time make -j8 # 8 : numbers of core
$ sudo make install
$ sudo rm -r <opencv_source_directory> #optional
~~~
<br>

### ● USB performance : Have to improve performance of sensors with USB
  + Link : [here](https://github.com/KumarRobotics/flea3#optimizing-usb-performance-under-linux)
  + For Xavier : [here](https://devtalk.nvidia.com/default/topic/1049581/jetson-agx-xavier/change-usbcore-usbfs_memory_mb/)
  ~~~shell
  $ sudo ./flash.sh -k kernel -C "usbcore.usbfs_memory_mb=1000" -k kernel-dtb jetson-xavier mmcblk0p1
  ~~~
<br>

### ● Calibration : Kalibr -> synchronization, time offset, extrinsic parameter
+ [Kalibr](https://github.com/ethz-asl/kalibr) -> synchronization, time offset <br>
+ For ZED cameras : [here](https://support.stereolabs.com/hc/en-us/articles/360012749113-How-can-I-use-Kalibr-with-the-ZED-Mini-camera-in-ROS-)

<br>

### ● Installation
+ git clone and build from source
~~~shell
$ cd ~/catkin_ws/src
$ git clone https://github.com/HKUST-Aerial-Robotics/VINS-Fusion #CPU
or 
$ git clone https://github.com/pjrambo/VINS-Fusion-gpu #GPU
$ cd .. && catkin build camera models # camera models first
$ catkin build
~~~
Before build VINS-Fusion, process below could be required.
<br>

+ For GPU version, if OpenCV with CUDA was built manually, build cv_bridge manually also
~~~shell
$ cd ~/catkin_ws/src && git clone https://github.com/ros-perception/vision_opencv
$ gedit vision_opencv/cv_bridge/CMakeLists.txt
~~~
Edit OpenCV PATHS in CMakeLists and include cmake file
~~~txt
find_package(OpenCV 3 REQUIRED PATHS /usr/local/share/OpenCV NO_DEFAULT_PATH
  COMPONENTS
    opencv_core
    opencv_imgproc
    opencv_imgcodecs
  CONFIG
)
include(/usr/local/share/OpenCV/OpenCVConfig.cmake) #under catkin_python_setup()
~~~
~~~shell
$ cd .. && catkin build cv_bridge
~~~
<br>

+ For GPU version, Edit CMakeLists.txt for loop_fusion and vins_estimator
~~~shell
$ cd ~/catkin_ws/src/VINS-Fusion-gpu/loop_fusion && gedit CMakeLists.txt
or
$ cd ~/catkin_ws/src/VINS-Fusion-gpu/vins_estimator && gedit CMakeLists.txt
~~~
~~~txt
##For loop_fusion : line 19
#find_package(OpenCV)
include(/usr/local/share/OpenCV/OpenCVConfig.cmake)

##For vins_estimator : line 20
#find_package(OpenCV REQUIRED)
include(/usr/local/share/OpenCV/OpenCVConfig.cmake)
~~~
<br>

# 4. Comparison & Application
+ /tf vs VINS-Mono on flightgoggles : [youtube](https://youtu.be/U4TJ7ZyfWD8)
+ Loop Fusion vs vins node on flightgoggles : [youtube](https://youtu.be/cvhI_1XQQt4)
+ Real World VINS-Mono with pointgrey cam, myAHRS+ imu on Jetson Xavier : [youtube](https://youtu.be/4qJYoND9OYk)
+ Real World VINS(GPU+version) with pointgrey, myAHRS at Intel i7-8700k, TITAN RTX : [youtube](https://youtu.be/UEZMZMFFhYs)
<br>

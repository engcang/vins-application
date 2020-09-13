# VINS-application
## Mainly focused on Build process and explanation
+ VINS-Mono and Fusion application of different sets of cameras and imu on different board including desktop and jetson xavier
## This repository contains many branches! as following : 
+ **Branch** : [ZED-mini](https://github.com/engcang/VINS-application/tree/zed-mini), [Pointgrey_myAHRS](https://github.com/engcang/VINS-application/tree/Pointgrey_MyAHRS+), [intel D435i](https://github.com/engcang/VINS-application/tree/Intel-D435i), [FlightGoggles](https://github.com/engcang/vins-application/tree/flightgoggles)
    + Including **config.yaml** files and **Calibration data**
    + git clone -b <branch_name> --single-branch https://github.com/engcang/VINS-application
+ Tested on : Jetson Xavier, Jetson TX2, Intel i7-6700k, i7-8700k
### Result clips : [here](#4-comparison--application-1)
<br>
<br>

# Index
### 1. [Algorithm & Gpu, Cpu version](#1-algorithm--gpu-cpu-version-1)
### 2. [Parameters](#2-parameters-1)
### 3. Prerequisites
#### ● [Ceres solver and Eigen](#-ceres-solver-and-eigen--mandatory-for-vins) : Mandatory for VINS (build Eigen first)
#### ● [OpenCV with CUDA](#-opencv-with-cuda--necessary-for-gpu-version-1) : Necessary for GPU version
#### ● [CV_Bridge and image_proc with Built OpenCV](#-cv_bridge-and-image_proc-with-built-opencv--necessary-for-whom-built-opencv-manually-from-above) : Necessary for GPU version
#### ● [USB performance](#-usb-performance--have-to-improve-performance-of-sensors-with-usb-1) : Have to improve performance of sensors with USB
#### ● [IMU-Camera Calibration](#-calibration--kalibr---synchronization-time-offset-extrinsic-parameter) : Synchronization, time offset, extrinsic parameter
#### ● [IMU-Camera rotational extrinsic](#-imu-camera-rotational-extrinsic-example) : Rotational extrinsic between IMU and Cam
#### ● [Installation](#-installation-1)
#### ● [Trouble shooting](#-trouble-shooting-1)
### 4. [Comparison & Application](#4-comparison--application-1)

<br><br><br>

# 1. Algorithm & GPU, CPU version
+ Mainly use Ceres-solver with Eigen, **performance of VINS is strongly proportional to CPU performance and some parameters**
+ [CPU version](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)
+ [GPU version](https://github.com/pjrambo/VINS-Fusion-gpu)
<br>

# 2. Parameters
+ Camera frame rate 
    + lower - low time delay, poor performance
    + higher - high time delay, better performance
    + has to be set from **camera launch file** : 10~30hz
##### from src/VINS/config/<config_file_name>.yaml
+ Max tracking Feature number **max_cnt**
    + 100~150, same correlation as camera frame rates
+ time offset **estimated_td : 1**, **td : value from [kalibr](#-calibration--kalibr---synchronization-time-offset-extrinsic-parameter)**
+ GPU acceleration **use_gpu : 1**, **use_gpu_acc_flow : 1** (for GPU version)
+ Thread numbers **multiple_thread : enabling multi-threads**
<br>

# 3. Prerequisites
### ● Ceres solver and Eigen : Mandatory for VINS
+ Eigen latest Stable version below [new home](https://gitlab.com/libeigen/eigen) or [old home](http://eigen.tuxfamily.org/index.php?title=Main_Page)
~~~shell
$ wget -O eigen.zip https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.zip #check version
$ unzip eigen.zip
$ cd eigen-3.3.7
& mkdir build && cd build
$ cmake .. && sudo make install
~~~
+ Eigen [3.3.90 version](https://eigen.tuxfamily.org/dox-devel/group__TutorialSlicingIndexing.html) or later for using ***slicing and Indexing*** as [here](https://eigen.tuxfamily.org/dox-devel/group__TutorialSlicingIndexing.html)
~~~shell
$ git clone https://gitlab.com/libeigen/eigen.git
$ cd eigen 
$ mkdir build && cd build
$ cmake .. && sudo make install
~~~

+ Ceres solver [home](http://ceres-solver.org/installation.html)
~~~shell
$ sudo apt-get install -y cmake libgoogle-glog-dev libatlas-base-dev libsuitesparse-dev
$ wget http://ceres-solver.org/ceres-solver-1.14.0.tar.gz
$ tar zxf ceres-solver-1.14.0.tar.gz
$ mkdir ceres-bin
$ mkdir solver && cd ceres-bin
$ cmake ../ceres-solver-1.14.0 -DEXPORT_BUILD_DIR=ON -DCMAKE_INSTALL_PREFIX="../solver"  #good for build without being root privileged and at wanted directory
$ make -j8 # 8 : number of cores
$ make test
$ make install
~~~
<br><br>

### ● OpenCV with CUDA : Necessary for GPU version
+ Install **CUDA** and **Graphic Driver** : 
  + for upper than **18.04**,
~~~shell
    $ sudo apt install gcc make
    $ sudo ubuntu-drivers devices
    $ sudo ubuntu-drivers autoinstall
    $ sudo reboot
    
    # get cuda install script at https://developer.nvidia.com/cuda-downloads
    $ sudo sh cuda_<version>_linux.run
    $ sudo reboot
    
    $ gedit ~/.bashrc
    # type
    export PATH=<CUDA_PATH>/bin:$PATH #ex: /usr/local/cuda-10.1
    export LD_LIBRARY_PATH=<CUDA_PATH>/lib64:$LD_LIBRARY_PATH #ex : /usr/local/cuda-10.1
~~~
  + check CUDA version using **nvcc --version**
~~~shell
# check installed cuda version
$ nvcc --version
# if nvcc --version does not print out CUDA,
$ gedit ~/.profile
# type below and save
export PATH=<CUDA_PATH>/bin:$PATH #ex: /usr/local/cuda-10.1
export LD_LIBRARY_PATH=<CUDA_PATH>/lib64:$LD_LIBRARY_PATH #ex : /usr/local/cuda-10.1
$ source ~/.profile
~~~

+ Build OpenCV with CUDA - references : [link 1](https://webnautes.tistory.com/1030), [link 2](https://github.com/jetsonhacks/buildOpenCVXavier/blob/master/buildOpenCV.sh)
    + for Xavier do as below or sh file from jetsonhacks [here](https://github.com/jetsonhacks/buildOpenCVXavier)
    + If want to use **C API (e.g. Darknet YOLO)** consider : 
        + **Recommend version is 3.4.0** because darknet has to use C API with OpenCV [refer](https://github.com/pjreddie/darknet/issues/551)
        + or **Patch as [here](https://github.com/opencv/opencv/issues/10963)** to use other version 
            + should **comment** the /usr/local/include/opencv2/highgui/highgui_c.h line 139 [as here](https://stackoverflow.com/questions/48611228/yolo-compilation-with-opencv-1-fails) after install
~~~shell
$ sudo apt-get purge libopencv* python-opencv
$ sudo apt-get update
$ sudo apt-get install -y build-essential pkg-config
$ sudo apt-get install -y cmake libavcodec-dev libavformat-dev libavutil-dev \
    libglew-dev libgtk2.0-dev libgtk-3-dev libjpeg-dev libpng-dev libpostproc-dev \
    libswscale-dev libtbb-dev libtiff5-dev libv4l-dev libxvidcore-dev \
    libx264-dev qt5-default zlib1g-dev libgl1 libglvnd-dev pkg-config \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev mesa-utils #libeigen3-dev # recommend to build from source : http://eigen.tuxfamily.org/index.php?title=Main_Page
$ sudo apt-get install python2.7-dev python3-dev python-numpy python3-numpy
$ mkdir <opencv_source_directory> && cd <opencv_source_directory>
$ wget -O opencv.zip https://github.com/opencv/opencv/archive/3.4.1.zip # check version
$ unzip opencv.zip
$ cd <opencv_source_directory>/opencv && mkdir build && cd build
# check your BIN version : http://arnon.dk/matching-sm-architectures-arch-and-gencode-for-various-nvidia-cards/
# 7.2 for Xavier, 5.2 for GTX TITAN X
$ cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D CMAKE_C_COMPILER=gcc-6 \
      -D CMAKE_CXX_COMPILER=g++-6 \
      -D CMAKE_INSTALL_PREFIX=/usr/local \
      -D WITH_CUDA=ON \
      -D CUDA_ARCH_BIN=7.2 \
      -D CUDA_ARCH_PTX="" \
      -D ENABLE_FAST_MATH=ON \
      -D CUDA_FAST_MATH=ON \
      -D WITH_CUBLAS=ON \
      -D WITH_LIBV4L=ON \
      -D WITH_GSTREAMER=ON \
      -D WITH_GSTREAMER_0_10=OFF \
      -D WITH_QT=ON \
      -D WITH_OPENGL=ON \
      -D BUILD_opencv_cudacodec=OFF \ #for cuda10-opencv3.4
      -D CUDA_NVCC_FLAGS="--expt-relaxed-constexpr" \
      -D WITH_TBB=ON \
      ../
$ time make -j8 # 8 : numbers of core
$ sudo make install
$ sudo rm -r <opencv_source_directory> #optional
~~~
#### ● when build **error** : 
+ Please include the appropriate gl headers before including cuda_gl_interop.h => reference [1](https://github.com/jetsonhacks/buildOpenCVXavier/blob/master/buildOpenCV.sh#L101), [2](https://github.com/jetsonhacks/buildOpenCVXavier/blob/master/patches/OpenGLHeader.patch), [3](https://devtalk.nvidia.com/default/topic/1007290/jetson-tx2/building-opencv-with-opengl-support-/post/5141945/#5141945)
+ modules/cudacodec/src/precomp.hpp:60:37: fatal error: dynlink_nvcuvid.h: No such file or directory
compilation terminated. --> **for CUDA version 10**
    + => reference [here](https://devtalk.nvidia.com/default/topic/1044773/cuda-setup-and-installation/error-in-installing-opencv-3-4-0-on-cuda-10/)
    + cmake ... -D BUILD_opencv_cudacodec=OFF ...
<br>

### ● CV_Bridge and image_proc with built OpenCV : Necessary for whom built OpenCV manually from above
#### ● CV_bridge
+ For GPU version, if OpenCV with CUDA was built manually, build cv_bridge manually also
~~~shell
$ cd ~/catkin_ws/src && git clone https://github.com/ros-perception/vision_opencv
# since ROS Noetic is added, we have to checkout to melodic tree
$ cd vision_opencv && git checkout origin/melodic
$ gedit vision_opencv/cv_bridge/CMakeLists.txt
~~~
+ Edit OpenCV PATHS in CMakeLists and include cmake file
~~~txt
#when error, try both lines
find_package(OpenCV 3 REQUIRED PATHS /usr/local/share/OpenCV NO_DEFAULT_PATH
#find_package(OpenCV 3 HINTS /usr/local/share/OpenCV NO_DEFAULT_PATH
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

#### ● image_proc
~~~shell
$ cd ~/catkin_ws/src && git clone https://github.com/ros-perception/image_pipeline
~~~
+ Edit OpenCV PATHS in CMakeLists in
~~~txt
1. depth_image_proc/CMakeLists.txt, 2. image_proc/CMakeLists.txt, 
3. image_view/CMakeLists.txt, 4. stereo_image_proc/CMakeLists.txt
find_package(OpenCV 3 REQUIRED PATHS /usr/local/share/OpenCV NO_DEFAULT_PATH)
5. image_publisher/CMakeLists.txt
find_package(OpenCV 3 REQUIRED PATHS /usr/local/share/OpenCV NO_DEFAULT_PATH COMPONENTS core)
6. image_rotate/CMakeLists.txt
find_package(OpenCV 3 REQUIRED PATHS /usr/local/share/OpenCV NO_DEFAULT_PATH COMPONENTS core imgproc)
~~~
~~~shell
$ cd ~/catkin_ws && catkin build
~~~
<br><br>

### ● USB performance : Have to improve performance of sensors with USB
  + Link : [here](https://github.com/KumarRobotics/flea3#optimizing-usb-performance-under-linux) for x86_64 desktops
  + TX1/TX2 : [here](https://www.matrix-vision.com/manuals/mvBlueFOX3/mvBC_page_quickstart.html#mvBC_subsubsection_quickstart_linux_requirements_optimising_usb)
  + For Xavier : [here](https://devtalk.nvidia.com/default/topic/1049581/jetson-agx-xavier/change-usbcore-usbfs_memory_mb/)
  ~~~shell
  $ sudo ./flash.sh -k kernel -C "usbcore.usbfs_memory_mb=1000" -k kernel-dtb jetson-xavier mmcblk0p1
  ~~~
<br>

### ● Calibration : Kalibr -> synchronization, time offset, extrinsic parameter
+ [Kalibr](https://github.com/ethz-asl/kalibr) -> synchronization, time offset <br>
+ For ZED cameras : [here](https://support.stereolabs.com/hc/en-us/articles/360012749113-How-can-I-use-Kalibr-with-the-ZED-Mini-camera-in-ROS-)
+ **ImportError: No module named Image** [reference](https://github.com/ethz-asl/kalibr/issues/67)
~~~shell
$ gedit kalibr/aslam_offline_calibration/kalibr/python/kalibr_camera_calibration/MulticamGraph.py
#import Image
from PIL import Image
~~~

<br>

### ● IMU-Camera rotational extrinsic example
+ Between ROS standard body(IMU) and camera
  <p align="center">
  <img src="https://github.com/engcang/vins-application/blob/master/extrinsic.png" width="600"/>
  </p>
+ **Left view :** Between ROS standard body(IMU) and down-pitched (look downward) camera
  <p align="center">
  <img src="https://github.com/engcang/vins-application/blob/master/pitching.png" width="600"/>
  </p>


<br>

### ● Installation
+ git clone and build from source
~~~shell
$ cd ~/catkin_ws/src
$ git clone https://github.com/HKUST-Aerial-Robotics/VINS-Fusion #CPU
or 
$ git clone https://github.com/pjrambo/VINS-Fusion-gpu #GPU
$ cd .. && catkin build camera_models # camera models first
$ catkin build
~~~
**Before build VINS-Fusion, process below could be required.**

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

### ● Trouble shooting
+ Aborted error when running **vins_node** : 
~~~shell
 $ echo "export MALLOC_CHECK_=0" >> ~/.bashrc
 $ source ~/.bashrc
~~~

# 4. Comparison & Application
+ /tf vs VINS-Mono on FlightGoggles : [youtube](https://youtu.be/U4TJ7ZyfWD8), with CPU [youtube](https://www.youtube.com/watch?v=1QUypn7GbXc)
+ Loop Fusion vs vins node on FlightGoggles : [youtube](https://youtu.be/cvhI_1XQQt4)
+ Real World VINS-Mono with pointgrey cam, myAHRS+ imu on Jetson Xavier : [youtube](https://youtu.be/4qJYoND9OYk), moved faster : [youtube](https://youtu.be/DN-Jao5aKRw)
+ Real World VINS(GPU+version) with pointgrey, myAHRS at Intel i7-8700k, TITAN RTX : [youtube](https://youtu.be/UEZMZMFFhYs) 
+ Real World VINS(GPU+version, Stereo) with Intel D435i, on Xavier, max CPU clocked : [youtube](https://youtu.be/b3l1TeNKyeU) and [youtube2](https://youtu.be/7yMDqiO2A2Q) : screen

+ VINS mono VS ROVIO : [youtube](https://youtu.be/n0N2qDcNcBQ)
+ VINS-Mono vs ROVIO vs ORB-SLAM2 : [youtube](https://youtu.be/XMyiNlIbDXU)
+ VINS-Fusion (Stereo) vs S-MSCKF on FlightGoggles : [youtube](https://youtu.be/s_Ol-k8rhwY)
<br>

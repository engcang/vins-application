# VINS-application
## Mainly focused on Build process and explanation
### ■ ROS1 algorithms:
#### ● `VINS-Fusion`, `VINS-Fusion-GPU`, `VINS-Fisheye`, `OpenVINS`, `EnVIO`, `ROVIO`, `S-MSCKF`, `ORB-SLAM2`, `DM-VIO`
### ■ ROS2 algorithms:
#### ● `NVIDIA Isaac Elbrus`

<br>

## This repository contains many branches! as following: 
### ■ ROS1 algorithms:
+ **Branch**: [OAK-D](https://github.com/engcang/vins-application/tree/OAK-D), [intel T265](https://github.com/engcang/vins-application/tree/Intel-T265), [intel D435i](https://github.com/engcang/VINS-application/tree/Intel-D435i), [ZED-mini](https://github.com/engcang/VINS-application/tree/zed-mini), [Pointgrey_myAHRS](https://github.com/engcang/VINS-application/tree/Pointgrey_MyAHRS+), [FlightGoggles](https://github.com/engcang/vins-application/tree/flightgoggles)
    + Including **config.yaml** files and **Calibration data**
    + git clone -b <branch_name> --single-branch https://github.com/engcang/vins-application
### ■ ROS2 algorithms:

---
---

### Result clips: [here](#4-comparison--application)
#### ● Tested on: Jetson Xavier NX, Jetson Xavier AGX, Jetson TX2, Intel i9-10900k, i7-6700k, i7-8700k, i5-9600k
### `VINS-Fusion` for PX4 with Masking: [here](https://github.com/engcang/vins-application/tree/master/vins-fusion-px4)
+ frame changed from `world` to `map`

<br>
<br>

# Index
### 0. Algorithms:
#### ■ ROS1 Algorithms:
+ VINS-Fusion [CPU version](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion) / [GPU version](https://github.com/pjrambo/VINS-Fusion-gpu)
    + Mainly uses `Ceres-solver`, `OpenCV` and `Eigen` and **performance of VINS is strongly proportional to CPU performance and some parameters**
+ [VINS-Fisheye](https://github.com/xuhao1/VINS-Fisheye): VINS-Fusion's extension with more `camera_models` and `CUDA` acceleration
    + only for `OpenCV 3.4.1` and `Jetson TX2` (I guess, I failed on i9-10900k + RTX3080)
+ [ROVIO](https://github.com/ethz-asl/rovio): Iterative EKF based VIO, direct method (using patch)
+ [S-MSCKF](https://github.com/KumarRobotics/msckf_vio): Stereo version of MSCKF VIO
+ [ORB-SLAM2](https://github.com/appliedAI-Initiative/orb_slam_2_ros): Feature based VO, Local and Global bundle adjustment
+ [OpenVINS](https://github.com/rpng/open_vins): MSCKF based VINS 
+ [EnVIO](https://github.com/lastflowers/envio): Iterated-EKF Ensemble VIO based on [ROVIO](https://github.com/ethz-asl/rovio)
+ [DM-VIO](https://github.com/lukasvst/dm-vio-ros): Monocular VIO with delayed marginalization and pose graph bundle adjustment based on [DSO](https://github.com/JakobEngel/dso_ros)
#### ■ ROS2 Algorithms:
+ [NVIDIA Isaac Elbrus](https://docs.nvidia.com/isaac/isaac/packages/visual_slam/doc/elbrus_visual_slam.html): GPU-accelerated Stereo Visual SLAM
    + `Ubuntu 20.04`: `CUDA` 11.4, 11.5 (not 11.6), `NVIDIA-graphic driver` from 470.103.01
    + `Jetpack`: 4.6.1 on `Jetson Xavier AGX`, `Jetson Xavier NX`

### 1. Parameters
+ [VINS-Fusion](#-vins-fusion)
### 2. Prerequisites
#### ● [Ceres solver and Eigen](#-ceres-solver-and-eigen-mandatory-for-vins): Mandatory for VINS (build Eigen first)
#### ● [CUDA](#-cuda-necessary-for-gpu-version-1): Necessary for GPU version
#### ● [cuDNN](#-cudnn-strong-library-for-neural-network-used-with-cuda): Necessary for GPU version
#### ● [OpenCV with CUDA and cuDNN](#-opencv-with-cuda-and-cudnn): Necessary for GPU version

#### ● CV_Bridge with Built OpenCV: Necessary for GPU version, and general ROS usage
+ **ROS1** - [OpenCV 3.x ver  /  OpenCV 4.x ver](#-ros1-cv_bridge)
+ **ROS2** - [OpenCV 4.x ver](#-ros2-cv_bridge)
#### ● [USB performance](#-usb-performance--have-to-improve-performance-of-sensors-with-usb): Have to improve performance of sensors with USB
#### ● [IMU-Camera Calibration](#-calibration--kalibr---synchronization-time-offset-extrinsic-parameter): Synchronization, time offset, extrinsic parameter
#### ● [IMU-Camera rotational extrinsic](#-imu-camera-rotational-extrinsic-example): Rotational extrinsic between IMU and Cam
### 3. Installation and Execution
#### ■ ROS1 Algorithms:
+ [VINS-Fusion](#-vins-fusion-1)  /  [VINS-Fisheye](#-vins-fisheye)  /  [OpenVINS](#-openvins)
+ [VINS-Fusion with OpenCV4](#-vins-fusion-1)  /  [S-MSCKF](#-s-msckf)  /  [ROVIO](#-rovio)  /  [ORB-SLAM2](#-orb-slam2)
+ [EnVIO](#-envio)  /  [DM-VIO](#-dm-vio)
+ `Trouble shooting` for [VINS-Fusion](#-trouble-shooting-for-vins-fusion)
#### ■ ROS2 Algorithms:
+ [NVIDIA Isaac Elbrus](#-nvidia-isaac-elbrus-1)
### 4. [Comparison & Application results](#4-comparison--application)
### 5. [VINS on mini onboard PCs](#5-vins-on-mini-onboard-pcs-1)

<br><br><br>


# 1. Parameters
### ● VINS-Fusion: 

<details><summary>[click to see]</summary>
    
+ Camera frame rate 
    + lower - low time delay, poor performance
    + higher - high time delay, better performance
    + has to be set from **camera launch file**: 10~30hz
+ Max tracking Feature number **max_cnt**
    + 100~150, same correlation as camera frame rates
+ time offset between IMU and cameras **estimated_td: 1**, **td : value from [kalibr](#-calibration--kalibr---synchronization-time-offset-extrinsic-parameter)**
+ GPU acceleration **use_gpu: 1**, **use_gpu_acc_flow: 1** (for GPU version)
+ Threads enabling - **multiple_thread: 1**

---

</details>

<br>

# 2. Prerequisites
### ● Ceres solver and Eigen: Mandatory for VINS

<details><summary>[click to see]</summary>
    
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

---

</details>

<br>

### ● CUDA: Necessary for GPU version
<details><summary>[click to see]</summary>

<br>

+ Install **CUDA** and **Graphic Driver**: 
~~~shell
    $ sudo apt install gcc make
    get the right version of CUDA(with graphic driver) .deb file at https://developer.nvidia.com/cuda-downloads
    follow the installation instructions there!
        # .run file can be used as nvidia graphic driver. But, .deb file is recommended to install tensorRT further.

        # if want to install only graphic driver, get graphic driver install script at https://www.nvidia.com/Download/index.aspx?lang=en-us
        # sudo ./NVIDIA_<graphic_driver_installer>.run --dkms
        # --dkms option is recommended when you also install NVIDIA driver, to register it along with kernel
        # otherwise, NVIDIA graphic driver will be gone after kernel upgrade via $ sudo apt upgrade
    $ sudo reboot

    $ gedit ~/.bashrc
    # type and save
    export PATH=<CUDA_PATH>/bin:$PATH #ex: /usr/local/cuda-11.1
    export LD_LIBRARY_PATH=<CUDA_PATH>/lib64:$LD_LIBRARY_PATH #ex : /usr/local/cuda-11.1
    $ . ~/.bashrc

    # check if installed well
    $ dpkg-query -W | grep cuda
~~~
+ check CUDA version using **nvcc --version**
~~~shell
# check installed cuda version
$ nvcc --version
# if nvcc --version does not print out CUDA,
$ gedit ~/.profile
# type below and save
export PATH=<CUDA_PATH>/bin:$PATH #ex: /usr/local/cuda-11.1
export LD_LIBRARY_PATH=<CUDA_PATH>/lib64:$LD_LIBRARY_PATH #ex : /usr/local/cuda-11.1
$ source ~/.profile
~~~

<br>

### ● Trouble shooting for NVIDIA driver or CUDA: please see /var/log/cuda-installer.log or /var/log/nvidia-install.log
+ Installation failed. See log at /var/log/cuda-installer.log for details => mostly because of `X server` is being used.
    + turn off `X server` and install.
~~~shell
# if you are using lightdm
$ sudo service lightdm stop

# or if you are using gdm3
$ sudo service gdm3

# then press Ctrl+Alt+F3 -> login with your ID/password
$ sudo sh cuda_<version>_linux.run
~~~
+ The kernel module failed to load. Secure boot is enabled on this system, so this is likely because it was not signed by a key that is trusted by the kernel.... 
    + turn off `Secure Boot` as below [reference](https://wiki.ubuntu.com/UEFI/SecureBoot/DKMS)
    + If you got this case, you should turn off `Secure Boot` and then turn off `X server` (as above) both.

---

</details>


### ● cuDNN: strong library for Neural Network used with CUDA
<details><summary>[click to see]</summary>
    
+ Download [here](https://developer.nvidia.com/cudnn)
+ install as below: [reference in Korean](https://cafepurple.tistory.com/39)
~~~shell
$ sudo tar zxf cudnn.tgz
$ sudo cp extracted_cuda/include/* <CUDA_PATH>/include/   #ex /usr/local/cuda-11.1/include/
$ sudo cp -P extracted_cuda/lib64/* <CUDA_PATH>/lib64/   #ex /usr/local/cuda-11.1/lib64/
$ sudo chmod a+r <CUDA_PATH>/lib64/libcudnn*   #ex /usr/local/cuda-11.1/lib64/libcudnn*
~~~

---

</details>

<br>

---

### ● OpenCV with CUDA and cuDNN 
#### ■ Ubuntu 18.04 - this repo mainly targets ROS1 for Ubuntu 18.04
<details><summary>[Click to see]</summary>
    
+ Build OpenCV with CUDA - references: [link 1](https://webnautes.tistory.com/1030), [link 2](https://github.com/jetsonhacks/buildOpenCVXavier/blob/master/buildOpenCV.sh)
    + for Xavier do as below or sh file from jetsonhacks [here](https://github.com/jetsonhacks/buildOpenCVXavier)
    + If want to use **C API (e.g. Darknet YOLO)** with `OpenCV3`, then: 
        + **Patch as [here](https://github.com/opencv/opencv/issues/10963)** to use other version **(3.4.1 is the best)**
            + should **comment** the /usr/local/include/opencv2/highgui/highgui_c.h line 139 [as here](https://stackoverflow.com/questions/48611228/yolo-compilation-with-opencv-1-fails) after install
+ **-D OPENCV_GENERATE_PKGCONFIG=YES** option is also needed for `OpenCV 4.X`
  + and copy the generated `opencv4.pc` file to `/usr/local/lib/pkgconfig` or `/usr/lib/aarch64-linux-gnu/pkgconfig` for jetson boards
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


# check version
$ wget -O opencv.zip https://github.com/opencv/opencv/archive/3.4.1.zip # check version
$ unzip opencv.zip
$ cd <opencv_source_directory>/opencv 

$ wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/3.4.1.zip # check version
$ unzip opencv_contrib.zip

$ mkdir build && cd build
    
# check your BIN version : http://arnon.dk/matching-sm-architectures-arch-and-gencode-for-various-nvidia-cards/
# 8.6 for RTX3080 7.2 for Xavier, 5.2 for GTX TITAN X, 6.1 for GTX TITAN X(pascal), 6.2 for TX2
# -D BUILD_opencv_cudacodec=OFF #for cuda10-opencv3.4
    
$ cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D CMAKE_C_COMPILER=gcc-6 \
      -D CMAKE_CXX_COMPILER=g++-6 \
      -D CMAKE_INSTALL_PREFIX=/usr/local \
      -D OPENCV_GENERATE_PKGCONFIG=YES \
      -D WITH_CUDA=ON \
      -D OPENCV_DNN_CUDA=ON \
      -D WITH_CUDNN=ON \
      -D CUDA_ARCH_BIN=8.6 \
      -D CUDA_ARCH_PTX=8.6 \
      -D ENABLE_FAST_MATH=ON \
      -D CUDA_FAST_MATH=ON \
      -D WITH_CUBLAS=ON \
      -D WITH_LIBV4L=ON \
      -D WITH_GSTREAMER=ON \
      -D WITH_GSTREAMER_0_10=OFF \
      -D WITH_CUFFT=ON \
      -D WITH_NVCUVID=ON \
      -D WITH_QT=ON \
      -D WITH_OPENGL=ON \
      -D WITH_IPP=OFF \
      -D WITH_V4L=ON \
      -D WITH_1394=OFF \
      -D WITH_GTK=ON \
      -D WITH_EIGEN=ON \
      -D WITH_FFMPEG=ON \
      -D WITH_TBB=ON \
      -D BUILD_opencv_cudacodec=OFF \
      -D CUDA_NVCC_FLAGS="--expt-relaxed-constexpr" \
      -D OPENCV_EXTRA_MODULES_PATH=../opencv_contrib-3.4.1/modules \
      ../
$ time make -j8 # 8 : numbers of core
$ sudo make install
$ sudo rm -r <opencv_source_directory> #optional for saving disk, but leave this folder to uninstall later, if you need.
~~~

<br>

### ● Trouble shooting for OpenCV build error:
+ Please include the appropriate gl headers before including cuda_gl_interop.h => reference [1](https://github.com/jetsonhacks/buildOpenCVXavier/blob/master/buildOpenCV.sh#L101), [2](https://github.com/jetsonhacks/buildOpenCVXavier/blob/master/patches/OpenGLHeader.patch), [3](https://devtalk.nvidia.com/default/topic/1007290/jetson-tx2/building-opencv-with-opengl-support-/post/5141945/#5141945)
+ modules/cudacodec/src/precomp.hpp:60:37: fatal error: dynlink_nvcuvid.h: No such file or directory
compilation terminated. --> **for CUDA version 10**
    + => reference [here](https://devtalk.nvidia.com/default/topic/1044773/cuda-setup-and-installation/error-in-installing-opencv-3-4-0-on-cuda-10/)
    + cmake ... -D BUILD_opencv_cudacodec=OFF ...
+ CUDA_nppicom_LIBRARY not found
    + $ sudo apt-get install nvidia-cuda-toolkit
    + or Edit *opencv/cmake/OpenCVDetectCUDA.cmake* as follows:
    ```cmake
        ...
        ...
        if(CUDA_FOUND)
            set(HAVE_CUDA 1)
            ocv_list_filterout(CUDA_nppi_LIBRARY "nppicom") #this line is added
            ocv_list_filterout(CUDA_npp_LIBRARY "nppicom") #this line is added
            if(WITH_CUFFT)
                set(HAVE_CUFFT 1)
            endif()
        ...
        ...
    ```
    
---

</details>

    
#### ■ Ubuntu 20.04 - this repo mainly targets ROS2 for Ubuntu 20.04

<details><summary>[Click to see]</summary>
    
+ Build OpenCV with CUDA - references: [link 1](https://webnautes.tistory.com/1479?category=704653)
+ **-D PYTHON3_PACKAGES_PATH=/usr/local/lib/python3.8/dist-packages** 
    + This is needed to prevent `No module name cv2` when `import cv2` in `Python3`
    
~~~bash
## optional, I just leave default OpenCV from ROS2, since I can set proper PATHS for desired OpenCV versions
## If you cannot, just do below:
$ sudo apt-get purge libopencv*
## (But you will have to sudo apt install ros-foxy-desktop again, when you need other packages related to this)

$ sudo apt-get purge python-opencv python3-opencv
$ pip uninstall opencv-python
$ sudo apt-get update
$ sudo apt-get install -y build-essential pkg-config
$ sudo apt-get install -y cmake libavcodec-dev libavformat-dev libavutil-dev \
    libglew-dev libgtk2.0-dev libgtk-3-dev libjpeg-dev libpng-dev libpostproc-dev \
    libswscale-dev libtbb-dev libtiff5-dev libv4l-dev libxvidcore-dev \
    libx264-dev qt5-default zlib1g-dev libgl1 libglvnd-dev pkg-config \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev mesa-utils #libeigen3-dev # recommend to build from source : http://eigen.tuxfamily.org/index.php?title=Main_Page
$ sudo apt-get install python3-dev python3-numpy
$ mkdir <opencv_source_directory> && cd <opencv_source_directory>


# check version
$ wget -O opencv.zip https://github.com/opencv/opencv/archive/4.5.5.zip # check version
$ unzip opencv.zip
$ cd <opencv_source_directory>/opencv 

$ wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.5.5.zip # check version
$ unzip opencv_contrib.zip

$ mkdir build && cd build
    
# check your CUDA_ARCH_BIN and CUDA_ARCH_PTX version : http://arnon.dk/matching-sm-architectures-arch-and-gencode-for-various-nvidia-cards/
# 8.6 for RTX3080 7.2 for Xavier, 5.2 for GTX TITAN X, 6.1 for GTX TITAN X(pascal), 6.2 for TX2
# -D BUILD_opencv_cudacodec=OFF #for cuda10-opencv3.4
    
$ cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D CMAKE_C_COMPILER=gcc-9 \
      -D CMAKE_CXX_COMPILER=g++-9 \
      -D CMAKE_INSTALL_PREFIX=/usr/local \
      -D OPENCV_GENERATE_PKGCONFIG=YES \
      -D PYTHON_EXECUTABLE=/usr/bin/python3.8 \
      -D PYTHON2_EXECUTABLE="" \
      -D BUILD_opencv_python3=ON \
      -D BUILD_opencv_python2=OFF \
      -D PYTHON3_PACKAGES_PATH=/usr/local/lib/python3.8/dist-packages \
      -D BUILD_NEW_PYTHON_SUPPORT=ON \
      -D OPENCV_SKIP_PYTHON_LOADER=ON \
      -D WITH_CUDA=ON \
      -D OPENCV_DNN_CUDA=ON \
      -D WITH_CUDNN=ON \
      -D CUDA_ARCH_BIN=8.6 \
      -D CUDA_ARCH_PTX=8.6 \
      -D ENABLE_FAST_MATH=ON \
      -D CUDA_FAST_MATH=ON \
      -D WITH_CUBLAS=ON \
      -D WITH_LIBV4L=ON \
      -D WITH_GSTREAMER=ON \
      -D WITH_GSTREAMER_0_10=OFF \
      -D WITH_CUFFT=ON \
      -D WITH_NVCUVID=ON \
      -D WITH_QT=ON \
      -D WITH_OPENGL=ON \
      -D WITH_IPP=OFF \
      -D WITH_V4L=ON \
      -D WITH_1394=OFF \
      -D WITH_GTK=ON \
      -D WITH_EIGEN=ON \
      -D WITH_FFMPEG=ON \
      -D WITH_TBB=ON \
      -D BUILD_opencv_cudacodec=OFF \
      -D CUDA_NVCC_FLAGS="--expt-relaxed-constexpr" \
      -D OPENCV_EXTRA_MODULES_PATH=../opencv_contrib-4.5.5/modules \
      ../
$ time make -j20 # 20 : numbers of core
$ sudo make install
$ sudo rm -r <opencv_source_directory> #optional for saving disk, but leave this folder to uninstall later, if you need.
~~~

<br>

### ● Trouble shooting for OpenCV build error:
+ No troubles found yet

---

</details>
    
---
    
### ● CV_Bridge with built OpenCV: Necessary for whom built OpenCV manually from above
### ■ ROS1-cv_bridge

<details><summary>[Click: CV_bridge with OpenCV 3.X version]</summary>
    
+ If OpenCV with CUDA were built manually, build cv_bridge manually also
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

---

</details>


<details><summary>[Click: CV_bridge with OpenCV 4.X version]</summary>
    
+ Referred [here](https://github.com/ros-perception/vision_opencv/issues/272#issuecomment-471311300)
~~~shell
$ cd ~/catkin_ws/src && git clone https://github.com/ros-perception/vision_opencv
# since ROS Noetic is added, we have to checkout to melodic tree
$ cd vision_opencv && git checkout origin/melodic
$ gedit vision_opencv/cv_bridge/CMakeLists.txt
~~~
+ Add options and edit OpenCV PATHS in CMakeLists
~~~txt
# add right after project()
set(CMAKE_CXX_STANDARD 11) 

# edit find_package(OpenCV)
#find_package(OpenCV 4 REQUIRED PATHS /usr/local/share/opencv4 NO_DEFAULT_PATH
find_package(OpenCV 4 REQUIRED
  COMPONENTS
    opencv_core
    opencv_imgproc
    opencv_imgcodecs
  CONFIG
)
include(/usr/local/lib/cmake/opencv4/OpenCVConfig.cmake)
~~~
+ Edit `cv_bridge/src/CMakeLists.txt`
~~~txt
# line number 35, Edit 3 -> 4
if (OpenCV_VERSION_MAJOR VERSION_EQUAL 4)
~~~
+ Edit `cv_bridge/src/module_opencv3.cpp`
~~~cpp
// line number 110
//    UMatData* allocate(int dims0, const int* sizes, int type, void* data, size_t* step, int flags, UMatUsageFlags usageFlags) const
    UMatData* allocate(int dims0, const int* sizes, int type, void* data, size_t* step, AccessFlag flags, UMatUsageFlags usageFlags) const

// line number 140
//    bool allocate(UMatData* u, int accessFlags, UMatUsageFlags usageFlags) const
    bool allocate(UMatData* u, AccessFlag accessFlags, UMatUsageFlags usageFlags) const
~~~
~~~shell
$ cd .. && catkin build cv_bridge
~~~

---

</details>

### ■ ROS2-cv_bridge

<details><summary>[Click: CV_bridge with OpenCV 4.X version]</summary>

+ If OpenCV with CUDA were built manually, build cv_bridge manually also
~~~bash
$ cd ~/colcon_ws/src && git clone https://github.com/ros-perception/vision_opencv
$ cd vision_opencv
$ git checkout origin/ros2

$ cd ~/colcon_ws
$ colcon build --symlink-install --packages-select cv_bridge image_geometry --allow-overriding cv_bridge image_geometry
$ source install/setup.bash
~~~

</details>
    
<br>

---

### ● USB performance : Have to improve performance of sensors with USB
<details><summary>[click to see]</summary>
    
  + Link : [here](https://github.com/KumarRobotics/flea3#optimizing-usb-performance-under-linux) for x86_64 desktops
  + TX1/TX2 : [here](https://www.matrix-vision.com/manuals/mvBlueFOX3/mvBC_page_quickstart.html#mvBC_subsubsection_quickstart_linux_requirements_optimising_usb)
  + For Xavier : [here](https://devtalk.nvidia.com/default/topic/1049581/jetson-agx-xavier/change-usbcore-usbfs_memory_mb/)
  ~~~shell
  $ sudo ./flash.sh -k kernel -C "usbcore.usbfs_memory_mb=1000" -k kernel-dtb jetson-xavier mmcblk0p1
  ~~~

---

</details>

<br>

---
    
### ● Calibration : Kalibr -> synchronization, time offset, extrinsic parameter
<details><summary>[click to see]</summary>
    
+ [Kalibr](https://github.com/ethz-asl/kalibr) -> synchronization, time offset <br>
+ For ZED cameras : [here](https://support.stereolabs.com/hc/en-us/articles/360012749113-How-can-I-use-Kalibr-with-the-ZED-Mini-camera-in-ROS-)
+ **When Calibrating Fisheye camera like T265**
    + Try with ***MEI*** camera model, as [here](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/issues/57), which is *omni-radtan* in Kalibr
    + and try this **Pull** to deal with NaNs [here](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/pull/110)

<br>

### ● Trouble shooting for Kalibr errors
+ **ImportError: No module named Image** [reference](https://github.com/ethz-asl/kalibr/issues/67)
~~~shell
$ gedit kalibr/aslam_offline_calibration/kalibr/python/kalibr_camera_calibration/MulticamGraph.py
#import Image
from PIL import Image
~~~
+ **focal length initialization error**
~~~shell
 $ gedit kalibr/aslam_cv/aslam_cameras/include/aslam/cameras/implementation/PinholeProjection.hpp
 # edit if sentence in line 781
 # comment from line 782 to 795
 f_guesses.push_back(2000.0) #initial guess of focal length!!!!
~~~
+ **cameras are not connected**
~~~shell
 $ gedit kalibr/aslam_offline_calibration/kalibr/python/kalibr_calibrate_cameras
 # comment from line 201 to 205
~~~

---

</details>

### ● IMU-Camera rotational extrinsic example
<details><summary>[click to see]</summary>
    
+ Between ROS standard body(IMU) and camera
  <p align="center">
  <img src="https://github.com/engcang/vins-application/blob/master/extrinsic.png" width="600"/>
  </p>
+ **Left view :** Between ROS standard body(IMU) and down-pitched (look downward) camera
  <p align="center">
  <img src="https://github.com/engcang/vins-application/blob/master/pitching.png" width="600"/>
  </p>

---

</details>

<br><br>

# 3. Installation and Execution
## ■ ROS1 Algorithms:
### ● VINS-Fusion

<details><summary>[with `OpenCV3`(original): click to see]</summary>
    
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

+ For `GPU` version, Edit `CMakeLists.txt` for `loop_fusion` and `vins_estimator`
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

---

</details>

<details><summary>[with `OpenCV4`: click to see]</summary>

+ git clone and build, few `cv` codes are changed from original repo.
~~~shell
$ cd ~/catkin_ws/src
$ git clone https://github.com/engcang/vins-application #Only CPU version yet
$ cd vins-application

$ mv vins_estimator ..
$ mv camera_models ..
$ cd ..
$ rm -r vins-application

$ cd .. 
$ catkin build
~~~

---

</details>

#### ● Trouble shooting for VINS-Fusion
<details><summary>[click to see]</summary>
    
+ Aborted error when running **vins_node** : 
~~~shell
 $ echo "export MALLOC_CHECK_=0" >> ~/.bashrc
 $ source ~/.bashrc
~~~
+ **If want to try to deal with NaNs**, refer [here](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/pull/110)

---

</details>

<br>

### ● VINS-Fisheye
#### only for `OpenCV 3.4.1` and `Jetson TX2` (I guess yet, I failed on i9-10900k + RTX3080)
<details><summary>[click to see]</summary>

+ Get `libSGM` and install with `OpenCV` option as below:
~~~shell
$ git clone https://github.com/fixstars/libSGM
$ cd libSGM
$ git submodule update --init

check and edit CMakeLists.txt
$ gedit CMakeLists.txt
Edit
BUILD_OPENCV_WRAPPER=ON and ENABLE_TESTS=ON

$ mkdir build && cd build
$ cmake .. -DBUILD_OPENCV_WRAPPER=ON -DENABLE_TESTS=ON
$ make -j6
$ sudo make install

do test
$ cd libSGM/build/test && ./sgm-test
~~~

+ Get `VINS-Fisheye` and install
~~~shell
$ cd ~/catkin_ws/src
$ git clone https://github.com/xuhao1/VINS-Fisheye
$ cd ..

build camera_models first
$ catkin build camera_models

$ gedit src/VINS-Fisheye/vins_estimator/CMakeLists.txt
edit as below:
set(ENABLE_BACKWARD false)
or
$ sudo apt install libdw-dev

$ catkin build
~~~

</details>

<br>


### ● OpenVINS

<details><summary>[click to see]</summary>

+ Get `OpenVINS` and install: refer [doc](https://docs.openvins.com/gs-installing.html), [git](https://github.com/rpng/open_vins)
~~~shell
$ cd ~/catkin_ws/src
$ git clone https://github.com/rpng/open_vins/
$ cd ..
$ catkin build
~~~

</details>

<br>

### ● EnVIO
<details><summary>[click to see]</summary>

+ Get `EnVIO` and install: refer [official git](https://github.com/lastflowers/envio/)
~~~bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/lastflowers/envio.git
$ cd ..

$ catkin_make

or, if you want to use it with catkin build,
then

$ gedit src/envio/CMakeLists.txt
comment two lines, line 4 and 5
#set(CMAKE_CXX_COMPILER "/usr/bin/g++-5")
#set(CMAKE_C_COMPILER "/usr/bin/gcc-5")

$ catkin build    
~~~

</details>

<br>

### ● S-MSCKF
    
<details><summary>[click to see]</summary>

#### ● Installation
```bash
 $ sudo apt-get install libsuitesparse-dev
 $ cd ~/catkin_ws/src && https://github.com/KumarRobotics/msckf_vio
 $ cd ~/catkin_ws && catkin build msckf_vio -DCMAKE_BUILD_TYPE=Release
```

</details>

<br>

### ● ROVIO
    
<details><summary>[click to see]</summary>

#### ● Requirements
    
+ `ROVIO` receives input image as `gray scale image` - convert the RGB image as [this file](https://github.com/engcang/utility_codes/blob/master/rgb2gray.py)
+ Config files can be generated directly from `Kalibr` results:
    
```bash
$ rosrun kalibr kalibr_rovio_config --cam <cam-chain.yaml filename>
```
    
+ After using kalibr to convert the calibration result files to rovio_config files,
    + Make sure to Edit `Camera1` and `Camera2` into `Camera0` and `Camera1` in `.info` file
    + Make sure to Add `Velocity Updates` block in `.info` file

#### ● Installation
    
+ Install [kindr](https://github.com/ANYbotics/kindr)

```bash
$ cd ~/catkin_ws/src && git clone https://github.com/ANYbotics/kindr
$ cd .. && catkin build -j8
```

+ Install `ROVIO`
    
```bash
$ cd ~/catkin_ws/src && git clone https://github.com/ethz-asl/rovio
$ cd rovio && git submodule update --init --recursive

$ cd ..
$ catkin build rovio --cmake-args -DCMAKE_BUILD_TYPE=Release

# With with opengl scene (optional)
$ sudo apt-get install freeglut3-dev libglew-dev
$ catkin build rovio --cmake-args -DCMAKE_BUILD_TYPE=Release -DMAKE_SCENE=ON
```

</details>

<br>

### ● ORB-SLAM2

<details><summary>[click to see]</summary>

#### ● Installation
```bash
 $ cd ~/catkin_ws/src && git clone https://github.com/appliedAI-Initiative/orb_slam_2_ros
 $ cd .. && rosdep install --from-paths src --ignore-src -r -y
 $ catkin build
```

+ highly recommend this pull request to speedup loading Vocabulary [here](https://github.com/raulmur/ORB_SLAM2/pull/21)

</details>

<br>

### ● DM-VIO

<details><summary>[click to see]</summary>

+ Install dependencies
```bash
$ sudo apt-get install cmake libsuitesparse-dev libeigen3-dev libboost-all-dev libyaml-cpp-dev libtbb-dev libgl1-mesa-dev libglew-dev pkg-config libegl1-mesa-dev libwayland-dev libxkbcommon-dev wayland-protocols -y

$ cd ~/your_workspace
$ git clone https://github.com/borglab/gtsam.git
$ cd gtsam
$ git checkout 4.2a6          # not strictly necessary but this is the version tested with.
$ mkdir build && cd build
$ cmake -DGTSAM_POSE3_EXPMAP=ON -DGTSAM_ROT3_EXPMAP=ON -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF ..
$ make -j
$ sudo make install

$ cd ~/your_workspace
$ git clone https://github.com/stevenlovegrove/Pangolin.git
$ cd Pangolin
$ git checkout v0.6
$ mkdir build && cd build
$ cmake ..
$ cmake --build .
$ sudo make install
```

+ Build `DM-VIO` and `DM-VIO-ROS`
```bash
$ cd ~/your_workspace
$ git clone https://github.com/lukasvst/dm-vio.git
$ cd dm-vio
$ mkdir build && cd build
$ cmake ..
$ make -j10
$ echo "export DMVIO_BUILD=`pwd`" >> ~/.bashrc && . ~/.bashrc

$ cd ~/your_workspace/src
$ git clone https://github.com/lukasvst/dm-vio-ros.git
$ cd ~/your_workspace
$ catkin build
$ . devel/setup.bash
$ sudo ldconfig
```

+ Run on `KAIST-VIO-Dataset`, refer [this config files](https://github.com/engcang/vins-application/tree/master/DM-VIO)
```bash
$ rosrun dmvio_ros node calib=camera_kaistvio.txt imuCalib=camchain_kaistvio.yaml settingsFile=setting_kaistvio.yaml mode=3 nogui=0 preset=1 quiet=1 useimu=1
```

</details>

<br>


## ■ ROS2 Algorithms:
### ● NVIDIA Isaac Elbrus

<details><summary>[click to see]</summary>

#### ● Requirements
+ PC option1 - Ubuntu 20.04
    + `CUDA`: 11.4-11.5 **(11.6 cannot install VPI 1.1.11)**
    + `NVIDIA Graphic driver` >= 470.103.01
    + **(Important)** `NVIDIA VPI` 1.1.11 (Only this version) - install with [this files](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/tree/main/docker/vpi) after CUDA installation
+ PC option2 - Jetpack 4.6.1 on Jetson Xavier AGX / NX
+ Topics: **`Raw stereo image` + `camera info topics`** + **(Important!) `/tf_static`** (including base_frame (e.g., camera_link) to left and right camera frame)

#### ● Installation
~~~bash
$ sudo apt install git-lfs
$ cd ~/colcon_ws/src
$ git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam &&
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline &&
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common
$ cd ..
$ rosdep install -i -r --from-paths src --rosdistro foxy -y
$ colcon build --symlink-install && source install/setup.bash
~~~

#### ● Run
+ Edit remapping topic names in the launch file below, before launching it.
~~~python
# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.

    ...

import launch
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

    ...
    
        remappings=[('stereo_camera/left/image', '/camera/infra1/image_rect_raw'),
                    ('stereo_camera/left/camera_info', '/camera/infra1/camera_info'),
                    ('stereo_camera/right/image', '/camera/infra2/image_rect_raw'),
                    ('stereo_camera/right/camera_info', '/camera/infra2/camera_info')]
    )

    ...
    
~~~

+ If you want to run it with `bag` file, then use or refer [this launch file](https://github.com/engcang/vins-application/blob/master/NVIDIA_Isaac_Elbrus/elbrus.launch.py)
    + since **`/tf_static`** cannot be recorded in `bag` file, `static_transform_publisher` should be added in the launch file as [these lines](https://github.com/engcang/vins-application/blob/master/NVIDIA_Isaac_Elbrus/elbrus.launch.py#L57-L62)

</details>

<br>

# 4. Comparison & Application
+ Conversion ROS topics into nav_msgs/Path to visualize in Rviz: use this [github](https://github.com/engcang/tf_to_trajectory)
+ Conversion compressed Images into raw Images: use this [code](https://github.com/engcang/utility_codes/blob/master/compressed_to_raw.py)

<br>

#### Simulation

+ `VINS-Mono` on `FlightGoggles`: [youtube](https://youtu.be/U4TJ7ZyfWD8), with CPU [youtube](https://www.youtube.com/watch?v=1QUypn7GbXc)
+ `ROVIO` on `FlightGoggles`: [youtube](https://youtu.be/3Xgwi7k6css)
+ `ORB-SLAM2` on `FlightGoggles`: [youtube](https://youtu.be/4iQf8rBA9mw)
+ `VINS with Loop fusion` vs `VINS` on `FlightGoggles`: [youtube](https://youtu.be/cvhI_1XQQt4)
+ `VINS-Mono` vs `ROVIO`: [youtube](https://youtu.be/n0N2qDcNcBQ)
+ `VINS-Mono` vs `ROVIO` vs `ORB-SLAM2`: [youtube](https://youtu.be/XMyiNlIbDXU)
+ `VINS-Fusion` (Stereo) vs `S-MSCKF` on `FlightGoggles`: [youtube](https://youtu.be/s_Ol-k8rhwY)
+ `VINS-Fusion` (Stereo) based autonomous flight and 3D mapping using RGB-D camera: [youtube](https://youtu.be/5t-6g7UWA7o)
    
#### Real world

+ Hand-held - `VINS-Mono` with pointgrey cam, myAHRS+ imu on Jetson Xavier AGX: [youtube](https://youtu.be/4qJYoND9OYk), moved faster : [youtube](https://youtu.be/DN-Jao5aKRw)
+ Hand-held - `ROVIO` with Intel D435i on Jetson Xavier AGX: [youtube](https://youtu.be/_o2KwT8jJN0)
+ Hand-held - `ORB-SLAM2` with Intel D435i on Jetson Xavier AGX: [youtube](https://youtu.be/-jueoC-YqF4)
+ Hand-held - `VINS(GPU version)` with pointgrey, myAHRS at Intel i7-8700k, TITAN RTX: [youtube](https://youtu.be/UEZMZMFFhYs) 
+ Hand-held - `VINS(GPU version, Stereo)` with Intel D435i, on Xavier AGX, max CPU clocked: [youtube](https://youtu.be/b3l1TeNKyeU) and [youtube2](https://youtu.be/7yMDqiO2A2Q) : screen
+ Hand-held - `VINS-Fusion (Stereo)` with Intel D435i and Pixhawk4 mini fused with T265 camera: [here](https://engcang.github.io/mavros_vision_pose/)
+ Hand-held - `VINS-Fusion (stereo)` with Intel D435i and Pixhawk4 mini on 1km long underground tunnel: [here](https://youtu.be/Gx0PSMCeR1g)
+ Hand-held - `VINS-Fusion GPU version` test using T265: [here](https://youtu.be/8w86LeB6fns)
+ Hand-held - `VINS-Fusion (stereo)` test using OAK-D: [here](https://youtu.be/Hjcjg9L4j9o)
+ Hand-held - `VINS-Fusion (stereo)` test using OAK-D PRO: [here](https://youtu.be/Xw-HIPbn0wg)
+ Real-Drone - `VINS-Fusion` with Intel D435i and Pixahwk4 mini on Real Hexarotor: [here](https://youtu.be/sfj1kxMVeMU)
+ Real-Drone - `VINS-Fusion` with Intel D435i and Pixahwk4 mini on Real Quadrotor: [here](https://youtu.be/S3XAOMek2mo)

+ `OpenVINS` on [KAIST VIO dataset](https://github.com/zinuok/kaistviodataset): result [youtube](https://youtu.be/Ye8xcKH4otY)
    + use this [launch file](https://github.com/engcang/vins-application/blob/master/openvins/kaist.launch) including parameters

+ `EnVIO` vs `VINS-Fusion` on [KAIST VIO dataset](https://github.com/zinuok/kaistviodataset): result [youtube](https://youtu.be/G2ZZoegoX9M)
+ `DM-VIO` vs `VINS-Mono` on [KAIST VIO dataset](https://github.com/zinuok/kaistviodataset): result [youtube](https://youtu.be/OJddYxLlSqo)

+ `NVIDIA Isaac Elbrus` in real-world: result [youtube](https://youtu.be/V24_xtkscjc)
    
<br>

# 5. VINS on mini onboard PCs
+ `Qualcomm RB5` vs `Khadas VIM3 Pro`  - [Video](https://youtu.be/hIu1buGkHBA)

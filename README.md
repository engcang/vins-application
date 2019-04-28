# VINS-application
+ Build Process and explanation [here](https://github.com/engcang/VINS-application)
+ Installation of Intel Realsense D435i for Jetson TX2 and Xavier
<br>

## Requirements
+ ~~For Jetpack 4.2, Ubuntu 18.04 both for Xavier, TX2 : try [this script](https://github.com/engcang/VINS-application/blob/Intel-D435i/jetpack4.2-d435i_tx2_xavier.sh)~~
  + ~~Edited the scripts file from jetsonhacks github : [here](https://github.com/jetsonhacks/buildLibrealsense2Xavier)~~
  + ~~copy and paste command lines while reading the comments, do not directly execute it~~
  + ~~and flash kernel from host pc to jetson board under recovery mode~~
  ~~~shell
  $ sudo ./flash.sh -k kernel jetson-<tx2 or xavier> mmcblk0p1
  ~~~

### ● Stable versions below
+ (necessary for ROS) Intel Realsense [SDK](https://github.com/IntelRealSense/librealsense)
  + For Xavier [here](https://github.com/jetsonhacks/buildLibrealsense2Xavier) -> SDK version is **v2.17.1** and need **JetPack 4.1 for L4T 31.1** or have to be flashed again...
    + video [link](https://youtu.be/Pp18JL6H2zc) or jetsonhacks [article](https://www.jetsonhacks.com/2019/01/21/intel-realsense-d435i-on-nvidia-jetson-agx-xavier/)
  + For TX2 [here](https://github.com/jetsonhacks/buildLibrealsense2TX) -> SDK version is **v2.13.0** and need  **JetPack3.2.1 for L4T 28.2 / L4T 28.2.1** or have to be flashed again...
    + video [link](https://youtu.be/mvDCOc-aoMU) or jetsonhacks [article](https://www.jetsonhacks.com/2018/07/10/librealsense-update-nvidia-jetson-tx-dev-kits/)
+ Intel Realsense2 ROS [here](https://github.com/intel-ros/realsense)
  + For Xavier : -> ROS Wrapper 2.0 version should be **v2.1.3** for SDK v2.17.1 [down link](https://github.com/intel-ros/realsense/archive/2.1.3.zip)
~~~shell
$ cd ~/catkin_ws/src && wget https://github.com/intel-ros/realsense/archive/<$version>.zip
$ unzip realsense-<$version>.zip
$ cd ..
$ catkin build realsense2_camera -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release -j8
$ source ./devel/setup.bash
~~~

### ● Disable **Emitter**
+ Disable **Emitter** using **/usr/local/bin/realsense-viewer**, save the **json** file and
  <p align="center">
  <img src="https://github.com/engcang/image-files/blob/master/vins/1.png" width="600" hspace="50"/>
  </p>

+ import the **json** file at the **rs_camera.launch** file as figure.
  <p align="center">
  <img src="https://github.com/engcang/image-files/blob/master/vins/2.png" width="500" hspace="30"/>
  </p>
  
### ● IMU Calibration recommended [here](https://www.intel.com/content/dam/support/us/en/documents/emerging-technologies/intel-realsense-technology/RealSense_Depth_D435i_IMU_Calib.pdf)


<br>

### ● First trial
+ used camera intrinsic from **/camera/infra1/rect_image_raw/camera_info**
+ set launch file **rs_camera.launch** as [reference](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/master/config/realsense_d435i/rs_camera.launch)
+ worked quite well

### ● After that,
+ used Kalibr and launch file as in this repository.

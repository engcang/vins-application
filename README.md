# VINS-application
+ Build Process and explanation [here](https://github.com/engcang/VINS-application)
+ Installation of Intel Realsense D435i for Jetson TX2 and Xavier
<br>

## Requirements - [SDK](https://github.com/IntelRealSense/librealsense)
### ● Necessary for ROS too.
#### ● For Xavier
+ **Old version**: [here](https://github.com/jetsonhacks/buildLibrealsense2Xavier) -> SDK version is **v2.17.1** and need **JetPack 4.1 for L4T 31.1** or have to be flashed again...
  + video [link](https://youtu.be/Pp18JL6H2zc) or jetsonhacks [article](https://www.jetsonhacks.com/2019/01/21/intel-realsense-d435i-on-nvidia-jetson-agx-xavier/)
#### ● For TX2
+ **More recent version**: [here](https://github.com/jetsonhacks/installRealSenseSDK)
  + **Kernel build is not needed anymore!!!**
+ **Old version**: [here](https://github.com/jetsonhacks/buildLibrealsense2TX) -> SDK version is **v2.13.0** and need  **JetPack3.2.1 for L4T 28.2 / L4T 28.2.1** or have to be flashed again...
  + video [link](https://youtu.be/mvDCOc-aoMU) or jetsonhacks [article](https://www.jetsonhacks.com/2018/07/10/librealsense-update-nvidia-jetson-tx-dev-kits/)


#### ● Xavier NX, x86_64 -> do as follows:
+ For Xavier NX, refer [here](https://github.com/zinuok/Xavier_NX) and [here](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md)
+ For Xavier NX and x86_64 desktop/laptop, refer [here](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md)
~~~shell
  $ sudo apt-get install git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev
  $ sudo apt-get install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev
  $ git clone https://github.com/IntelRealSense/librealsense.git
  $ cd librealsense && mkdir build && cd build
  $ cmake .. -DCMAKE_BUILD_TYPE=Release
  $ sudo make uninstall && make clean
  $ time make -j8 && sudo make install
~~~
  + Trouble shooting : 'Failed to set power state error' or 'UDEV-Rules are missing'
  ~~~
    $ sudo cp [librealsense path]/config/99-realsense-libusb.rules /etc/udev/rules.d/99-realsense-libusb.rules && sudo udevadm control --reload-rules && udevadm trigger
    $ reboot
  ~~~

<br><br>

### ● ROS package
+ Intel Realsense2 ROS [here](https://github.com/intel-ros/realsense)
  + For Xavier : -> ROS Wrapper 2.0 version should be **v2.1.3** for SDK v2.17.1 [down link](https://github.com/intel-ros/realsense/archive/2.1.3.zip)
~~~shell
$ cd ~/catkin_ws/src && wget https://github.com/intel-ros/realsense/archive/<$version>.zip
$ unzip realsense-<$version>.zip
$ cd ..
$ catkin build realsense2_camera -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release -j8
$ source ./devel/setup.bash
~~~
+ Trouble shooting **"Could not find a package configuration file provided by "ddynamic_reconfigure"**
  + `$ sudo apt install ros-<distro>-ddynamic-reconfigure`

<br>

### ● Disable **Emitter**
+ Disable **Emitter** using **/usr/local/bin/realsense-viewer**, save the **json** file and
  <p align="center">
  <img src="https://github.com/engcang/image-files/blob/master/vins/1.png" width="600" hspace="50"/>
  </p>

+ Check ***"controls-laserstate=off"*** in the .json file
+ import the **json** file at the **rs_camera.launch** file as figure.
  <p align="center">
  <img src="https://github.com/engcang/image-files/blob/master/vins/2.png" width="500" hspace="30"/>
  </p>
  
### ● IMU Calibration recommended [here](https://www.intel.com/content/dam/support/us/en/documents/emerging-technologies/intel-realsense-technology/RealSense_Depth_D435i_IMU_Calib.pdf)
  <p align="center">
  <img src="https://github.com/engcang/VINS-application/blob/Intel-D435i/imu_calibration.png" width="500" hspace="30"/>
  </p>

<br>

### ● First trial
+ used camera intrinsic from **/camera/infra1/rect_image_raw/camera_info**
+ set launch file **rs_camera.launch** as [reference](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/master/config/realsense_d435i/rs_camera.launch)
+ worked quite well

### ● After that,
+ used [Kalibr](https://github.com/ethz-asl/kalibr) and launch file as in this [repository](https://github.com/engcang/vins-application#-calibration--kalibr---synchronization-time-offset-extrinsic-parameter).
+ a lot referred [here](https://github.com/intel-ros/realsense/issues/563) for imu models, configuration, emitter disabling, and VIO result

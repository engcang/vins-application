# VINS-application: Intel-D435i
+ Build Process and explanation [here](https://github.com/engcang/VINS-application)
<br>

## Index
1. Prerequisites
+ [SDK](#requirements---sdk)
+ [ROS packages](#ros-package)
+ [Disable IR Emittier](#disable-emitter)
+ [IMU calibration](#imu-calibration-recommended-here)
+ [Kalibr (Calibration for Cam-IMU)](#kalibrcalibration-for-cameras-imu)
2. [Results](#results)

<br>

## Requirements - [SDK](https://github.com/IntelRealSense/librealsense)
<details><summary>click to see</summary>

### ● Necessary for basic use / ROS version - referred [here](https://github.com/zinuok/Xavier_NX) and [here](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md)
##### ● Jetson boards: add *-DFORCE_RSUSB_BACKEND=ON -DBUILD_WITH_CUDA=true*
~~~shell
  $ sudo apt-get install git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev
  $ sudo apt-get install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev
  $ git clone https://github.com/IntelRealSense/librealsense.git
  $ cd librealsense && mkdir build && cd build
  $ cmake .. -DCMAKE_BUILD_TYPE=Release
  $ sudo make uninstall && make clean
  $ time make -j8 && sudo make install
~~~

### ● Trouble shooting
  + **DS5 group_devices is empty** -> add CMake option
  ~~~
    $ cmake .. -DCMAKE_BUILD_TYPE=Release -DFORCE_RSUSB_BACKEND=ON
  ~~~
  + **Failed to set power state error** or **UDEV-Rules are missing**
  ~~~
    $ sudo cp [librealsense path]/config/99-realsense-libusb.rules /etc/udev/rules.d/99-realsense-libusb.rules && sudo udevadm control --reload-rules && udevadm trigger
    $ reboot
  ~~~

</details>

<br>

## ROS package
<details><summary>click to see</summary>
  
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

</details>

<br>

## Disable **Emitter**

<details><summary>click to see</summary>
  
+ Disable **Emitter** using **/usr/local/bin/realsense-viewer**, save the **json** file and
  <p align="center">
  <img src="https://github.com/engcang/image-files/blob/master/vins/1.png" width="600" hspace="50"/>
  </p>

+ Check ***"controls-laserstate=off"*** in the .json file
+ import the **json** file at the **rs_camera.launch** file as figure.
  <p align="center">
  <img src="https://github.com/engcang/image-files/blob/master/vins/2.png" width="500" hspace="30"/>
  </p>

</details>

<br>

## IMU Calibration recommended [here](https://www.intel.com/content/dam/support/us/en/documents/emerging-technologies/intel-realsense-technology/RealSense_Depth_D435i_IMU_Calib.pdf)
  <p align="center">
  <img src="https://github.com/engcang/VINS-application/blob/Intel-D435i/imu_calibration.png" width="500" hspace="30"/>
  </p>

<br>

## Kalibr(calibration for cameras-IMU)

<details><summary>click to see</summary>


### Remember to use `pinhole-radtan` model in Kalibr and `pinhole` model in VINS-Fusion <br> use `linear_interpolation` for `unite_imu_method` in realsense camera `launch` file

+ Used [Kalibr](https://github.com/ethz-asl/kalibr) as [here](https://support.stereolabs.com/hc/en-us/articles/360012749113-How-can-I-use-Kalibr-with-the-ZED-Mini-camera-in-ROS-) for ZED-mini camera
+ a lot referred [here](https://github.com/intel-ros/realsense/issues/563) for imu models, configuration, emitter disabling, and VIO result
+ First, calibrate cameras
~~~shell
$ kalibr_calibrate_cameras --bag Kalibr_data.bag --topics /camera/infra1/image_rect_raw /camera/infra2/image_rect_raw --models pinhole-radtan pinhole-radtan --target april_grid.yaml
~~~
+ Then, calibrate IMU with cameras
~~~shell
$ kalibr_calibrate_imu_camera --bag Kalibr_data.bag --cam camchain-Kalibr_data.yaml --imu imu-params.yaml --target april_grid.yaml
~~~
+ for `imu-params.yaml`, I used
~~~python
#Accelerometers
accelerometer_noise_density: 0.001865   #Noise density (continuous-time)
accelerometer_random_walk:   0.0002   #Bias random walk

#Gyroscopes
gyroscope_noise_density:     0.0018685   #Noise density (continuous-time)
gyroscope_random_walk:       0.000004   #Bias random walk

rostopic:                    /camera/imu      #the IMU ROS topic
update_rate:                 400.0     #Hz (for discretization of the values above)
~~~

</details>

<br>

## Results
+ Real World VINS(GPU+version, Stereo) with Intel D435i, on Xavier, max CPU clocked: [youtube](https://youtu.be/b3l1TeNKyeU) and [youtube2](https://youtu.be/7yMDqiO2A2Q) : screen
+ VINS-Fusion (Stereo) with Intel D435i and Pixhawk4 mini fused with T265 camera: [here](https://engcang.github.io/mavros_vision_pose/)
+ VINS-Fusion (stereo) with Intel D435i and Pixhawk4 mini on 1km long underground tunnel: [here](https://youtu.be/Gx0PSMCeR1g)

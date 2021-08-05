# VINS-application: Intel-T265
+ Build Process and explanation [here](https://github.com/engcang/VINS-application)
<br>

## Index
1. Prerequisites
+ [SDK](#requirements---sdk)
+ [ROS packages](#ros-package)
+ [Kalibr (Calibration for Cam-IMU)](#kalibrcalibration-for-cameras-imu)
2. [Results](#results)

<br>

## Requirements - [SDK](https://github.com/IntelRealSense/librealsense)
<details><summary>click to see</summary>

### ● Necessary for basic use / ROS version - referred [here](https://github.com/zinuok/Xavier_NX) and [here](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md)
#### ● Jetson boards: add *-DFORCE_RSUSB_BACKEND=ON -DBUILD_WITH_CUDA=true*
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

## Kalibr(calibration for cameras-IMU)

<details><summary>click to see</summary>

### Remember to use `omni-radtan` model in Kalibr and `MEI` model in VINS-Fusion <br> use `linear_interpolation` for `unite_imu_method` in realsense camera `launch` file
  
+ Used [Kalibr](https://github.com/ethz-asl/kalibr) as [here](https://support.stereolabs.com/hc/en-us/articles/360012749113-How-can-I-use-Kalibr-with-the-ZED-Mini-camera-in-ROS-) for ZED-mini camera
+ a lot referred [here](https://www.jianshu.com/p/194d6c9ef9a4), [here2](https://www.intelrealsense.com/how-to-getting-imu-data-from-d435i-and-t265/), and [here3](https://www.bosch-sensortec.com/products/motion-sensors/imus/bmi055/) for imu models, configuration, and VIO result
+ First, calibrate cameras
~~~shell
$ kalibr_calibrate_cameras --bag Kalibr_data.bag --topics /camera/fisheye1/image_raw /camera/fisheye2/image_raw --models omni-radtan omni-radtan --target april_grid.yaml
~~~
+ Then, calibrate IMU with cameras
~~~shell
$ kalibr_calibrate_imu_camera --bag Kalibr_data.bag --cam camchain-Kalibr_data.yaml --imu imu-params.yaml --target april_grid.yaml
~~~
+ for `imu-params.yaml`, I used
~~~python
#Accelerometers
accelerometer_noise_density: 1.85e-03   #Noise density (continuous-time)
accelerometer_random_walk:   2.548e-05   #Bias random walk

#Gyroscopes
gyroscope_noise_density:     1.094e-02   #Noise density (continuous-time)
gyroscope_random_walk:       5.897e-04   #Bias random walk

rostopic:                    /camera/imu      #the IMU ROS topic
update_rate:                 200.0      #Hz (for discretization of the values above)
~~~

</details>

<br>

## Results
+ VINS-Fusion GPU version test with T265: [here](https://youtu.be/8w86LeB6fns)

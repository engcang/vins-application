# VINS-application
+ Build Process and explanation [here](https://github.com/engcang/VINS-application)

## Requirements
+ (necessary for ROS) Intel Realsense [SDK](https://github.com/IntelRealSense/librealsense)
  + For Xavier [here](https://github.com/jetsonhacks/buildLibrealsense2Xavier) -> SDK version is **v2.17.1** and need **JetPack 4.1 for L4T 31.1** or have to be flashed again
+ Intel Realsense2 ROS [here](https://github.com/intel-ros/realsense) For Xavier : -> ROS Wrapper 2.0 version should be **v2.1.3** [down link](https://github.com/intel-ros/realsense/archive/2.1.3.zip)
~~~shell
$ cd ~/catkin_ws/src && wget https://github.com/intel-ros/realsense/archive/2.1.3.zip
$ unzip realsense-2.1.3.zip
$ cd ..
$ catkin build realsense2 -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release -j8
$ source ./devel/setup.bash
~~~

<br>

+ used camera intrinsic from **/camera/infra1/rect_image_raw/camera_info**
+ set launch file ** ** as below [reference](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/issues/6)
~~~xml
"unite_imu_method" = "linear_interpolation"
"gyro_fps" = "200"
"accel_fps" = "250"
"enable_sync" = "true"
"enable_imu" = "true"
~~~

# VINS-application
+ Build Process and explanation [here](https://github.com/engcang/VINS-application)

## Requirements
+ (necessary for ROS) Intel Realsense SDK [SDK](https://github.com/IntelRealSense/librealsense), For Xavier [here](https://github.com/jetsonhacks/buildLibrealsense2Xavier) -> SDK version is **v2.17.1** and need **JetPack 4.1 for L4T 31.1**
+ Intel Realsense2 ROS [here](https://github.com/intel-ros/realsense) For Xavier : -> ROS Wrapper 2.0 version should be **v2.1.3** [down link](https://github.com/intel-ros/realsense/archive/2.1.3.zip)
+ myAHRS+ imu sensor [homepage](http://withrobot.com/en/), [github for ROS](https://github.com/robotpilot/myahrs_driver)
~~~shell
# for permission of imu sensor
$ sudo chmod a+rw /dev/ttyACM0
$ sudo usermod -a -G dialout $USER
~~~

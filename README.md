# VINS-application
+ Build Process and explanation [here](https://github.com/engcang/VINS-application)

## Requirements
+ (necessary for ROS) PointGrey [SDK](https://www.ptgrey.com/support/downloads)
+ PointGrey ROS [Driver](https://github.com/ros-drivers/pointgrey_camera_driver)
+ myAHRS+ imu sensor [homepage](http://withrobot.com/en/), [github for ROS](https://github.com/robotpilot/myahrs_driver)
~~~shell
# for permission of imu sensor
$ sudo chmod a+rw /dev/ttyACM0
$ sudo usermod -a -G dialout $USER
~~~

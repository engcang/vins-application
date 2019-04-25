# VINS-application
+ Build Process and explanation [here](https://github.com/engcang/VINS-application)

## Requirements
### ● For Jetpack 4.2, Ubuntu 18.04 both for Xavier, TX2 : try [this script](https://github.com/engcang/VINS-application/blob/Intel-D435i/jetpack4.2-d435i_tx2_xavier.sh)
  #### ● Edited the scripts file from jetsonhacks github : [here](https://github.com/jetsonhacks/buildLibrealsense2Xavier)
<br>

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

<br>

+ used camera intrinsic from **/camera/infra1/rect_image_raw/camera_info**
+ set launch file **rs_camera.launch** as below [reference](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/master/config/realsense_d435i/rs_camera.launch)
~~~xml
<launch>
  <arg name="serial_no"           default=""/>
  <arg name="json_file_path"      default=""/>
  <arg name="camera"              default="camera"/>
  <arg name="tf_prefix"           default="$(arg camera)"/>

  <arg name="fisheye_width"       default="640"/>
  <arg name="fisheye_height"      default="480"/>
  <arg name="enable_fisheye"      default="true"/>

  <arg name="depth_width"         default="640"/>
  <arg name="depth_height"        default="480"/>
  <arg name="enable_depth"        default="true"/>

  <arg name="infra1_width"        default="640"/>
  <arg name="infra1_height"       default="480"/>
  <arg name="enable_infra1"       default="true"/>

  <arg name="infra2_width"        default="640"/>
  <arg name="infra2_height"       default="480"/>
  <arg name="enable_infra2"       default="true"/>

  <arg name="color_width"         default="640"/>
  <arg name="color_height"        default="480"/>
  <arg name="enable_color"        default="true"/>

  <arg name="fisheye_fps"         default="30"/>
  <arg name="depth_fps"           default="30"/>
  <arg name="infra1_fps"          default="30"/>
  <arg name="infra2_fps"          default="30"/>
  <arg name="color_fps"           default="30"/>
  <arg name="gyro_fps"            default="200"/>
  <arg name="accel_fps"           default="250"/>
  <arg name="enable_imu"          default="true"/>

  <arg name="enable_pointcloud"         default="false"/>
  <arg name="pointcloud_texture_stream" default="RS2_STREAM_COLOR"/>
  <arg name="pointcloud_texture_index"  default="0"/>

  <arg name="enable_sync"           default="true"/>
  <arg name="align_depth"           default="true"/>

  <arg name="filters"               default=""/>
  <arg name="clip_distance"         default="-2"/>
  <arg name="linear_accel_cov"      default="0.01"/>
  <arg name="initial_reset"         default="false"/>
  <arg name="unite_imu_method"      default="linear_interpolation"/>
  <arg name="hold_back_imu_for_frames"      default="true"/>
  
  <group ns="$(arg camera)">
    <include file="$(find realsense2_camera)/launch/includes/nodelet.launch.xml">
      <arg name="tf_prefix"                value="$(arg tf_prefix)"/>
      <arg name="serial_no"                value="$(arg serial_no)"/>
      <arg name="json_file_path"           value="$(arg json_file_path)"/>

      <arg name="enable_pointcloud"        value="$(arg enable_pointcloud)"/>
      <arg name="pointcloud_texture_stream" value="$(arg pointcloud_texture_stream)"/>
      <arg name="pointcloud_texture_index"  value="$(arg pointcloud_texture_index)"/>
      <arg name="enable_sync"              value="$(arg enable_sync)"/>
      <arg name="align_depth"              value="$(arg align_depth)"/>

      <arg name="fisheye_width"            value="$(arg fisheye_width)"/>
      <arg name="fisheye_height"           value="$(arg fisheye_height)"/>
      <arg name="enable_fisheye"           value="$(arg enable_fisheye)"/>

      <arg name="depth_width"              value="$(arg depth_width)"/>
      <arg name="depth_height"             value="$(arg depth_height)"/>
      <arg name="enable_depth"             value="$(arg enable_depth)"/>

      <arg name="color_width"              value="$(arg color_width)"/>
      <arg name="color_height"             value="$(arg color_height)"/>
      <arg name="enable_color"             value="$(arg enable_color)"/>

      <arg name="infra1_width"             value="$(arg infra1_width)"/>
      <arg name="infra1_height"            value="$(arg infra1_height)"/>
      <arg name="enable_infra1"            value="$(arg enable_infra1)"/>

      <arg name="infra2_width"             value="$(arg infra2_width)"/>
      <arg name="infra2_height"            value="$(arg infra2_height)"/>
      <arg name="enable_infra2"            value="$(arg enable_infra2)"/>

      <arg name="fisheye_fps"              value="$(arg fisheye_fps)"/>
      <arg name="depth_fps"                value="$(arg depth_fps)"/>
      <arg name="infra1_fps"               value="$(arg infra1_fps)"/>
      <arg name="infra2_fps"               value="$(arg infra2_fps)"/>
      <arg name="color_fps"                value="$(arg color_fps)"/>
      <arg name="gyro_fps"                 value="$(arg gyro_fps)"/>
      <arg name="accel_fps"                value="$(arg accel_fps)"/>
      <arg name="enable_imu"               value="$(arg enable_imu)"/>

      <arg name="filters"                  value="$(arg filters)"/>
      <arg name="clip_distance"            value="$(arg clip_distance)"/>
      <arg name="linear_accel_cov"         value="$(arg linear_accel_cov)"/>
      <arg name="initial_reset"            value="$(arg initial_reset)"/>
      <arg name="unite_imu_method"         value="$(arg unite_imu_method)"/>
    </include>
  </group>
</launch>
~~~

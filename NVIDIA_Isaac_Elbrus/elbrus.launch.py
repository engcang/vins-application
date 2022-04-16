# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import launch

from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    ld = launch.LaunchDescription()

    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
                    'enable_rectified_pose': True,
                    'denoise_input_images': False,
                    'rectified_images': True,
                    'enable_debug_mode': False,
                    'debug_dump_path': '/tmp/elbrus',
                    'enable_slam_visualization': True,
                    'enable_landmarks_view': True,
                    'enable_observations_view': True,
                    'map_frame': 'map',
                    'odom_frame': 'odom',
                    'base_frame': 'camera_link',
                    'input_left_camera_frame': 'camera_infra1_frame',
                    'input_right_camera_frame': 'camera_infra2_frame',
                    'path_max_size': 8192
                    }],
        remappings=[('stereo_camera/left/image', '/camera/infra1/image_rect_raw'),
                    ('stereo_camera/left/camera_info', '/camera/infra1/camera_info'),
                    ('stereo_camera/right/image', '/camera/infra2/image_rect_raw'),
                    ('stereo_camera/right/camera_info', '/camera/infra2/camera_info')]
    )

    visual_slam_launch_container = ComposableNodeContainer(
        name='visual_slam_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        #prefix=['xterm -e gdb -ex run --args'],
        composable_node_descriptions=[
            visual_slam_node
        ],
        output='screen'
    )
    

    node1 = Node(package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       arguments = ["0", "0", "0", "0", "0", "0", "camera_link", "camera_infra1_frame"])
    node2 = Node(package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       arguments = ["0", "-0.050198", "0", "0", "0", "0", "camera_link", "camera_infra2_frame"])

    ld.add_action(visual_slam_launch_container)
    ld.add_action(node1)
    ld.add_action(node2)
    
    return ld

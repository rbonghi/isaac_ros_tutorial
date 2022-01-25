# Copyright (c) 2022, NVIDIA CORPORATION. All rights reserved.
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.

import os
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    # From https://github.com/stereolabs/zed-ros2-examples/blob/master/zed_display_rviz2/launch/display_zedm.launch.py
    
    pkg_zed = get_package_share_directory('zed_wrapper')
    pkg_tutorial = get_package_share_directory('isaac_ros_zed')

    # Camera model
    # use:
    #  - 'zed' for "ZED" camera
    #  - 'zedm' for "ZED mini" camera
    #  - 'zed2' for "ZED2" camera
    #  - 'zed2i' for "ZED2i" camera
    camera_model = 'zedm'
    camera_name = camera_model
    
    publish_urdf = 'true'  # Publish static frames from camera URDF
    
    # Robot base frame. Note: overrides the parameter `pos_tracking.base_frame` in `common.yaml`.
    base_frame = 'base_link'
    # Position X of the camera with respect to the base frame [m].
    cam_pos_x = '0.0'
    # Position Y of the camera with respect to the base frame [m].
    cam_pos_y = '0.0'
    # Position Z of the camera with respect to the base frame [m].
    cam_pos_z = '0.0'
    # Roll orientation of the camera with respect to the base frame [rad].
    cam_roll = '0.0'
    # Pitch orientation of the camera with respect to the base frame [rad].
    cam_pitch = '0.0'
    # Yaw orientation of the camera with respect to the base frame [rad].
    cam_yaw = '0.0'

    # ZED Configurations from local config
    config_common_path = os.path.join(pkg_tutorial, 'config', 'common.yaml')

    if(camera_model != 'zed'):
        config_camera_path = os.path.join(pkg_zed, 'config', camera_model + '.yaml')
    else:
        config_camera_path = ''

    # URDF/xacro file to be loaded by the Robot State Publisher node
    xacro_path = os.path.join(pkg_zed, 'urdf', 'zed_descr.urdf.xacro')

    # ZED Wrapper node
    zed_wrapper_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('zed_wrapper'),
            '/launch/include/zed_camera.launch.py'
        ]),
        launch_arguments={
            'camera_model': camera_model,
            'camera_name': camera_name,
            'node_name': 'zed_node',
            'config_common_path': config_common_path,
            'config_camera_path': config_camera_path,
            'publish_urdf': publish_urdf,
            'xacro_path': xacro_path,
            'svo_path': '',
            'base_frame': base_frame,
            'cam_pos_x': cam_pos_x,
            'cam_pos_y': cam_pos_y,
            'cam_pos_z': cam_pos_z,
            'cam_roll': cam_roll,
            'cam_pitch': cam_pitch,
            'cam_yaw': cam_yaw
        }.items()
    )

    visual_odometry_node = ComposableNode(
        name='visual_odometry_node',
        package='isaac_ros_visual_odometry',
        plugin='isaac_ros::visual_odometry::VisualOdometryNode',
        namespace='camera',
        parameters=[{
                    'enable_rectified_pose': False,
                    'denoise_input_images': False,
                    'rectified_images': True,
                    'enable_debug_mode': False,
                    'enable_imu': True,
                    'debug_dump_path': '/tmp/elbrus',
                    'left_camera_frame': f"{camera_name}_left_camera_frame",
                    'right_camera_frame': f"{camera_name}_right_camera_frame",
                    'imu_frame': f"{camera_name}_imu_link",
                    'fixed_frame': 'odom',
                    'current_smooth_frame': 'base_link',
                    'current_rectified_frame': 'base_link_rectified'
                    }],
        remappings=[('stereo_camera/left/camera_info',
                     f"/{camera_name}/zed_node/left/camera_info"),
                    ('stereo_camera/right/camera_info',
                     f"/{camera_name}/zed_node/right/camera_info"),
                    ('stereo_camera/left/image',
                     f"/{camera_name}/zed_node/left/image_rect_gray"),
                    ('stereo_camera/right/image',
                     f"/{camera_name}/zed_node/right/image_rect_gray"),
                    ('visual_odometry/imu',
                     f"/{camera_name}/zed_node/imu/data")],
    )

    visual_odometry_launch_container = ComposableNodeContainer(
        name='visual_odometry_launch_container',
        namespace='camera',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            visual_odometry_node
        ],
        output='screen'
    )

    return launch.LaunchDescription([zed_wrapper_launch, visual_odometry_launch_container])

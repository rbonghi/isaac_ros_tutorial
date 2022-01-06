# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import os
import launch
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

import xacro
import tempfile

def to_urdf(xacro_path, parameters=None):
    """Convert the given xacro file to URDF file.
    * xacro_path -- the path to the xacro file
    * parameters -- to be used when xacro file is parsed.
    """
    urdf_path = tempfile.mktemp(prefix="%s_" % os.path.basename(xacro_path))

    # open and process file
    doc = xacro.process_file(xacro_path, mappings=parameters)
    # open the output file
    out = xacro.open_output(urdf_path)
    out.write(doc.toprettyxml(indent='  '))

    return urdf_path

def generate_launch_description():
    """Launch file which brings up visual odometry node configured for RealSense."""
    realsense_camera_node = Node(
        package='realsense2_camera',
        node_executable='realsense2_camera_node',
        namespace='camera',
        parameters=[{
                'infra_height': 360,
                'infra_width': 640,
                'enable_color': False,
                'enable_depth': False,
                'stereo_module.emitter_enabled': 2, #https://github.com/IntelRealSense/realsense-ros/issues/817
                'infra_fps': 90.0,
                'unite_imu_method': 'linear_interpolation' # copy | linear_interpolation
        }],
        )

    visual_odometry_node = ComposableNode(
        name='visual_odometry_node',
        package='isaac_ros_visual_odometry',
        namespace='camera',
        plugin='isaac_ros::visual_odometry::VisualOdometryNode',
        parameters=[{
                    'enable_rectified_pose': False,
                    'denoise_input_images': False,
                    'rectified_images': True,
                    'enable_debug_mode': False,
                    'enable_imu': True,
                    'debug_dump_path': '/tmp/elbrus',
                    'left_camera_frame': 'camera_infra1_frame',
                    'right_camera_frame': 'camera_infra2_frame',
                    'fixed_frame': 'odom',
                    'imu_frame': 'camera_imu_optical_frame',
                    'current_smooth_frame': 'base_link',
                    'current_rectified_frame': 'base_link_rect'
                    }],
        remappings=[('stereo_camera/left/image', 'infra1/image_rect_raw'),
                    ('stereo_camera/left/camera_info', 'infra1/camera_info'),
                    ('stereo_camera/right/image', 'infra2/image_rect_raw'),
                    ('stereo_camera/right/camera_info', 'infra2/camera_info'),
                    ('visual_odometry/imu', 'imu')]
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

    xacro_path = os.path.join(get_package_share_directory('realsense2_description'), 'urdf', 'test_d435i_camera.urdf.xacro')
    urdf = to_urdf(xacro_path, {'use_nominal_extrinsics' : 'true', 'add_plug' : 'true'})

    model_node = Node(
        node_name='model_node',
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='',
        output='screen',
        arguments = [urdf]
        )

    return launch.LaunchDescription([visual_odometry_launch_container, realsense_camera_node, model_node])
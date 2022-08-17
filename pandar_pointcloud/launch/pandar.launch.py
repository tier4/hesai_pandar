
import launch
import os
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from ament_index_python import get_package_share_directory

import yaml


def launch_setup(context, *args, **kwargs):

    def create_parameter_dict(*args):
        result = {}
        for x in args:
            result[x] = LaunchConfiguration(x)
        return result

    nodes = []

    nodes.append(ComposableNode(
        package='pandar_pointcloud',
        plugin='pandar_pointcloud::PandarCloud',
        name='pandar_cloud',
        parameters=[{**create_parameter_dict('scan_phase', 'start_angle', 'end_angle', 'model', 'return_mode', 'run_mode', 'background', 'device_ip', 'calibration'),
        }],
        remappings=[('pandar_points_objects', 'pointcloud_filter'),
                    ('pandar_points_background', 'pointcloud_background')],
        extra_arguments=[{
            'use_intra_process_comms': LaunchConfiguration('use_intra_process')
        }],
    )
    )

    container = ComposableNodeContainer(
        name='pandar_node_container',
        namespace='',
        package='rclcpp_components',
        executable=LaunchConfiguration('container_executable'),
        composable_node_descriptions=nodes,
    )

    driver_component = ComposableNode(
        package='pandar_driver',
        plugin='pandar_driver::PandarDriver',
        name='pandar_driver',
        parameters=[{**create_parameter_dict('pcap', 'device_ip', 'lidar_port', 'gps_port', 'scan_phase', 'model', 'frame_id'),
                     }],
    )

    loader = LoadComposableNodes(
        composable_node_descriptions=[driver_component],
        target_container=container,
        condition=launch.conditions.IfCondition(LaunchConfiguration('launch_driver')),
    )

    return [container, loader]


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value))


    package_dir = get_package_share_directory("pandar_pointcloud")
    add_launch_arg('launch_driver', 'True')
    add_launch_arg('pcap', '')
    add_launch_arg('device_ip', '192.168.1.201')
    add_launch_arg('lidar_port', '2368')
    add_launch_arg('gps_port', '10121')
    add_launch_arg('scan_phase', '0.0')
    add_launch_arg('start_angle', '120.0')
    add_launch_arg('end_angle', '240.0')
    add_launch_arg('model', 'PandarQT')
    add_launch_arg('run_mode', 'Subtract')
    add_launch_arg('return_mode', 'First')
    add_launch_arg('background', os.path.join(package_dir, 'config/background.exr'))
    add_launch_arg('frame_id', 'pandar')
    add_launch_arg('calibration', os.path.join(package_dir, 'config/qt.csv'))

    add_launch_arg('container_name', 'pandar_composable_node_container')
    add_launch_arg('use_multithread', 'False')
    add_launch_arg('use_intra_process', 'False')

    set_container_executable = SetLaunchConfiguration(
        'container_executable',
        'component_container',
        condition=UnlessCondition(LaunchConfiguration('use_multithread'))
    )

    set_container_mt_executable = SetLaunchConfiguration(
        'container_executable',
        'component_container_mt',
        condition=IfCondition(LaunchConfiguration('use_multithread'))
    )

    return launch.LaunchDescription(launch_arguments +
                                    [set_container_executable,
                                     set_container_mt_executable] +
                                    [OpaqueFunction(function=launch_setup)])

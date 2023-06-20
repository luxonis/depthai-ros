import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):
    params_file = LaunchConfiguration("params_file")
    depthai_prefix = get_package_share_directory("depthai_ros_driver")
    name = LaunchConfiguration('name').perform(context)
    
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(depthai_prefix, 'launch', 'rgbd_pcl.launch.py')),
            launch_arguments={"name": name,
                              "params_file": params_file,
                              "rectify_rgb": "true"}.items()),

        LoadComposableNodes(
            target_container=name+"_container",
            composable_node_descriptions=[
                    ComposableNode(
                        package="depthai_filters",
                        plugin="depthai_filters::SpatialBB",
                        remappings=[
                                    ('stereo/camera_info', name+'/stereo/camera_info'),
                                    ('nn/spatial_detections', name+'/nn/spatial_detections'),
                                    ('rgb/preview/image_raw', name+'/rgb/preview/image_raw')]
                    ),
            ],
        ),

    ]


def generate_launch_description():
    depthai_filters_prefix = get_package_share_directory("depthai_filters")

    declared_arguments = [
        DeclareLaunchArgument("name", default_value="oak"),
        DeclareLaunchArgument("params_file", default_value=os.path.join(depthai_filters_prefix, 'config', 'spatial_bb.yaml')),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
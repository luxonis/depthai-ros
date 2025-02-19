import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):
    depthai_prefix = get_package_share_directory("depthai_ros_driver")
    name = LaunchConfiguration('name').perform(context)
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(depthai_prefix, 'launch', 'oak_t.launch.py')),
            launch_arguments={"name": name}.items()),

        LoadComposableNodes(
            target_container=name+"_container",
            composable_node_descriptions=[
                    ComposableNode(
                        package="depthai_filters",
                        name="thermal_temp",
                        plugin="depthai_filters::ThermalTemp",
                        remappings=[('/thermal/raw_data/image_raw', name+'/thermal/raw_data/image_raw'),],
                    ),
            ],
        ),

    ]


def generate_launch_description():

    declared_arguments = [
        DeclareLaunchArgument("name", default_value="oak"),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )


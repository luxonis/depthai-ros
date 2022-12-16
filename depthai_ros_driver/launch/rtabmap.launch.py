import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    name = LaunchConfiguration('name').perform(context)
    depthai_prefix = get_package_share_directory("depthai_ros_driver")

    params_file= LaunchConfiguration("params_file")
    parameters = [
        {
            "frame_id": name,
            "subscribe_depth": True,
            "approx_sync": True,
            "Rtabmap/DetectionRate": "3.5",
        }
    ]

    remappings = [
        ("rgb/image", name+"/rgb/image_raw"),
        ("rgb/camera_info", name+"/rgb/camera_info"),
        ("depth/image", name+"/stereo/image_raw"),
    ]
    return [
        Node(
                package="rtabmap_ros",
                executable="rgbd_odometry",
                output="screen",
                parameters=parameters,
                remappings=remappings,
            ),
            Node(
                package="rtabmap_ros",
                executable="rtabmap",
                output="screen",
                parameters=parameters,
                remappings=remappings,
                arguments=["-d"],
            ),
            Node(
                package="rtabmap_ros",
                executable="rtabmapviz",
                output="screen",
                parameters=parameters,
                remappings=remappings,
            ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(depthai_prefix, 'launch', 'camera.launch.py')),
            launch_arguments={"name": name,
                              "params_file": params_file}.items())
    ]


def generate_launch_description():
    depthai_prefix = get_package_share_directory("depthai_ros_driver")
    declared_arguments = [
        DeclareLaunchArgument("name", default_value="oak"),
        DeclareLaunchArgument("params_file", default_value=os.path.join(depthai_prefix, 'config', 'rgbd.yaml')),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )

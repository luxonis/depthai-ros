import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    depthai_prefix = get_package_share_directory("depthai_ros_driver")
    params_file = LaunchConfiguration("params_file")
    name = LaunchConfiguration("name").perform(context)
    type = LaunchConfiguration("type").perform(context)
    print("#########")
    print(type)
    size = LaunchConfiguration("size").perform(context)
    square = LaunchConfiguration("square").perform(context)
    mono_args = ["--size", size,
                 "--square", square,
                 "--camera_name", "oak/rgb",
                 "--no-service-check"]
    stereo_args = ["--size", size,
                   "--square", square,
                   "--camera_name", "/rgb",
                   "--approximate", "0.1",
                   "--camera_name", "oak",
                   "left_camera", "left",
                   "right_camera", "right",
                   "--no-service-check"]
    mono_remappings = [('/image', '{}/rgb/image_raw'.format(name))]
    stereo_remappings = [('/left', '{}/left/image_raw'.format(name)),
                         ('/right', '{}/right/image_raw'.format(name))]
    args = []
    remappings = []
    if type == "mono":
        args = mono_args
        remappings = mono_remappings
    else:
        args = stereo_args
        remappings = stereo_remappings

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(depthai_prefix, 'launch', 'camera.launch.py')),
            launch_arguments={"name": name,
                              "params_file": params_file}.items()
        ),
        Node(
            name="calibrator",
            namespace="",
            package="camera_calibration",
            executable="cameracalibrator",
            arguments=args,
            remappings=remappings
        )
    ]


def generate_launch_description():
    depthai_prefix = get_package_share_directory("depthai_ros_driver")
    declared_arguments = [
        DeclareLaunchArgument("name", default_value="oak"),
        DeclareLaunchArgument("size", default_value="8x6"),
        DeclareLaunchArgument("square", default_value="0.108"),
        DeclareLaunchArgument("type", default_value="mono"),
        DeclareLaunchArgument("params_file", default_value=os.path.join(depthai_prefix, 'config', 'calibration.yaml')),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    name = LaunchConfiguration('name').perform(context)
    depthai_prefix = get_package_share_directory("depthai_ros_driver")
    rosbag_path = LaunchConfiguration("rosbag_path").perform(context)
    rviz_config = os.path.join(depthai_prefix, "config", "rviz", "rgbd.rviz")

    params_file= LaunchConfiguration("params_file")

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(depthai_prefix, 'launch', 'camera.launch.py')),
            launch_arguments={"name": name,
                              "params_file": params_file,
                              "use_rviz": LaunchConfiguration("use_rviz"),
                              "rviz_config": rviz_config}.items()),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', '-l', rosbag_path],
            output='screen'
        )
    ]


def generate_launch_description():
    depthai_prefix = get_package_share_directory("depthai_ros_driver")
    declared_arguments = [
        DeclareLaunchArgument("name", default_value="oak"),
        DeclareLaunchArgument("params_file", default_value=os.path.join(depthai_prefix, 'config', 'stereo_from_rosbag.yaml')),
        DeclareLaunchArgument("use_rviz", default_value="True"),
        DeclareLaunchArgument("rosbag_path", default_value="")
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )

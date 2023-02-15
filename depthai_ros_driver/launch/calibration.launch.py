import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):
    name = LaunchConfiguration("name").perform(context)
    type = LaunchConfiguration("type").perform(context)
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
    if type=="mono":
        args = mono_args
        remappings = mono_remappings
    else:
        args = stereo_args
        remappings = stereo_remappings

    return [
        ComposableNodeContainer(
            name=name+"_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=[
                    ComposableNode(
                        package="depthai_ros_driver",
                        plugin="depthai_ros_driver::Camera",
                        name=name,
                        parameters=[{"camera.i_pipeline_type": "RGBStereo",
                                     "camera.i_nn_type": "none",
                                     "right.i_publish_topic": True,
                                     "left.i_publish_topic": True}],
                    ),
            ],
            output="screen",
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
        DeclareLaunchArgument("type", default_value="mono")

    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )

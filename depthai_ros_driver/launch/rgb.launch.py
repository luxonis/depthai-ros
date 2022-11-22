import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    depthai_prefix = get_package_share_directory("depthai_ros_driver")
    rviz_config = os.path.join(depthai_prefix, "config", "rgbd.rviz")
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="False"),
            Node(
                condition=IfCondition(LaunchConfiguration("use_rviz")),
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="log",
                arguments=["-d", rviz_config],
            ),
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(
            #         os.path.join(depthai_prefix, "launch", "description.launch.py")
            #     )
            # ),
            ComposableNodeContainer(
                name="container",
                namespace="",
                package="rclcpp_components",
                executable="component_container",
                composable_node_descriptions=[
                    # Driver itself
                    ComposableNode(
                        package="depth_image_proc",
                        plugin="depth_image_proc::ConvertMetricNode",
                        name="convert_metric_node",
                        remappings=[
                            ("image_raw", "/camera/depth/image_raw"),
                            ("camera_info", "/camera/depth/camera_info"),
                            ("image", "/camera/depth/converted_depth"),
                        ],
                    ),
                    ComposableNode(
                        package="depth_image_proc",
                        plugin="depth_image_proc::PointCloudXyzNode",
                        name="point_cloud_xyz_node",
                        remappings=[
                            ("camera_info", "/camera/depth/camera_info"),
                            ("image", "/camera/depth/image_raw"),
                            (
                                "image_rect",
                                "/camera/depth/converted_depth",
                            ),
                        ],
                    ),
                    ComposableNode(
                        package="depthai_ros_driver",
                        plugin="depthai_ros_driver::Camera",
                        name="camera",
                    ),
                ],
                output="screen",
            ),
        ]
    )
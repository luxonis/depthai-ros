import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):
    log_level = "info"
    if context.environment.get("DEPTHAI_DEBUG") == "1":
        log_level = "debug"

    params_file = LaunchConfiguration("params_file")
    depthai_prefix = get_package_share_directory("depthai_ros_driver")

    name = LaunchConfiguration("name").perform(context)
    rgb_topic_name = name + "/rgb/image_raw"
    if LaunchConfiguration("rectify_rgb").perform(context) == "true":
        rgb_topic_name = name + "/rgb/image_rect"

    parent_frame = LaunchConfiguration("parent_frame", default="oak-d-base-frame")
    cam_pos_x = LaunchConfiguration("cam_pos_x", default="0.0")
    cam_pos_y = LaunchConfiguration("cam_pos_y", default="0.0")
    cam_pos_z = LaunchConfiguration("cam_pos_z", default="0.0")
    cam_roll = LaunchConfiguration("cam_roll", default="0.0")
    cam_pitch = LaunchConfiguration("cam_pitch", default="0.0")
    cam_yaw = LaunchConfiguration("cam_yaw", default="0.0")
    use_composition = LaunchConfiguration("rsp_use_composition", default="true")
    imu_from_descr = LaunchConfiguration("imu_from_descr", default="false")
    publish_tf_from_calibration = LaunchConfiguration(
        "publish_tf_from_calibration", default="false"
    )
    override_cam_model = LaunchConfiguration("override_cam_model", default="false")

    tf_params = {}
    if publish_tf_from_calibration.perform(context) == "true":
        cam_model = ""
        if override_cam_model.perform(context) == "true":
            cam_model = camera_model.perform(context)
        tf_params = {
            "camera": {
                "i_publish_tf_from_calibration": True,
                "i_tf_tf_prefix": name,
                "i_tf_camera_model": cam_model,
                "i_tf_base_frame": name,
                "i_tf_parent_frame": parent_frame.perform(context),
                "i_tf_cam_pos_x": cam_pos_x.perform(context),
                "i_tf_cam_pos_y": cam_pos_y.perform(context),
                "i_tf_cam_pos_z": cam_pos_z.perform(context),
                "i_tf_cam_roll": cam_roll.perform(context),
                "i_tf_cam_pitch": cam_pitch.perform(context),
                "i_tf_cam_yaw": cam_yaw.perform(context),
                "i_tf_imu_from_descr": imu_from_descr.perform(context),
            }
        }

    return [
        ComposableNodeContainer(
            name=name + "_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=[
                ComposableNode(
                    package="depthai_ros_driver",
                    plugin="depthai_ros_driver::Camera",
                    name=name,
                    parameters=[params_file, tf_params],
                )
            ],
            arguments=["--ros-args", "--log-level", log_level],
            output="both",
        ),
        LoadComposableNodes(
            condition=IfCondition(LaunchConfiguration("rectify_rgb")),
            target_container=name + "_container",
            composable_node_descriptions=[
                ComposableNode(
                    package="image_proc",
                    plugin="image_proc::RectifyNode",
                    name=name + "_rectify_color_node",
                    remappings=[
                        ("image", name + "/rgb/image_raw"),
                        ("camera_info", name + "/rgb/camera_info"),
                        ("image_rect", name + "/rgb/image_rect"),
                        ("image_rect/compressed", name + "/rgb/image_rect/compressed"),
                        (
                            "image_rect/compressedDepth",
                            name + "/rgb/image_rect/compressedDepth",
                        ),
                        ("image_rect/theora", name + "/rgb/image_rect/theora"),
                    ],
                )
            ],
        ),
        LoadComposableNodes(
            target_container=name + "_container",
            composable_node_descriptions=[
                ComposableNode(
                    package="depth_image_proc",
                    plugin="depth_image_proc::PointCloudXyzrgbNode",
                    name=name + "_point_cloud_xyzrgb_node",
                    remappings=[
                        ("depth_registered/image_rect", name + "/stereo/image_raw"),
                        ("rgb/image_rect_color", rgb_topic_name),
                        ("rgb/camera_info", name + "/rgb/camera_info"),
                        ("points", name + "/points"),
                    ],
                ),
            ],
        ),
    ]


def generate_launch_description():
    depthai_prefix = get_package_share_directory("depthai_ros_driver")
    declared_arguments = [
        DeclareLaunchArgument("name", default_value="oak"),
        DeclareLaunchArgument("camera_model", default_value="OAK-D"),
        DeclareLaunchArgument("parent_frame", default_value="oak-d-base-frame"),
        DeclareLaunchArgument("cam_pos_x", default_value="0.0"),
        DeclareLaunchArgument("cam_pos_y", default_value="0.0"),
        DeclareLaunchArgument("cam_pos_z", default_value="0.0"),
        DeclareLaunchArgument("cam_roll", default_value="0.0"),
        DeclareLaunchArgument("cam_pitch", default_value="0.0"),
        DeclareLaunchArgument("cam_yaw", default_value="0.0"),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(depthai_prefix, "config", "rgbd.yaml"),
        ),
        DeclareLaunchArgument("rectify_rgb", default_value="False"),
        DeclareLaunchArgument("rsp_use_composition", default_value="true"),
        DeclareLaunchArgument(
            "publish_tf_from_calibration",
            default_value="false",
            description="Enables TF publishing from camera calibration file.",
        ),
        DeclareLaunchArgument(
            "imu_from_descr",
            default_value="false",
            description="Enables IMU publishing from URDF.",
        ),
        DeclareLaunchArgument(
            "override_cam_model",
            default_value="false",
            description="Overrides camera model from calibration file.",
        ),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )

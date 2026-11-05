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
    tf_prefix = LaunchConfiguration("tf_prefix").perform(context)
    tf_prefix = tf_prefix.strip("/") or name
    rgb_topic_name = name + "/rgb/image_raw"
    if LaunchConfiguration("rectify_rgb").perform(context) == "true":
        rgb_topic_name = name + "/rgb/image_rect"

    parent_frame = LaunchConfiguration("parent_frame", default="oak_parent_frame")
    cam_pos_x = LaunchConfiguration("cam_pos_x", default="0.0")
    cam_pos_y = LaunchConfiguration("cam_pos_y", default="0.0")
    cam_pos_z = LaunchConfiguration("cam_pos_z", default="0.0")
    cam_roll = LaunchConfiguration("cam_roll", default="0.0")
    cam_pitch = LaunchConfiguration("cam_pitch", default="0.0")
    cam_yaw = LaunchConfiguration("cam_yaw", default="0.0")
    use_composition = LaunchConfiguration("rsp_use_composition", default="false")
    camera_model = LaunchConfiguration("camera_model", default="OAK-D")
    imu_from_descr = LaunchConfiguration("imu_from_descr", default="false")
    imu_sensor = LaunchConfiguration("imu_sensor", default="")
    imu_frame = LaunchConfiguration("imu_frame", default="auto")
    pose_source = LaunchConfiguration("pose_source", default="legacy")
    model_selection = LaunchConfiguration("model_selection", default="auto")
    fallback_model = LaunchConfiguration("fallback_model", default="")
    mesh_model = LaunchConfiguration("mesh_model", default="")
    legacy_frame_aliases = LaunchConfiguration("legacy_frame_aliases", default="true")
    publish_tf_from_calibration = LaunchConfiguration(
        "publish_tf_from_calibration", default="true"
    )
    override_cam_model = LaunchConfiguration("override_cam_model", default="false")

    tf_params = {"driver": {"i_tf_prefix": tf_prefix}}
    if publish_tf_from_calibration.perform(context) == "true":
        cam_model = ""
        if override_cam_model.perform(context) == "true":
            cam_model = camera_model.perform(context)
        tf_params["driver"].update(
            {
                "i_publish_tf_from_calibration": True,
                "i_tf_device_name": name,
                "i_tf_device_model": cam_model,
                "i_tf_base_frame": name,
                "i_tf_parent_frame": parent_frame.perform(context),
                "i_tf_cam_pos_x": cam_pos_x.perform(context),
                "i_tf_cam_pos_y": cam_pos_y.perform(context),
                "i_tf_cam_pos_z": cam_pos_z.perform(context),
                "i_tf_cam_roll": cam_roll.perform(context),
                "i_tf_cam_pitch": cam_pitch.perform(context),
                "i_tf_cam_yaw": cam_yaw.perform(context),
                "i_tf_imu_from_descr": imu_from_descr.perform(context),
                "i_tf_imu_sensor": imu_sensor.perform(context),
                "i_tf_imu_frame": imu_frame.perform(context),
                "i_tf_pose_source": pose_source.perform(context),
                "i_tf_model_selection": model_selection.perform(context),
                "i_tf_fallback_model": fallback_model.perform(context),
                "i_tf_mesh_model": mesh_model.perform(context),
                "i_tf_legacy_frame_aliases": legacy_frame_aliases.perform(context),
            }
        )
    else:
        tf_params["driver"]["i_publish_tf_from_calibration"] = False

    return [
        ComposableNodeContainer(
            name=name + "_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=[
                ComposableNode(
                    package="depthai_ros_driver",
                    plugin="depthai_ros_driver::Driver",
                    name=name,
                    parameters=[params_file, tf_params],
                )
            ],
            arguments=["--ros-args", "--log-level", log_level],
            output="both",
        ),
    ]


def generate_launch_description():
    depthai_prefix = get_package_share_directory("depthai_ros_driver")
    declared_arguments = [
        DeclareLaunchArgument("name", default_value="oak"),
        DeclareLaunchArgument(
            "tf_prefix",
            default_value="",
            description="Prefix for camera-related TF frames. Defaults to the node name.",
        ),
        DeclareLaunchArgument("camera_model", default_value="OAK-D"),
        DeclareLaunchArgument("parent_frame", default_value="oak_parent_frame"),
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
        DeclareLaunchArgument("rsp_use_composition", default_value="false"),
        DeclareLaunchArgument(
            "publish_tf_from_calibration",
            default_value="false",
            description="Enables TF publishing from camera calibration file.",
        ),
        DeclareLaunchArgument("imu_sensor", default_value="", description="IMU key or chip name; detected from hardware in calibration TF mode."),
        DeclareLaunchArgument("pose_source", default_value="legacy"),
        DeclareLaunchArgument("model_selection", default_value="auto"),
        DeclareLaunchArgument("fallback_model", default_value=""),
        DeclareLaunchArgument("mesh_model", default_value=""),
        DeclareLaunchArgument("legacy_frame_aliases", default_value="true"),
        DeclareLaunchArgument("imu_frame", default_value="auto", choices=["auto", "optical", "native"]),
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

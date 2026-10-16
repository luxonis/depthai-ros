import os
import subprocess
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):
    bringup_dir = get_package_share_directory("depthai_descriptions")
    use_base_descr = LaunchConfiguration("use_base_descr", default="false")

    camera_model = LaunchConfiguration("camera_model", default="OAK-D")
    imu_sensor = LaunchConfiguration("imu_sensor", default="")
    imu_frame = LaunchConfiguration("imu_frame", default="auto")
    pose_source = LaunchConfiguration("pose_source", default="legacy")
    model_selection = LaunchConfiguration("model_selection", default="auto")
    fallback_model = LaunchConfiguration("fallback_model", default="")
    mesh_model = LaunchConfiguration("mesh_model", default="")
    legacy_frame_aliases = LaunchConfiguration("legacy_frame_aliases", default="true")
    tf_prefix = LaunchConfiguration("tf_prefix", default="oak")
    base_frame = LaunchConfiguration("base_frame", default="oak")
    parent_frame = LaunchConfiguration("parent_frame", default="oak_parent_frame")
    cam_pos_x = LaunchConfiguration("cam_pos_x", default="0.0")
    cam_pos_y = LaunchConfiguration("cam_pos_y", default="0.0")
    cam_pos_z = LaunchConfiguration("cam_pos_z", default="0.0")
    cam_roll = LaunchConfiguration("cam_roll", default="1.5708")
    cam_pitch = LaunchConfiguration("cam_pitch", default="0.0")
    cam_yaw = LaunchConfiguration("cam_yaw", default="1.5708")
    namespace = LaunchConfiguration("namespace", default="")
    rs_compat = LaunchConfiguration("rs_compat", default="false")
    use_composition = LaunchConfiguration("use_composition", default="false")

    rsp_name = LaunchConfiguration("rsp_name").perform(context)
    if not rsp_name:
        rsp_name = LaunchConfiguration("tf_prefix").perform(context).strip("/").replace("/", "_")
    has_imu = LaunchConfiguration("has_imu", default="auto").perform(context)
    if has_imu == "auto" and use_base_descr.perform(context) == "true":
        has_imu = "false"
    mappings = {
        "camera_name": tf_prefix.perform(context), "camera_model": camera_model.perform(context),
        "imu_sensor": imu_sensor.perform(context), "imu_frame": imu_frame.perform(context),
        "has_imu": has_imu, "use_base_descr": use_base_descr.perform(context),
        "base_frame": base_frame.perform(context), "parent_frame": parent_frame.perform(context),
        "cam_pos_x": cam_pos_x.perform(context), "cam_pos_y": cam_pos_y.perform(context),
        "cam_pos_z": cam_pos_z.perform(context), "cam_roll": cam_roll.perform(context),
        "cam_pitch": cam_pitch.perform(context), "cam_yaw": cam_yaw.perform(context),
        "rs_compat": rs_compat.perform(context),
        "pose_source": pose_source.perform(context),
        "model_selection": model_selection.perform(context),
        "fallback_model": fallback_model.perform(context),
        "mesh_model": mesh_model.perform(context),
        "legacy_frame_aliases": legacy_frame_aliases.perform(context),
    }
    command = [sys.executable, os.path.join(bringup_dir, "scripts", "resolve_description.py")]
    command.extend(f"{key}:={value}" for key, value in mappings.items())
    result = subprocess.run(command, capture_output=True, text=True)
    if result.returncode != 0 or not result.stdout.strip():
        raise RuntimeError(f"Unable to generate robot description: {result.stderr}")
    if result.stderr.strip():
        from launch.logging import get_logger
        get_logger("depthai_descriptions").warning(result.stderr.strip())
    robot_description = {"robot_description": ParameterValue(result.stdout, value_type=str)}
    return [
        Node(
            package="robot_state_publisher",
            condition=UnlessCondition(use_composition),
            executable="robot_state_publisher",
            name=rsp_name + "_state_publisher",
            namespace=namespace,
            parameters=[robot_description],
        ),
        LoadComposableNodes(
            target_container=f"{namespace.perform(context)}/{rsp_name}_container",
            condition=IfCondition(use_composition),
            composable_node_descriptions=[
                ComposableNode(
                    package="robot_state_publisher",
                    plugin="robot_state_publisher::RobotStatePublisher",
                    name=rsp_name + "_state_publisher",
                    namespace=namespace,
                    parameters=[robot_description],
                )
            ],
        ),
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("pose_source", default_value="legacy"),
        DeclareLaunchArgument("model_selection", default_value="auto"),
        DeclareLaunchArgument("fallback_model", default_value=""),
        DeclareLaunchArgument("mesh_model", default_value=""),
        DeclareLaunchArgument("legacy_frame_aliases", default_value="true"),
        DeclareLaunchArgument(
            "rsp_name",
            default_value="",
            description="Robot state publisher node and container name. Defaults to tf_prefix.",
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description='Specifies the namespace of the robot state publisher node. Default value will be ""',
        ),
        DeclareLaunchArgument(
            "camera_model",
            default_value="OAK-D",
            description="Board model name from board_models.yaml or model_config.yaml.",
        ),
        DeclareLaunchArgument(
            "has_imu", default_value="auto", choices=["auto", "true", "false"],
            description="Publish IMU TF from this description; auto selects available poses in the full description.",
        ),
        DeclareLaunchArgument(
            "imu_sensor", default_value="",
            description="IMU variant key, e.g. BNO, BMI or LSM6DSV. Required for boards with multiple variants and no legacy IMU pose.",
        ),
        DeclareLaunchArgument(
            "imu_frame", default_value="auto", choices=["auto", "optical", "native"],
            description="Axes for board-derived IMU poses. Optical matches the ROS driver's RDF convention.",
        ),
        DeclareLaunchArgument(
            "tf_prefix",
            default_value="oak",
            description="Prefix applied to camera-related TF frame names.",
        ),
        DeclareLaunchArgument(
            "base_frame",
            default_value="oak",
            description="Name of the base link.",
        ),
        DeclareLaunchArgument(
            "parent_frame",
            default_value="oak_parent_frame",
            description="Name of the parent link from other a robot TF for example that can be connected to the base of the OAK.",
        ),
        DeclareLaunchArgument(
            "cam_pos_x",
            default_value="0.0",
            description="Position X of the camera with respect to the base frame.",
        ),
        DeclareLaunchArgument(
            "cam_pos_y",
            default_value="0.0",
            description="Position Y of the camera with respect to the base frame.",
        ),
        DeclareLaunchArgument(
            "cam_pos_z",
            default_value="0.0",
            description="Position Z of the camera with respect to the base frame.",
        ),
        DeclareLaunchArgument(
            "cam_roll",
            default_value="0.0",
            description="Roll orientation of the camera with respect to the base frame.",
        ),
        DeclareLaunchArgument(
            "cam_pitch",
            default_value="0.0",
            description="Pitch orientation of the camera with respect to the base frame.",
        ),
        DeclareLaunchArgument(
            "cam_yaw",
            default_value="0.0",
            description="Yaw orientation of the camera with respect to the base frame.",
        ),
        DeclareLaunchArgument(
            "use_composition",
            default_value="false",
            description="Use composition to start the robot_state_publisher node. Default value will be false",
        ),
        DeclareLaunchArgument(
            "use_base_descr",
            default_value="false",
            description="Launch base description. Default value will be false",
        ),
        DeclareLaunchArgument(
            "rs_compat",
            default_value="false",
            description="Enable RealSense compatibility mode. Default value will be false",
        ),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )

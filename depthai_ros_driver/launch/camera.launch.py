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


def is_launch_config_true(context, name):
    return LaunchConfiguration(name).perform(context) == "true"


def setup_launch_prefix(context, *args, **kwargs):
    use_gdb = LaunchConfiguration("use_gdb", default="false")
    use_valgrind = LaunchConfiguration("use_valgrind", default="false")
    use_perf = LaunchConfiguration("use_perf", default="false")

    launch_prefix = ""

    if use_gdb.perform(context) == "true":
        launch_prefix += "xterm -e gdb -ex run --args"
    if use_valgrind.perform(context) == "true":
        launch_prefix += "valgrind --tool=callgrind"
    if use_perf.perform(context) == "true":
        launch_prefix += (
            "perf record -g --call-graph dwarf --output=perf.out.node_name.data --"
        )

    return launch_prefix


def launch_setup(context, *args, **kwargs):
    log_level = "info"
    if context.environment.get("DEPTHAI_DEBUG") == "1":
        log_level = "debug"

    urdf_launch_dir = os.path.join(
        get_package_share_directory("depthai_descriptions"), "launch"
    )

    parent_frame = LaunchConfiguration(
        "parent_frame", default="oak-d-base-frame"
    ).perform(context)
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
    params_file = LaunchConfiguration("params_file")
    camera_model = LaunchConfiguration("camera_model", default="OAK-D")
    rs_compat = LaunchConfiguration("rs_compat", default="false")
    pointcloud_enable = LaunchConfiguration("pointcloud.enable", default="false")
    rectify_rgb = LaunchConfiguration("rectify_rgb", default="true")
    namespace = LaunchConfiguration("namespace", default="").perform(context)
    name = LaunchConfiguration("name").perform(context)

    # If RealSense compatibility is enabled, we need to override some parameters, topics and node names
    parameter_overrides = {}
    color_sens_name = "rgb"
    stereo_sens_name = "stereo"
    points_topic_name = f"{name}/points"
    if rs_compat.perform(context) == "true":
        color_sens_name = "color"
        stereo_sens_name = "depth"
        if name == "oak":
            name = "camera"
        points_topic_name = f"{name}/depth/color/points"
        if namespace== "":
            namespace = "camera"
        if parent_frame == "oak-d-base-frame":
            parent_frame = f"{name}_link"
        parameter_overrides = {
            "camera": {
                "i_rs_compat": True,
            },
            "color": {
                "i_publish_topic": is_launch_config_true(context, "enable_color"),
            },
            "depth": {
                "i_publish_topic": is_launch_config_true(context, "enable_depth"),
            },
        }
        parameter_overrides["depth"] = {
                "i_publish_left_rect": True,
                "i_publish_right_rect": True,
            }

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
                "i_tf_parent_frame": parent_frame,
                "i_tf_cam_pos_x": cam_pos_x.perform(context),
                "i_tf_cam_pos_y": cam_pos_y.perform(context),
                "i_tf_cam_pos_z": cam_pos_z.perform(context),
                "i_tf_cam_roll": cam_roll.perform(context),
                "i_tf_cam_pitch": cam_pitch.perform(context),
                "i_tf_cam_yaw": cam_yaw.perform(context),
                "i_tf_imu_from_descr": imu_from_descr.perform(context),
            }
        }

    launch_prefix = setup_launch_prefix(context)

    return [
        Node(
            condition=IfCondition(LaunchConfiguration("use_rviz").perform(context)),
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", LaunchConfiguration("rviz_config")],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(urdf_launch_dir, "urdf_launch.py")
            ),
            launch_arguments={
                "namespace": namespace,
                "tf_prefix": name,
                "camera_model": camera_model,
                "base_frame": name,
                "parent_frame": parent_frame,
                "cam_pos_x": cam_pos_x,
                "cam_pos_y": cam_pos_y,
                "cam_pos_z": cam_pos_z,
                "cam_roll": cam_roll,
                "cam_pitch": cam_pitch,
                "cam_yaw": cam_yaw,
                "use_composition": use_composition,
                "use_base_descr": publish_tf_from_calibration,
                "rs_compat": rs_compat,
            }.items(),
        ),
        ComposableNodeContainer(
            name=f"{name}_container",
            namespace=namespace,
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=[
                ComposableNode(
                    package="depthai_ros_driver",
                    plugin="depthai_ros_driver::Camera",
                    name=name,
                    namespace=namespace,
                    parameters=[
                        params_file,
                        tf_params,
                        parameter_overrides,
                    ],
                )
            ],
            arguments=["--ros-args", "--log-level", log_level],
            prefix=[launch_prefix],
            output="both",
        ),
        LoadComposableNodes(
            condition=IfCondition(rectify_rgb),
            target_container=f"{namespace}/{name}_container",
            composable_node_descriptions=[
                ComposableNode(
                    package="image_proc",
                    plugin="image_proc::RectifyNode",
                    name="rectify_color_node",
                    namespace=namespace,
                    remappings=[
                        ("image", f"{name}/{color_sens_name}/image_raw"),
                        ("camera_info", f"{name}/{color_sens_name}/camera_info"),
                        ("image_rect", f"{name}/{color_sens_name}/image_rect"),
                        (
                            "image_rect/compressed",
                            f"{name}/{color_sens_name}/image_rect/compressed",
                        ),
                        (
                            "image_rect/compressedDepth",
                            f"{name}/{color_sens_name}/image_rect/compressedDepth",
                        ),
                        (
                            "image_rect/theora",
                            f"{name}/{color_sens_name}/image_rect/theora",
                        ),
                    ],
                )
            ],
        ),
        LoadComposableNodes(
            condition=IfCondition(pointcloud_enable),
            target_container=f"{namespace}/{name}_container",
            composable_node_descriptions=[
                ComposableNode(
                    package="depth_image_proc",
                    plugin="depth_image_proc::PointCloudXyzrgbNode",
                    name="point_cloud_xyzrgb_node",
                    namespace=namespace,
                    remappings=[
                        (
                            "depth_registered/image_rect",
                            f"{name}/{stereo_sens_name}/image_raw",
                        ),
                        (
                            "rgb/image_rect_color",
                            f"{name}/{color_sens_name}/image_rect",
                        ),
                        ("rgb/camera_info", f"{name}/{color_sens_name}/camera_info"),
                        ("points", points_topic_name),
                    ],
                ),
            ],
        ),
    ]


def generate_launch_description():
    depthai_prefix = get_package_share_directory("depthai_ros_driver")

    declared_arguments = [
        DeclareLaunchArgument("name", default_value="oak"),
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("parent_frame", default_value="oak-d-base-frame"),
        DeclareLaunchArgument("camera_model", default_value="OAK-D"),
        DeclareLaunchArgument("cam_pos_x", default_value="0.0"),
        DeclareLaunchArgument("cam_pos_y", default_value="0.0"),
        DeclareLaunchArgument("cam_pos_z", default_value="0.0"),
        DeclareLaunchArgument("cam_roll", default_value="0.0"),
        DeclareLaunchArgument("cam_pitch", default_value="0.0"),
        DeclareLaunchArgument("cam_yaw", default_value="0.0"),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(depthai_prefix, "config", "camera.yaml"),
        ),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=os.path.join(depthai_prefix, "config", "rviz", "rgbd.rviz"),
        ),
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
        DeclareLaunchArgument("use_gdb", default_value="false"),
        DeclareLaunchArgument("use_valgrind", default_value="false"),
        DeclareLaunchArgument("use_perf", default_value="false"),
        DeclareLaunchArgument(
            "rs_compat",
            default_value="false",
            description="Enables compatibility with RealSense nodes.",
        ),
        DeclareLaunchArgument("rectify_rgb", default_value="true"),
        DeclareLaunchArgument("pointcloud.enable", default_value="false"),
        DeclareLaunchArgument("enable_color", default_value="true"),
        DeclareLaunchArgument("enable_depth", default_value="true"),
        DeclareLaunchArgument("enable_infra1", default_value="false"),
        DeclareLaunchArgument("enable_infra2", default_value="false"),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )

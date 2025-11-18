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
from launch.events import Shutdown


def is_launch_config_true(context, name):
    return LaunchConfiguration(name).perform(context) == "true"


def setup_launch_prefix(context, *args, **kwargs):
    use_gdb = LaunchConfiguration("use_gdb", default="false")
    use_valgrind = LaunchConfiguration("use_valgrind", default="false")
    valgrind_args = LaunchConfiguration(
        "valgrind_args",
        default="--leak-check=full --show-leak-kinds=all --track-origins=yes --verbose --log-file=memcheck.log",
    )
    use_perf = LaunchConfiguration("use_perf", default="false")

    launch_prefix = ""

    if use_gdb.perform(context) == "true":
        launch_prefix += "xterm -e gdb -ex run --args"
    if use_valgrind.perform(context) == "true":
        launch_prefix += f"valgrind {valgrind_args.perform(context)}"
        print(launch_prefix)
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
        "parent_frame", default="oak_parent_frame"
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
        "publish_tf_from_calibration", default="true"
    )
    override_cam_model = LaunchConfiguration("override_cam_model", default="false")
    params_file = LaunchConfiguration("params_file")
    camera_model = LaunchConfiguration("camera_model", default="OAK-D")
    rs_compat = LaunchConfiguration("rs_compat", default="false")
    pointcloud_enable = LaunchConfiguration("pointcloud.enable", default="false")
    namespace = LaunchConfiguration("namespace", default="").perform(context)
    name = LaunchConfiguration("name").perform(context)

    # If RealSense compatibility is enabled, we need to override some parameters, topics and node names
    parameter_overrides = {}
    points_topic_name = f"{name}/rgbd/points"
    if rs_compat.perform(context) == "true":
        depth_profile = LaunchConfiguration("depth_module.depth_profile").perform(
            context
        )
        color_profile = LaunchConfiguration("rgb_camera.color_profile").perform(context)
        infra_profile = LaunchConfiguration("depth_module.infra_profile").perform(
            context
        )
        # split profile string (0,0,0 or 0x0x0 or 0X0X0) into with (int) height(int) and fps(double)
        # find delimiter
        delimiter = ","
        if "x" in depth_profile:
            delimiter = "x"
        elif "X" in depth_profile:
            delimiter = "X"
        depth_profile = depth_profile.split(delimiter)
        color_profile = color_profile.split(delimiter)
        infra_profile = infra_profile.split(delimiter)

        if name == "oak":
            name = "camera"
        points_topic_name = f"{name}/depth/color/points"
        if namespace == "":
            namespace = "camera"
        if parent_frame == "oak_parent_frame":
            parent_frame = f"{name}_link"
        parameter_overrides = {
            "driver": {
                "i_rs_compat": True,
            },
            "pipeline_gen": {
                "i_enable_sync": True,
            },
            "color": {
                "i_publish_topic": is_launch_config_true(context, "enable_color"),
                "i_synced": True,
                "i_width": int(color_profile[0]),
                "i_height": int(color_profile[1]),
                "i_fps": float(color_profile[2]),
            },
            "depth": {
                "i_publish_topic": is_launch_config_true(context, "enable_depth"),
                "i_synced": True,
                "i_subpixel": True,
                "i_width": int(depth_profile[0]),
                "i_height": int(depth_profile[1]),
                "i_fps": float(depth_profile[2]),
                "i_left_rect_publish_topic": is_launch_config_true(
                    context, "enable_infra1"
                ),
                "i_right_rect_publish_topic": is_launch_config_true(
                    context, "enable_infra2"
                ),
            },
            "infra1": {
                "i_width": int(infra_profile[0]),
                "i_height": int(infra_profile[1]),
                "i_fps": float(infra_profile[2]),
            },
            "infra2": {
                "i_width": int(infra_profile[0]),
                "i_height": int(infra_profile[1]),
                "i_fps": float(infra_profile[2]),
            },
        }
        if pointcloud_enable.perform(context) == "true":
            parameter_overrides["pipeline_gen"]["i_enable_rgbd"] = True

    params = {}
    if publish_tf_from_calibration.perform(context) == "true":
        cam_model = ""
        if override_cam_model.perform(context) == "true":
            cam_model = camera_model.perform(context)
        params = {
            "driver": {
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
    else:
        params = {"driver": {"i_publish_tf_from_calibration": False}}
    if pointcloud_enable.perform(context) == "true":
        params["pipeline_gen"] = {"i_enable_rgbd": True}

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
                    plugin="depthai_ros_driver::Driver",
                    name=name,
                    namespace=namespace,
                    parameters=[
                        params_file,
                        params,
                        parameter_overrides,
                    ],
                    remappings=[(f"{name}/rgbd/points", points_topic_name)],
                )
            ],
            arguments=["--ros-args", "--log-level", log_level],
            prefix=[launch_prefix],
            output="both",
        ),
    ]


def generate_launch_description():
    depthai_prefix = get_package_share_directory("depthai_ros_driver")

    declared_arguments = [
        DeclareLaunchArgument("name", default_value="oak"),
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("parent_frame", default_value="oak_parent_frame"),
        DeclareLaunchArgument("camera_model", default_value="OAK-D-PRO"),
        DeclareLaunchArgument("cam_pos_x", default_value="0.0"),
        DeclareLaunchArgument("cam_pos_y", default_value="0.0"),
        DeclareLaunchArgument("cam_pos_z", default_value="0.0"),
        DeclareLaunchArgument("cam_roll", default_value="0.0"),
        DeclareLaunchArgument("cam_pitch", default_value="0.0"),
        DeclareLaunchArgument("cam_yaw", default_value="0.0"),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(depthai_prefix, "config", "driver.yaml"),
        ),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=os.path.join(depthai_prefix, "config", "rviz", "rgbd.rviz"),
        ),
        DeclareLaunchArgument("rsp_use_composition", default_value="true"),
        DeclareLaunchArgument(
            "publish_tf_from_calibration",
            default_value="true",
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
        DeclareLaunchArgument(
            "valgrind_args",
            default_value="--leak-check=full --show-leak-kinds=all --track-origins=yes --verbose --log-file=memcheck.log",
        ),
        DeclareLaunchArgument("use_perf", default_value="false"),
        DeclareLaunchArgument(
            "rs_compat",
            default_value="false",
            description="Enables compatibility with RealSense nodes.",
        ),
        DeclareLaunchArgument("pointcloud.enable", default_value="false"),
        DeclareLaunchArgument("enable_color", default_value="true"),
        DeclareLaunchArgument("enable_depth", default_value="true"),
        DeclareLaunchArgument("enable_infra1", default_value="false"),
        DeclareLaunchArgument("enable_infra2", default_value="false"),
        DeclareLaunchArgument("depth_module.depth_profile", default_value="640,400,30"),
        DeclareLaunchArgument("rgb_camera.color_profile", default_value="640,400,30"),
        DeclareLaunchArgument("depth_module.infra_profile", default_value="640,400,30"),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )

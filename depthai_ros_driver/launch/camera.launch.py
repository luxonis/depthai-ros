import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):
    log_level = 'info'
    if(context.environment.get('DEPTHAI_DEBUG')=='1'):
        log_level='debug'

    

    urdf_launch_dir = os.path.join(get_package_share_directory('depthai_descriptions'), 'launch')
    
    params_file = LaunchConfiguration("params_file")
    camera_model = LaunchConfiguration('camera_model',  default = 'OAK-D')

    name = LaunchConfiguration('name').perform(context)

    parent_frame = LaunchConfiguration('parent_frame',  default = 'oak-d-base-frame')
    cam_pos_x    = LaunchConfiguration('cam_pos_x',     default = '0.0')
    cam_pos_y    = LaunchConfiguration('cam_pos_y',     default = '0.0')
    cam_pos_z    = LaunchConfiguration('cam_pos_z',     default = '0.0')
    cam_roll     = LaunchConfiguration('cam_roll',      default = '0.0')
    cam_pitch    = LaunchConfiguration('cam_pitch',     default = '0.0')
    cam_yaw      = LaunchConfiguration('cam_yaw',       default = '0.0')
    use_composition = LaunchConfiguration('rsp_use_composition', default='true')
    imu_from_descr = LaunchConfiguration('imu_from_descr', default='false')
    publish_tf_from_calibration = LaunchConfiguration('publish_tf_from_calibration', default='false')
    override_cam_model = LaunchConfiguration('override_cam_model', default='false')

    tf_params = {}
    if(publish_tf_from_calibration.perform(context) == 'true'):
        cam_model = ''
        if override_cam_model.perform(context) == 'true':
            cam_model = camera_model.perform(context)
        tf_params = {'camera': {
            'i_publish_tf_from_calibration': True,
            'i_tf_tf_prefix': name,
            'i_tf_camera_model': cam_model,
            'i_tf_base_frame': name,
            'i_tf_parent_frame': parent_frame.perform(context),
            'i_tf_cam_pos_x': cam_pos_x.perform(context),
            'i_tf_cam_pos_y': cam_pos_y.perform(context),
            'i_tf_cam_pos_z': cam_pos_z.perform(context),
            'i_tf_cam_roll': cam_roll.perform(context),
            'i_tf_cam_pitch': cam_pitch.perform(context),
            'i_tf_cam_yaw': cam_yaw.perform(context),
            'i_tf_imu_from_descr': imu_from_descr.perform(context),
        }
        }
    
    use_gdb      = LaunchConfiguration('use_gdb',       default = 'false')
    use_valgrind = LaunchConfiguration('use_valgrind',  default = 'false')
    use_perf     = LaunchConfiguration('use_perf',      default = 'false')

    launch_prefix = ''

    if (use_gdb.perform(context) == 'true'):
        launch_prefix += "gdb -ex run --args "
    if (use_valgrind.perform(context) == 'true'):
        launch_prefix += "valgrind --tool=callgrind"
    if (use_perf.perform(context) == 'true'):
        launch_prefix += "perf record -g --call-graph dwarf --output=perf.out.node_name.data --"
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
                os.path.join(urdf_launch_dir, 'urdf_launch.py')),
            launch_arguments={'tf_prefix': name,
                              'camera_model': camera_model,
                              'base_frame': name,
                              'parent_frame': parent_frame,
                              'cam_pos_x': cam_pos_x,
                              'cam_pos_y': cam_pos_y,
                              'cam_pos_z': cam_pos_z,
                              'cam_roll': cam_roll,
                              'cam_pitch': cam_pitch,
                              'cam_yaw': cam_yaw,
                              'use_composition': use_composition,
                              'use_base_descr': publish_tf_from_calibration}.items()),

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
                        parameters=[params_file, tf_params],
                    )
            ],
            arguments=['--ros-args', '--log-level', log_level],
            prefix=[launch_prefix],
            output="both",
        ),

    ]


def generate_launch_description():
    depthai_prefix = get_package_share_directory("depthai_ros_driver")

    declared_arguments = [
        DeclareLaunchArgument("name", default_value="oak"),
        DeclareLaunchArgument("parent_frame", default_value="oak-d-base-frame"),
        DeclareLaunchArgument("camera_model", default_value="OAK-D"),
        DeclareLaunchArgument("cam_pos_x", default_value="0.0"),
        DeclareLaunchArgument("cam_pos_y", default_value="0.0"),
        DeclareLaunchArgument("cam_pos_z", default_value="0.0"),
        DeclareLaunchArgument("cam_roll", default_value="0.0"),
        DeclareLaunchArgument("cam_pitch", default_value="0.0"),
        DeclareLaunchArgument("cam_yaw", default_value="0.0"),
        DeclareLaunchArgument("params_file", default_value=os.path.join(depthai_prefix, 'config', 'camera.yaml')),
        DeclareLaunchArgument("use_rviz", default_value='false'),
        DeclareLaunchArgument("rviz_config", default_value=os.path.join(depthai_prefix, "config", "rviz", "rgbd.rviz")),
        DeclareLaunchArgument("rsp_use_composition", default_value='true'),
        DeclareLaunchArgument("publish_tf_from_calibration", default_value='false', description='Enables TF publishing from camera calibration file.'),
        DeclareLaunchArgument("imu_from_descr", default_value='false', description='Enables IMU publishing from URDF.'),
        DeclareLaunchArgument("override_cam_model", default_value='false', description='Overrides camera model from calibration file.'),
        DeclareLaunchArgument("use_gdb", default_value='false'),
        DeclareLaunchArgument("use_valgrind", default_value='false'),
        DeclareLaunchArgument("use_perf", default_value='false')
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )




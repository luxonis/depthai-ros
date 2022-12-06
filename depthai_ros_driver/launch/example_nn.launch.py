import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):

    depthai_prefix = get_package_share_directory("depthai_ros_driver")

    urdf_launch_dir = os.path.join(get_package_share_directory('depthai_bridge'), 'launch')

    nn_config_path = os.path.join(depthai_prefix, "config", "nn", LaunchConfiguration(
        "nn_type").perform(context)+".json")
    mxid = LaunchConfiguration('mxid').perform(context)
    camera_model = LaunchConfiguration('camera_model',  default = 'OAK-D')
    tf_prefix    = LaunchConfiguration('tf_prefix',     default = 'oak')
    tf_prefix_str = LaunchConfiguration('tf_prefix').perform(context)
    parent_frame = LaunchConfiguration('parent_frame',  default = 'oak-d-base-frame')
    cam_pos_x    = LaunchConfiguration('cam_pos_x',     default = '0.0')
    cam_pos_y    = LaunchConfiguration('cam_pos_y',     default = '0.0')
    cam_pos_z    = LaunchConfiguration('cam_pos_z',     default = '0.0')
    cam_roll     = LaunchConfiguration('cam_roll',      default = '0.0')
    cam_pitch    = LaunchConfiguration('cam_pitch',     default = '0.0')
    cam_yaw      = LaunchConfiguration('cam_yaw',       default = '0.0')
    
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(urdf_launch_dir, 'urdf_launch.py')),
            launch_arguments={'tf_prefix': tf_prefix,
                              'camera_model': camera_model,
                              'base_frame': tf_prefix,
                              'parent_frame': parent_frame,
                              'cam_pos_x': cam_pos_x,
                              'cam_pos_y': cam_pos_y,
                              'cam_pos_z': cam_pos_z,
                              'cam_roll': cam_roll,
                              'cam_pitch': cam_pitch,
                              'cam_yaw': cam_yaw}.items()),

        ComposableNodeContainer(
            name=tf_prefix_str+"_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=[

                    ComposableNode(
                        package="depthai_ros_driver",
                        plugin="depthai_ros_driver::Camera",
                        name=tf_prefix_str,
                        parameters=[{
                            "camera.i_mx_id": mxid,
                            "nn.i_nn_config_path": nn_config_path
                            }],
                    ),
            ],
            output="screen",
        ),
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("nn_type", default_value="segmentation"),
        DeclareLaunchArgument("tf_prefix", default_value="oak"),
        DeclareLaunchArgument("use_rviz", default_value="False"),
        DeclareLaunchArgument("parent_frame", default_value="oak-d-base-frame"),
        DeclareLaunchArgument("mxid", default_value=""),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )

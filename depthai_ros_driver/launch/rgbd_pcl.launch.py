import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch.conditions import IfCondition


def launch_setup(context, *args, **kwargs):
    params_file = LaunchConfiguration("params_file")
    urdf_launch_dir = os.path.join(get_package_share_directory('depthai_bridge'), 'launch')
    rviz_config = os.path.join(get_package_share_directory("depthai_ros_driver"), "config", "rviz", "rgbd.rviz")
    
    camera_model = LaunchConfiguration('camera_model',  default = 'OAK-D')

    name = LaunchConfiguration('name').perform(context)

    parent_frame = LaunchConfiguration('parent_frame',  default = 'oak-d-base-frame')
    cam_pos_x    = LaunchConfiguration('cam_pos_x',     default = '0.0')
    cam_pos_y    = LaunchConfiguration('cam_pos_y',     default = '0.0')
    cam_pos_z    = LaunchConfiguration('cam_pos_z',     default = '0.0')
    cam_roll     = LaunchConfiguration('cam_roll',      default = '0.0')
    cam_pitch    = LaunchConfiguration('cam_pitch',     default = '0.0')
    cam_yaw      = LaunchConfiguration('cam_yaw',       default = '0.0')
    
    
    return [
        Node(
                condition=IfCondition(LaunchConfiguration("use_rviz")),
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="log",
                arguments=["-d", rviz_config],
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
                              'cam_yaw': cam_yaw}.items()),
        ComposableNodeContainer(
            name=name+"_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=[
                    ComposableNode(
                        package="depth_image_proc",
                        plugin="depth_image_proc::ConvertMetricNode",
                        name="convert_metric_node",
                        remappings=[('image_raw', name+'/stereo/image_raw'),
                                            ('camera_info', name+'/stereo/camera_info'),
                                            ('image', name+'/stereo/converted_depth')]
                    ),
                    ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzrgbNode',
                    name='point_cloud_xyzrgb_node',
                    remappings=[('depth_registered/image_rect', name+'/stereo/converted_depth'),
                                ('rgb/image_rect_color', name+'/rgb/image_raw'),
                                ('rgb/camera_info', name+'/rgb/camera_info'),
                                ('points', name+'/points')]
                    ),
                    ComposableNode(
                        package="depthai_ros_driver",
                        plugin="depthai_ros_driver::Camera",
                        name=name,
                        parameters=[params_file],
                    ),
            ],
            output="screen",
        ),

    ]


def generate_launch_description():
    depthai_prefix = get_package_share_directory("depthai_ros_driver")
    declared_arguments = [
        DeclareLaunchArgument("name", default_value="oak"),
        DeclareLaunchArgument("parent_frame", default_value="oak-d-base-frame"),
        DeclareLaunchArgument("cam_pos_x", default_value="0.0"),
        DeclareLaunchArgument("cam_pos_y", default_value="0.0"),
        DeclareLaunchArgument("cam_pos_z", default_value="0.0"),
        DeclareLaunchArgument("cam_roll", default_value="0.0"),
        DeclareLaunchArgument("cam_pitch", default_value="0.0"),
        DeclareLaunchArgument("cam_yaw", default_value="0.0"),
        DeclareLaunchArgument("params_file", default_value=os.path.join(depthai_prefix, 'config', 'rgbd.yaml')),
        DeclareLaunchArgument("use_rviz", default_value="False"),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )

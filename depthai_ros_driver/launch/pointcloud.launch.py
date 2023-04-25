import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):
    params_file = LaunchConfiguration("params_file")
    depthai_prefix = get_package_share_directory("depthai_ros_driver")

    name = LaunchConfiguration('name').perform(context)
    
    return [

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(depthai_prefix, 'launch', 'camera.launch.py')),
            launch_arguments={"name": name,
                              "params_file": params_file,
                              "parent_frame": LaunchConfiguration("parent_frame"),
                               "cam_pos_x": LaunchConfiguration("cam_pos_x"),
                               "cam_pos_y": LaunchConfiguration("cam_pos_y"),
                               "cam_pos_z": LaunchConfiguration("cam_pos_z"),
                               "cam_roll": LaunchConfiguration("cam_roll"),
                               "cam_pitch": LaunchConfiguration("cam_pitch"),
                               "cam_yaw": LaunchConfiguration("cam_yaw"),
                               "use_rviz": LaunchConfiguration("use_rviz")
                               }.items()),

        LoadComposableNodes(
            target_container=name+"_container",
            composable_node_descriptions=[
                    ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyziNode',
                    name='point_cloud_xyzi',

                    remappings=[('depth/image_rect', name+'/stereo/image_raw'),
                                ('intensity/image_rect', name+'/right/image_raw'),
                                ('intensity/camera_info', name+'/right/camera_info'),
                                ('points', name+'/points')
                                ]),
            ],
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
        DeclareLaunchArgument("params_file", default_value=os.path.join(depthai_prefix, 'config', 'pcl.yaml')),
        DeclareLaunchArgument("use_rviz", default_value="False"),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, ComposableNodeContainer

from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):

    depthai_prefix = get_package_share_directory("depthai_ros_driver")
    rviz_config = os.path.join(depthai_prefix, "config", "multicam.rviz")
    # put mx_ids here
    cams = {"oak_d_w": {"mxid": "", "nn": "segmentation"},
            "oak_d_lite": {"mxid": "", "nn": "yolo"}, 
            "oak_d_pro": {"mxid": "", "nn": "mobilenet"}}
    nodes = []
    for cam_name, info in cams.items():
        mxid = info["mxid"]
        nn = info["nn"]
        node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(depthai_prefix, 'launch', 'example_nn.launch.py')),
                launch_arguments={'tf_prefix': cam_name,
                            'mxid': mxid,
                            'nn_family': nn
                            'nn_type': 'rgb'}.items())
                            
        nodes.append(node)
    
    rviz = Node(
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
    )

    nodes.append(rviz)
    return nodes


def generate_launch_description():
    declared_arguments = [DeclareLaunchArgument("use_rviz", default_value="False")]

    return LaunchDescription(
        declared_arguments
        + [
            OpaqueFunction(function=launch_setup),
        ]
    )
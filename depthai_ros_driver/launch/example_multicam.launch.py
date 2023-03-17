import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node



def launch_setup(context, *args, **kwargs):

    depthai_prefix = get_package_share_directory("depthai_ros_driver")
    params_file = os.path.join(depthai_prefix, "config", "multicam_example.yaml")
    # put mx_ids here
    cams = ["oak_d_w", "oak_d_lite"]
    nodes = []
    i=0.0
    for cam_name in cams:
        node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(depthai_prefix, 'launch', 'camera.launch.py')),
            launch_arguments={"name": cam_name,
                              "parent_frame": "map",
                              "params_file": params_file,
                              "cam_pos_y": str(i)}.items())
        nodes.append(node)
        i=i+0.1
    spatial_rgbd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(depthai_prefix, 'launch',
                         'rgbd_pcl.launch.py')
        ),
        launch_arguments={"name": "oak_d_pro",
                          "parent_frame": "map",
                          "params_file": params_file,
                          "cam_pos_y": str(-0.1)}.items())

    obj_det = Node(
        package="depthai_ros_driver",
        executable="obj_pub.py",
        remappings=[("/oak/nn/detections", "/oak_d_pro/nn/detections"),
        ("/oak/nn/detection_markers", "/oak_d_pro/nn/detection_markers")]
    )

    nodes.append(spatial_rgbd)
    nodes.append(obj_det)
    return nodes


def generate_launch_description():

    return LaunchDescription(
        [
            OpaqueFunction(function=launch_setup),
        ]
    )

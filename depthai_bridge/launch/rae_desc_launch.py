from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    bringup_dir = get_package_share_directory('depthai_bridge')
    xacro_path = os.path.join(bringup_dir, 'urdf', 'rae.urdf.xacro')


    rsp_node =  Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': Command(
                [
                    'xacro', ' ', xacro_path, ' ',
                ])}]
        )

    joint_state_publisher = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher')


    ld = LaunchDescription()
    ld.add_action(joint_state_publisher)
    ld.add_action(rsp_node)
    return ld

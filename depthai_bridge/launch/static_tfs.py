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
                    # arguments = ['--x', '0.0', '--y', '0.0', '--z', '2.0', '--roll', '-1.5708', '--pitch', '-1.5708', '--yaw', '1.5708', '--frame-id', 'world', '--child-frame-id', 'imu']

    return LaunchDescription([
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    arguments = ['--x', '0.0', '--y', '0.0', '--z', '0.0', '--roll', '3.14159', '--pitch', '1.5708', '--yaw', '0', '--frame-id', 'world', '--child-frame-id', 'imu']
                ),
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    arguments = ['--x', '0.00079', '--y', '-0.075448', '--z', '0.0', '--roll', '3.14159', '--pitch', '0', '--yaw', '1.5708', '--frame-id', 'imu', '--child-frame-id', 'left_camera']
                ),
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    arguments = ['--x', '0.00079', '--y', '0.0000448', '--z', '0.0', '--roll', '3.14159', '--pitch', '0', '--yaw', '1.5708', '--frame-id', 'imu', '--child-frame-id', 'right_camera']
                ),

                ## ------------ EUROC MODEL-----------
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    arguments = ['--x', '-0.0216401454975', '--y', '-0.064676986768', '--z', '0.00981073058949', '--roll', '0.0037574', '--pitch', '0.0257773', '--yaw', '1.5559253', '--frame-id', 'euroc', '--child-frame-id', 'euroc_left']
                ),

                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    arguments = ['--x', '-0.0198435579556', '--y', '0.0453689425024', '--z', '0.00786212447038', '--roll', '0.0179073', '--pitch', '0.0253925', '--yaw', '1.5582367', '--frame-id', 'euroc', '--child-frame-id', 'euroc_right']
                ),

                ## ------------ REALSENSE MODEL-----------
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    arguments = ['--x', '-0.00107491', '--y', '0.00339225', '--z', '0.02843741', '--roll', '0.0105477', '--pitch', '0.0002774', '--yaw', '0.0045972', '--frame-id', 'realsense', '--child-frame-id', 'realsense_left']
                ),

                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    arguments = ['--x', '0.04882315', '--y', '0.00363688', '--z', '0.02863922', '--roll', '0.0104211', '--pitch', '-0.0025562', '--yaw', '0.0045419', '--frame-id', 'realsense', '--child-frame-id', 'realsense_right']
                ),

                ## OAS-D NON Image Frame
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    arguments = ['--x', '0.0', '--y', '0.0', '--z', '0.0', '--roll', '-1.571', '--pitch', '1.571', '--yaw', '-3.142', '--frame-id', 'world_oak', '--child-frame-id', 'imu_oak']
                ),

                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    arguments = ['--x', '-0.075', '--y', '0.0', '--z', '0.00079', '--roll', '0', '--pitch', '0', '--yaw', '0', '--frame-id', 'world_oak', '--child-frame-id', 'left_camera_oak']
                ),

                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    arguments = ['--x', '0.0', '--y', '0.0', '--z', '0.00079', '--roll', '0', '--pitch', '0', '--yaw', '0', '--frame-id', 'world_oak', '--child-frame-id', 'right_camera_oak']
                )

            ])

#  [-90.000, 90.000, -180.000]
# - Rotation: in RPY (radian) [-1.571, 1.571, -3.142]

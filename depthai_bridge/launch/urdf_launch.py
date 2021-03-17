from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    bringup_dir = get_package_share_directory('depthai_bridge')
    urdf_path = os.path.join(bringup_dir, 'urdf', 'depthai_descr.urdf')
    urdf = open(urdf_path).read()
    # doc = xacro.process_file(urdf_path, mappings={'simulate_obstacles' : 'false'})

    return launch.LaunchDescription([
        Node(
            name='robot_state_publisher',
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': urdf}]
        )
    ])

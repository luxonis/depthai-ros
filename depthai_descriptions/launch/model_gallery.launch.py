"""Show all supported model/chip descriptions without camera hardware."""
import importlib.util
import json
import tempfile
import yaml
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.conditions import IfCondition
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context):
    share = Path(get_package_share_directory('depthai_descriptions'))
    spec = importlib.util.spec_from_file_location('model_gallery', share / 'scripts/model_gallery.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    values = {name: LaunchConfiguration(name).perform(context)
              for name in ('models', 'pose_source', 'imu_frame', 'columns', 'spacing', 'scale')}
    values['columns'], values['spacing'] = int(values['columns']), float(values['spacing'])
    gallery = module.build_gallery(**values, share=share)
    for notice in gallery['notices']:
        get_logger('model_gallery').warning(notice)
    get_logger('model_gallery').info(f"Displaying {len(gallery['entries'])} model/IMU entries; cyan=cameras, magenta=IMU; axes: X red, Y green, Z blue.")
    # Keep each launch's fitted RViz configuration isolated and remove it on shutdown.
    with tempfile.NamedTemporaryFile(mode='w', suffix='.rviz', prefix='model_gallery_', delete=False) as output:
        yaml.safe_dump(module.rviz_config(gallery, share / 'config/model_gallery.rviz'), output)
        rviz_path = output.name
    get_logger('model_gallery').info(f"Display scale: {gallery['scale']:.3g}x (visualization only; gallery TF distances are scaled).")
    return [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=lambda context: Path(rviz_path).unlink(missing_ok=True))])),
        Node(package='robot_state_publisher', executable='robot_state_publisher', name='model_gallery_state_publisher',
             parameters=[{'robot_description': ParameterValue(gallery.pop('urdf'), value_type=str)}],
             remappings=[('robot_description', '/model_gallery/robot_description')]),
        Node(package='depthai_descriptions', executable='model_gallery.py', name='model_gallery_markers',
             parameters=[{'manifest': ParameterValue(json.dumps(gallery), value_type=str),
                          'axis_length': ParameterValue(LaunchConfiguration('axis_length'), value_type=float)}]),
        Node(package='rviz2', executable='rviz2', name='model_gallery_rviz',
             arguments=['-d', rviz_path],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('models', default_value='', description='Comma/space-separated models; empty shows every supported model and IMU variant.'),
        DeclareLaunchArgument('pose_source', default_value='board', choices=['board', 'legacy']),
        DeclareLaunchArgument('imu_frame', default_value='auto', choices=['auto', 'native', 'optical']),
        DeclareLaunchArgument('columns', default_value='8'),
        DeclareLaunchArgument('scale', default_value='auto', description='Display scale: auto adapts to entry count; 1 preserves real dimensions.'),
        DeclareLaunchArgument('spacing', default_value='0.75', description='Grid spacing in metres before display scaling.'),
        DeclareLaunchArgument('axis_length', default_value='0.04', description='Sensor-axis length in metres before display scaling.'),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=launch_setup),
    ])

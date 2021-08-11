from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    bringup_dir = get_package_share_directory('depthai_bridge')
    xacro_path = os.path.join(bringup_dir, 'urdf', 'depthai_descr.urdf.xacro')
    # urdf = open(urdf_path).read()
    # doc = xacro.process_file(urdf_path, mappings={'simulate_obstacles' : 'false'})
    print(xacro_path)
    camera_name = LaunchConfiguration('camera_name')
    camera_model = LaunchConfiguration('camera_model')
    base_frame = LaunchConfiguration('base_frame')
    cam_pos_x = LaunchConfiguration('cam_pos_x')
    cam_pos_y = LaunchConfiguration('cam_pos_y')
    cam_pos_z = LaunchConfiguration('cam_pos_z')
    cam_roll = LaunchConfiguration('cam_roll')
    cam_pitch = LaunchConfiguration('cam_pitch')
    cam_yaw = LaunchConfiguration('cam_yaw')

    declare_camera_name_cmd = DeclareLaunchArgument(
        'camera_name',
        default_value='oak-d',
        description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`.')

    declare_camera_model_cmd = DeclareLaunchArgument(
        'camera_model',
        default_value='BW1098OAK',
        description='The model of the camera. Using a wrong camera model can disable camera features. Valid models: `BW1098OAK`.')

    # declare_config_common_path_cmd = DeclareLaunchArgument(
    #     'config_common_path',
    #     default_value=default_config_common,
    #     description='Path to the `common.yaml` file.')

    # declare_config_camera_path_cmd = DeclareLaunchArgument(
    #     'config_camera_path',
    #     description='Path to the `<camera_model>.yaml` file.')

    # declare_publish_urdf_cmd = DeclareLaunchArgument(
    #     'publish_urdf',
    #     default_value='true',
    #     description='Enable URDF processing and starts Robot State Published to propagate static TF.')

    # declare_xacro_path_cmd = DeclareLaunchArgument(
    #     'xacro_path',
    #     description='Path to the camera URDF file as a xacro file.')

    # declare_svo_path_cmd = DeclareLaunchArgument(
    #     'svo_path',
    #     default_value='',
    #     description='Path to an input SVO file. Note: overrides the parameter `general.svo_file` in `common.yaml`.')

    declare_base_frame_cmd = DeclareLaunchArgument(
        'base_frame',
        default_value='oak-d_frame',
        description='Name of the base link.')

    declare_pos_x_cmd = DeclareLaunchArgument(
        'cam_pos_x',
        default_value='0.0',
        description='Position X of the camera with respect to the base frame.')

    declare_pos_y_cmd = DeclareLaunchArgument(
        'cam_pos_y',
        default_value='0.0',
        description='Position Y of the camera with respect to the base frame.')

    declare_pos_z_cmd = DeclareLaunchArgument(
        'cam_pos_z',
        default_value='0.0',
        description='Position Z of the camera with respect to the base frame.')

    declare_roll_cmd = DeclareLaunchArgument(
        'cam_roll',
        default_value='1.5708',
        description='Roll orientation of the camera with respect to the base frame.')

    declare_pitch_cmd = DeclareLaunchArgument(
        'cam_pitch',
        default_value='0.0',
        description='Pitch orientation of the camera with respect to the base frame.')

    declare_yaw_cmd = DeclareLaunchArgument(
        'cam_yaw',
        default_value='1.5708',
        description='Yaw orientation of the camera with respect to the base frame.')

    rsp_node =  Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='oak_state_publisher',
            parameters=[{'robot_description': Command(
                [
                    'xacro', ' ', xacro_path, ' ',
                    'camera_name:=', camera_name, ' ',
                    'camera_model:=', camera_model, ' ',
                    'base_frame:=', base_frame, ' ',
                    'cam_pos_x:=', cam_pos_x, ' ',
                    'cam_pos_y:=', cam_pos_y, ' ',
                    'cam_pos_z:=', cam_pos_z, ' ',
                    'cam_roll:=', cam_roll, ' ',
                    'cam_pitch:=', cam_pitch, ' ',
                    'cam_yaw:=', cam_yaw
                ])}]
        )
    
    ld = LaunchDescription()
    ld.add_action(declare_camera_name_cmd)
    ld.add_action(declare_camera_model_cmd)
    ld.add_action(declare_base_frame_cmd)
    ld.add_action(declare_pos_x_cmd)
    ld.add_action(declare_pos_y_cmd)
    ld.add_action(declare_pos_z_cmd)
    ld.add_action(declare_roll_cmd)
    ld.add_action(declare_pitch_cmd)
    ld.add_action(declare_yaw_cmd)

    ld.add_action(rsp_node)
    return ld


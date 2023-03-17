import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, launch_description_sources
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
import launch_ros.actions
import launch_ros.descriptions


def generate_launch_description():
    default_rviz = os.path.join(get_package_share_directory('depthai_examples'),
                                'rviz', 'pointCloud.rviz')
    urdf_launch_dir = os.path.join(get_package_share_directory('depthai_descriptions'), 'launch')

    tf_prefix    = LaunchConfiguration('tf_prefix',    default = 'oak')
    camera_model = LaunchConfiguration('camera_model', default = 'OAK-D')
    base_frame   = LaunchConfiguration('base_frame',   default = 'oak-d_frame')
    parent_frame = LaunchConfiguration('parent_frame', default = 'oak-d-base-frame')
    publish_urdf = LaunchConfiguration('publish_urdf', default = True)

    cam_pos_x = LaunchConfiguration('cam_pos_x',       default = '0.0')
    cam_pos_y = LaunchConfiguration('cam_pos_y',       default = '0.0')
    cam_pos_z = LaunchConfiguration('cam_pos_z',       default = '0.0')
    cam_roll  = LaunchConfiguration('cam_roll',        default = '0.0')
    cam_pitch = LaunchConfiguration('cam_pitch',       default = '0.0')
    cam_yaw   = LaunchConfiguration('cam_yaw',         default = '0.0')

    lrcheck      = LaunchConfiguration('lrcheck',      default = True)
    extended     = LaunchConfiguration('extended',     default = False)
    subpixel     = LaunchConfiguration('subpixel',     default = True)
    confidence   = LaunchConfiguration('confidence',   default = 200)
    LRchecktresh = LaunchConfiguration('LRchecktresh', default = 5)

    colorResolution = LaunchConfiguration('colorResolution', default = "1080p")
    useVideo        = LaunchConfiguration('useVideo',        default = True)
    usePreview      = LaunchConfiguration('usePreview',      default = False)
    useDepth        = LaunchConfiguration('useDepth',        default = True)
    previewWidth    = LaunchConfiguration('previewWidth',    default = 300)
    previewHeight   = LaunchConfiguration('previewHeight',   default = 300)

    # IR Brightness. OAK-D-Pro only.
    dotProjectormA   = LaunchConfiguration('dotProjectormA',     default = 0.0)
    floodLightmA     = LaunchConfiguration('floodLightmA',       default = 0.0)

    declare_camera_model_cmd = DeclareLaunchArgument(
        'camera_model',
        default_value=camera_model,
        description='The model of the camera. Using a wrong camera model can disable camera features. Valid models: `OAK-D, OAK-D-LITE`.')

    declare_tf_prefix_cmd = DeclareLaunchArgument(
        'tf_prefix',
        default_value=tf_prefix,
        description='The name of the camera. It can be different from the camera model and it will be used in naming TF.')

    declare_base_frame_cmd = DeclareLaunchArgument(
        'base_frame',
        default_value=base_frame,
        description='Name of the base link.')

    declare_parent_frame_cmd = DeclareLaunchArgument(
        'parent_frame',
        default_value=parent_frame,
        description='Name of the parent link from other a robot TF for example that can be connected to the base of the OAK.')

    declare_publish_urdf_cmd = DeclareLaunchArgument(
        'publish_urdf',
        default_value=publish_urdf,
        description='Whether to publish the urdf')

    declare_pos_x_cmd = DeclareLaunchArgument(
        'cam_pos_x',
        default_value=cam_pos_x,
        description='Position X of the camera with respect to the base frame.')

    declare_pos_y_cmd = DeclareLaunchArgument(
        'cam_pos_y',
        default_value=cam_pos_y,
        description='Position Y of the camera with respect to the base frame.')

    declare_pos_z_cmd = DeclareLaunchArgument(
        'cam_pos_z',
        default_value=cam_pos_z,
        description='Position Z of the camera with respect to the base frame.')

    declare_roll_cmd = DeclareLaunchArgument(
        'cam_roll',
        default_value=cam_roll,
        description='Roll orientation of the camera with respect to the base frame.')

    declare_pitch_cmd = DeclareLaunchArgument(
        'cam_pitch',
        default_value=cam_pitch,
        description='Pitch orientation of the camera with respect to the base frame.')

    declare_yaw_cmd = DeclareLaunchArgument(
        'cam_yaw',
        default_value=cam_yaw,
        description='Yaw orientation of the camera with respect to the base frame.')

    declare_lrcheck_cmd = DeclareLaunchArgument(
        'lrcheck',
        default_value=lrcheck,
        description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`.')

    declare_extended_cmd = DeclareLaunchArgument(
        'extended',
        default_value=extended,
        description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`.')

    declare_subpixel_cmd = DeclareLaunchArgument(
        'subpixel',
        default_value=subpixel,
        description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`.')

    declare_confidence_cmd = DeclareLaunchArgument(
        'confidence',
        default_value=confidence,
        description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`.')

    declare_LRchecktresh_cmd = DeclareLaunchArgument(
        'LRchecktresh',
        default_value=LRchecktresh,
        description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`.')

    declare_colorResolution_cmd = DeclareLaunchArgument(
        'colorResolution',
        choices=['1080p', '4K'],
        default_value=colorResolution,
        description='The resolution of the color camera')

    declare_useVideo_cmd = DeclareLaunchArgument(
        'useVideo',
        default_value=useVideo,
        description='Whether to publish a video of color image')

    declare_usePreview_cmd = DeclareLaunchArgument(
        'usePreview',
        default_value=usePreview,
        description='Whether to publish a preview of color image')

    declare_useDepth_cmd = DeclareLaunchArgument(
        'useDepth',
        default_value=useDepth,
        description='Whether to publish the depth image')

    declare_previewWidth_cmd = DeclareLaunchArgument(
        'previewWidth',
        default_value=previewWidth,
        description='Width of preview image')

    declare_previewHeight_cmd = DeclareLaunchArgument(
        'previewHeight',
        default_value=previewHeight,
        description='Height of preview image')

    declare_dotProjectormA_cmd = DeclareLaunchArgument(
        'dotProjectormA',
        default_value=dotProjectormA,
        description='Brightness of IR Dot projector on OAK-D-Pro in mA.')

    declare_floodLightmA_cmd = DeclareLaunchArgument(
        'floodLightmA',
        default_value=floodLightmA,
        description='Brightness of IR Flood LED on OAK-D-Pro in mA.')

    urdf_launch = IncludeLaunchDescription(
                            launch_description_sources.PythonLaunchDescriptionSource(
                                    os.path.join(urdf_launch_dir, 'urdf_launch.py')),
                            launch_arguments={'tf_prefix'   : tf_prefix,
                                              'camera_model': camera_model,
                                              'base_frame'  : base_frame,
                                              'parent_frame': parent_frame,
                                              'cam_pos_x'   : cam_pos_x,
                                              'cam_pos_y'   : cam_pos_y,
                                              'cam_pos_z'   : cam_pos_z,
                                              'cam_roll'    : cam_roll,
                                              'cam_pitch'   : cam_pitch,
                                              'cam_yaw'     : cam_yaw}.items(),
                            condition=IfCondition(publish_urdf))

    rgb_stereo_node = launch_ros.actions.Node(
            package='depthai_examples', executable='rgb_stereo_node',
            output='screen',
            parameters=[{'tf_prefix': tf_prefix},
                        {'lrcheck': lrcheck},
                        {'extended': extended},
                        {'subpixel': subpixel},
                        {'confidence': confidence},
                        {'LRchecktresh': LRchecktresh},
                        {'colorResolution': colorResolution},
                        {'useVideo': useVideo},
                        {'usePreview': usePreview},
                        {'useDepth': useDepth},
                        {'previewWidth': previewWidth},
                        {'previewHeight': previewHeight},
                        {'dotProjectormA': dotProjectormA},
                        {'floodLightmA': floodLightmA}])

    metric_converter_node = launch_ros.actions.ComposableNodeContainer(
            name='container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # Driver itself
                launch_ros.descriptions.ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::ConvertMetricNode',
                    name='convert_metric_node',
                    remappings=[('image_raw', '/stereo/depth'),
                                ('camera_info', '/stereo/camera_info'),
                                ('image', '/stereo/converted_depth')]
                ),
            ],
            output='screen',)

    point_cloud_node = launch_ros.actions.ComposableNodeContainer(
            name='container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # Driver itself
                launch_ros.descriptions.ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyziNode',
                    name='point_cloud_xyzi',

                    remappings=[('depth/image_rect', '/stereo/converted_depth'),
                                ('intensity/image_rect', '/right/image'),
                                ('intensity/camera_info', '/right/camera_info'),
                                ('points', '/stereo/points')]
                ),
            ],
            output='screen',)

    rviz_node = launch_ros.actions.Node(
            package='rviz2', executable='rviz2', output='screen',
            arguments=['--display-config', default_rviz])

    ld = LaunchDescription()
    ld.add_action(declare_tf_prefix_cmd)
    ld.add_action(declare_camera_model_cmd)

    ld.add_action(declare_base_frame_cmd)
    ld.add_action(declare_parent_frame_cmd)

    ld.add_action(declare_publish_urdf_cmd)

    ld.add_action(declare_pos_x_cmd)
    ld.add_action(declare_pos_y_cmd)
    ld.add_action(declare_pos_z_cmd)
    ld.add_action(declare_roll_cmd)
    ld.add_action(declare_pitch_cmd)
    ld.add_action(declare_yaw_cmd)

    ld.add_action(declare_lrcheck_cmd)
    ld.add_action(declare_extended_cmd)
    ld.add_action(declare_subpixel_cmd)
    ld.add_action(declare_confidence_cmd)
    ld.add_action(declare_LRchecktresh_cmd)

    ld.add_action(declare_colorResolution_cmd)
    ld.add_action(declare_useVideo_cmd)
    ld.add_action(declare_usePreview_cmd)
    ld.add_action(declare_useDepth_cmd)
    ld.add_action(declare_previewWidth_cmd)
    ld.add_action(declare_previewHeight_cmd)

    ld.add_action(declare_dotProjectormA_cmd)
    ld.add_action(declare_floodLightmA_cmd)

    ld.add_action(rgb_stereo_node)
    ld.add_action(urdf_launch)

    # ld.add_action(metric_converter_node)
    # ld.add_action(point_cloud_node)
    # ld.add_action(rviz_node)
    return ld


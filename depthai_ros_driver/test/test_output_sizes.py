"""Hardware regression for rotation after NN depth alignment and stereo decimation."""
import os
import time
import unittest
import launch
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo

DEVICE = os.getenv('DEPTHAI_ROTATION_TEST_DEVICE', '')
MODE = os.getenv('DEPTHAI_ROTATION_SIZE_CASE', 'decimation')
ANGLE = int(os.getenv('DEPTHAI_ROTATION_TEST_ANGLE', '90'))
FACTOR = int(os.getenv('DEPTHAI_ROTATION_DECIMATION_FACTOR', '2'))
if MODE not in ('nn', 'spatial', 'decimation', 'aligned') or FACTOR not in (2, 3):
    raise ValueError('Select nn/spatial/decimation/aligned and decimation factor 2 or 3')
params = {'driver.i_output_rotation_deg': ANGLE, 'driver.i_enable_ir': False,
          'driver.i_publish_tf_from_calibration': False, 'pipeline_gen.i_enable_imu': False}
params['driver.i_ip' if '.' in DEVICE else 'driver.i_device_id'] = DEVICE
if MODE == 'nn':
    params.update({'rgb.i_enable_nn': True, 'rgb_nn.i_enable_passthrough': True})
    TOPIC = '/oak/rgb_nn/passthrough'
    SIZE = (512, 384)
elif MODE == 'spatial':
    params.update({'pipeline_gen.i_nn_type': 'spatial', 'nn.i_enable_passthrough_depth': True})
    TOPIC = '/oak/nn/passthrough_depth'
    # OAK4 aligns to the 512x384 model; the USB RGBD pipeline retains 640x400 depth.
    SIZE = (512, 384) if '.' in DEVICE else (640, 400)
else:
    params.update({'stereo.i_aligned': MODE == 'aligned', 'stereo.i_enable_decimation_filter': True,
                   'stereo.i_decimation_filter_decimation_factor': FACTOR})
    TOPIC = '/oak/stereo'
    SIZE = (320, 200) if FACTOR == 2 else (224, 133 if '.' in DEVICE else 134)
    if MODE == 'aligned' and '.' in DEVICE:
        SIZE = (640, 400)


@launch_testing.markers.keep_alive
def generate_test_description():
    if not DEVICE:
        return launch.LaunchDescription([launch_testing.actions.ReadyToTest()])
    return launch.LaunchDescription([ComposableNodeContainer(
        name='sizes', namespace='', package='rclcpp_components', executable='component_container',
        composable_node_descriptions=[ComposableNode(package='depthai_ros_driver',
            plugin='depthai_ros_driver::Driver', name='oak', parameters=[params])], output='screen'),
        launch_testing.actions.ReadyToTest()])

@unittest.skipUnless(DEVICE, "Set DEPTHAI_ROTATION_TEST_DEVICE for hardware integration")
class TestSizes(unittest.TestCase):
    def test_stream(self):
        rclpy.init()
        node = rclpy.create_node('check_sizes')
        messages = {}
        image_count = 0

        def receive(message, suffix):
            nonlocal image_count
            messages[suffix] = message
            if suffix == "/image_raw":
                image_count += 1

        subscriptions = [node.create_subscription(t, TOPIC + suffix,
            lambda m,k=suffix: receive(m,k), qos_profile_sensor_data)
            for t,suffix in [(Image, '/image_raw'), (CameraInfo, '/camera_info')]]
        try:
            deadline = time.monotonic() + 60
            while (len(messages) < 2 or image_count < 5) and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.1)
            self.assertEqual(len(messages), 2)
            self.assertGreaterEqual(image_count, 5)
            expected = SIZE[::-1] if ANGLE % 180 else SIZE
            if MODE == 'nn' or (MODE == 'spatial' and '.' in DEVICE):
                expected = SIZE
            for key,msg in messages.items():
                print(key, msg.width, msg.height, msg.header.frame_id, flush=True)
                self.assertEqual((msg.width,msg.height),expected)
            self.assertEqual(len(messages['/image_raw'].data), messages['/image_raw'].step * expected[1])
            self.assertEqual(messages['/image_raw'].header.frame_id, messages['/camera_info'].header.frame_id)
            self.assertEqual(messages['/image_raw'].header.frame_id.endswith('_rotated'), bool(ANGLE))
            info = messages['/camera_info']
            self.assertGreater(info.k[0],0)
            self.assertGreater(info.k[4],0)
            print('K:',list(info.k),flush=True)
        finally:
            node.destroy_node()
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
@unittest.skipUnless(DEVICE, "Set DEPTHAI_ROTATION_TEST_DEVICE for hardware integration")
class TestShutdown(unittest.TestCase):
    def test_exit(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)

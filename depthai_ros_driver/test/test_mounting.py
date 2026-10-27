"""Hardware integration check. Select a device explicitly with DEPTHAI_ROTATION_TEST_DEVICE."""
import os
import time
import unittest

import launch
import numpy as np
import launch_testing
import launch_testing.actions
import launch_testing.asserts
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2
from tf2_msgs.msg import TFMessage
from depthai_ros_msgs.msg import TrackedFeatures
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters

DEVICE = os.getenv("DEPTHAI_ROTATION_TEST_DEVICE", "")
ANGLE = int(os.getenv("DEPTHAI_ROTATION_TEST_ANGLE", "90"))
RGB_TRACKER = os.getenv("DEPTHAI_ROTATION_TEST_TRACKER", "left_rect") == "rgb"
FEATURE_TOPIC = ("rgb" if RGB_TRACKER else "left_rect") + "_feature_tracker/tracked_features"


def generate_test_description():
    actions = []
    if DEVICE:
        params = {
            "driver.i_output_rotation_deg": ANGLE,
            "driver.i_enable_ir": False,
            "driver.i_publish_tf_from_calibration": False,
            "pipeline_gen.i_enable_rgbd": True,
            "stereo.i_left_rect_publish_topic": True,
            "stereo.i_right_rect_publish_topic": True,
            ("rgb.i_enable_feature_tracker" if RGB_TRACKER else "stereo.i_left_rect_enable_feature_tracker"): True,
            "rgb.i_height": 384,
            "stereo.i_height": 384,
            "imu.i_enable_rotation": False,
            "imu.i_enable_mag": False,
            "imu.i_acc_mode": "ACCELEROMETER_RAW",
            "imu.i_gyro_mode": "GYROSCOPE_RAW",
        }
        params["driver.i_ip" if "." in DEVICE else "driver.i_device_id"] = DEVICE
        actions.append(ComposableNodeContainer(
            name="mounting_container", namespace="", package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=[ComposableNode(
                package="depthai_ros_driver", plugin="depthai_ros_driver::Driver",
                name="oak", parameters=[params]), ComposableNode(
                package="depth_image_proc", plugin="depth_image_proc::PointCloudXyzNode",
                name="depth_cloud", remappings=[("image_rect", "/oak/stereo/image_raw"),
                    ("camera_info", "/oak/stereo/camera_info"), ("points", "/oak/depth_points")])], output="screen"))
    actions.append(launch_testing.actions.ReadyToTest())
    return launch.LaunchDescription(actions)


@unittest.skipUnless(DEVICE, "Set DEPTHAI_ROTATION_TEST_DEVICE for hardware integration")
class TestMounting(unittest.TestCase):
    def test_geometry_and_frames(self):
        rclpy.init()
        node = rclpy.create_node("mounting_test")
        messages, transforms = {}, {}
        counts = {}
        def receive(topic, message):
            messages[topic] = message
            counts[topic] = counts.get(topic, 0) + 1
        subscriptions = []
        topics = {
            "rgb/image_raw": Image, "rgb/camera_info": CameraInfo,
            "stereo/image_raw": Image, "stereo/camera_info": CameraInfo,
            "left/image_rect": Image, "right/image_rect": Image,
            FEATURE_TOPIC: TrackedFeatures,
            "rgbd/points": PointCloud2, "depth_points": PointCloud2, "imu/data": Imu,
        }
        try:
            for topic, msg_type in topics.items():
                subscriptions.append(node.create_subscription(
                    msg_type, "/oak/" + topic,
                    lambda msg, key=topic: receive(key, msg), qos_profile_sensor_data))
            subscriptions.append(node.create_subscription(
                TFMessage, "/tf_static",
                lambda msg: transforms.update({t.child_frame_id: t for t in msg.transforms}),
                QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)))
            deadline = time.monotonic() + 45
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
                if len(messages) == len(topics) and min(counts.values()) >= 5:
                    # Cameras can publish invalid depth during startup exposure settling.
                    clouds = [messages[t] for t in ("rgbd/points", "depth_points")]
                    if all(np.count_nonzero(np.ndarray((c.height, c.width), dtype="<f4",
                           buffer=c.data, offset=8, strides=(c.row_step, c.point_step)) > 0) > 100 for c in clouds):
                        break
            self.assertEqual(set(messages), set(topics))
            self.assertTrue(all(count >= 5 for count in counts.values()), counts)
            size = (384, 640) if ANGLE % 180 else (640, 384)
            for topic, msg in messages.items():
                if isinstance(msg, Imu):
                    self.assertFalse(msg.header.frame_id.endswith("_rotated"))
                    continue
                self.assertEqual(msg.header.frame_id.endswith("_rotated"), bool(ANGLE), topic)
                if ANGLE:
                    self.assertIn(msg.header.frame_id, transforms)
                    tf = transforms[msg.header.frame_id]
                    self.assertEqual(tf.header.frame_id + "_rotated", tf.child_frame_id)
                if isinstance(msg, TrackedFeatures):
                    for feature in msg.features:
                        self.assertTrue(0 <= feature.position.x < size[0])
                        self.assertTrue(0 <= feature.position.y < size[1])
                if isinstance(msg, (Image, CameraInfo)):
                    self.assertEqual((msg.width, msg.height), size, topic)
                if isinstance(msg, Image):
                    self.assertEqual(len(msg.data), msg.step * msg.height)
                if isinstance(msg, CameraInfo):
                    self.assertGreater(msg.k[0], 0)
                    self.assertGreater(msg.k[4], 0)
                    self.assertAlmostEqual(msg.k[1], 0, places=4)
                    self.assertAlmostEqual(msg.k[3], 0, places=4)
                if isinstance(msg, PointCloud2):
                    self.assertEqual((msg.width, msg.height), size)
                    self.assertEqual(len(msg.data), msg.row_step * msg.height)
            # Both organized clouds must project onto the rotated pixel grid.
            info = messages["stereo/camera_info"]
            for topic in ("rgbd/points", "depth_points"):
                cloud = messages[topic]
                xyz = np.ndarray((cloud.height, cloud.width, 3), dtype="<f4",
                                 buffer=cloud.data, strides=(cloud.row_step, cloud.point_step, 4))
                valid = np.isfinite(xyz).all(axis=2) & (xyz[:, :, 2] > 0)
                self.assertGreater(valid.sum(), 100)
                u = xyz[:, :, 0][valid] / xyz[:, :, 2][valid] * info.k[0] + info.k[2]
                v = xyz[:, :, 1][valid] / xyz[:, :, 2][valid] * info.k[4] + info.k[5]
                rows, cols = np.indices(valid.shape)
                np.testing.assert_allclose(u, cols[valid], atol=0.01, rtol=0)
                np.testing.assert_allclose(v, rows[valid], atol=0.01, rtol=0)
            client = node.create_client(SetParameters, "/oak/set_parameters")
            self.assertTrue(client.wait_for_service(timeout_sec=5))
            request = SetParameters.Request(parameters=[Parameter(
                name="driver.i_output_rotation_deg",
                value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=(ANGLE+90)%360))])
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5)
            self.assertIsNotNone(future.result())
            self.assertFalse(future.result().results[0].successful)
        finally:
            node.destroy_node()
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
@unittest.skipUnless(DEVICE, "Set DEPTHAI_ROTATION_TEST_DEVICE for hardware integration")
class TestShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)

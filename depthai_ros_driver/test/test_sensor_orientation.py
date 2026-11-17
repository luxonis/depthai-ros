"""Compare live 0/180-degree readout on a stationary scene.

Run twice, with DEPTHAI_ROTATION_TEST_ANGLE=0 then 180 and the same
DEPTHAI_SENSOR_ORIENTATION_REFERENCE directory. Select the camera explicitly.
The reference directory retains images, calibration and measured stream rates.
"""
import json
import os
from pathlib import Path
import time
import unittest

import cv2
import launch
import launch_testing
import launch_testing.actions
import launch_testing.asserts
from launch_testing.util import KeepAliveProc
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

DEVICE = os.getenv("DEPTHAI_ROTATION_TEST_DEVICE", "")
ANGLE = int(os.getenv("DEPTHAI_ROTATION_TEST_ANGLE", "0"))
REFERENCE = os.getenv("DEPTHAI_SENSOR_ORIENTATION_REFERENCE", "")


def generate_test_description():
    actions = []
    if not (DEVICE and REFERENCE):
        actions.append(KeepAliveProc())
    if DEVICE and REFERENCE:
        params = {
            "driver.i_output_rotation_deg": ANGLE,
            "driver.i_enable_ir": False,
            "driver.i_publish_tf_from_calibration": False,
            "pipeline_gen.i_pipeline_type": "RGBStereo",
            "pipeline_gen.i_enable_imu": False,
        }
        params["driver.i_ip" if "." in DEVICE else "driver.i_device_id"] = DEVICE
        for camera in ("rgb", "left", "right"):
            if ANGLE == 180:
                # Check this assumed baseline against an unmodified AUTO capture.
                params[camera + ".i_native_sensor_orientation"] = os.getenv("DEPTHAI_SENSOR_NATIVE_ORIENTATION", "NORMAL")
            params.update({camera + ".i_width": 640, camera + ".i_height": 400,
                           camera + ".i_undistorted": os.getenv("DEPTHAI_SENSOR_UNDISTORTED", "1") == "1",
                           camera + ".r_set_man_exposure": True,
                           camera + ".r_exposure": 30000 if camera == "rgb" else 10000,
                           camera + ".r_iso": 1600 if camera == "rgb" else 400})
        actions.append(ComposableNodeContainer(
            name="sensor_orientation_container", namespace="", package="rclcpp_components",
            executable="component_container", output="screen",
            composable_node_descriptions=[ComposableNode(
                package="depthai_ros_driver", plugin="depthai_ros_driver::Driver",
                name="oak", parameters=[params])]))
    actions.append(launch_testing.actions.ReadyToTest())
    return launch.LaunchDescription(actions)


@unittest.skipUnless(DEVICE and REFERENCE, "Select device and reference directory")
class TestSensorOrientation(unittest.TestCase):
    def test_half_turn_matches_native_image(self):
        self.assertIn(ANGLE, (0, 180))
        output = Path(REFERENCE)
        output.mkdir(parents=True, exist_ok=True)
        rclpy.init()
        node = rclpy.create_node("sensor_orientation_test")
        messages, infos, counts, first, last = {}, {}, {}, {}, {}
        subscriptions = []

        def receive(camera, msg):
            messages[camera] = msg
            counts[camera] = counts.get(camera, 0) + 1
            first.setdefault(camera, time.monotonic())
            last[camera] = time.monotonic()

        try:
            for camera in ("rgb", "left", "right"):
                subscriptions.append(node.create_subscription(
                    Image, "/oak/" + camera + "/image_raw",
                    lambda msg, c=camera: receive(c, msg), qos_profile_sensor_data))
                subscriptions.append(node.create_subscription(
                    CameraInfo, "/oak/" + camera + "/camera_info",
                    lambda msg, c=camera: infos.update({c: msg}), qos_profile_sensor_data))
            deadline = time.monotonic() + 50
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
                if (len(infos) == 3 and len(first) == 3
                        and min(last[c] - first[c] for c in first) >= 20):
                    break
            self.assertEqual(set(messages), {"rgb", "left", "right"})
            self.assertEqual(set(infos), set(messages))
            metrics = {}
            for camera, msg in messages.items():
                with self.subTest(camera=camera):
                    info = infos[camera]
                    self.assertEqual((msg.width, msg.height), (640, 400))
                    self.assertEqual(msg.header.frame_id, info.header.frame_id)
                    self.assertEqual(msg.header.frame_id.endswith("_rotated"), ANGLE == 180)
                    # Driver publishes three-channel BGR images for these camera outputs.
                    self.assertEqual(msg.encoding, "bgr8")
                    image = np.ndarray((msg.height, msg.width, 3), dtype=np.uint8,
                                       buffer=msg.data, strides=(msg.step, 3, 1)).copy()
                    cv2.imwrite(str(output / f"{camera}-{ANGLE}.png"), image)
                    calibration = np.array(list(info.k) + list(info.p) + list(info.d))
                    metrics[camera] = {"fps": (counts[camera] - 1) / (last[camera] - first[camera])}
                    self.assertGreater(metrics[camera]["fps"], 10)
                    reference = output / (camera + "-native.npz")
                    if ANGLE == 0:
                        np.savez(reference, image=image, calibration=calibration)
                    else:
                        baseline_rates = json.loads((output / "metrics-0.json").read_text())
                        self.assertGreaterEqual(metrics[camera]["fps"], baseline_rates[camera]["fps"] * 0.8,
                                                "Rotated stream regressed more than 20% from its native rate")
                        native = np.load(reference)
                        actual = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY).astype(np.float32)
                        expected = cv2.rotate(cv2.cvtColor(native["image"], cv2.COLOR_BGR2GRAY), cv2.ROTATE_180).astype(np.float32)
                        upright = cv2.rotate(expected, cv2.ROTATE_180)
                        self.assertGreater(float(expected.std()), 5, "Scene needs visible texture")
                        correlation = float(cv2.matchTemplate(actual, expected, cv2.TM_CCOEFF_NORMED)[0, 0])
                        wrong = float(cv2.matchTemplate(actual, upright, cv2.TM_CCOEFF_NORMED)[0, 0])
                        metrics[camera].update(correlation_180=correlation, correlation_0=wrong)
                        self.assertGreater(correlation, 0.75, camera)
                        self.assertGreater(correlation, wrong + 0.1, camera)
                        baseline = native["calibration"]
                        expected_calibration = baseline.copy()
                        expected_calibration[2] = 639 - baseline[2]
                        expected_calibration[5] = 399 - baseline[5]
                        expected_calibration[11] = 639 - baseline[11]
                        expected_calibration[15] = 399 - baseline[15]
                        expected_calibration[[12, 16]] *= -1
                        if len(baseline) >= 25:  # plumb_bob/rational tangential coefficients
                            expected_calibration[23:25] *= -1
                        if len(baseline) >= 35:
                            expected_calibration[33:35] *= -1
                        np.testing.assert_allclose(calibration, expected_calibration, atol=1e-5)
            (output / f"metrics-{ANGLE}.json").write_text(json.dumps(metrics, indent=2))
            print("SENSOR_ORIENTATION_METRICS", ANGLE, metrics, flush=True)
        finally:
            node.destroy_node()
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
@unittest.skipUnless(DEVICE and REFERENCE, "Select device and reference directory")
class TestShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)

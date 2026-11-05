import os
import unittest
import launch
import launch_testing
import launch_testing.actions
import launch_testing.markers
import launch_testing.asserts
import pytest
import rclpy
import rclpy.node
import rclpy.parameter
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger
from rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import (
    Parameter,
    ParameterDescriptor,
    ParameterType,
    ParameterValue,
)
import time
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from depthai_ros_driver.test_helper import TestHelper


@pytest.mark.rostest
def generate_test_description():
    depthai_prefix = get_package_share_directory("depthai_ros_driver")
    params_file = os.path.join(depthai_prefix, "config", "driver.yaml")
    rviz_config = os.path.join(depthai_prefix, "config", "rviz", "rgbd.rviz")

    driver_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(depthai_prefix, "launch", "driver.launch.py")
        ),
        launch_arguments={
            "name": "oak",
            "namespace": "",
            "parent_frame": "oak_parent_frame",
            "camera_model": "OAK-D-PRO",
            "cam_pos_x": "0.0",
            "cam_pos_y": "0.0",
            "cam_pos_z": "0.0",
            "cam_roll": "0.0",
            "cam_pitch": "0.0",
            "cam_yaw": "0.0",
            "params_file": params_file,
            "use_rviz": "false",
            "rviz_config": rviz_config,
            "rsp_use_composition": "true",
            "publish_tf_from_calibration": "true",
            "imu_from_descr": "false",
            "override_cam_model": "false",
            "use_gdb": "false",
            "use_valgrind": "false",
            "use_perf": "false",
            "rs_compat": "false",
            "pointcloud.enable": "false",
            "enable_color": "true",
            "enable_depth": "true",
            "enable_infra1": "false",
            "enable_infra2": "false",
            "depth_module.depth_profile": "640,400,30",
            "rgb_camera.color_profile": "640,400,30",
            "depth_module.infra_profile": "640,400,30",
        }.items(),
    )

    return launch.LaunchDescription(
        [
            driver_launch,
            launch.actions.TimerAction(
                period=1.0, actions=[launch_testing.actions.ReadyToTest()]
            ),
        ]
    ), {"driver_launch": driver_launch}


class TestDriverLaunch(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test")
        self.testHelper = TestHelper(self.node)

    def tearDown(self):
        self.node.destroy_node()

    def test_driver_output(self, proc_output):
        proc_output.assertWaitFor("Driver ready!", timeout=10.0, stream="stderr")

    def test_published_rgb_image(self, proc_output):
        self.assertTrue(
            self.testHelper.testIncomingMessages(Image, "/oak/rgb/image_raw")
        )
        self.assertTrue(
            self.testHelper.testIncomingMessages(CameraInfo, "/oak/rgb/camera_info")
        )

    def test_published_stereo_image(self, proc_output):
        self.assertTrue(
            self.testHelper.testIncomingMessages(Image, "/oak/stereo/image_raw")
        )
        self.assertTrue(
            self.testHelper.testIncomingMessages(CameraInfo, "/oak/stereo/camera_info")
        )

    def test_published_imu_messages(self, proc_output):
        self.testHelper.testIncomingMessages(Imu, "/oak/imu/data_raw")

    def test_stop_start_camera(self, proc_output):
        self.assertTrue(self.testHelper.testTriggerService("/oak/stop_driver"))
        proc_output.assertWaitFor("Driver stopped!", timeout=10.0, stream="stderr")

        # Both saves must respond with a failure instead of crashing or timing
        # out when their device-dependent operations cannot run after stop().
        for service in ("save_pipeline", "save_calibration"):
            client = self.node.create_client(Trigger, "/oak/" + service)
            try:
                self.assertTrue(client.wait_for_service(timeout_sec=5.0))
                future = client.call_async(Trigger.Request())
                rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
                self.assertTrue(future.done(), service + " did not respond")
                response = future.result()
                self.assertFalse(response.success)
                self.assertIn("not connected", response.message)
            finally:
                self.node.destroy_client(client)

        self.assertTrue(self.testHelper.testTriggerService("/oak/start_driver"))
        proc_output.assertWaitFor("Driver ready!", timeout=10.0, stream="stderr")

    def test_save_calibration(self, proc_output):
        self.assertTrue(self.testHelper.testTriggerService("/oak/save_calibration"))
        proc_output.assertWaitFor(
            "Saving calibration to", timeout=10.0, stream="stderr"
        )

    def test_save_pipeline(self, proc_output):
        self.assertTrue(self.testHelper.testTriggerService("/oak/save_pipeline"))
        proc_output.assertWaitFor(
            "Saving pipeline schema to", timeout=10.0, stream="stderr"
        )

    def test_set_parameters(self, proc_output):

        parameters = [
            Parameter(
                name="driver.r_laser_dot_intensity",
                value=ParameterValue(type=3, double_value=0.4),
            )
        ]
        self.assertTrue(self.testHelper.setParameters(parameters, False))

        value = self.testHelper.getParameter("driver.r_laser_dot_intensity")
        self.assertAlmostEqual(value.double_value, 0.4)

    def test_override_camera_info(self, proc_output):
        self.testHelper
        parameters = [
            Parameter(
                name="rgb.i_calibration_file",
                value=ParameterValue(
                    type=4,
                    string_value="package://depthai_ros_driver/config/calibration/rgb.yaml",
                ),
            )
        ]
        self.assertTrue(self.testHelper.setParameters(parameters))
        value = self.testHelper.getParameter("rgb.i_calibration_file")
        self.assertEqual(
            value.string_value,
            "package://depthai_ros_driver/config/calibration/rgb.yaml",
        )

        def cb(msg):
            self.assertEqual(msg.distortion_model, "plumb_bob")
            return msg

        self.assertTrue(
            self.testHelper.testIncomingMessages(CameraInfo, "/oak/rgb/camera_info", cb)
        )


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)

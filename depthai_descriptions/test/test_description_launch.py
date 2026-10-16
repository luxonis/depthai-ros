import importlib.util
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument


ROOT = Path(__file__).resolve().parents[2]


def load_launch(package, filename):
    spec = importlib.util.spec_from_file_location('test_launch', ROOT / package / 'launch' / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class DescriptionLaunchTest(unittest.TestCase):
    def context(self, module, **values):
        context = LaunchContext()
        context.launch_configurations.update(values)
        for action in module.generate_launch_description().entities:
            if isinstance(action, DeclareLaunchArgument):
                action.execute(context)
        return context

    def test_driver_launch_forwards_selection_and_defers_initial_imu(self):
        for filename in ('driver.launch.py', 'driver_as_part_of_a_robot.launch.py'):
            for calibration in ('true', 'false'):
                with self.subTest(filename=filename, calibration=calibration), tempfile.TemporaryDirectory() as logs:
                    module = load_launch('depthai_ros_driver', filename)
                    with patch.dict(os.environ, ROS_LOG_DIR=logs), \
                         patch.object(module, 'get_package_share_directory', side_effect=lambda package: str(ROOT / package)), \
                         patch.object(module, 'ComposableNode') as node, \
                         patch.object(module, 'ComposableNodeContainer'), \
                         patch.object(module, 'LoadComposableNodes'), \
                         patch.object(module, 'IncludeLaunchDescription') as include:
                        context = self.context(module, publish_tf_from_calibration=calibration,
                                               imu_from_descr='true', imu_sensor='BMI', imu_frame='native',
                                               pose_source='board', model_selection='exact', fallback_model='OAK-D',
                                               mesh_model='OAK-D-S2', legacy_frame_aliases='false')
                        module.launch_setup(context)
                        driver = next(call.kwargs for call in node.call_args_list if call.kwargs.get('plugin') == 'depthai_ros_driver::Driver')
                        params = next(p['driver'] for p in driver['parameters'] if isinstance(p, dict) and 'driver' in p)
                        if calibration == 'true':
                            self.assertEqual(params['i_tf_imu_sensor'], 'BMI')
                            self.assertEqual(params['i_tf_imu_frame'], 'native')
                            for key, value in dict(pose_source='board', model_selection='exact', fallback_model='OAK-D',
                                                   mesh_model='OAK-D-S2', legacy_frame_aliases='false').items():
                                self.assertEqual(params['i_tf_' + key], value)
                        else:
                            self.assertFalse(params['i_publish_tf_from_calibration'])
                        if filename == 'driver.launch.py':
                            passed = dict(include.call_args.kwargs['launch_arguments'])
                            self.assertEqual(passed['has_imu'], 'false' if calibration == 'true' else 'auto')
                            for key, value in dict(pose_source='board', model_selection='exact', fallback_model='OAK-D',
                                                   mesh_model='OAK-D-S2', legacy_frame_aliases='false').items():
                                self.assertEqual(passed[key].perform(context), value)
                            sensor = passed['imu_sensor']
                            if not isinstance(sensor, str):
                                sensor = sensor.perform(context)
                            self.assertEqual(sensor, '' if calibration == 'true' else 'BMI')

    def test_description_launch_forwards_has_imu_and_rejects_failure(self):
        module = load_launch('depthai_descriptions', 'urdf_launch.py')
        with tempfile.TemporaryDirectory() as logs, patch.dict(os.environ, ROS_LOG_DIR=logs), \
             patch.object(module, 'get_package_share_directory', return_value=str(ROOT / 'depthai_descriptions')), \
             patch.object(module.subprocess, 'run') as run, patch.object(module, 'Node'), \
             patch.object(module, 'LoadComposableNodes'), patch.object(module, 'ComposableNode'):
            context = self.context(module, camera_model='OAK-D-S2', use_base_descr='true',
                                   has_imu='true', imu_sensor='BMI')
            run.return_value.returncode = 0
            run.return_value.stderr = ''
            run.return_value.stdout = '<robot name="test"><link name="base"/></robot>'
            module.launch_setup(context)
            command = run.call_args.args[0]
            self.assertIn('has_imu:=true', command)
            self.assertIn('imu_sensor:=BMI', command)
            self.assertIn('use_base_descr:=true', command)
            run.return_value.returncode = 2
            run.return_value.stderr = 'bad chip'
            with self.assertRaisesRegex(RuntimeError, 'bad chip'):
                module.launch_setup(context)


if __name__ == '__main__':
    unittest.main()

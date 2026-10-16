import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch
from xml.etree import ElementTree

import yaml

DESCRIPTION = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('resolve_description', DESCRIPTION / 'scripts/resolve_description.py')
resolver = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(resolver)


class DescriptionResolverTest(unittest.TestCase):
    def expand(self, model, **mappings):
        detected = mappings.pop('detected', '')
        with patch('ament_index_python.packages.get_package_share_directory', return_value=str(DESCRIPTION)):
            return resolver.expand(dict(camera_model=model, pose_source='board', **mappings), detected_imu=detected, share=DESCRIPTION)

    def test_exact_models_aliases_and_no_mesh(self):
        for model in ('OAK-D-S2', 'OAK-D-LITE', 'OAK-D-PRO-W', 'OAK-D-PRO-POE'):
            with self.subTest(model=model):
                result = self.expand(model, use_base_descr='true')
                self.assertEqual(result['model'], model)
        self.assertEqual(self.expand('OAK-4-PRO')['model'], 'OAK4-D')
        self.assertTrue(ElementTree.fromstring(self.expand('OAK-D-PRO-POE')['urdf']).find('.//mesh').attrib['filename'].endswith('OAK-D-S2-POE.stl'))
        for model in ('NONEXISTENT', 'OAK-FFC-4P', 'OAK-D-W-POE-C57'):
            with self.subTest(model=model), self.assertRaises(ValueError):
                self.expand(model)

    def test_detected_chips_and_override(self):
        for detected, key in (('BNO086', 'BNO'), ('BMI270', 'BMI'), ('BMI', 'BMI')):
            result = self.expand('OAK-D-S2', use_base_descr='true', has_imu='true', detected=detected)
            self.assertEqual(result['imu_sensor'], key)
            self.assertIsNotNone(ElementTree.fromstring(result['urdf']).find("joint[@name='oak_imu_joint']"))
        result = self.expand('OAK-D-S2', has_imu='true', detected='BNO086', imu_sensor='BMI')
        self.assertEqual(result['imu_sensor'], 'BMI')
        self.assertEqual(self.expand('OAK4-D', has_imu='true', detected='LSM6DSV')['imu_sensor'], 'LSM6DSV')
        for detected in ('', 'UNSUPPORTED_CHIP'):
            with self.subTest(detected=detected), self.assertRaises(ValueError):
                self.expand('OAK-D-S2', has_imu='true', detected=detected)
        # Device detection must not affect descriptions which do not own IMU TF.
        self.expand('OAK-D-S2', has_imu='false', detected='UNSUPPORTED_CHIP')

    def test_frame_mode_and_tf_ownership(self):
        for mode in ('native', 'optical'):
            result = self.expand('OAK-D-S2', use_base_descr='true', has_imu='true', imu_sensor='BMI', imu_frame=mode)
            root = ElementTree.fromstring(result['urdf'])
            self.assertIsNone(root.find("joint[@name='oak_left_camera_joint']"))
            pose = root.find("joint[@name='oak_imu_joint']/origin")
            boards = yaml.safe_load((DESCRIPTION / 'urdf/include/board_models.yaml').read_text())
            self.assertEqual(pose.attrib['rpy'], boards['models']['OAK-D-S2']['imu_variants']['BMI'][mode + '_rpy'])
        for base in ('true', 'false'):
            root = ElementTree.fromstring(self.expand('OAK-D-S2', use_base_descr=base, has_imu='false')['urdf'])
            self.assertIsNone(root.find("joint[@name='oak_imu_joint']"))
            self.assertEqual(root.find("joint[@name='oak_left_camera_joint']") is None, base == 'true')

    def test_terminal_origin_metadata(self):
        sr = self.expand('OAK-D-SR', use_base_descr='true', has_imu='true', imu_sensor='BMI')
        self.assertEqual(sr['camera_poses']['CAM_C']['xyz'], [0, 0, 0])
        thermal = self.expand('OAK-T', use_base_descr='true')
        self.assertEqual(thermal['camera_poses']['CAM_E']['xyz'], [0, 0, 0])
        self.assertEqual(thermal['camera_poses']['CAM_A']['xyz'], [0, 0.025, 0])

    def test_camera_names_match_driver_socket_conventions(self):
        for model in ('OAK-D-S2', 'OAK-D-SR-POE', 'OAK-T', 'PERCEPT', 'RAE-D-E', 'TOF-FFC-A-B'):
            for rs in (False, True):
                with self.subTest(model=model, rs=rs):
                    result = self.expand(model, rs_compat=str(rs).lower(), has_imu='false')
                    names = dict(zip(('CAM_A', 'CAM_B', 'CAM_C', 'CAM_D', 'CAM_E'),
                                     ('color', 'infra2', 'infra1', 'infra3', 'infra4') if rs
                                     else ('rgb', 'left', 'right', 'left_back', 'right_back')))
                    if model == 'OAK-T':
                        names = dict(zip(names, ('rgb', 'left', 'right', 'left_back', 'thermal')))
                    elif model == 'OAK-D-SR-POE':
                        names.update(CAM_A='tof', CAM_B='left', CAM_C='right')
                    root = ElementTree.fromstring(result['urdf'])
                    frames = {link.attrib['name'] for link in root.findall('link')}
                    for socket in result['camera_poses']:
                        suffix = '_frame' if rs else '_camera_frame'
                        optical = '_optical_frame' if rs else '_camera_optical_frame'
                        self.assertIn('oak_' + names[socket] + suffix, frames)
                        self.assertIn('oak_' + names[socket] + optical, frames)

    def test_custom_xacro_and_failure_output(self):
        with tempfile.TemporaryDirectory() as directory:
            custom = Path(directory) / 'custom file.xacro'
            custom.write_text('<robot name="custom"><link name="base"/></robot>')
            result = resolver.expand({'camera_model': 'CUSTOM'}, custom, share=DESCRIPTION)
            self.assertEqual(result['camera_poses'], {})
            command = [sys.executable, str(DESCRIPTION / 'scripts/resolve_description.py'),
                       '--description=' + str(custom), '--output=json',
                       '--custom-args=camera_model:=CUSTOM camera_name:="a b"']
            valid = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(valid.returncode, 0, valid.stderr)
            self.assertIn('<robot', json.loads(valid.stdout)['urdf'])
            for invalid in ('', '<not_robot/>', '<robot><broken>'):
                custom.write_text(invalid)
                failed = subprocess.run(command, capture_output=True, text=True)
                self.assertNotEqual(failed.returncode, 0)
                self.assertEqual(failed.stdout, '')
                self.assertIn('Failed to expand', failed.stderr)

    def test_custom_flags_mappings_and_shell_literals(self):
        with tempfile.TemporaryDirectory() as directory:
            custom = Path(directory) / 'custom.xacro'
            custom.write_text('<robot name="custom" xmlns:xacro="http://ros.org/wiki/xacro">'
                              '<xacro:arg name="camera_name" default="default"/>'
                              '<link name="$(arg camera_name)"/></robot>')
            command = [sys.executable, str(DESCRIPTION / 'scripts/resolve_description.py'),
                       '--description=' + str(custom), '--output=json', 'camera_name:=original']
            for flag in ('--inorder', '-i', '--verbosity=2', '--verbosity 2'):
                result = subprocess.run(command + ['--custom-args=' + flag + ' camera_name:="literal $HOME ; echo injected"'],
                                        capture_output=True, text=True)
                self.assertEqual(result.returncode, 0, result.stderr)
                root = ElementTree.fromstring(json.loads(result.stdout)['urdf'])
                self.assertEqual(root.find('link').attrib['name'], 'literal $HOME ; echo injected')
            for args in ('--verbosity nope', '--output=/tmp/out', 'camera_name:=test ; echo injected'):
                result = subprocess.run(command + ['--custom-args=' + args], capture_output=True, text=True)
                self.assertNotEqual(result.returncode, 0)
                self.assertEqual(result.stdout, '')


if __name__ == '__main__':
    unittest.main()

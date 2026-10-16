"""Migration contract against unmodified Xacros pinned to develop (see fixtures/SOURCE)."""
from pathlib import Path
from unittest.mock import patch
from xml.etree import ElementTree as ET
import unittest

import xacro
import yaml
from test_description_resolver import resolver, DESCRIPTION
import test_board_to_urdf as conversion_tests


class LegacyCompatibilityTest(unittest.TestCase):
    assertVector = conversion_tests.ExistingUrdfTest.assertVector
    assertOrigin = conversion_tests.ExistingUrdfTest.assertOrigin

    def expand(self, model, fixture=False, base=False, **args):
        share = DESCRIPTION / 'test/fixtures/develop' if fixture else DESCRIPTION
        path = share / 'urdf' / ('base_descr.urdf.xacro' if base else 'depthai_descr.urdf.xacro')
        with patch('ament_index_python.packages.get_package_share_directory', return_value=str(share)):
            return ET.fromstring(xacro.process_file(str(path), mappings=dict(camera_model=model, **args)).toxml())

    def test_all_develop_models_default_geometry_names_and_visuals(self):
        legacy = yaml.safe_load((DESCRIPTION / 'urdf/include/legacy_models.yaml').read_text())
        for model in legacy['models']:
            for rs in ('false', 'true'):
                for base in (False, True):
                    with self.subTest(model=model, rs=rs, base=base):
                        options = dict(rs_compat=rs)
                        if base:
                            options['has_imu'] = 'true' if model in ('OAK-D', 'OAK-D-PRO', 'OAK-D-POE') else 'false'
                        expected = self.expand(model, fixture=True, base=base, **options)
                        actual = self.expand(model, base=base, **options)
                        self.assertEqual({n.attrib['name'] for n in actual.findall('link')},
                                         {n.attrib['name'] for n in expected.findall('link')})
                        aj = {n.attrib['name']: n for n in actual.findall('joint')}
                        ej = {n.attrib['name']: n for n in expected.findall('joint')}
                        self.assertEqual(set(aj), set(ej))
                        for name, joint in ej.items():
                            self.assertEqual(aj[name].find('parent').attrib, joint.find('parent').attrib)
                            self.assertEqual(aj[name].find('child').attrib, joint.find('child').attrib)
                            self.assertOrigin(aj[name].find('origin').attrib, joint.find('origin').attrib)
                        for name in ('mesh', 'color'):
                            self.assertEqual([n.attrib for n in actual.findall('.//' + name)],
                                             [n.attrib for n in expected.findall('.//' + name)])
                        self.assertEqual([n.attrib for n in actual.findall('.//visual/origin')],
                                         [n.attrib for n in expected.findall('.//visual/origin')])

    def resolve(self, **args):
        with patch('ament_index_python.packages.get_package_share_directory', return_value=str(DESCRIPTION)):
            return resolver.expand(args, share=DESCRIPTION)

    def test_legacy_driver_aliases_and_explicit_exact_model(self):
        aliases = yaml.safe_load((DESCRIPTION / 'urdf/include/legacy_models.yaml').read_text())['model_aliases']
        for model, target in aliases.items():
            with self.subTest(model=model):
                result = self.resolve(camera_model=model, use_base_descr='true')
                self.assertEqual(result['model'], target)
                self.assertEqual(result['camera_poses'], {})
                self.assertIn('DEPRECATED model_selection=legacy', result['diagnostics'])
        exact = self.resolve(camera_model='OAK-D-LITE', use_base_descr='true', model_selection='exact')
        self.assertEqual(exact['model'], 'OAK-D-LITE')
        self.assertEqual(exact['pose_source'], 'legacy')
        board = self.resolve(camera_model='OAK-D-LITE', use_base_descr='true', pose_source='board')
        self.assertEqual(board['model'], 'OAK-D-LITE')
        self.assertEqual(board['pose_source'], 'board')

    def test_explicit_fallback_and_new_model_board_default(self):
        with self.assertRaisesRegex(ValueError, 'Unknown camera model'):
            self.resolve(camera_model='UNKNOWN')
        result = self.resolve(camera_model='UNKNOWN', fallback_model='OAK-D')
        self.assertEqual(result['model'], 'OAK-D')
        self.assertIn('Explicit fallback_model: UNKNOWN -> OAK-D', result['diagnostics'])
        self.assertEqual(self.resolve(camera_model='PERCEPT')['pose_source'], 'board')

    def test_legacy_imu_ignores_board_chip_selection(self):
        default = self.resolve(camera_model='OAK-D-PRO')['urdf']
        explicit = self.resolve(camera_model='OAK-D-PRO', imu_sensor='BMI', imu_frame='native')['urdf']
        self.assertEqual(default, explicit)

    def test_deprecated_aliases_are_identity_and_optional(self):
        for model, alias, canonical in [('OAK-D-SR-POE', 'rgb', 'tof'), ('OAK-T', 'cam_e', 'thermal'),
                                        ('PERCEPT', 'cam_d', 'left_back')]:
            with self.subTest(model=model):
                root = ET.fromstring(self.resolve(camera_model=model, pose_source='board', has_imu='false')['urdf'])
                joint = root.find(f"joint/child[@link='oak_{alias}_camera_frame']/..")
                self.assertEqual(joint.find('parent').attrib['link'], f'oak_{canonical}_camera_frame')
                self.assertOrigin(joint.find('origin').attrib, dict(xyz='0 0 0', rpy='0 0 0'))
                disabled = ET.fromstring(self.resolve(camera_model=model, pose_source='board', legacy_frame_aliases='false')['urdf'])
                self.assertIsNone(disabled.find(f"link[@name='oak_{alias}_camera_frame']"))
        rs = ET.fromstring(self.resolve(camera_model='OAK-D', pose_source='board', rs_compat='true')['urdf'])
        joint = rs.find("joint/child[@link='oak_color_camera_frame']/..")
        self.assertEqual(joint.find('parent').attrib['link'], 'oak_color_frame')

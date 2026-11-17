import importlib.util
from pathlib import Path
import unittest
from unittest.mock import patch
from xml.etree import ElementTree as ET

import yaml

SHARE = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('model_gallery', SHARE / 'scripts/model_gallery.py')
gallery = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(gallery)


class ModelGalleryTest(unittest.TestCase):
    def build(self, **args):
        with patch('ament_index_python.packages.get_package_share_directory', return_value=str(SHARE)):
            return gallery.build_gallery(share=SHARE, **args)

    def test_all_models_variants_and_single_connected_tree(self):
        result = self.build()
        boards = yaml.safe_load((SHARE / 'urdf/include/board_models.yaml').read_text())['models']
        expected = {(model, sensor) for model, config in boards.items()
                    for sensor in (list(config['imu_variants']) or [''])}
        expected.add(('OAK-D-IOT-75', ''))
        self.assertEqual({(e['model'], e['sensor']) for e in result['entries']}, expected)
        root = ET.fromstring(result['urdf'])
        names = [e.get('name') for e in root.findall('link')]
        self.assertEqual(len(names), len(set(names)))
        joints = root.findall('joint')
        self.assertEqual(len(joints), len({j.get('name') for j in joints}))
        children = [j.find('child').get('link') for j in joints]
        self.assertEqual(len(children), len(set(children)))
        self.assertEqual(set(names) - set(children), {'gallery_world'})
        for joint in joints:
            self.assertIn(joint.find('parent').get('link'), names)
        for frame in result['frames']:
            self.assertIn(frame['frame'], names)
        for entry in result['entries']:
            if entry['sensor']:
                self.assertIn(entry['frame'] + '_imu_frame', names)
        self.assertEqual(len(result['notices']), 5)

    def test_filter_layout_and_legacy_selection(self):
        result = self.build(models='OAK-D-SR,OAK-T', columns=1, spacing=0.8, scale='1')
        self.assertEqual([e['xyz'] for e in result['entries']], [[0.0, 0.0, 0], [0.0, -0.8, 0], [0.0, -1.6, 0]])
        root = ET.fromstring(result['urdf'])
        for index in (0, 1, 2):
            origin = root.find(f"joint[@name='gallery_{index:03d}_right_camera_joint']/origin")
            self.assertEqual(list(map(float, origin.get('xyz').split())), [0, 0, 0])
        legacy = self.build(models='OAK-D-S2', pose_source='legacy')
        self.assertEqual(len(legacy['entries']), 1)
        self.assertEqual(legacy['entries'][0]['pose_source'], 'legacy')
        for args in ({'models': 'OAK-FFC-4P'}, {'columns': 0}, {'spacing': float('nan')}):
            with self.assertRaises(ValueError):
                self.build(**args)

    def test_scale_preserves_alignment_and_fits_view(self):
        import math
        original = self.build(models='OAK-D-SR,OAK-T', scale='1')
        enlarged = self.build(models='OAK-D-SR,OAK-T', scale='10')
        a, b = ET.fromstring(original['urdf']), ET.fromstring(enlarged['urdf'])
        for first, second in zip(a.iter('origin'), b.iter('origin')):
            self.assertEqual(first.get('rpy'), second.get('rpy'))
            self.assertEqual([float(v) * 10 for v in first.get('xyz').split()],
                             list(map(float, second.get('xyz').split())))
        for mesh in b.iter('mesh'):
            self.assertEqual(mesh.get('scale'), '10.0 10.0 10.0')
        auto = self.build(models='OAK-D-SR,OAK-T')
        self.assertAlmostEqual(auto['scale'], 50 / math.sqrt(3))
        config = gallery.rviz_config(enlarged, SHARE / 'config/model_gallery.rviz')
        view = config['Visualization Manager']['Views']['Current']
        self.assertEqual(view['Focal Point']['X'], 7.5)
        self.assertGreater(view['Distance'], 15)
        for value in ('0', '-1', 'nan', 'inf'):
            with self.assertRaises(ValueError):
                self.build(models='OAK-T', scale=value)

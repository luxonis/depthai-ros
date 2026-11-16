import math
import contextlib
import io
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
from types import SimpleNamespace
import tempfile
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import compare_board_urdf as compare


class ComparisonTest(unittest.TestCase):
    def test_each_imu_is_selected_for_each_convention(self):
        extrinsics = {'to_cam': 'CAM_A', 'specTranslation': dict(x=0, y=0, z=0),
                      'rotationMatrix': compare.geometry.IDENTITY}
        board = {'cameras': {'CAM_A': {}}, 'imuExtrinsics': {'sensors': {
            sensor: {'extrinsics': extrinsics} for sensor in ['BNO', 'BMI']}}}
        identity = (compare.geometry.IDENTITY, [0, 0, 0])
        def expand(descriptions, model, sensor, mode):
            poses = {'oak': identity, 'oak_rgb_camera_frame': identity}
            if sensor:
                poses['oak_imu_frame'] = (compare.geometry.OPTICAL_TO_BODY, [0, 0, 0])
            return poses
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            (path / 'TEST.json').write_text(json.dumps(board))
            args = SimpleNamespace(boards=path, descriptions=path, reference=None,
                                   imu_frame='both', imu_sensor=None,
                                   position_tolerance=0.001, angle_tolerance=0.0001)
            with patch.object(compare, 'expand', side_effect=expand) as mocked, contextlib.redirect_stdout(io.StringIO()):
                self.assertFalse(compare.compare_model('TEST', args))
            selected = {(call.args[2], call.args[3]) for call in mocked.call_args_list if call.args[2]}
            self.assertEqual(selected, {('BNO', 'native'), ('BMI', 'native'),
                                        ('BNO', 'optical'), ('BMI', 'optical')})

    def test_deprecated_aliases_do_not_count_as_extra_sensors(self):
        descriptions = Path(__file__).resolve().parents[1]
        boards = descriptions.parent / 'depthai-core/shared/depthai_boards/boards'
        if not boards.is_dir():
            self.skipTest('depthai_boards checkout is unavailable')
        args = SimpleNamespace(boards=boards, descriptions=descriptions, reference=None,
                               imu_frame='both', imu_sensor=None,
                               position_tolerance=0.001, angle_tolerance=0.0001)
        for model in ('OAK-D-SR-POE', 'OAK-T', 'PERCEPT'):
            with self.subTest(model=model), contextlib.redirect_stdout(io.StringIO()) as output:
                different = compare.compare_model(model, args)
            self.assertFalse(different, output.getvalue())

    def test_equivalent_rotations(self):
        first = compare.pose({'rpy': '0 0 0'})
        second = compare.pose({'rpy': f'0 0 {2 * math.pi}'})
        self.assertTrue(compare.difference(first, second, 0.001, 0.0001)[2])

    def test_difference_units_and_sign(self):
        board = compare.pose({'xyz': '0.006 0 0', 'rpy': f'0 0 {math.pi / 2}'})
        urdf = compare.pose({})
        delta, angle, matches = compare.difference(board, urdf, 0.001, 0.0001)
        self.assertEqual(delta, [6, 0, 0])
        self.assertAlmostEqual(angle, 90)
        self.assertFalse(matches)

    def test_rotated_joint_chain(self):
        robot = ET.fromstring(f'''<robot><link name="root"/><link name="a"/><link name="b"/>
        <joint type="fixed"><parent link="root"/><child link="a"/>
        <origin xyz="1 0 0" rpy="0 0 {math.pi / 2}"/></joint>
        <joint type="fixed"><parent link="a"/><child link="b"/>
        <origin xyz="1 0 0"/></joint></robot>''')
        poses = compare.link_poses(robot)
        self.assertAlmostEqual(poses['b'][1][0], 1)
        self.assertAlmostEqual(poses['b'][1][1], 1)
        aligned = compare.geometry.compose(compare.geometry.inverse(poses['a']), poses['b'])
        self.assertAlmostEqual(aligned[1][0], 1)
        self.assertAlmostEqual(aligned[1][1], 0)

    def test_disconnected_tree(self):
        with self.assertRaisesRegex(ValueError, 'one root'):
            compare.link_poses(ET.fromstring('<robot><link name="a"/><link name="b"/></robot>'))


if __name__ == '__main__':
    unittest.main()

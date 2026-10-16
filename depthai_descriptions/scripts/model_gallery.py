#!/usr/bin/env python3
"""Build a single URDF gallery and publish labels and sensor-axis markers."""
import json
import math
from pathlib import Path
import sys
from xml.etree import ElementTree as ET

import yaml
from ament_index_python.packages import get_package_share_directory

# Also works when imported by the installed launch file.
sys.path.insert(0, str(Path(__file__).resolve().parent))
from resolve_description import expand


def build_gallery(models='', pose_source='board', imu_frame='auto', columns=8, spacing=0.75,
                  share=None, scale='auto'):
    if columns < 1 or not math.isfinite(spacing) or spacing <= 0:
        raise ValueError('columns must be positive and spacing must be finite and positive')
    if pose_source not in ('board', 'legacy') or imu_frame not in ('auto', 'native', 'optical'):
        raise ValueError('Invalid pose_source or imu_frame')
    share = Path(share or get_package_share_directory('depthai_descriptions'))
    boards = yaml.safe_load((share / 'urdf/include/board_models.yaml').read_text())
    legacy = yaml.safe_load((share / 'urdf/include/legacy_models.yaml').read_text())['models']
    selected = list(dict.fromkeys(models.replace(',', ' ').split())) if models.strip() else sorted(set(boards['models']) | set(legacy))
    robot = ET.Element('robot', name='depthai_model_gallery')
    ET.SubElement(robot, 'link', name='gallery_world')
    entries, frames, notices = [], [], []
    if pose_source == 'legacy':
        notices.append('DEPRECATED pose_source=legacy: displaying frozen develop poses where available; select pose_source:=board to migrate.')
    for model in selected:
        if model not in boards['models'] and model not in legacy:
            raise ValueError(f'Cannot display {model}: ' + boards['unsupported'].get(model, 'unknown model'))
        source = pose_source if model in boards['models'] else 'legacy'
        uses_legacy = source == 'legacy' and model in legacy
        variants = [''] if uses_legacy else list(boards['models'][model]['imu_variants']) or ['']
        for sensor in variants:
            index = len(entries)
            prefix = f'gallery_{index:03d}'
            x, y = (index % columns) * spacing, -(index // columns) * spacing
            result = expand(dict(camera_model=model, pose_source=source, model_selection='exact',
                                 imu_sensor=sensor, imu_frame=imu_frame, legacy_frame_aliases='false',
                                 camera_name=prefix, base_frame=prefix, parent_frame='gallery_world',
                                 cam_pos_x=str(x), cam_pos_y=str(y), cam_pos_z='0'), share=share)
            root = ET.fromstring(result['urdf'])
            for element in root:
                if element.tag == 'link' and element.get('name') == 'gallery_world':
                    continue
                for material in element.iter('material'):
                    material.set('name', prefix + '_' + material.get('name', 'material'))
                robot.append(element)
            title = model + (f' / {sensor}' if sensor else '')
            title += ' [' + result['pose_source'] + ']'
            if source != pose_source:
                title += ' (no board description)'
                notices.append(f'{model}: displaying deprecated legacy geometry; no board counterpart exists.')
            entries.append(dict(model=model, sensor=sensor, pose_source=result['pose_source'],
                                frame=prefix, title=title, xyz=[x, y, 0]))
            for link in root.findall('link'):
                name = link.get('name')
                if name.endswith('_camera_frame') or name.endswith('_imu_frame'):
                    frames.append(dict(frame=name, label=name.removeprefix(prefix + '_').removesuffix('_frame'),
                                       imu=name.endswith('_imu_frame')))
    if not models.strip():
        notices.extend(f'{model}: skipped incomplete board ({reason})' for model, reason in boards['unsupported'].items())
    if not entries:
        raise ValueError('Select at least one model')
    # Display-only uniform scaling: preserve all mesh/sensor/IMU relationships.
    factor = max(1.0, 50.0 / math.sqrt(len(entries))) if scale == 'auto' else float(scale)
    if not math.isfinite(factor) or factor <= 0:
        raise ValueError('scale must be auto or a finite positive number')
    for origin in robot.iter('origin'):
        if origin.get('xyz'):
            origin.set('xyz', ' '.join(str(float(v) * factor) for v in origin.get('xyz').split()))
    for mesh in robot.iter('mesh'):
        mesh.set('scale', ' '.join(str(float(v) * factor) for v in mesh.get('scale', '1 1 1').split()))
    for entry in entries:
        entry['xyz'] = [v * factor for v in entry['xyz']]
    return dict(urdf=ET.tostring(robot, encoding='unicode'), entries=entries, frames=frames,
                notices=notices, scale=factor, spacing=spacing * factor)


def rviz_config(gallery, template):
    config = yaml.safe_load(Path(template).read_text())
    manager = config['Visualization Manager']
    points = [entry['xyz'] for entry in gallery['entries']]
    low = [min(p[i] for p in points) for i in (0, 1)]
    high = [max(p[i] for p in points) for i in (0, 1)]
    view = manager['Views']['Current']
    view['Focal Point'] = dict(X=(low[0] + high[0]) / 2, Y=(low[1] + high[1]) / 2, Z=0.0)
    view['Distance'] = 1.8 * (max(high[i] - low[i] for i in (0, 1)) + gallery['spacing'])
    for display in manager['Displays']:
        if display['Class'] == 'rviz_default_plugins/Grid':
            display['Cell Size'] = gallery['spacing']
        elif display['Class'] == 'rviz_default_plugins/TF':
            display['Marker Scale'] *= gallery['scale']
    return config


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from geometry_msgs.msg import Point
    from visualization_msgs.msg import Marker, MarkerArray

    rclpy.init()
    node = Node('model_gallery_markers')
    manifest = json.loads(node.declare_parameter('manifest', '{}').value)
    length = float(node.declare_parameter('axis_length', 0.04).value)
    if not math.isfinite(length) or length <= 0:
        raise ValueError('axis_length must be finite and positive')
    display_scale = manifest.get('scale', 1.0)
    length *= display_scale
    publisher = node.create_publisher(MarkerArray, '/model_gallery/markers',
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    array = MarkerArray()

    def marker(frame, namespace, kind, color):
        m = Marker()
        m.header.frame_id = frame
        m.ns, m.id, m.type, m.action = namespace, len(array.markers), kind, Marker.ADD
        m.pose.orientation.w = 1.0
        m.color.r, m.color.g, m.color.b, m.color.a = (*color, 1.0)
        m.frame_locked = True
        array.markers.append(m)
        return m

    for entry in manifest['entries']:
        m = marker(entry['frame'], 'model_labels', Marker.TEXT_VIEW_FACING, (1.0, 1.0, 1.0))
        m.text = entry['title']
        m.pose.position.z = 0.20 * display_scale
        m.scale.z = 0.045 * display_scale
    for frame in manifest['frames']:
        color = (1.0, 0.2, 0.8) if frame['imu'] else (0.1, 0.9, 1.0)
        m = marker(frame['frame'], 'imu_positions' if frame['imu'] else 'sensor_positions', Marker.SPHERE, color)
        m.scale.x = m.scale.y = m.scale.z = length * 0.2
        m = marker(frame['frame'], 'sensor_labels', Marker.TEXT_VIEW_FACING, color)
        m.text = frame['label']
        m.pose.position.z = length * 1.4
        m.scale.z = length * 0.4
        for axis, color in enumerate(((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))):
            m = marker(frame['frame'], 'sensor_axes', Marker.ARROW, color)
            endpoint = [0.0, 0.0, 0.0]
            endpoint[axis] = length
            m.points = [Point(), Point(x=endpoint[0], y=endpoint[1], z=endpoint[2])]
            m.scale.x, m.scale.y, m.scale.z = length * 0.04, length * 0.1, length * 0.15
    # Repeat for RViz restarts; transient-local also serves late subscribers.
    node.create_timer(1.0, lambda: publisher.publish(array))
    publisher.publish(array)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

import depthai as dai
import math
import sys
from scipy.spatial.transform import Rotation as R
from pathlib import Path
import json

from geometry_msgs.msg import TransformStamped, Transform

import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q


class ImuStaticFramePublisher(Node):

    def __init__(self):
        super().__init__('imu_static_tf2_broadcaster')
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.tf_list = []
            
    def transforms_from_json(self, board):
        #  'world'
        for name in board['cameras']:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.child_frame_id = name + '_json'

            camera = board['cameras'][name]
            if 'extrinsics' in camera:
                extrinsic = camera['extrinsics']
                t.header.frame_id = extrinsic['to_cam'] + '_json'
                t.transform.translation.x = float(extrinsic['specTranslation']['x'])/ 100
                t.transform.translation.y = float(extrinsic['specTranslation']['y'])/ 100
                t.transform.translation.z = float(extrinsic['specTranslation']['z'])/ 100

                rot = extrinsic['rotation']
                quat = quaternion_from_euler(float(rot['r']), float(rot['p']), float(rot['y']))
                
                t.transform.rotation.x = quat[0]
                t.transform.rotation.y = quat[1]
                t.transform.rotation.z = quat[2]
                t.transform.rotation.w = quat[3]
            else:
                # print(f'No to Camera. Set to MAP')
                t.header.frame_id = 'map'
                t.transform.translation.x = 0.0
                t.transform.translation.y = 0.0
                t.transform.translation.z = 0.0

                t.transform.rotation.x = 0.0
                t.transform.rotation.y = 0.0
                t.transform.rotation.z = 0.0
                t.transform.rotation.w = 1.0
            # print(f'Publishing {t}')
            self.tf_list.append(t)
        
        if 'imuExtrinsics' in board:
            extrinsic = board['imuExtrinsics']['sensors']['BNO']['extrinsics']
            
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.child_frame_id = 'imu_json'

            t.header.frame_id = extrinsic['to_cam'] + '_json'
            t.transform.translation.x = float(extrinsic['specTranslation']['x']) / 100
            t.transform.translation.y = float(extrinsic['specTranslation']['y']) / 100
            t.transform.translation.z = float(extrinsic['specTranslation']['z']) / 100

            rot = extrinsic['rotation']
            print(rot)
            # rot_x = R.from_euler('zyx', [rot['y'], rot['p'], rot['r']], degrees=True)
            rot_x = R.from_euler('zyx', [90, 0, 90], degrees=True)
            quat = rot_x.as_quat()

            
            t.transform.rotation.x = quat[0]
            t.transform.rotation.y = quat[1]
            t.transform.rotation.z = quat[2]
            t.transform.rotation.w = quat[3]
            self.tf_list.append(t)
    
    def transformer(self, transformationMat):
        # Rotation matrix to quaternion
        transform = Transform()
        transformationMat = np.array(transformationMat)
        r = R.from_matrix(transformationMat[:3, :3])
        quat = r.as_quat()

        transform.translation.x = transformationMat[0, 3] / 100
        transform.translation.y = transformationMat[1, 3] / 100
        transform.translation.z = transformationMat[2, 3] / 100

        transform.rotation.x = quat[0]
        transform.rotation.y = quat[1]
        transform.rotation.z = quat[2]
        transform.rotation.w = quat[3]
        return transform

    # THis is WIP
    def transform_from_handler(self, calibHandler):
        stamp = self.get_clock().now().to_msg()

        t = TransformStamped()        
        t.header.stamp = stamp

        t.header.frame_id = 'map'
        t.child_frame_id = 'cam_c_handler'

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_list.append(t)
        
        extrinsics = calibHandler.getCameraExtrinsics(dai.CameraBoardSocket.CAM_B, dai.CameraBoardSocket.CAM_C, True)
        t = TransformStamped()        
        t.header.stamp = stamp
        t.header.frame_id = 'cam_c_handler'
        t.child_frame_id = 'cam_b_handler'
        t.transform = self.transformer(extrinsics)
        self.tf_list.append(t)

        extrinsics = calibHandler.getCameraExtrinsics(dai.CameraBoardSocket.CAM_A, dai.CameraBoardSocket.CAM_C, True)
        t = TransformStamped()        
        t.header.stamp = stamp
        t.header.frame_id = 'cam_c_handler'
        t.child_frame_id = 'cam_a_handler'
        t.transform = self.transformer(extrinsics)
        self.tf_list.append(t)

        # extrinsics = calibHandler.getCameraToImuExtrinsics(dai.CameraBoardSocket.CAM_B, True)
        extrinsics = calibHandler.getImuToCameraExtrinsics(dai.CameraBoardSocket.CAM_B, True)
        t = TransformStamped()        
        t.header.stamp = stamp
        t.header.frame_id = 'cam_b_handler'
        t.child_frame_id = 'imu_handler'
        t.transform = self.transformer(extrinsics)
        # print(f'IMU wrt Cam B ------------> \n {np.array(extrinsics)}')
        self.tf_list.append(t)
    
    def publish_tf(self):
        self.tf_static_broadcaster.sendTransform(self.tf_list)


def main():
    logger = rclpy.logging.get_logger('logger')
    print(sys.argv)
    if len(sys.argv) != 2:
        logger.info('Invalid number of parameters. Usage: \n')
        sys.exit(1)
    print(sys.argv[1])
    path = Path(sys.argv[1])
    
    with open(path, 'r') as f:
        board = json.load(f)['board_config']
    device = dai.Device()
    calibHandler = device.readCalibration()
    
    rclpy.init()

    tf_node = ImuStaticFramePublisher()
    tf_node.transform_from_handler(calibHandler)
    tf_node.transforms_from_json(board)
    tf_node.publish_tf()
    try:
        rclpy.spin(tf_node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
 
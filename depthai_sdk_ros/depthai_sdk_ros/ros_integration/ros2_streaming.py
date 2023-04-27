
 

import logging
from threading import Thread
from typing import Dict, Any
from queue import Queue
import rclpy
from .ros_base import RosBase
from .depthai2ros2 import DepthAi2Ros2
from sensor_msgs.msg import Image, CameraInfo
import sys
def ros_thread(bridge, queue: Queue):
    rclpy.init()
    node = rclpy.create_node('DepthAI_SDK')
    publishers = dict()

    while rclpy.ok():
            msgs: Dict[str, Any] = queue.get(block=True)
            for topic, msg in msgs.items():
                if topic not in publishers:
                    logging.info(f'SDK started publishing ROS messages to{topic}')
                    if(type(msg)==Image):
                        prefix = topic.split('/')
                        cam_info_topic = '/' + prefix[1] + '/camera_info'
                        publishers[cam_info_topic] = node.create_publisher(CameraInfo, cam_info_topic, 10)
                    publishers[topic] = node.create_publisher(type(msg), topic, 10)
                if(type(msg)==Image):
                    prefix = topic.split('/')
                    cam_info_topic = '/' + prefix[1] + '/camera_info'
                    info = bridge.get_calib(topic.split('/')[1], msg)
                    publishers[cam_info_topic].publish(info)
                publishers[topic].publish(msg)

                rclpy.spin_once(node, timeout_sec=0.001)  # 1ms timeout


class Ros2Streaming(RosBase):

    def __init__(self, device):
        super().__init__()
        self.queue = Queue(30)
        self.bridge = DepthAi2Ros2(device)
        self.process = Thread(target=ros_thread, args=(self.bridge, self.queue,))
        self.process.start()

    # def update(self): # By RosBase
    # def new_msg(self): # By RosBase

    def new_ros_msg(self, topic: str, ros_msg):
        self.queue.put({topic: ros_msg})
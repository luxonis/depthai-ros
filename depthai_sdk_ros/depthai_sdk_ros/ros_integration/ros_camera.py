import os
from depthai_sdk import OakCamera
from depthai import UsbSpeed
from depthai import Pipeline
from typing import Dict, Any, Optional, List, Union, Callable
from depthai_sdk.oak_outputs.xout.xout_frames import XoutFrames
from depthai_sdk.components.component import Component
from depthai_sdk.classes.output_config import BaseConfig
from depthai_sdk.oak_outputs.xout.xout_base import XoutBase




class RosStreamConfig(BaseConfig):
    outputs: List[Callable]
    ros = None

    def __init__(self, outputs: List[Callable]):
        self.outputs = outputs

    def setup(self, pipeline: Pipeline, device, names: List[str]) -> List[XoutBase]:
        xouts: List[XoutFrames] = []
        for output in self.outputs:
            xoutbase: XoutFrames = output(pipeline, device)
            xoutbase.setup_base(None)
            xouts.append(xoutbase)

        envs = os.environ
        if 'ROS_VERSION' not in envs:
            raise Exception('ROS installation not found! Please install or source the ROS you would like to use.')

        version = envs['ROS_VERSION']
        if version == '1':
            raise Exception('ROS1 publsihing is not yet supported!')
            from depthai_sdk.integrations.ros.ros1_streaming import Ros1Streaming
            self.ros = Ros1Streaming()
        elif version == '2':
            from .ros2_streaming import Ros2Streaming
            self.ros = Ros2Streaming(device)
        else:
            raise Exception(f"ROS version '{version}' not recognized! Should be either '1' or '2'")
        self.ros.update(xouts)
        return [self]

    def new_msg(self, name, msg):
        self.ros.new_msg(name, msg)
    def check_queue(self, block):
        pass  # No queues
    def start_fps(self):
        pass

    # def is_ros1(self) -> bool:
    #     try:
    #         import rospy
    #         return True
    #     except:
    #         return False
    #
    # def is_ros2(self):
    #     try:
    #         import rclpy
    #         return True
    #     except:
    #         return False



class ROSCamera(OakCamera):
    def __init__(self, device: str | None = None, usb_speed: str | UsbSpeed | None = None, replay: str | None = None, rotation: int = 0, args: bool | Dict = True):
        super().__init__(device, usb_speed, replay, rotation, args)
    def ros_stream(self, output: Union[List, Callable, Component]):
        self._out_templates.append(RosStreamConfig(self._get_component_outputs(output)))

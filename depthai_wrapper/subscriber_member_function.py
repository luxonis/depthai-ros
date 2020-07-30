# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
import sys
import cv2
import pickle
import json

from depthai_helpers.config_manager import DepthConfigManager
from depthai_helpers.arg_manager import SharedArgs, CliArgs

from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import ParameterType

from std_msgs.msg import String, UInt8MultiArray


class DepthAISubscriber(Node):
    meta = []
    def __init__(self):
        # note the topic name will change if a device is specified. the name will be <stream>+<device_id>. eg preview3.
        super().__init__('depthai_subscriber')

        # TODO: have the publisher tell us what kind if network to decode for.
        from depthai_helpers.mobilenet_ssd_handler import decode_mobilenet_ssd, show_mobilenet_ssd
        show_nn=show_mobilenet_ssd

        # setup our params
        self.paramName = "cliArgs"
        paramDefault = ""
        paramDesc = ParameterDescriptor(type=ParameterType.PARAMETER_STRING,
                                              description="arguments that match the command line script depthai-demo.py input")
        self.declare_parameter(self.paramName, paramDefault, paramDesc)

        # get param
        my_param = self.get_parameter(self.paramName).get_parameter_value().string_value
        new_argv = [sys.argv[0]]
        if my_param is not "":
            paramList = my_param.split(" ")
            for param in paramList:
                new_argv.append(param)

        # setup argv for parsing.
        self.old_argv = sys.argv
        sys.argv = new_argv

        # parse params
        cliArgs = CliArgs()
        args = vars(cliArgs.parse_args())
        print(args)

        self.configMan = DepthConfigManager(args)

        self.previewsub = self.create_subscription(
            UInt8MultiArray,
            'preview'+args['device_id'],
            self.preview_callback,
            10)
        self.previewsub  # prevent unused variable warning

        self.metasub = self.create_subscription(
            String,
            'meta'+args['device_id'],
            self.meta_callback,
            10)
        self.metasub  # prevent unused variable warning

    def preview_callback(self, msg):
        serializedFrame = bytearray(msg.data)
        frame = pickle.loads(serializedFrame)

        nn_frame = self.configMan.show_nn(self.meta, frame, labels=self.configMan.labels, config=self.configMan.jsonConfig)

        cv2.imshow("preview", nn_frame)
        key = cv2.waitKey(1)


    def meta_callback(self, msg):
        self.meta = json.loads(msg.data)



def main(args=None):
    rclpy.init(args=args)

    depthai_subscriber = DepthAISubscriber()

    rclpy.spin(depthai_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    depthai_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

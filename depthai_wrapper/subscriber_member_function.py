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
import json

import msgpack
import msgpack_numpy as m
import numpy as np

from depthai_helpers.config_manager import DepthConfigManager
from depthai_helpers.arg_manager import SharedArgs, CliArgs

from rclpy.node import Node
from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import SetParameters, GetParameters, ListParameters



from std_msgs.msg import String, UInt8MultiArray


class DepthAISubscriber(Node):
    meta = []

    def __init__(self):
        # note the topic name will change if a device is specified. the name will be <stream>+<device_id>. eg preview3.
        super().__init__('depthai_subscriber')

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
        self.args = vars(cliArgs.parse_args())
        print(self.args)

        self.configMan = DepthConfigManager(self.args)

        # "." is not allowed in topic names so we replaced "." with "_" on the server.
        self.targetDev = self.args['device_id'].replace(".", "_")

        # get nn2depth params, if necessary.
        if self.args['draw_bb_depth']:
            self.nn2depth = self.get_nn2depth_sync(self.targetDev)
            print(self.nn2depth)


        # subscribe to topics
        self.previewSubName = 'preview'+self.targetDev
        self.previewSub = self.create_subscription(
            UInt8MultiArray,
            self.previewSubName,
            self.preview_callback,
            10)
        self.previewSub  # prevent unused variable warning

        self.leftSubName = 'left'+self.targetDev
        self.leftSub = self.create_subscription(
            UInt8MultiArray,
            self.leftSubName,
            self.left_callback,
            10)
        self.leftSub  # prevent unused variable warning

        self.rightSubName = 'right'+self.targetDev
        self.rightSub = self.create_subscription(
            UInt8MultiArray,
            self.rightSubName,
            self.right_callback,
            10)
        self.rightSub  # prevent unused variable warning

        self.disparitySubName = 'disparity'+self.targetDev
        self.disparitySub = self.create_subscription(
            UInt8MultiArray,
            self.disparitySubName,
            self.disparity_callback,
            10)
        self.disparitySub  # prevent unused variable warning

        self.depthSubName = 'depth'+self.targetDev
        self.depthSub = self.create_subscription(
            UInt8MultiArray,
            self.depthSubName,
            self.depth_callback,
            10)
        self.depthSub  # prevent unused variable warning

        self.d2hSubName = 'd2h'+self.targetDev
        self.d2hSub = self.create_subscription(
            String,
            self.d2hSubName,
            self.d2h_callback,
            10)
        self.depthSub  # prevent unused variable warning

        self.metaSubName = 'meta'+self.targetDev
        self.metaSub = self.create_subscription(
            String,
            self.metaSubName,
            self.meta_callback,
            10)
        self.metaSub  # prevent unused variable warning


    def get_nn2depth_sync(self, device_id):
        req = GetParameters.Request()
        req.names = ['nn2depth' + device_id]
        client = self.create_client(GetParameters, '/depthai_publisher/get_parameters')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, nn2depth specified so depthai_wrapper needs to be running before starting a listener.')

        self.future = client.call_async(req)
        rclpy.spin_until_future_complete(self,self.future)
        if self.future.done():
            try:
                response = self.future.result()
            except Exception as e:
                self.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                return response.values[0].string_value

        return ""



    def preview_callback(self, msg):
        serializedFrame = bytearray(msg.data)
        frame = msgpack.unpackb(serializedFrame, object_hook=m.decode)

        nn_frame = self.configMan.show_nn(self.meta, frame, labels=self.configMan.labels, config=self.configMan.jsonConfig)
        cv2.imshow(self.previewSubName, nn_frame)
        key = cv2.waitKey(1)

    def left_callback(self, msg):
        self.left_right_disparity_callback(msg, self.leftSubName)

    def right_callback(self, msg):
        self.left_right_disparity_callback(msg, self.rightSubName)

    def disparity_callback(self, msg):
        self.left_right_disparity_callback(msg, self.disparitySubName)

    def left_right_disparity_callback(self, msg, streamName):
        serializedFrame = bytearray(msg.data)
        if len(serializedFrame) > 0:
            frame = msgpack.unpackb(serializedFrame, object_hook=m.decode)

            if self.args['draw_bb_depth']:
                camera = self.args['cnn_camera']
                if streamName == 'disparity':
                    if camera == 'left_right':
                        camera = 'right'
                elif camera != 'rgb':
                    camera = packet.getMetadata().getCameraName()
                self.configMan.show_nn(self.meta, frame, labels=self.configMan.labels, config=self.configMan.jsonConfig)

            cv2.imshow(streamName, frame)
            key = cv2.waitKey(1)

    def depth_callback(self, msg):
        serializedFrame = bytearray(msg.data)
        frame = msgpack.unpackb(serializedFrame, object_hook=m.decode)

        if len(frame.shape) == 2:
            if frame.dtype == np.uint8: # grayscale
                cv2.putText(frame, self.depthSubName, (25, 25), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255))
            else: # uint16
                frame = (65535 // frame).astype(np.uint8)
                #colorize depth map, comment out code below to obtain grayscale
                frame = cv2.applyColorMap(frame, cv2.COLORMAP_HOT)
                # frame = cv2.applyColorMap(frame, cv2.COLORMAP_JET)
                cv2.putText(frame, self.depthSubName, (25, 25), cv2.FONT_HERSHEY_SIMPLEX, 1.0, 255)
        else: # bgr
            cv2.putText(frame, self.depthSubName, (25, 25), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255))

        if self.args['draw_bb_depth']:
            camera = self.args['cnn_camera']
            if camera == 'left_right':
                camera = 'right'
            self.configMan.show_nn(self.meta, frame, labels=self.configMan.labels, config=self.configMan.jsonConfig, nn2depth=self.nn2depth)
        cv2.imshow(self.depthSubName, frame)
        key = cv2.waitKey(1)


    def d2h_callback(self, msg):
        dict_ = json.loads(msg.data)
        print('meta_d2h Temp',
            ' CSS:' + '{:6.2f}'.format(dict_['sensors']['temperature']['css']),
            ' MSS:' + '{:6.2f}'.format(dict_['sensors']['temperature']['mss']),
            ' UPA:' + '{:6.2f}'.format(dict_['sensors']['temperature']['upa0']),
            ' DSS:' + '{:6.2f}'.format(dict_['sensors']['temperature']['upa1']))

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

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
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray, String, UInt8MultiArray
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import ParameterType


# get path to DepthAI stuff.
import sys
#sys.path.append("/home/jngai/Desktop/depthai/")

# pull in the depthai stuff.
import depthai
import cv2
import pickle
import json

from time import time, sleep, monotonic

from depthai_helpers import utils
from depthai_helpers.cli_utils import cli_print, PrintColors

from depthai_helpers.object_tracker_handler import show_tracklets

from depthai_helpers.config_manager import DepthConfigManager
from depthai_helpers.arg_manager import SharedArgs, CliArgs

from depthai_helpers.mobilenet_ssd_handler import decode_mobilenet_ssd_convert_json

"""
class Ros2Args(SharedArgs):
    args = []
    ros2Node = None
    ros2ParamDict = {}
    ros2TypeDict = {
        "bool":ParameterType.PARAMETER_BOOL,
        "int":ParameterType.PARAMETER_INTEGER,
        "float":ParameterType.PARAMETER_DOUBLE,
        "double":ParameterType.PARAMETER_DOUBLE,
        "str":ParameterType.PARAMETER_STRING, 
        "byteArray":ParameterType.PARAMETER_BYTE_ARRAY,
        "boolArray":ParameterType.PARAMETER_BOOL_ARRAY,
        "intArray":ParameterType.PARAMETER_INTEGER_ARRAY,
        "floatArray":ParameterType.PARAMETER_DOUBLE_ARRAY,
        "doubleArray":ParameterType.PARAMETER_DOUBLE_ARRAY,
        "strArray":ParameterType.PARAMETER_STRING_ARRAY
        }

    def __init__(self, ros2Node):
        super().__init__()
        self.ros2Node = ros2Node

    def printArgs(self):
        print(self.args.longName)

    def initializeParams(self):
        #TODO: use actual params to setup the package. 
        # ROS2 doesn't really have a None option for arguments and our CLI arg parser relies on this. 
        # We'll need to substitute None with placeholders here and translate back?
        # For now we'll just take the cli arguments as a parameter, make a sys.argv out of it and run argparse.
        for arg in self.args:
            cleanLongName = arg.longName.strip('-')
            print(cleanLongName)
            print(arg.ros2Type)

            paramDesc = ParameterDescriptor(type=self.ros2TypeDict[arg.ros2Type],
                                                  description=arg.help)
            self.ros2Node.declare_parameter(cleanLongName, arg.default, paramDesc)

            self.ros2ParamDict[cleanLongName] = rclpy.parameter.Parameter(
                cleanLongName,
                rclpy.Parameter.Type.STRING,
                arg.default
            )
"""

#-------------------------------------------------------
# TODO: should move the watchdog stuff to a shared class
#-------------------------------------------------------
process_watchdog_timeout=10 #seconds
def reset_process_wd():
    global wd_cutoff
    wd_cutoff=monotonic()+process_watchdog_timeout
    return
#-------------------------------------------------------

class DepthAIPublisher(Node):
    def __init__(self):
        super().__init__('depthai_publisher')

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

        configMan = DepthConfigManager(args)
        cmd_file, debug_mode = configMan.getCommandFile()
        usb2_mode = configMan.getUsb2Mode()
        decode_nn = configMan.decode_nn
        show_nn = configMan.show_nn
        labels = configMan.labels

        # This json file is sent to DepthAI. It communicates what options you'd like to enable and what model you'd like to run.
        config = configMan.jsonConfig

        # setup publishers
        self.previewPublisher = self.create_publisher(UInt8MultiArray, 'preview'+args['device_id'], 10)
        self.previewmsg = UInt8MultiArray()
        self.nnResultPublisher = self.create_publisher(String, 'meta'+args['device_id'], 10)
        self.nnmsg = String()

        # Create a list of enabled streams ()
        stream_names = [stream if isinstance(stream, str) else stream['name'] for stream in configMan.stream_list]
        enable_object_tracker = 'object_tracker' in stream_names

        device = None
        if debug_mode: 
            print('Cmd file: ', cmd_file, ' args["device_id"]: ', args['device_id'])
            device = depthai.Device(cmd_file, args['device_id'])
        else:
            device = depthai.Device(args['device_id'], usb2_mode)

        print(stream_names)
        print('Available streams: ' + str(device.get_available_streams()))

        # create the pipeline, here is the first connection with the device
        pipeline = device.create_pipeline(config=config)

        if pipeline is None:
            print('Pipeline is not created.')
            exit(3)


        #-------------------------------------------------------
        # another chunk of stuff that could go in a shared class?
        #-------------------------------------------------------
        nn2depth = device.get_nn_to_depth_bbox_mapping()

        t_start = time()
        frame_count = {}
        frame_count_prev = {}
        nnet_prev = {}
        nnet_prev["entries_prev"] = {}
        nnet_prev["nnet_source"] = {}
        frame_count['nn'] = {}
        frame_count_prev['nn'] = {}

        NN_cams = {'rgb', 'left', 'right'}

        for cam in NN_cams:
            nnet_prev["entries_prev"][cam] = []
            nnet_prev["nnet_source"][cam] = []
            frame_count['nn'][cam] = 0
            frame_count_prev['nn'][cam] = 0
        #-------------------------------------------------------


        stream_windows = []
        for s in stream_names:
            if s == 'previewout':
                for cam in NN_cams:
                    stream_windows.append(s + '-' + cam)
            else:
                stream_windows.append(s)

        for w in stream_windows:
            frame_count[w] = 0
            frame_count_prev[w] = 0

        tracklets = None


        #-------------------------------------------------------
        # start watchdog
        reset_process_wd()
        #-------------------------------------------------------

        while True:
            # retreive data from the device
            # data is stored in packets, there are nnet (Neural NETwork) packets which have additional functions for NNet result interpretation
            nnet_packets, data_packets = pipeline.get_available_nnet_and_data_packets(True)

            #-------------------------------------------------------
            # TODO: should move the watchdog stuff to a shared class
            #-------------------------------------------------------
            packets_len = len(nnet_packets) + len(data_packets)
            if packets_len != 0:
                reset_process_wd()
            else:
                cur_time=monotonic()
                if cur_time > wd_cutoff:
                    print("process watchdog timeout")
                    os._exit(10)
            #-------------------------------------------------------

            for _, nnet_packet in enumerate(nnet_packets):
                meta = nnet_packet.getMetadata()
                camera = 'rgb'
                if meta != None:
                    camera = meta.getCameraName()
                nnet_prev["nnet_source"][camera] = nnet_packet
                nnet_prev["entries_prev"][camera] = decode_nn(nnet_packet, config=config)

                #TODO: Is there a reason we aren't returning a json object instead of having custom decode_nn methods for each network?
                # Gonna do this the dirty way for now...
                serializedEntry = decode_mobilenet_ssd_convert_json(decode_nn(nnet_packet, config=config))

                self.nnmsg.data = str(serializedEntry)
                self.nnResultPublisher.publish(self.nnmsg)
                
                frame_count['metaout'] += 1
                frame_count['nn'][camera] += 1


            for packet in data_packets:
                window_name = packet.stream_name
                if packet.stream_name not in stream_names:
                    continue # skip streams that were automatically added
                packetData = packet.getData()
                if packetData is None:
                    print('Invalid packet data!')
                    continue
                elif packet.stream_name == 'previewout':
                    #broadcast to previewout
                    meta = packet.getMetadata()
                    camera = 'rgb'
                    if meta != None:
                        camera = meta.getCameraName()

                    window_name = 'previewout-' + camera
                    # the format of previewout image is CHW (Chanel, Height, Width), but OpenCV needs HWC, so we
                    # change shape (3, 300, 300) -> (300, 300, 3)
                    data0 = packetData[0,:,:]
                    data1 = packetData[1,:,:]
                    data2 = packetData[2,:,:]
                    frame = cv2.merge([data0, data1, data2])

                    #------------------------------------------------
                    serializedFrame = pickle.dumps(frame)
                    intarr = list(serializedFrame)

                    #start = time()
                    # this takes 0.04 seconds for a 300x300 image. That's way too slow. 
                    # Oh, this is actually because __debug__ is enabled and it's importing a bunch of stuff inline... This should be fixed on ros2's side or we need a way to disable __debug__.
                    self.previewmsg.data = intarr
                    #end = time()

                    self.previewPublisher.publish(self.previewmsg)

                    #print(end - start)
                    #------------------------------------------------

                elif packet.stream_name == 'left' or packet.stream_name == 'right' or packet.stream_name == 'disparity':
                    frame_bgr = packetData
                    cv2.putText(frame_bgr, packet.stream_name, (25, 25), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0))
                    cv2.putText(frame_bgr, "fps: " + str(frame_count_prev[window_name]), (25, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0))
                    if args['draw_bb_depth']:
                        camera = args['cnn_camera']
                        if packet.stream_name == 'disparity':
                            if camera == 'left_right':
                                camera = 'right'
                        elif camera != 'rgb':
                            camera = packet.getMetadata().getCameraName()
                        show_nn(nnet_prev["entries_prev"][camera], frame_bgr, labels=labels, config=config, nn2depth=nn2depth)

                    
                    cv2.imshow(window_name, frame_bgr)
                elif packet.stream_name.startswith('depth'):
                    frame = packetData

                    if len(frame.shape) == 2:
                        if frame.dtype == np.uint8: # grayscale
                            cv2.putText(frame, packet.stream_name, (25, 25), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255))
                            cv2.putText(frame, "fps: " + str(frame_count_prev[window_name]), (25, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255))
                        else: # uint16
                            frame = (65535 // frame).astype(np.uint8)
                            #colorize depth map, comment out code below to obtain grayscale
                            frame = cv2.applyColorMap(frame, cv2.COLORMAP_HOT)
                            # frame = cv2.applyColorMap(frame, cv2.COLORMAP_JET)
                            cv2.putText(frame, packet.stream_name, (25, 25), cv2.FONT_HERSHEY_SIMPLEX, 1.0, 255)
                            cv2.putText(frame, "fps: " + str(frame_count_prev[window_name]), (25, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, 255)
                    else: # bgr
                        cv2.putText(frame, packet.stream_name, (25, 25), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255))
                        cv2.putText(frame, "fps: " + str(frame_count_prev[window_name]), (25, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, 255)

                    if args['draw_bb_depth']:
                        camera = args['cnn_camera']
                        if camera == 'left_right':
                            camera = 'right'
                        show_nn(nnet_prev["entries_prev"][camera], frame, labels=labels, config=config, nn2depth=nn2depth)
                    cv2.imshow(window_name, frame)

                elif packet.stream_name == 'jpegout':
                    jpg = packetData
                    mat = cv2.imdecode(jpg, cv2.IMREAD_COLOR)
                    cv2.imshow('jpegout', mat)

                elif packet.stream_name == 'video':
                    videoFrame = packetData
                    videoFrame.tofile(video_file)
                    #mjpeg = packetData
                    #mat = cv2.imdecode(mjpeg, cv2.IMREAD_COLOR)
                    #cv2.imshow('mjpeg', mat)

                elif packet.stream_name == 'meta_d2h':
                    str_ = packet.getDataAsStr()
                    dict_ = json.loads(str_)

                    print('meta_d2h Temp',
                        ' CSS:' + '{:6.2f}'.format(dict_['sensors']['temperature']['css']),
                        ' MSS:' + '{:6.2f}'.format(dict_['sensors']['temperature']['mss']),
                        ' UPA:' + '{:6.2f}'.format(dict_['sensors']['temperature']['upa0']),
                        ' DSS:' + '{:6.2f}'.format(dict_['sensors']['temperature']['upa1']))
                elif packet.stream_name == 'object_tracker':
                    tracklets = packet.getObjectTracker()

                frame_count[window_name] += 1

            key = cv2.waitKey(1)



def main(args=None):
    rclpy.init(args=args)
    print('depthai.__version__ == %s' % depthai.__version__)
    depthai_publisher = DepthAIPublisher()

    rclpy.spin(depthai_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    depthai_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

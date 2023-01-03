#!/usr/bin/env python3
# Copyright (c) [2022] [Adam Serafin]

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the 'Software'), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED 'AS IS', WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import rospy
from vision_msgs.msg import Detection3DArray
from visualization_msgs.msg import ImageMarker, MarkerArray, Marker
from geometry_msgs.msg import Point, Pose, Vector3
from std_msgs.msg import ColorRGBA, String
from foxglove_msgs.msg import ImageMarkerArray


class ObjectPublisher():

    def __init__(self):
        rospy.init_node('object_publisher')
        self._sub_ = rospy.Subscriber(
            '/oak/nn/detections', Detection3DArray, self.publish_data, queue_size=1)
        self._det_pub = rospy.Publisher(
            '/oak/nn/detection_markers', ImageMarkerArray, queue_size=1)
        self._text_pub = rospy.Publisher(
            '/oak/nn/text_markers', MarkerArray, queue_size=1)
        self._unique_id = 0

        rospy.loginfo('ObjectPublisher node Up!')

    def publish_data(self, msg: Detection3DArray):
        markerArray = ImageMarkerArray()
        textMarker = MarkerArray()
        i = 0
        if self._unique_id > 50:
            self._unique_id = 0
        for det in msg.detections:
            bbox = det.bbox
            det.results[0]
            label = f'{det.results[0].id}_{i + self._unique_id}'
            det_pose = det.results[0].pose.pose
            textMarker.markers.append(Marker(
                    header=msg.header,
                    id=i + self._unique_id,
                    scale=Vector3(x=0.1, y=0.1, z=0.1),
                    type=Marker.TEXT_VIEW_FACING,
                    color=ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0),
                    action=0,
                    text=label,
                    pose=det_pose
                )
                )

            markerArray.markers.append(ImageMarker(
                    header=msg.header,
                    id=i + self._unique_id,
                    scale=1.0,
                    filled=1,
                    type=ImageMarker.LINE_STRIP,
                    outline_color=ColorRGBA(r=0.0, g=255.0, b=255.0, a=255.0),
                    fill_color=ColorRGBA(b=255.0, a=0.2),
                    points=[
                        Point(x=bbox.center.position.x - bbox.size.x/2, y=bbox.center.position.y + bbox.size.y/2, z=0.0),
                        Point(x=bbox.center.position.x + bbox.size.x/2, y=bbox.center.position.y + bbox.size.y/2, z=0.0),
                        Point(x=bbox.center.position.x + bbox.size.x/2, y=bbox.center.position.y - bbox.size.y/2, z=0.0),
                        Point(x=bbox.center.position.x - bbox.size.x/2, y=bbox.center.position.y - bbox.size.y/2, z=0.0),
                        Point(x=bbox.center.position.x - bbox.size.x/2, y=bbox.center.position.y + bbox.size.y/2, z=0.0),
                    ],
                ))

            i += 1
            self._unique_id += 1

        self._det_pub.publish(markerArray)
        self._text_pub.publish(textMarker)


def main(args=None):
    ObjectPublisher()
    rospy.spin()


if __name__ == '__main__':
    main()

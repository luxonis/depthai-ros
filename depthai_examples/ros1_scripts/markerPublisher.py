import rospy
from depthai_ros_msgs.msg import SpatialDetectionArray
from foxglove_msgs.msg import ImageMarkerArray
from visualization_msgs.msg import ImageMarker

from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA, String



class SpatialDetectionsVisualizationNode:
    def __init__(self):
       rospy.init_node("my_node")
       rospy.Subscriber("spatialDetections", SpatialDetectionArray, self.detectionCallback, queue_size=1)

       # On initialization, set up a Publisher for ImageMarkerArrays
       self.pubMarkers = rospy.Publisher("spatialDetectionMarkers", ImageMarkerArray, queue_size=1)
    #    self.pubTextMarkers = rospy.Publisher("spatialDetectionTextMarkers", ImageMarkerArray, queue_size=1)
       
       rospy.spin()

    def detectionCallback(self, spatialMsgArray):
        markers = ImageMarkerArray()
        textMarker = ImageMarkerArray()
        
        for spatialMsg in spatialMsgArray.detections:
            bbox = spatialMsg.bbox
            detectionID = None
            score = 0
            label = None

            for result in spatialMsg.results:
                if result.score > score:
                    detectionID = result.id
                    score = result.score

            position = spatialMsg.position
            label = f'ID: {detectionID} \n score: {score} \n x: {position.x} \n y: {position.y} \n z: {position.z}'

            """ textMarker.markers.append(ImageMarker(
                    header=spatialMsgArray.header,
                    id=detectionID,
                    scale=2,
                    filled=1,
                    type=ImageMarker.TEXT,
                    outline_color=ColorRGBA(0, 1, 1, 1),
                    fill_color=ColorRGBA(b=255.0, a=0.2),
                    text=String(data=label),
                    position=Point(bbox.center.x - bbox.size_x/2, bbox.center.y + bbox.size_y/2, 0)
                )
                ) """
            markers.markers.append(ImageMarker(
                    header=spatialMsgArray.header,
                    id=detectionID,
                    scale=1,
                    filled=1,
                    type=ImageMarker.LINE_STRIP,
                    outline_color=ColorRGBA(0, 1, 1, 1),
                    fill_color=ColorRGBA(b=255.0, a=0.2),
                    points=[
                        Point(bbox.center.x - bbox.size_x/2, bbox.center.y + bbox.size_y/2, 0),
                        Point(bbox.center.x + bbox.size_x/2, bbox.center.y + bbox.size_y/2, 0),
                        Point(bbox.center.x + bbox.size_x/2, bbox.center.y - bbox.size_y/2, 0),
                        Point(bbox.center.x - bbox.size_x/2, bbox.center.y - bbox.size_y/2, 0),
                        Point(bbox.center.x - bbox.size_x/2, bbox.center.y + bbox.size_y/2, 0),
                    ],
                )
                )
        self.pubMarkers.publish(markers)
        # self.pubTextMarkers.publish(textMarker)
        

def main():
    SpatialDetectionsVisualizationNode()


if __name__ == "__main__":
    main()
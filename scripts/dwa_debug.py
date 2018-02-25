#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from dwa_debug_tools.msg import EvaluationDataBase
from dwa_debug_tools.msg import EvaluationValue
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='totalEval', offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name='distEval', offset=16, datatype=PointField.FLOAT32, count=1),
    PointField(name='timeEval', offset=20, datatype=PointField.FLOAT32, count=1),
]

class CustomPointCloud(object):
    def __init__(self):
        rospy.init_node('publish_custom_point_cloud')
        self.pubDw = rospy.Publisher('/dynamic_window', PointCloud2, queue_size=1)
        self.pubDwaCmd = rospy.Publisher('/dwa_cmd_vel', MarkerArray, queue_size=1)
        self.subscriver = rospy.Subscriber("/dwa_eval", EvaluationDataBase, self.onDWAEvalMsg)
        self.frameId = 'base'

    def onDWAEvalMsg(self, evalDB):
        self.POINTS = [[]]

        markerArray = MarkerArray()
        maxTotalInput = []
        maxTimeInput = []
        maxDistInput = []
        maxTotalEval = 0.0
        maxTimeEval = 0.0
        maxDistEval = 0.0
        for i in range(len(evalDB.value)):
            self.POINTS.append([evalDB.value[i].vx, evalDB.value[i].vy, 0.0, evalDB.value[i].totalEval, evalDB.value[i].distEval, evalDB.value[i].timeEval])
            if maxTotalEval < evalDB.value[i].totalEval:
                maxTotalEval = evalDB.value[i].totalEval
                maxTotalInput = [evalDB.value[i].vx, evalDB.value[i].vy]
            if maxTimeEval < evalDB.value[i].timeEval:
                maxTimeEval = evalDB.value[i].timeEval
                maxTimeInput = [evalDB.value[i].vx, evalDB.value[i].vy]
            if maxDistEval < evalDB.value[i].distEval:
                maxDistEval = evalDB.value[i].distEval
                maxDistInput = [evalDB.value[i].vx, evalDB.value[i].vy]

        del self.POINTS[0]

        HEADER = Header(frame_id=self.frameId)
        point_cloud = pc2.create_cloud(HEADER, FIELDS, self.POINTS)
        self.pubDw.publish(point_cloud)

        self.setMarker(maxTotalInput, markerArray, [1,1,0])
        self.setMarker(maxTimeInput, markerArray, [1,1,1])
        self.setMarker(maxDistInput, markerArray, [0,0,0])
        self.pubDwaCmd.publish(markerArray)



    def setMarker(self, maxInput, markerArray, color):
        marker = Marker()
        marker.header.frame_id = self.frameId
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.id = len(markerArray.markers)
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = maxInput[0]
        marker.pose.position.y = maxInput[1]
        marker.pose.position.z = 0.0
        markerArray.markers.append(marker)



def main():
    try:
        custom_point_cloud = CustomPointCloud()
        rospy.spin()
        # custom_point_cloud.publish_points()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':

    main()

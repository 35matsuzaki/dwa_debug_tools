#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from dwa_debug_tools.msg import EvaluationDataBase
from dwa_debug_tools.msg import EvaluationValue

RANGE_X0 = -1 # min col x
RANGE_X1 = 11 # max col x
RANGE_Y0 = -1 # min row y
RANGE_Y1 = 13 # max row y
MESH_SIZE = 0.1 #[m]
FILE_NAME = "/home/effinger/ws_w1/src/magnetic_localization_ros/maps/10cm/mag_map.txt"


# 点の座標を定義するフレームの名前
HEADER = Header(frame_id='base')

# PointCloud2のフィールドの一覧
FIELDS = [
    # 点の座標(x, y, z)
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    # 点の色(RGB)
    # 赤: 0xff0000, 緑:0x00ff00, 青: 0x0000ff
    # PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
    # 独自に定義したフィールド
    # 例えば点の確からしさとか、観測時刻とか
    PointField(name='totalEval', offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name='distEval', offset=16, datatype=PointField.FLOAT32, count=1),
    PointField(name='timeEval', offset=20, datatype=PointField.FLOAT32, count=1),
    # PointField(name='my_field2', offset=20, datatype=PointField.FLOAT32, count=1),
]

def drange(begin, end, step):
    n = begin
    while n+step < end:
        yield n
        n += step


class CustomPointCloud(object):
    def __init__(self):
        rospy.init_node('publish_custom_point_cloud')
        self.publisher = rospy.Publisher('/custom_point_cloud', PointCloud2, queue_size=1)
        self.subscriver = rospy.Subscriber("/dwa_eval", EvaluationDataBase, self.onDWAEvalMsg)


    def onDWAEvalMsg(self, evalDB):
        self.POINTS = [[]]
        for i in range(len(evalDB.value)):
            self.POINTS.append([evalDB.value[i].vx, evalDB.value[i].vy, 0.0, evalDB.value[i].totalEval, evalDB.value[i].distEval, evalDB.value[i].timeEval])
        del self.POINTS[0]

        point_cloud = pc2.create_cloud(HEADER, FIELDS, self.POINTS)
        self.publisher.publish(point_cloud)


def main():
    try:
        custom_point_cloud = CustomPointCloud()
        rospy.spin()
        # custom_point_cloud.publish_points()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':

    main()

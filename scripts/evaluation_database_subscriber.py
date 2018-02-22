#!/usr/bin/env python
import rospy
import random
import math
import copy
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
import sys
import matplotlib.cm as cm
import rospy

from dwa_debug_tools.msg import EvaluationDataBase
from dwa_debug_tools.msg import EvaluationValue

fig = plt.figure()
ax1 = fig.add_subplot(221)
ax2 = fig.add_subplot(222)
ax3 = fig.add_subplot(223)
ax4 = fig.add_subplot(224)
area_dw=[-1.4, 1.4, -1.4, 1.4]

vxScatter = []
vyScatter = []
evalScatter = []
distScatter =[]
timeScatter = []
isDWAEvalMsg = False
def drawScale():
    ax2.cla()
    ax3.cla()
    ax4.cla()

    ax2.set_title('Distance Evaluation')
    ax2.set_xlabel('vx [rad/s]')
    ax2.set_ylabel('vy [m/s]')
    ax2.axis(area_dw)
    ax2.grid(True)
    ax2.set_aspect('equal', adjustable='box')
    plt.tight_layout()

    ax3.set_title('Time Evaluation')
    ax3.set_xlabel('vx [rad/s]')
    ax3.set_ylabel('vy [m/s]')
    ax3.axis(area_dw)
    ax3.grid(True)
    ax3.set_aspect('equal', adjustable='box')
    plt.tight_layout()

    ax4.set_title('Total Evaluation')
    ax4.set_xlabel('vx [rad/s]')
    ax4.set_ylabel('vy [m/s]')
    ax4.axis(area_dw)
    ax4.grid(True)
    ax4.set_aspect('equal', adjustable='box')
    plt.tight_layout()


def onDWAEvalMsg(evalDB):
    global vxScatter, vyScatter, evalScatter, distScatter, timeScatter, isDWAEvalMsg
    isDWAEvalMsg = True
    vxScatter = []
    vyScatter = []
    evalScatter = []
    distScatter =[]
    timeScatter = []
    # print "echo"

    for i in range(len(evalDB.value)):
        vxScatter.append(evalDB.value[i].vx)
        vyScatter.append(evalDB.value[i].vy)
        evalScatter.append(evalDB.value[i].totalEval)
        distScatter.append(evalDB.value[i].distEval)
        timeScatter.append(evalDB.value[i].timeEval)

def evalDBChecker():
    rospy.init_node('evalDB_checker', anonymous=True)
    rospy.Subscriber("dwa_eval", EvaluationDataBase, onDWAEvalMsg)


if __name__ == '__main__':
    evalDBChecker()
    # global vxScatter, vyScatter, evalScatter, distScatter, timeScatter, isDWAEvalMsg
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        print isDWAEvalMsg
        if isDWAEvalMsg:
            drawScale()
            print "echo:", vxScatter[0]
            ax1.scatter(vxScatter,vyScatter,c=distScatter, cmap="jet", s = 40)
            ax2.scatter(vxScatter,vyScatter,c=distScatter, cmap="jet", s = 40)
            ax3.scatter(vxScatter,vyScatter,c=timeScatter, cmap="jet", s = 40)
            ax4.scatter(vxScatter,vyScatter,c=evalScatter, cmap="jet", s = 40)
            plt.pause(1)
        rate.sleep()
        rospy.spin()

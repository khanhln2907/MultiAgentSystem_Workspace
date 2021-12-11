#! /usr/bin/env python

import os
import time
import math
import random
import numpy as np
# from matplotlib import pyplot as plt
import cv2
import message_filters
import scipy.ndimage as ndimage
# import seaborn as sns
# import matplotlib.pyplot as plt
#from skimage.draw import circle
#from skimage.feature import peak_local_max
import rospy
from cv_bridge import CvBridge
from rospy.numpy_msg import numpy_msg
#from tip.msg import Vector
# from rospy_tutorials.msg import Floats

# some message type
from geometry_msgs.msg import PoseStamped, Vector3
from sensor_msgs.msg import Image, CameraInfo
from platform import python_version

import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull
from voronoi import voronoi 
from voronoi_draw.msg import CentralizedMsg

class aposn:
	def __init__(self):
		self.a1_x = 200*np.random.rand() + 30
		self.a1_y = 200*np.random.rand() + 30
		self.a2_x = 200*np.random.rand() + 30
		self.a2_y = 200*np.random.rand() + 30
		self.a3_x = 200*np.random.rand() + 30
		self.a3_y = 200*np.random.rand() + 30
		self.a4_x = 200*np.random.rand() + 30
		self.a4_y = 200*np.random.rand() + 30

apos = aposn()

	
def drawback():
    bridge = CvBridge()
    #rgb = np.full((2820,4020,3), 255, dtype=np.uint8)
    #cv2.rectangle(rgb, (20, 20), (4000, 4000), (255,0,0), thickness=4)
    rgb = np.full((564,804,3), 255, dtype=np.uint8)
    cv2.rectangle(rgb, (4, 4), (800, 560), (255,0,0), thickness=4)
    pins = np.array([[apos.a1_x,apos.a1_y], [apos.a2_x,apos.a2_y], [apos.a3_x,apos.a3_y], [apos.a4_x,apos.a4_y]])	
    bnd = np.array([[20,20], [20,2800], [4000,2800], [4000, 20]]) / 5
    pnts,vorn = voronoi(pins,bnd)
    pntsv = [ [] for row in pnts]

    for i, obj in enumerate(pnts):
        pntsv[i] =  np.array(vorn[i])
        if len(pntsv[i]) >= 3:
            vorhull = ConvexHull(pntsv[i])		
            for simplex in vorhull.simplices:
                temp_1 = pntsv[i][simplex, 0]
                temp_2 = pntsv[i][simplex, 1]
                temp_x_1  = temp_1.item(0)
                temp_x_2  = temp_1.item(1)
                temp_y_1  = temp_2.item(0)
                temp_y_2  = temp_2.item(1)
                cv2.line(rgb, (int(temp_x_1), int(temp_y_1)), (int(temp_x_2), int(temp_y_2)), (255,0,0), thickness=4)
                rgb_addline = bridge.cv2_to_imgmsg(cv2.flip(rgb,1), 'rgb8')
                dynamic_painting_pub.publish(rgb_addline)

def callback(data):
	tmp = np.array(data.valueArr) / 5
	rospy.loginfo(tmp)
	apos.a1_x = tmp[2]
	apos.a1_y = tmp[3]
	apos.a2_x = tmp[8]
	apos.a2_y = tmp[9]
	apos.a3_x = tmp[14]
	apos.a3_y = tmp[15]
	apos.a4_x = tmp[20]
	apos.a4_y = tmp[21]



if __name__ == '__main__':
	bridge = CvBridge()
	rospy.init_node('stream_voronoi')
	rate = rospy.Rate(5)
	dynamic_painting_pub = rospy.Publisher('/img/paint', Image, queue_size=1000)
	info = rospy.Subscriber('centralNode/info', CentralizedMsg, callback)
	
	while not rospy.is_shutdown():
		drawback()
		rate.sleep()

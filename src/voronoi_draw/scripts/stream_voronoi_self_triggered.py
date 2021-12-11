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
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Image, CameraInfo
from platform import python_version

import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull
from voronoi import voronoi 

class aposn:
	def __init__(self):
		self.a1_x = 200*np.random.rand() + 30
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
	rgb = np.full((564,804,3), 255, dtype=np.uint8)
	cv2.rectangle(rgb, (4, 4), (800, 560), (255,0,0), thickness=4)
	pins = np.array([[apos.a1_x,apos.a1_y], [apos.a2_x,apos.a2_y], [apos.a3_x,apos.a3_y], [apos.a4_x,apos.a4_y]])
	bnd = np.array([[4,4], [4,560], [800,560], [800, 4]])
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

	


def callback_3(data):
	apos.a1_x = data.linear.x/5
	apos.a1_y = data.linear.y/5

def callback_5(data):
	apos.a2_x = data.linear.x/5
	apos.a2_y = data.linear.y/5

def callback_6(data):
	apos.a3_x = data.linear.x/5
	apos.a3_y = data.linear.y/5

def callback_7(data):
	apos.a4_x = data.linear.x/5
	apos.a4_y = data.linear.y/5


if __name__ == '__main__':

	bridge = CvBridge()
	rospy.init_node('stream_voronoi')
	rate = rospy.Rate(4)
	dynamic_painting_pub = rospy.Publisher('/img/paint', Image, queue_size=1)

	vt_3 = rospy.Subscriber('/san/vt_ctr', Twist, callback_3)
	vt_5 = rospy.Subscriber('/wu/vt_ctr', Twist, callback_5)
	vt_6 = rospy.Subscriber('/liu/vt_ctr', Twist, callback_6)
	vt_7 = rospy.Subscriber('/qi/vt_ctr', Twist, callback_7)
	
	while not rospy.is_shutdown():
		drawback()
		rate.sleep()

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
from voronoi_custom import voronoi
from voronoi_custom import getConvexBndMatrix 
from tip.msg import UnicycleInfoMsg
from tip.msg import ControlMsg
from BLF_Controller import *
from Centralized_Controller import *

# List handler to carry the information of the user
centralCom = Centralized_Controller()

# Update the state into the Voronoi Handler internally, the data will be used to comput the Tesselations
def updateAgentInfo(data):
	centralCom.updateState(data)


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
	bridge = CvBridge()
	#rgb = np.full((2820,4020,3), 255, dtype=np.uint8)
	#cv2.rectangle(rgb, (20, 20), (4000, 4000), (255,0,0), thickness=4)
	rgb = np.full((564,804,3), 255, dtype=np.uint8)
	cv2.rectangle(rgb, (4, 4), (800, 560), (255,0,0), thickness=4)
	vmList = []
	# Create an array that contains all agents' position
	for i in range(0, centralCom._nAgent):
		vmList.append([centralCom._AgentList[i].Vm[0], centralCom._AgentList[i].Vm[1]]) 
	pins = np.array(vmList) / 5

	bnd = np.array([[20,20], [20,2800], [4000,2800], [4000, 20]]) / 5
	aMat, bVec = getConvexBndMatrix(bnd)
	pnts,vorn,_,_ = voronoi(pins, aMat, bVec)
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

# def drawback():
# 	SCALE = 5
# 	rgb = np.full((2820/SCALE,4020/SCALE,3), 255, dtype=np.uint8)
# 	cv2.rectangle(rgb, (20/SCALE, 20/SCALE), (4000/SCALE, 4000/SCALE),\
# 						(255,0,0), thickness=4)
# 	pins = []
# 	for i in range(0, centralCom._nAgent):
# 		pins.append([centralCom._AgentList[i].VmX, centralCom._AgentList[i].VmY]) 
# 	bnd = np.array([[20,20], [20,2800], [4000,2800], [4000, 20]])/SCALE
# 	pnts,vorn, _,_ = voronoi(pins, centralCom.aMat, centralCom.bVec)	
# 	pntsv = [ [] for row in pnts]

# 	for i, obj in enumerate(pnts):
# 		pntsv[i] =  np.array(vorn[i])
# 		if len(pntsv[i]) >= 3:
# 			vorhull = ConvexHull(pntsv[i])		

# 			for simplex in vorhull.simplices:
# 				temp_1 = pntsv[i][simplex, 0]
# 				temp_2 = pntsv[i][simplex, 1]
# 				temp_x_1  = temp_1.item(0)
# 				temp_x_2  = temp_1.item(1)
# 				temp_y_1  = temp_2.item(0)
# 				temp_y_2  = temp_2.item(1)
# 				cv2.line(rgb, (int(temp_x_1), int(temp_y_1)), (int(temp_x_2), int(temp_y_2)), (255,0,0), thickness=4)
	
# 				rgb_addline = bridge.cv2_to_imgmsg(cv2.flip(rgb,1), 'rgb8')
# 				dynamic_painting_pub.publish(rgb_addline)

	


if __name__ == '__main__':

	bridge = CvBridge()
	rospy.init_node('stream_voronoi')
	rate = rospy.Rate(15)
	dynamic_painting_pub = rospy.Publisher('/img/paint', Image, queue_size=1)

	N_AGENT = 4
	centralCom.begin(N_AGENT, 0, 0, 0)

	Info3 = rospy.Subscriber('/san/CoverageInfo', UnicycleInfoMsg, updateAgentInfo)
	Info5 = rospy.Subscriber('/wu/CoverageInfo', UnicycleInfoMsg, updateAgentInfo)
	Info6 = rospy.Subscriber('/liu/CoverageInfo', UnicycleInfoMsg, updateAgentInfo)
	Info7 = rospy.Subscriber('/qi/CoverageInfo', UnicycleInfoMsg, updateAgentInfo)
	
	while not rospy.is_shutdown():
		try:		
			# Execute if the central node is in operating state
			if(centralCom.startFlag == True):
				tic = time.time()
				centralCom.updateCoverage()
				# Capture execution time
				toc = time.time() - tic

				# Print with low frequency for debugging
				if((time.time() - centralCom.lastPrintTime) * 1000 > 0):
					centralCom.printDebugInfo()
					centralCom.lastPrintTime = time.time()
					drawback()
			# ROS routine
			rate.sleep()

		except:
			"Error occurred! Stop all motors"
			centralCom.safeExit()

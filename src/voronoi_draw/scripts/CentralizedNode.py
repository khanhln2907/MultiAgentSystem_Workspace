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

import numpy as np
from CentralizedControllerBase import CentralizedControllerBase
from Agent import SimUnicycleCoverageAgent,ROSUnicycleCoverageAgent, LoggingInfo
import sys
sys.path.append('/home/qingchen/catkin_ws/src/voronoi_draw')

# These are logged by python Logging module, no ROS
import logging
from CoverageInterfaces import *
logFileName = "/home/qingchen/catkin_ws/src/voronoi_draw/scripts/" + "Logging/LogSim%d.log" %(time.time())
logging.basicConfig(filename = logFileName, encoding='utf-8', level=logging.DEBUG)

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

def updateAgentInfo(data):
	global cntRegisteredROSAgent, agentList, config, readyFlag

	# Update all the subscribed topic from ROS
	# SENSORS - State Feedback
	# Obtain the new data from agents and assign into the controller lists
	# Find the registered Agent ID and assign the values
	isAssigned = False
	for agent in agentList:
		if ((agent.ID == -1) or (agent.ID == np.int32(data.packet.TransmitterID))):
			# Assign new agent
			if(agent.ID == -1):
				cntRegisteredROSAgent += 1
				# Update the state
				agent.ID = np.int32(data.packet.TransmitterID)
			
			# Update the new pose
			pose3 = np.array([np.float32(data.packet.AgentPosX), np.float32(data.packet.AgentPosY), np.float32(data.packet.AgentTheta)])
			agent.updatePose(pose3, config.radius)
			# Check all agents are registered
			isAssigned = True
			break

	if(isAssigned == False):
		print("List is full ! New Agent detected")
	

class SimParam:
	nAgent = 6
	SIM = 0
	dt = 0.01
	boundaries = np.array([[20.0,20.0], [20.0,2800.0], [4000.0,2800.0], [4000.0, 20.0]])
	vConst = 16.0
	P = 1.0
	EPS_SIGMOID = 2.0
	Q_2x2 = 1.0 * np.identity(2)
	wThres = 127.0
	gain = 40.0
	wOrbit = 40.0
	radius = 200.0


# Initialize the simulation's parameters
np.random.seed(2)
config = SimParam()

cntRegisteredROSAgent = 0

agentList = []  
# SOME ONETIME CONFIG
controlParam = ControlParameter()
controlParam.eps = config.EPS_SIGMOID
controlParam.gain = config.gain
controlParam.P = config.P
controlParam.Q_2x2 = config.Q_2x2
controlParam.wOrbit = config.wOrbit
controlParam.vConst = config.vConst
controlParam.wThres = config.wThres


poseSim = [np.array([3924.21, 2331.41, 0.97]), \
			np.array([963.49, 2194.09, 0.73]), \
			np.array([1516.73, 145.00, 6.12]), \
			np.array([3818.81, 132.36, 0.21])]
for i in range(config.nAgent):
	if(config.SIM):
		config.vConst = 20.0
		config.wOrbit = 0.25
		config.wThres = 1.5
		config.gain = 0.8
		agent = SimUnicycleCoverageAgent(i, config.dt, poseSim[i])
		agent.begin(1, 2, 1, 2, config.boundaries)
		agent.setParameter(controlParam)
		agent.updateVM(80) # Uate the VM for the first time
		agentList.append(agent)
		pass
	else:
		pose3 = np.float32(np.array([0.0, 0.0, 0.0]))
		agent = ROSUnicycleCoverageAgent(-1, pose3)
		agent.begin(1, 2, 1, 2, config.boundaries)
		agent.setParameter(controlParam)
		agentList.append(agent)
com = CentralizedControllerBase(agentList, config.boundaries)

if __name__ == '__main__':
	# Declaration of ROS nodes
	bridge = CvBridge()
	rospy.init_node('stream_voronoi')
	rate = rospy.Rate(500)
	dynamic_painting_pub = rospy.Publisher('/img/paint', Image, queue_size=1)

	Info1 = rospy.Subscriber('/yi/CoverageInfo', UnicycleInfoMsg, updateAgentInfo)
	Info2 = rospy.Subscriber('/er/CoverageInfo', UnicycleInfoMsg, updateAgentInfo)
	Info3 = rospy.Subscriber('/san/CoverageInfo', UnicycleInfoMsg, updateAgentInfo)
	Info4 = rospy.Subscriber('/xi/CoverageInfo', UnicycleInfoMsg, updateAgentInfo)
	Info5 = rospy.Subscriber('/wu/CoverageInfo', UnicycleInfoMsg, updateAgentInfo)
	Info6 = rospy.Subscriber('/liu/CoverageInfo', UnicycleInfoMsg, updateAgentInfo)
	Publisher = rospy.Publisher('centralNode/controlInput', ControlMsg, queue_size = 1)	

	# Reinitialize the logger	
	logger = logging.getLogger('__name__')  
	fh = logging.FileHandler(logFileName)  
	logger.setLevel(logging.DEBUG)  
	logger.addHandler(fh)  

	drawbackCnt = 0
	SCALE = 2000
	while not rospy.is_shutdown():
		if(cntRegisteredROSAgent == config.nAgent or config.SIM):
			# Get the pose from each agent's nodes
			pntsArr = []
			for i in range(config.nAgent):
				_, vm2 = agentList[i].getPose()
				pntsArr.append(vm2)

			# Compute the centralized controller and update the actual states
			totV, controlInput = com.updateCoverage(pntsArr)
			print(totV)

			# Logging routines
			str = "Logging \n"
			for i in range(config.nAgent):
				str += agentList[i].getLogStr()
				str += "\n"	
			logger.info(str)
			print(str)

			# Send the control output to each agent
			for i in range(config.nAgent):
				if(config.SIM):
					agentList[i].move(config.vConst, controlInput[i], config.wOrbit)
				else:
					#agentList[i].command(16.0, 20, Publisher)
					# controlInput[i] = 40
					agentList[i].command(config.vConst, controlInput[i], Publisher)

			drawbackCnt = +1
			if(drawbackCnt % SCALE):

				#drawback()
				pass
		else:
			print("Waiting for all agents ")
			#time.sleep(0.1)

		if(not config.SIM):
			rate.sleep()








	
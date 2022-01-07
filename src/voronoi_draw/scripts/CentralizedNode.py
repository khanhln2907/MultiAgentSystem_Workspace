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
			agent.updatePose(pose3, config.vConst, config.wOrbit)
			# Check all agents are registered
			isAssigned = True
			break

	if(isAssigned == False):
		print("List is full ! New Agent detected")
	

def drawback():
	rgb = np.full((2820,4020,3), 255, dtype=np.uint8)
	cv2.rectangle(rgb, (20, 20), (4000, 4000), (255,0,0), thickness=4)
	pins = []
	for i in range(0, centralCom._nAgent):
		pins.append([centralCom._AgentList[i].VmX, centralCom._AgentList[i].VmY]) 
	bnd = np.array([[20,20], [20,2800], [4000,2800], [4000, 20]])
	[pnts,vorn,centroidArr] = voronoi(pins,centralCom.aMat, centralCom.bVec)
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

	


# if __name__ == '__main__':

# 	bridge = CvBridge()
# 	rospy.init_node('stream_voronoi')
# 	rate = rospy.Rate(15)
# 	dynamic_painting_pub = rospy.Publisher('/img/paint', Image, queue_size=1)

# 	N_AGENT = 4
# 	centralCom.begin(N_AGENT, 0, 0, 0)

# 	Info3 = rospy.Subscriber('/san/CoverageInfo', UnicycleInfoMsg, updateAgentInfo)
# 	Info5 = rospy.Subscriber('/wu/CoverageInfo', UnicycleInfoMsg, updateAgentInfo)
# 	Info6 = rospy.Subscriber('/liu/CoverageInfo', UnicycleInfoMsg, updateAgentInfo)
# 	Info7 = rospy.Subscriber('/qi/CoverageInfo', UnicycleInfoMsg, updateAgentInfo)
	
# 	while not rospy.is_shutdown():

		
# 		# Execute if the central node is in operating state
# 		if(centralCom.startFlag == True):
# 			tic = time.time()
# 			centralCom.updateCoverage()
# 			centralCom.publishDebugInfo()
# 			# Capture execution time
# 			toc = time.time() - tic

# 			# Print with low frequency for debugging
# 			if((time.time() - centralCom.lastPrintTime) * 1000 > 50):
# 				sumV = 0
# 				for agent in centralCom._AgentList:
# 					sumV += agent.lastVBLF
# 				str = "\n"
# 				str += "Execute control. Time %f [s]. Sum VBLF> %.4f \nReport: \n" %(toc, sumV)
				
# 				for agent in centralCom._AgentList:	
# 					str += "%d -> P[%4.1f %4.1f %1.1f] VM[%4.4f %4.4f] C[%4.4f %4.4f] Vel[%3.2f %2.2f] V: %.3f dV: [%.4f %.4f] Err: %.2f \n"\
# 					%(agent.ID, agent.PosX, agent.PosY, agent.Theta, \
# 					agent.VmX, agent.VmY,\
# 					agent.TargetX, agent.TargetY,\
# 					agent.angularVel, agent.testW,\
# 					agent.lastVBLF, agent.dVBLF[0], agent.dVBLF[1], \
# 					math.sqrt(pow(agent.VmX - agent.TargetX,2) + pow(agent.VmY - agent.TargetY,2)))						
# 				rospy.loginfo(str)
# 				rospy.loginfo(centralCom.adjacentMat)
# 				centralCom.lastPrintTime = time.time()
# 		# ROS routine
# 		rate.sleep()

def fix_logging(level=logging.WARNING):
    console = logging.StreamHandler()
    console.setLevel(level)
    logging.getLogger('').addHandler(console)
    formatter = logging.Formatter('%(levelname)-8s:%(name)-12s: %(message)s')
    console.setFormatter(formatter)
    logging.getLogger('').addHandler(console)




class SimParam:
	nAgent = 4
	SIM = 0
	dt = 0.01
	boundaries = np.array([[20,20], [20,2800], [4000,2800], [4000, 20]])
	wOrbit = 0.5
	vConst = 16
	P = 3
	EPS_SIGMOID = 5
	Q_2x2 = 5 * np.identity(2)

# Initialize the simulation's parameters
np.random.seed(6)
config = SimParam()

cntRegisteredROSAgent = 0

agentList = []  
for i in range(config.nAgent):
	if(config.SIM):
		rXY = 2000;    
		pose3 = np.array([rXY * np.random.rand(), rXY * np.random.rand(), np.random.rand()])
		agent = SimUnicycleCoverageAgent(-1, config.dt, pose3)
		agent.begin(1, 2, 1, 2, config.boundaries)
		agentList.append(agent)
		pass
	else:
		pose3 = np.float32(np.array([0.0, 0.0, 0.0]))
		agent = ROSUnicycleCoverageAgent(-1, pose3)
		agent.begin(1, 2, 1, 2, config.boundaries)
		agentList.append(agent)
com = CentralizedControllerBase(agentList, config.boundaries)

if __name__ == '__main__':
	# Declaration of ROS nodes
	bridge = CvBridge()
	rospy.init_node('stream_voronoi')
	rate = rospy.Rate(15)
	dynamic_painting_pub = rospy.Publisher('/img/paint', Image, queue_size=1)

	Info3 = rospy.Subscriber('/san/CoverageInfo', UnicycleInfoMsg, updateAgentInfo)
	Info5 = rospy.Subscriber('/wu/CoverageInfo', UnicycleInfoMsg, updateAgentInfo)
	Info6 = rospy.Subscriber('/liu/CoverageInfo', UnicycleInfoMsg, updateAgentInfo)
	Info7 = rospy.Subscriber('/qi/CoverageInfo', UnicycleInfoMsg, updateAgentInfo)
	Publisher = rospy.Publisher('centralNode/controlInput', ControlMsg, queue_size = 1)	

	# Reinitialize the logger	
	logger = logging.getLogger('__name__')  
	fh = logging.FileHandler(logFileName)  
	logger.setLevel(logging.DEBUG)  
	logger.addHandler(fh)  

	while not rospy.is_shutdown():
		if(cntRegisteredROSAgent == config.nAgent or config.SIM):
			# Get the pose from each agent's nodes
			pntsArr = []
			for i in range(config.nAgent):
				_, vm2 = agentList[i].getPose()
				pntsArr.append(vm2)

			# Compute the centralized controller and update the actual states
			totV, controlInput = com.updateCoverage(pntsArr)

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
					agentList[i].command(config.vConst, controlInput[i], Publisher)









	
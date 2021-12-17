#! /usr/bin/env python
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
from voronoi_draw.msg import CentralizedMsg
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull
from voronoi_custom import voronoi
from voronoi_custom import getConvexBndMatrix 
from voronoi_custom import getAdjacentList
from tip.msg import UnicycleInfoMsg
from tip.msg import ControlMsg
from BLF_Controller import *
from VoronoiLib import *

class Centralized_Controller:
	def __init__(self):
		# Agent information
		self._nAgent = 0
		self._cntRegisteredAgent = 0
		self._AgentList = []
		self.adjacentMat = 0		# Adjacent matrix used to determines the adjacent agents
		# Values to handle the coverage area, convex: a1 * x + a2 * y + b < 0
		self.boundaries = 0
		self.nBound = 0
		self.aMat = 0
		self.bVec = 0
		self.VoronoiVertices = 0
		# Variable for logging at low frequency
		self.lastPrintTime = time.time()
		self.startingTime = time.time()
		self.startFlag = False
		self.controlInputPublisher = rospy.Publisher('/centralNode/controlInput', ControlMsg, queue_size=1)
		self.infoPublisher = rospy.Publisher('/centralNode/info', CentralizedMsg, queue_size=1)
		# Variable for logging Msg
		self.cntMsg = 0

	def begin(self,nAgents, a1, a2, b):
		self._nAgent = nAgents
		self._a1 = a1
		self._a2 = a2
		self._b = b
		# Initialize the list of agents information with the desired settings
		boundaries = np.array([[20,20], [20,2800], [4000,2800], [4000, 20]])
		[self.aMat, self.bVec] = getConvexBndMatrix(boundaries)
		self.bndCoeff = np.zeros([len(self.bVec),3])
		for j in range(len(self.bVec)):
			self.bndCoeff[j,0] = self.aMat[j,0]
			self.bndCoeff[j,1] = self.aMat[j,1]
			self.bndCoeff[j,2] = self.bVec[j]
		
		for i in range(self._nAgent):
			agentHandler = BLF_Controller()
			agentHandler.begin(1, 2, 1, 2, boundaries)
			self._AgentList.append(agentHandler)
		
		rospy.loginfo("Init list of %d agents", self._nAgent)

	# Update all the subscribed topic from ROS
	def updateState(self, data):
		# SENSORS - State Feedback
		# Obtain the new data from agents and assign into the controller lists
		# Find the registered Agent ID and assign the values
		isAssigned = False
		for agent in self._AgentList:
			if ((agent.ID == -1) or (agent.ID == np.int32(data.packet.TransmitterID))):
				# Assign new agent
				if(agent.ID == -1):
					self._cntRegisteredAgent += 1
				# Update the state
				agent.updateState(data)
				# Check all agents are registered
				isAssigned = True
				break

		if(isAssigned == False):
			rospy.loginfo("List is full ! New Agent detected")
		
		# CONTROL METHOD 
		# Start only all agents registered for the agentlist
		if(self._cntRegisteredAgent == self._nAgent):
			self.startFlag = True

		if(self.startFlag == True):
			# Recompute the Voronoi tesselations here and update the target for each agent
			# (Consider using some kind of filter to reduce jerking from high frequency data, for instance threshold for the changes of the position, ...) ...
			# MOVE OUT SIDE AND EXECUTE EXTERNALLY DUE TO HIGH COMPUTATIONAL COST
			# self.updateCoverage()
			# Compute the control output and publish to agents
			# self.controlCoverage()		
			# Do something else ...
			pass
			
	def drawback(self):
		bridge = CvBridge()
		rgb = np.full((2820,4020,3), 255, dtype=np.uint8)
		cv2.rectangle(rgb, (20, 20), (4000, 4000), (255,0,0), thickness=4)

		vmList = []
		# Create an array that contains all agents' position
		for i in range(0, self._nAgent):
			vmList.append([self._AgentList[i].VmX, self._AgentList[i].VmY]) 

		vmArr = np.array(vmList)
		bnd = np.array([[20,20], [20,2800], [4000,2800], [4000, 20]])
		#pnts,vorn, centroid = voronoi(vmArr,bnd)
		pntsv = [ [] for row in vmArr]

		for i, obj in enumerate(vmArr):
			pntsv[i] =  np.array(self.VoronoiVertices[i])
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
					self.CVPublisher.publish(rgb_addline)
		return 0


	def updateCoverage(self):
		tmpTessel = []
		for i in range(0, self._nAgent):
			tmpTessel.append([self._AgentList[i].VmX, self._AgentList[i].VmY]) 

		pntsArr = np.array(tmpTessel)
		# Boundary lines of the coverage area. This is still hard coded until now
		# Get the centroids and the vertices
		[pntsIn , self.VoronoiVertices, centroidArr, partitionMasses] = voronoi(pntsArr, self.aMat, self.bVec)
		[self.adjacentMat, commonverMat] = getAdjacentList(self.VoronoiVertices, centroidArr)

		# Return the adjacent matrix in relation to the order of centroid
		# If the total amount of centroid returns is lower than the total amount of agents
		# Mission must be terminated. Error detection
		if(len(centroidArr) != self._nAgent):
			rospy.logerr("VORONOI CONFIGURATION INVALID. EMERGENCY STOP ACTIVATED")
			# Send Stop Cmd to each agent
			for i in range(0, self._nAgent):
				v = 0
				w = 0
				# Publish the message
				msg = ControlMsg()
				msg.ID = self._AgentList[i].ID
				msg.translation = v
				msg.rotation = w			
				self.controlInputPublisher.publish(msg)
		# Update the control input if everything is working fine	
		else: 			
			# Compute the Lyapunov partial derivative from each agent
			lyapunovTelegraphList = []
			for agentID in range(0, self._nAgent):
				# Information of agent <agentID>
				myAgent = VoronoiPrivateData()
				myAgent.C = np.array(centroidArr[agentID])
				myAgent.z = np.array([self._AgentList[agentID].VmX, self._AgentList[agentID].VmY])
				myAgent.dCi_dzi = 0

				# Parsed neighbor information
				telegraph = []
				for neighborID in range(0, self._nAgent):
					neighborReport = NeighborVoronoiInfo()
					if(self.adjacentMat[agentID][neighborID] == 1):
						neighborReport.neighborID = self._AgentList[neighborID].ID
						neighborReport.vm_coord_2d = np.array([self._AgentList[neighborID].VmX, self._AgentList[neighborID].VmY])
						neighborReport.com_v1_2d = commonverMat[agentID][neighborID][0]
						neighborReport.com_v2_2d = commonverMat[agentID][neighborID][1]
						telegraph.append(neighborReport)

				retTelegraph = self._AgentList[agentID].updateVoronoiInfo(myAgent,telegraph, partitionMasses[agentID], self.bndCoeff)
				lyapunovTelegraphList.append(retTelegraph)

			#rospy.loginfo(lyapunovTelegraphList)
			# Perfrom the routing of the telegraph for the feedback of the Lyapunov derivative
			for agentID in range(0, self._nAgent):
				myTel = []
				# Go through the "network" to get my data
				for j in range(0, self._nAgent):
					if j is not agentID:
						for msg in lyapunovTelegraphList[j]:
							if (msg.neighborID == self._AgentList[agentID].ID): # If the message is for me
								myTel.append(msg) 	
				
				[v, w] = self._AgentList[agentID].updateControlInput(myTel)
				# Publish the message
				msg = ControlMsg()
				msg.ID = self._AgentList[agentID].ID
				msg.translation = v
				msg.rotation = w			
				self.controlInputPublisher.publish(msg)
	

	def publishDebugInfo(self):
		msg = CentralizedMsg()
		self.cntMsg += 1
		debugInfo = []
		debugInfo.append(self.cntMsg)
		for agents in (self._AgentList):
			debugInfo.append(agents.ID)
			debugInfo.append(agents.PosX)
			debugInfo.append(agents.PosY)
			#debugInfo.append(agents.Theta)
			debugInfo.append(agents.VmX)
			debugInfo.append(agents.VmY)
			#debugInfo.append(agents.TargetX)
			#debugInfo.append(agents.TargetY)
			debugInfo.append(agents.lastVBLF)
		# Publish
		msg.valueArr = debugInfo
		self.infoPublisher.publish(msg)	


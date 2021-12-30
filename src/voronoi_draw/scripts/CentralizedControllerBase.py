#! /usr/bin/env python
#! /usr/bin/env python
import os
import time
import math
import random
import numpy as np
import scipy.ndimage as ndimage
# some message type
from scipy.spatial import ConvexHull
from voronoi_custom import voronoi
from voronoi_custom import getConvexBndMatrix 
from voronoi_custom import getAdjacentList
from BLFController import *
from controlAlgo import *
from VoronoiLib import *
from UnicycleAgent import *

class CentralizedControllerBase:
	def __init__(self, agentList, bndPnt):
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
		# Variable for logging Msg
		self.cntMsg = 0

		self._AgentList = agentList
		self._nAgent = len(self._AgentList)
		# Initialize the list of agents information with the desired settings
		[self.aMat, self.bVec] = getConvexBndMatrix(bndPnt)
		self.bndCoeff = np.zeros([len(self.bVec),3])
		for j in range(len(self.bVec)):
			self.bndCoeff[j,0] = self.aMat[j,0]
			self.bndCoeff[j,1] = self.aMat[j,1]
			self.bndCoeff[j,2] = self.bVec[j]
		print("Init list of %d agents", self._nAgent)
	

	# Update all the subscribed topic from ROS
	# def updateState(self, data):
	# 	# SENSORS - State Feedback
	# 	# Obtain the new data from agents and assign into the controller lists
	# 	# Find the registered Agent ID and assign the values
	# 	isAssigned = False
	# 	for agent in self._AgentList:
	# 		if ((agent.ID == -1) or (agent.ID == np.int32(data.packet.TransmitterID))):
	# 			# Assign new agent
	# 			if(agent.ID == -1):
	# 				self._cntRegisteredAgent += 1
	# 			# Update the state
	# 			agent.updateState(data)
	# 			# Check all agents are registered
	# 			isAssigned = True
	# 			break

	# 	if(isAssigned == False):
	# 		#rospy.loginfo("List is full ! New Agent detected")
	# 		print("List is full ! New Agent detected")
		
	# 	# CONTROL METHOD 
	# 	# Start only all agents registered for the agentlist
	# 	if(self._cntRegisteredAgent == self._nAgent):
	# 		self.startFlag = True

	# This method must be defined by the child class to provide the publishing methods to each agent
	def publishControlMsg(self, ID, v,w):
		#str = "Publish to agent %d, v = %.4f, w = %.4f" %(ID, v, w)
		#print(str)
		raise NotImplementedError()

	def updateCoverageDep(self, pntsArr):
		# Boundary lines of the coverage area. This is still hard coded until now
		# Get the centroids and the vertices
		[pntsIn , self.VoronoiVertices, centroidArr, partitionMasses] = voronoi(pntsArr, self.aMat, self.bVec)
		[self.adjacentMat, commonverMat] = getAdjacentList(self.VoronoiVertices, centroidArr)

		# Return the adjacent matrix in relation to the order of centroid
		# If the total amount of centroid returns is lower than the total amount of agents
		# Mission must be terminated. Error detection
		if(len(centroidArr) != self._nAgent):
			print("VORONOI CONFIGURATION INVALID. EMERGENCY STOP ACTIVATED")
			#rospy.logerr("VORONOI CONFIGURATION INVALID. EMERGENCY STOP ACTIVATED")
			# Send Stop Cmd to each agent
			for agentID in range(0, self._nAgent):
				self.publishControlMsg(self._AgentList[agentID].ID, 0,0)

		# Update the control input if everything is working fine	
		else: 
			# Update BLF State of all agent
			dVidziList = []
			lapunovMat = [[None] for i in range(self._nAgent)]
			controlParam = controlParameter()
			controlParam.eps = 5
			controlParam.gain = 3
			controlParam.P = 3
			controlParam.Q_2x2 = 5 * np.identity(2)
			# Compute the partial derivative for each agent
			totVi = 0
			for agentID in range(0, self._nAgent):
				myAgent = vorPrivateData()
				myAgent.C = np.array(centroidArr[agentID])
				myAgent.z = np.array([self._AgentList[agentID].vm2[0], self._AgentList[agentID].vm2[1]])
				myAgent.dCi_dzi = 0
				dCi_dzj_list = np.zeros((self._nAgent,self._nAgent,2,2))
				for neighborID in range(0, self._nAgent):
					if(self.adjacentMat[agentID][neighborID] == 1):
						adjCoord_2d = np.array([self._AgentList[neighborID].vm2[0], self._AgentList[neighborID].vm2[1]])
						dCi_dzi_AdjacentJ, dCi_dzj = Voronoi2DCalCVTPartialDerivative(myAgent.z, myAgent.C, partitionMasses[agentID],\
													adjCoord_2d, commonverMat[agentID][neighborID][0], commonverMat[agentID][neighborID][1])
						myAgent.dCi_dzi += dCi_dzi_AdjacentJ
						dCi_dzj_list[agentID,neighborID,:,:] = dCi_dzj[:,:]
				
				# Compute the Lyapunov feedback after having the adjacent information
				[Vi, dVidzi, dVidzj_Arr] = Voronoi2DCaldVdz(myAgent, dCi_dzj_list[agentID,:,:,:], self.bndCoeff, controlParam)
				dVidziList.append(dVidzi)
				lapunovMat[agentID] = [[Vi], [dVidzi], [dVidzj_Arr]]
				totVi += Vi

			print("Lyapunov: ", totVi)
			# Compute the control output for all agents
			controlInput = []
			for agentID in range(0, self._nAgent):
				dV = dVidziList[agentID]
				for neighborID in range(0, self._nAgent):
					if(self.adjacentMat[neighborID][agentID] == 1):
						
						dV += lapunovMat[neighborID][2][0][agentID]
				self._AgentList[agentID].updateLyapunov(centroidArr[agentID], lapunovMat[agentID][0][0], dV)
				[v, w] = self._AgentList[agentID].controlBLF(dV)
				# Publish the message
				controlInput.append(w)
				#self.publishControlMsg(self._AgentList[agentID].ID, v,w)
			return [controlInput, lapunovMat]
	
	# Compute Voronoi Tessellation for each agent
	# Perform information routing to each control handle
	# Mapping the partial derivative of the Lyapunov Feedback to each adjacent agent
	# Publish the control input
	def updateCoverage(self):
		tmpTessel = []
		for i in range(0, self._nAgent):
			tmpTessel.append([self._AgentList[i].vm2[0], self._AgentList[i].vm2[1]]) 

		pntsArr = np.array(tmpTessel)
		# Boundary lines of the coverage area. This is still hard coded until now
		# Get the centroids and the vertices
		[pntsIn , self.VoronoiVertices, centroidArr, partitionMasses] = voronoi(pntsArr, self.aMat, self.bVec)
		[self.adjacentMat, commonverMat] = getAdjacentList(self.VoronoiVertices, centroidArr)

		# Return the adjacent matrix in relation to the order of centroid
		# If the total amount of centroid returns is lower than the total amount of agents
		# Mission must be terminated. Error detection
		if(len(centroidArr) != self._nAgent):
			# Send Stop Cmd to each agent
			for i in range(0, self._nAgent):
				v = 0
				w = 0
				# Publish the message
				# msg = ControlMsg()
				# msg.ID = self._AgentList[i].ID
				# msg.translation = v
				# msg.rotation = w			
				# self.controlInputPublisher.publish(msg)
		# Update the control input if everything is working fine	
		else: 			
			# Compute the Lyapunov partial derivative from each agent
			lyapunovTelegraphList = []
			totV = 0
			for agentID in range(0, self._nAgent):
				# Information of agent <agentID>
				myCVT = np.array(centroidArr[agentID])

				# Parsed neighbor information
				telegraph = []
				for neighborID in range(0, self._nAgent):
					neighborReport = NeighborVoronoiInfo()
					if(self.adjacentMat[agentID][neighborID] == 1):
						neighborReport.neighborID = self._AgentList[neighborID].ID
						neighborReport.vm_coord_2d = np.array([self._AgentList[neighborID].vm2[0], self._AgentList[neighborID].vm2[1]])
						neighborReport.com_v1_2d = commonverMat[agentID][neighborID][0]
						neighborReport.com_v2_2d = commonverMat[agentID][neighborID][1]
						telegraph.append(neighborReport)

				# Transmit all neighbor info to the agent to obtain the Lyapunov partial derivative
				Vi, retTelegraph = self._AgentList[agentID].updateVoronoiInfo(myCVT,telegraph, partitionMasses[agentID], self.bndCoeff)
				lyapunovTelegraphList.append(retTelegraph)

				totV += Vi
			
			# Perfrom the routing of the telegraph for the feedback of the Lyapunov derivative
			controlInput = []
			for agentID in range(0, self._nAgent):
				myTel = []
				# Go through the "network" to get my data
				for j in range(0, self._nAgent):
					#if j is not agentID:
					for msg in lyapunovTelegraphList[j]:
						if (msg.neighborID == self._AgentList[agentID].ID): # If the message is for me
							myTel.append(msg) 	
				
				[v, w] = self._AgentList[agentID].calcControlInput(myTel)
				# Publish the message
				# msg = ControlMsg()
				# msg.ID = self._AgentList[agentID].ID
				# msg.translation = v
				# msg.rotation = w			
				# self.controlInputPublisher.publish(msg)
				controlInput.append(w)
			return totV, controlInput
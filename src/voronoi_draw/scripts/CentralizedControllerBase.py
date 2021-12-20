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

class CentralizedControllerBase:
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
		# Variable for logging Msg
		self.cntMsg = 0

	def begin(self,nAgents, bndPnt):
		self._nAgent = nAgents
		# Initialize the list of agents information with the desired settings
		[self.aMat, self.bVec] = getConvexBndMatrix(bndPnt)
		self.bndCoeff = np.zeros([len(self.bVec),3])
		for j in range(len(self.bVec)):
			self.bndCoeff[j,0] = self.aMat[j,0]
			self.bndCoeff[j,1] = self.aMat[j,1]
			self.bndCoeff[j,2] = self.bVec[j]
		
		for i in range(self._nAgent):
			agentHandler = BLFController()
			agentHandler.begin(1, 2, 1, 2, bndPnt)
			self._AgentList.append(agentHandler)
		
		#rospy.loginfo("Init list of %d agents", self._nAgent)
		print("Init list of %d agents", self._nAgent)



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
			#rospy.loginfo("List is full ! New Agent detected")
			print("List is full ! New Agent detected")
		
		# CONTROL METHOD 
		# Start only all agents registered for the agentlist
		if(self._cntRegisteredAgent == self._nAgent):
			self.startFlag = True

	# This method must be defined by the child class to provide the publishing methods to each agent
	def publishControlMsg(self, ID, v,w):
		str = "Publish to agent %d, v = %.4f, w = %.4f" %(ID, v, w)
		print(str)
		#raise NotImplementedError()

	def updateCoverage(self, pntsArr):
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
			for agentID in range(0, self._nAgent):
				myAgent = vorPrivateData()
				myAgent.C = np.array(centroidArr[agentID])
				myAgent.z = np.array([self._AgentList[agentID].VmX, self._AgentList[agentID].VmY])
				myAgent.dCi_dzi = 0
				dCi_dzj_list = np.zeros((self._nAgent,self._nAgent,2,2))
				for neighborID in range(0, self._nAgent):
					if(self.adjacentMat[agentID][neighborID] == 1):
						adjCoord_2d = np.array([self._AgentList[neighborID].VmX, self._AgentList[neighborID].VmY])
						dCi_dzi_AdjacentJ, dCi_dzj = Voronoi2D_calCVTPartialDerivative(myAgent.z, myAgent.C, partitionMasses[agentID],\
													adjCoord_2d, commonverMat[agentID][neighborID][0], commonverMat[agentID][neighborID][1])
						myAgent.dCi_dzi += dCi_dzi_AdjacentJ
						dCi_dzj_list[agentID,neighborID,:,:] = dCi_dzj[:,:]
				
				# Compute the Lyapunov feedback after having the adjacent information
				[Vi, dVidzi, dVidzj_Arr] = Voronoi2D_cal_dV_dz(myAgent, dCi_dzj_list[agentID,:,:,:], self.bndCoeff, controlParam)
				dVidziList.append(dVidzi)
				lapunovMat[agentID] = [[Vi], [dVidzi], [dVidzj_Arr]]

			# Compute the control output for all agents
			for agentID in range(0, self._nAgent):
				dV = dVidziList[agentID]
				for neighborID in range(0, self._nAgent):
					if(self.adjacentMat[neighborID][agentID] == 1):
						
						dV += lapunovMat[neighborID][2][0][agentID]
				self._AgentList[agentID].updateLyapunov(centroidArr[agentID], lapunovMat[agentID][0][0], dV)
				[v, w] = self._AgentList[agentID].controlBLF(dV)
				# Publish the message
				self.publishControlMsg(self._AgentList[agentID].ID, v,w)

	# def updateCoverageRev(self):
	# 	tmpTessel = []
	# 	for i in range(0, self._nAgent):
	# 		tmpTessel.append([self._AgentList[i].VmX, self._AgentList[i].VmY]) 

	# 	pntsArr = np.array(tmpTessel)
	# 	# Boundary lines of the coverage area. This is still hard coded until now
	# 	# Get the centroids and the vertices
	# 	[pntsIn , self.VoronoiVertices, centroidArr, partitionMasses] = voronoi(pntsArr, self.aMat, self.bVec)
	# 	[self.adjacentMat, commonverMat] = getAdjacentList(self.VoronoiVertices, centroidArr)

	# 	# Return the adjacent matrix in relation to the order of centroid
	# 	# If the total amount of centroid returns is lower than the total amount of agents
	# 	# Mission must be terminated. Error detection
	# 	if(len(centroidArr) != self._nAgent):
	# 		rospy.logerr("VORONOI CONFIGURATION INVALID. EMERGENCY STOP ACTIVATED")
	# 		# Send Stop Cmd to each agent
	# 		for i in range(0, self._nAgent):
	# 			v = 0
	# 			w = 0
	# 			# Publish the message
	# 			msg = ControlMsg()
	# 			msg.ID = self._AgentList[i].ID
	# 			msg.translation = v
	# 			msg.rotation = w			
	# 			self.controlInputPublisher.publish(msg)
	# 	# Update the control input if everything is working fine	
	# 	else: 
	# 		# Update BLF State of all agent
	# 		dVidziList = []
	# 		lapunovMat = [[None] for i in range(self._nAgent)]
	# 		controlParam = controlParameter()
	# 		controlParam.eps = 5
	# 		controlParam.gain = 3
	# 		controlParam.P = 3
	# 		controlParam.Q_2x2 = 5 * np.identity(2)
	# 		# Compute the partial derivative for each agent
	# 		for agentID in range(0, self._nAgent):
	# 			# Information of agent <agentID>
	# 			myAgent = vorPrivateData()
	# 			myAgent.C = np.array(centroidArr[agentID])
	# 			myAgent.z = np.array([self._AgentList[agentID].VmX, self._AgentList[agentID].VmY])
	# 			myAgent.dCi_dzi = 0
	# 			dCi_dzj_list = np.zeros((self._nAgent,self._nAgent,2,2))

	# 			# Parsed neighbor information
				
	# 			telegraph = []
	# 			adjCoord_2d = np.zeros((self._nAgent,2,2))
	# 			for neighborID in range(0, self._nAgent):
	# 				neighborReport = NeighborInfo()
	# 				if(self.adjacentMat[agentID][neighborID] == 1):
	# 					neighborReport.neighborID = self._AgentList[neighborID].ID
	# 					neighborReport.vm_coord_2d = np.array([self._AgentList[neighborID].VmX, self._AgentList[neighborID].VmY])
	# 					neighborReport.com_v1_2d = commonverMat[agentID][neighborID][0]
	# 					neighborReport.com_v2_2d = commonverMat[agentID][neighborID][1]
	# 					telegraph.append(neighborReport)

	# 			self._AgentList[agentID].updateVoronoiInfo(myAgent,telegraph)
	# 					#adjCoord_2d = np.array([self._AgentList[neighborID].VmX, self._AgentList[neighborID].VmY])
	# 					#dCi_dzi_AdjacentJ, dCi_dzj = Voronoi2D_calCVTPartialDerivative(myAgent.z, myAgent.C, partitionMasses[agentID],\
	# 					#							adjCoord_2d, commonverMat[agentID][neighborID][0], commonverMat[agentID][neighborID][1])
	# 					#myAgent.dCi_dzi += dCi_dzi_AdjacentJ
	# 					#dCi_dzj_list[agentID,neighborID,:,:] = dCi_dzj[:,:]
				




	# 			for neighborID in range(0, self._nAgent):
	# 				if(self.adjacentMat[agentID][neighborID] == 1):
	# 					adjCoord_2d = np.array([self._AgentList[neighborID].VmX, self._AgentList[neighborID].VmY])
	# 					dCi_dzi_AdjacentJ, dCi_dzj = Voronoi2D_calCVTPartialDerivative(myAgent.z, myAgent.C, partitionMasses[agentID],\
	# 												adjCoord_2d, commonverMat[agentID][neighborID][0], commonverMat[agentID][neighborID][1])
	# 					myAgent.dCi_dzi += dCi_dzi_AdjacentJ
	# 					dCi_dzj_list[agentID,neighborID,:,:] = dCi_dzj[:,:]
				
	# 			# Compute the Lyapunov feedback after having the adjacent information
	# 			#rospy.loginfo(dCi_dzj_list[agentID,:,:,:])
	# 			[Vi, dVidzi, dVidzj_Arr] = Voronoi2D_cal_dV_dz(myAgent, dCi_dzj_list[agentID,:,:,:], self.bndCoeff, controlParam)
	# 			dVidziList.append(dVidzi)
	# 			lapunovMat[agentID] = [[Vi], [dVidzi], [dVidzj_Arr]]

	# 		# Compute the control output for all agents
	# 		for agentID in range(0, self._nAgent):
	# 			dV = dVidziList[agentID]
	# 			for neighborID in range(0, self._nAgent):
	# 				if(self.adjacentMat[neighborID][agentID] == 1):
						
	# 					dV += lapunovMat[neighborID][2][0][agentID]
	# 			#rospy.loginfo(centroidArr[agentID])
	# 			self._AgentList[agentID].updateLyapunov(centroidArr[agentID], lapunovMat[agentID][0][0], dV)
	# 			[v, w] = self._AgentList[agentID].controlBLF(dV)
	# 			# Publish the message
	# 			msg = ControlMsg()
	# 			msg.ID = self._AgentList[agentID].ID
	# 			msg.translation = v
	# 			msg.rotation = w			
	# 			self.controlInputPublisher.publish(msg)
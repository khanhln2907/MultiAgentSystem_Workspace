#! /usr/bin/env python
import time
import numpy as np
# some message type
from voronoi_custom import voronoi
from voronoi_custom import getConvexBndMatrix 
from voronoi_custom import getAdjacentList
from CoverageInterfaces import *

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
		print(self.bndCoeff)
	# Compute Voronoi Tessellation for each agent
	# Perform information routing to each control handle
	# Mapping the partial derivative of the Lyapunov Feedback to each adjacent agent
	# Publish the control input
	def updateCoverage(self, pnts2Arr):
		# Get the centroids and the vertices
		[pntsIn , self.VoronoiVertices, centroidArr, partitionMasses] = voronoi(pnts2Arr, self.aMat, self.bVec)
		print(pnts2Arr)
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
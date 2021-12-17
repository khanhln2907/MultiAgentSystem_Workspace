import numpy as np
import rospy
import math
from voronoi_custom import getConvexBndMatrix 

class NeighborInfo:
	neighborID = 0
	vm_coord_2d = 0
	cvt_coord_2d = 0
	com_v1_2d = 0
	com_v2_2d = 0


# State Feedback from the sensos, qualisys, ...
class BLF_Controller:
	def __init__(self):
		# ID
		self.ID = -1
		# Coverage Information
		# [Parameter] Convex region: Ax - b <= 0
		self.ABnd = 0
		self.bBnd = 0
		# State Feedback
		self.PosX = 200*np.random.rand() + 30
		self.PosY = 200*np.random.rand() + 30
		self.Theta = 0
		self.VmX = 200*np.random.rand() + 30
		self.VmY = 200*np.random.rand() + 30
		self.VBLF = 0
		self.verticesList = 0
		# Target
		self.TargetX = 0
		self.TargetY = 0
		# Algorithm Variables
		self.lastVmX = 0
		self.lastVmY = 0
		self.lastVBLF = 0
		self.curVBLF = 0 
		self.dVBLF = 0
		self.gain = 0
		self.testW = 0
		# Constraints
		self.vConst = 0
		self.wThres = 0
		self.wOrbit = 0
		# Output
		self.angularVel = 0

	def updateVM(self):
		# Save it befoe updating new values
		self.lastVmX = self.VmX
		self.lastVmY = self.VmY
		# Calculate the VM ourself so dont need to change the config in cpp file
		self.VmX = self.PosX - self.vConst/self.wOrbit * math.sin(self.Theta) 
		self.VmY = self.PosY + self.vConst/self.wOrbit * math.cos(self.Theta)

	def updateState(self, data):	
		self.ID = np.int32(data.packet.TransmitterID)
		self.PosX = np.float32(data.packet.AgentPosX)
		self.PosY = np.float32(data.packet.AgentPosY)
		self.Theta = np.float32(data.packet.AgentTheta)
		
		# Update this internally for centralized controller
		self.updateVM()	# Update the virtual center with radius 200 locally to avoid conflict with another node
		
	def updateLyapunov(self, newCVT_2d, V, dV):
		self.TargetX = newCVT_2d[0]
		self.TargetY = newCVT_2d[1]
		self.dVBLF = dV
		self.lastVBLF = V
		return 0
		
	# # Obain the new Voronoi information and return the partial derivatives / or Lyapuno feedback for adjacent agents
	# def updateVoronoiInfo(self, myCVT, adjPart):
	# 	nNeighbor = len(adjPart)

	# 	dCi_dzi = 0
	# 	for i in range(nNeighbor):
	# 		myZ = np.array([self.VmX, self.VmY])
	# 		myCVT = np.array([self.VmX, self.VmY])
	# 		dCi_dzi_AdjacentJ, dCi_dzj = Voronoi2D_calCVTPartialDerivative(, myAgent.C, partitionMasses[agentID],\
	# 									adjCoord_2d, commonverMat[agentID][neighborID][0], commonverMat[agentID][neighborID][1])
	# 		#myAgent.dCi_dzi += dCi_dzi_AdjacentJ
	# 		#dCi_dzj_list[agentID,neighborID,:,:] = dCi_dzj[:,:]
		
	# 	for neighborID in range(0, self._nAgent):
	# 		if(self.adjacentMat[agentID][neighborID] == 1):
	# 			adjCoord_2d = np.array([self._AgentList[neighborID].VmX, self._AgentList[neighborID].VmY])
	# 			dCi_dzi_AdjacentJ, dCi_dzj = Voronoi2D_calCVTPartialDerivative(myAgent.z, myAgent.C, partitionMasses[agentID],\
	# 										adjCoord_2d, commonverMat[agentID][neighborID][0], commonverMat[agentID][neighborID][1])
	# 			myAgent.dCi_dzi += dCi_dzi_AdjacentJ
	# 			dCi_dzj_list[agentID,neighborID,:,:] = dCi_dzj[:,:]


	# 	return 0

	def begin(self, controlGain, v0, w0, wCO, boundaries):
		self.gain = controlGain
		self.vConst = v0
		self.wOrbit = w0
		self.wThres = wCO
		[self.ABnd, self.bBnd] = getConvexBndMatrix(boundaries)
		# filter the boundary lines in case perpendicular to Oy
		eps = 0.01
		for j in range(len(self.bBnd)):
			self.ABnd[j,0] = round(self.ABnd[j,0])
			self.ABnd[j,1] = round(self.ABnd[j,1])
			self.bBnd[j,0] = round(self.bBnd[j,0])
		#rospy.loginfo([self.ABnd, self.bBnd])

	def controlBLF(self, dV):	
		# Constant parameter =================================
		# This should be initalised at the beginning, however config here for easy tuning
		self.wThres = 127
		self.vConst = 16
		self.gain = np.double(30) 
		self.wOrbit = 30
		eps = 5
		# Control output ====================================
		w = self.wOrbit + self.gain * calc_sigmoid(dV[0] * math.cos(self.Theta) + dV[1] * math.sin(self.Theta), eps)

		# Output cutoff
		if(w > self.wThres):
			w = self.wThres
			rospy.logwarn("ANGULAR THRESHOLD VIOLATED: %.2f !", w)
		elif(w < -self.wThres):
			w = -self.wThres
			rospy.logwarn("ANGULAR THRESHOLD VIOLATED: %.2f !", w)

		# Constant Heading Velocity
		self.vConst = 16
		self.angularVel = w
		return [self.vConst, self.angularVel]



	def computeVBLF(self):
		nBndLines = len(self.bBnd)
		tmpV = 0
		for j in range(nBndLines):
			tmpV += 1 // (self.bBnd[j] - self.ABnd[j][0] * self.VmX + self.ABnd[j][1] * self.VmY)
		V = tmpV * (math.pow(self.VmX - self.TargetX, 2) + math.pow(self.VmY - self.TargetY, 2)) / 2 
		return V

	def computeSimple(self):
		w = self.wOrbit + self.gain * np.sign((self.VmX - self.TargetX) * math.cos(self.Theta) 
							+ (self.VmY - self.TargetY) * math.sin(self.Theta))
		return w

	
		
def calc_sigmoid(x, eps):
	return x / (abs(x) + eps)
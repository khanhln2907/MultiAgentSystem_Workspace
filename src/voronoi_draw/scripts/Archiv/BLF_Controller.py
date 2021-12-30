import numpy as np
import rospy
import math
from voronoi_custom import getConvexBndMatrix 
from controlAlgo import *

class NeighborVoronoiInfo:
	neighborID = 0
	vm_coord_2d = 0
	cvt_coord_2d = 0
	com_v1_2d = 0
	com_v2_2d = 0

class NeighborLyapunoInfo:
	publisherID = 0
	neighborID = 0
	dCidzj = np.zeros([2,2])
	dVidzj = np.zeros([2,1])		# i is my index, j is the neighbor

	def getInfo(self):
		str = ""
		str += "i: %d > j: %d.  dVidzj: [%.8f, %.8f] dCidzj: [%.8f ,%.8f; %.8f ,%.8f] \n" %(self.publisherID, self.neighborID,\
														 self.dVidzj[0], self.dVidzj[1],\
														self.dCidzj[0,0], self.dCidzj[0,1], self.dCidzj[1,0], self.dCidzj[1,1])
		return str

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
		self.Vm = np.array([200*np.random.rand() + 30, 200*np.random.rand() + 30])
		self.VBLF = 0
		self.verticesList = 0
		

		# Algorithm Variables
		self.lastVmX = 0
		self.lastVmY = 0
		self.lastVBLF = 0
		self.curVBLF = 0 
		self.gain = 0

		# Constraints
		self.vConst = 0
		self.wThres = 0
		self.wOrbit = 0
		# Output
		self.angularVel = 0

		# Voronoi State
		self.CVT = np.array([0, 0])

		# Lyapunov State
		self.lastLyapunovReport = []
		self.dVidzi = 0

		# Evaluation of the computation
		self.dVBLF = 0
		self.dCidzi = np.array([[0.0, 0.0],[0.0, 0.0]])
		self.dC = np.array([0.0, 0.0])
		self.dVM = np.array([0.0, 0.0])
		self.dV = np.array([0.0, 0.0])
		self.updateCnt = 0

	def begin(self, controlGain, v0, w0, wCO, boundaries):
		self.gain = controlGain
		self.vConst = v0
		self.wOrbit = w0
		self.wThres = wCO
		[self.ABnd, self.bBnd] = getConvexBndMatrix(boundaries)
		for j in range(len(self.bBnd)):
			self.ABnd[j,0] = round(self.ABnd[j,0])
			self.ABnd[j,1] = round(self.ABnd[j,1])
			self.bBnd[j,0] = round(self.bBnd[j,0])
		
	def updateState(self, data):	
		self.ID = np.int32(data.packet.TransmitterID)
		x = np.float32(data.packet.AgentPosX)
		y = np.float32(data.packet.AgentPosY)
		the = np.float32(data.packet.AgentTheta)
		self.PosX = x
		self.PosY = y
		self.Theta = the
		# Update virtual mass
		# Save it befoe updating new values
		self.lastVmX = self.Vm[0]
		self.lastVmY = self.Vm[1]
		# Calculate the VM ourself so dont need to change the config in cpp file
		radius = self.vConst // self.wOrbit
		radius = 200
		self.Vm[0] = self.PosX -  radius* math.sin(self.Theta) 
		self.Vm[1] = self.PosY +  radius* math.cos(self.Theta)

		tmpdVMx = self.Vm[0] - self.lastVmX
		tmpdVMy = self.Vm[1] - self.lastVmY

		self.updateCnt += 1
		if(tmpdVMx == 0.0 and tmpdVMy == 0.0):
			pass
		else:
			self.dVM[0] = tmpdVMx
			self.dVM[1] = tmpdVMy
		#rospy.loginfo([self.Vm, [self.lastVmX, self.lastVmY], self.dVM])
		
	# Obain the new Voronoi information and return the partial derivatives / or Lyapuno feedback for adjacent agents
	def updateVoronoiInfo(self, myCVT, neighborTelegraph, mVi, bndCoeff):
		# sOME ONETIME CONFIG
		controlParam = ControlParameter()
		controlParam.eps = 5
		controlParam.gain = 3
		controlParam.P = 3
		controlParam.Q_2x2 = 5 * np.identity(2)

		# Update the CVT
		self.dC = myCVT - self.CVT
		self.CVT = myCVT 
		myZ = np.array([self.Vm[0], self.Vm[1]])

		nNeighbor = len(neighborTelegraph)

		dCi_dzi = 0
		dCi_dzj_list = []
		for i in range(nNeighbor):
			adjCoord_2d = np.array(neighborTelegraph[i].vm_coord_2d)
			ver1 = np.array(neighborTelegraph[i].com_v1_2d)
			ver2 = np.array(neighborTelegraph[i].com_v2_2d)
			dCi_dzi_AdjacentJ, dCi_dzj = Voronoi2DCalCVTPartialDerivative(myZ, myCVT, mVi, adjCoord_2d, ver1, ver2)
			
			dCi_dzi += dCi_dzi_AdjacentJ
			dCi_dzj_list.append(dCi_dzj)


		myAgent = VoronoiPrivateData()
		myAgent.C = myCVT
		myAgent.z = myZ
		myAgent.dCi_dzi = dCi_dzi
		self.dCidzi = dCi_dzi
		[V, self.dVidzi, dVidzj_Arr] = Voronoi2DCaldVdz(myAgent, dCi_dzj_list, bndCoeff, controlParam)
		self.dVBLF = V - self.lastVBLF
		self.lastVBLF = V

		dVidzjTelegraph = []
		for i in range(nNeighbor):
			tmp = NeighborLyapunoInfo()
			tmp.publisherID = self.ID
			tmp.neighborID = neighborTelegraph[i].neighborID
			tmp.dCidzj = dCi_dzj_list[i]
			tmp.dVidzj = dVidzj_Arr[i]
			dVidzjTelegraph.append(tmp)

		return dVidzjTelegraph
	
	def updateControlInput(self, lyapunovTelegraph):
		# Save the last sampled record for debugging purpose
		self.lastLyapunovReport = lyapunovTelegraph

		sumdV = self.dVidzi		
		for msg in lyapunovTelegraph:
			sumdV += msg.dVidzj

		# Constant parameter =================================
		# This should be initalised at the beginning, however config here for easy tuning
		self.wThres = 127
		self.vConst = 16
		self.gain = np.double(30) 
		self.wOrbit = 30
		eps = 5
		# Control output ====================================
		w = self.wOrbit + self.gain * calcSigmoid(sumdV[0] * math.cos(self.Theta) + sumdV[1] * math.sin(self.Theta), eps)

		# Output cutoff
		if(w > self.wThres):
			w = self.wThres
			rospy.logwarn("ANGULAR THRESHOLD VIOLATED: %.2f !", w)
		elif(w < -self.wThres):
			w = -self.wThres
			rospy.logwarn("ANGULAR THRESHOLD VIOLATED: %.2f !", w)

		self.angularVel = w
		return [self.vConst, self.angularVel]

	
	def getDebugInfo(self):
		lyapuStr = "dVidzi: [%.4f %4f]\n" %(self.dVidzi[0], self.dVidzi[1])
		for report in self.lastLyapunovReport:
			lyapuStr += report.getInfo()
	

		str = "Agent: %d\nP[%4.1f %4.1f %1.1f] VM[%4.4f %4.4f] C[%4.4f %4.4f] Vel[%3.2f %2.2f] V: %.3f  Err: %.2f\
					.\nFeedback:\n%s dVM [%.9f %.9f] dC [%.4f %.4f] dV: %.5f Cnt: %d\n"\
					%(self.ID, self.PosX, self.PosY, self.Theta, \
					self.Vm[0], self.Vm[1],\
					self.CVT[0], self.CVT[1],\
					self.vConst, self.angularVel,\
					self.lastVBLF,\
					math.sqrt(pow(self.Vm[0] - self.CVT[0],2) + pow(self.Vm[1] - self.CVT[1],2)),\
					lyapuStr, self.dVM[0], self.dVM[1], self.dC[0], self.dC[1], self.dVBLF,\
					self.updateCnt					)

		str += "dCidzi: [%.6f, %.6f; %.6f, %.6f] \n" %(self.dCidzi[0,0], self.dCidzi[0,1], self.dCidzi[1,0], self.dCidzi[1,1])
		
		return str

		

def calcSigmoid(x, eps):
	return x / (abs(x) + eps)
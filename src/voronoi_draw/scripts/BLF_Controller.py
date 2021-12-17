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
	dVidzj = 0		# i is my index, j is


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

		# Voronoi State
		self.CVT = np.array([0, 0])

		# Lyapunov State
		self.dVidzi = 0

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

	def updateVM(self, radius):
		# Save it befoe updating new values
		self.lastVmX = self.VmX
		self.lastVmY = self.VmY
		# Calculate the VM ourself so dont need to change the config in cpp file
		self.VmX = self.PosX - self.vConst / self.wOrbit * math.sin(self.Theta) 
		self.VmY = self.PosY + self.vConst / self.wOrbit * math.cos(self.Theta)

	def updateState(self, data):	
		self.ID = np.int32(data.packet.TransmitterID)
		self.PosX = np.float32(data.packet.AgentPosX)
		self.PosY = np.float32(data.packet.AgentPosY)
		self.Theta = np.float32(data.packet.AgentTheta)
		
		# Update this internally for centralized controller
		self.updateVM(200)	# Update the virtual center with radius 200 locally to avoid conflict with another node
		
	def updateLyapunov(self, newCVT_2d, V, dV):
		self.TargetX = newCVT_2d[0]
		self.TargetY = newCVT_2d[1]
		self.dVBLF = dV
		self.lastVBLF = V
		return 0
		
	# Obain the new Voronoi information and return the partial derivatives / or Lyapuno feedback for adjacent agents
	def updateVoronoiInfo(self, myCVT, neighborTelegraph, mVi, bndCoeff):
		# sOME ONETIME CONFIG
		controlParam = ControlParameter()
		controlParam.eps = 5
		controlParam.gain = 3
		controlParam.P = 3
		controlParam.Q_2x2 = 5 * np.identity(2)

		# Update the CVT
		self.CVT =myCVT 

		nNeighbor = len(neighborTelegraph)

		dCi_dzi = 0
		dCi_dzj_list = []
		for i in range(nNeighbor):
			myZ = np.array([self.VmX, self.VmY])
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
		[self.lastVBLF, self.dVidzi, dVidzj_Arr] = Voronoi2DCaldVdz(myAgent, dCi_dzj_list, bndCoeff, controlParam)

		dVidzjTelegraph = []
		for i in range(nNeighbor):
			tmp = NeighborLyapunoInfo()
			tmp.publisherID = self.ID
			tmp.neighborID = neighborTelegraph[i].neighborID
			tmp.dVidzj = dVidzj_Arr[i]
			dVidzjTelegraph.append(tmp)

		return dVidzjTelegraph
	
	def updateControlInput(self, lyapunovTelegraph):
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

	
		
def calcSigmoid(x, eps):
	return x / (abs(x) + eps)
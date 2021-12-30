# import numpy as np
# import math
# from voronoi_custom import getConvexBndMatrix
# from AgentBase import AgentBase 
# from UnicycleAgent import UnicycleAgent

# class NeighborInfo:
# 	neighborID = 0
# 	vm_coord_2d = 0
# 	cvt_coord_2d = 0
# 	com_v1_2d = 0
# 	com_v2_2d = 0


# # State Feedback from the sensos, qualisys, ...
# class BLFController(UnicycleAgent):
# 	def __init__(self):
# 		self.ID = -1
# 		# Coverage Information
# 		# [Parameter] Convex region: Ax - b <= 0
# 		self.ABnd = 0
# 		self.bBnd = 0
# 		# State Feedback
		
# 		self.VBLF = 0
# 		self.verticesList = 0
# 		# Target
# 		self.CVT = np.array([0,0])
		
# 		# Algorithm Variables
# 		self.lastVBLF = 0
# 		self.curVBLF = 0 
# 		self.dVBLF = 0
# 		self.gain = 0
# 		self.testW = 0

# 		# Constraints
# 		self.vConst = 0
# 		self.wThres = 0
# 		self.wOrbit = 0
# 		# Output
# 		self.angularVel = 0

		
# 	def updateLyapunov(self, newCVT_2d, V, dV):
# 		self.CVT[0] = newCVT_2d[0]
# 		self.CVT[1] = newCVT_2d[1]
# 		self.dVBLF = dV
# 		self.lastVBLF = V
# 		return 0
		
# 	# 	return 0

# 	def begin(self, controlGain, v0, w0, wCO, boundaries):
# 		self.gain = controlGain
# 		self.vConst = v0
# 		self.wOrbit = w0
# 		self.wThres = wCO
# 		[self.ABnd, self.bBnd] = getConvexBndMatrix(boundaries)
# 		for j in range(len(self.bBnd)):
# 			self.ABnd[j,0] = round(self.ABnd[j,0])
# 			self.ABnd[j,1] = round(self.ABnd[j,1])
# 			self.bBnd[j,0] = round(self.bBnd[j,0])

# 	def controlBLF(self, dV):	
# 		# Constant parameter =================================
# 		# This should be initalised at the beginning, however config here for easy tuning
# 		self.wThres = 1.5
# 		self.vConst = 16
# 		self.gain = np.double(1) 
# 		self.wOrbit = 0.5
# 		eps = 5
# 		# Control output ====================================
# 		w = self.wOrbit + self.gain * calc_sigmoid(dV[0] * math.cos(self.pose3[2]) + dV[1] * math.sin(self.pose3[2]), eps)
# 		# Output cutoff
# 		if(w > self.wThres):
# 			w = self.wThres
# 			print("ANGULAR THRESHOLD VIOLATED")
# 		elif(w < -self.wThres):
# 			w = -self.wThres
# 			print("ANGULAR THRESHOLD VIOLATED")

# 		# Constant Heading Velocity
# 		self.vConst = 16
# 		self.angularVel = w
# 		return [self.vConst, self.angularVel]



# 	# def computeVBLF(self):
# 	# 	nBndLines = len(self.bBnd)
# 	# 	tmpV = 0
# 	# 	for j in range(nBndLines):
# 	# 		tmpV += 1 // (self.bBnd[j] - self.ABnd[j][0] * self.vm2[0] + self.ABnd[j][1] * self.vm2[1])
# 	# 	V = tmpV * (math.pow(self.vm2[0] - self.CVT[0], 2) + math.pow(self.vm2[1] - self.CVT[1], 2)) / 2 
# 	# 	return V

# 	# def computeSimple(self):
# 	# 	w = self.wOrbit + self.gain * np.sign((self.vm2[0] - self.CVT[0]) * math.cos(self.pose3[2]) 
# 	# 						+ (self.vm2[1] - self.CVT[1]) * math.sin(self.pose3[2]))
# 	# 	return w

	
		
# def calc_sigmoid(x, eps):
# 	return x / (abs(x) + eps)
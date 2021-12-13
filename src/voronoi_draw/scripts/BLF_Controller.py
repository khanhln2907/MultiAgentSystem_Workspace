import numpy as np
import rospy
import math
from voronoi_custom import getConvexBndMatrix 

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

	def updateVM(self, radius):
		# Save it befoe updating new values
		self.lastVmX = self.VmX
		self.lastVmY = self.VmY
		# Calculate the VM ourself so dont need to change the config in cpp file
		radius = 200 # Using constant orbit Radius for now
		self.VmX = self.PosX - 200 * math.sin(self.Theta) 
		self.VmY = self.PosY + 200 * math.cos(self.Theta)

	def updateState(self, data):	
		#rospy.loginfo("Update new Info Agent: %d", agent.ID)
		self.ID = np.int32(data.packet.TransmitterID)
		self.PosX = np.float32(data.packet.AgentPosX)
		self.PosY = np.float32(data.packet.AgentPosY)
		self.Theta = np.float32(data.packet.AgentTheta)
		
		# Update this internally for centralized controller
		self.updateVM(200)	# Update the virtual center with radius 200 locally to avoid conflict with another node
		
	def updateBLFState(self, newCentroidX, newCentroidY):
		self.TargetX = newCentroidX
		self.TargetY = newCentroidY
		posFilterThres = 0
		dx = self.VmX - self.lastVmX
		dy = self.VmY - self.lastVmY
		isValid = abs(dx) > posFilterThres and abs(dy) > posFilterThres
		if(isValid):
			nBndLines = len(self.bBnd)
			tmpV = 0
			# Compute own BLF function
			for j in range(nBndLines):
				# Multiply by 10000 to increase accuracy 
				newS = 10000 // (-1 * self.bBnd[j,0] - self.ABnd[j,0]*self.VmX - self.ABnd[j,1]*self.VmY)
				tmpV += newS
				#if(self.ID == 20005):
				#	rospy.loginfo([self.ABnd[j,0], self.ABnd[j,1], -1 * self.bBnd[j,0], self.VmX, self.VmY, newS, tmpV])

			innerProd = math.pow(self.VmX - self.TargetX, 2) + math.pow(self.VmY - self.TargetY, 2)
			V = tmpV * innerProd 
			
			# This condition guarantees that agent maintains inside the coverage region
			if(V < 0):
				rospy.logwarn("ID: %d -> BLF violated", self.ID)

			# Update the distributed state and return to the centralized controller
			self.dVBLF = V - self.lastVBLF
			self.lastVBLF = V
		else:
			self.dVBLF = 0

		return self.dVBLF

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
		self.gain = np.double(25) 
		self.wOrbit = 30
		# Control output ====================================
		# Controller selection
		#w = self.simpleController()
		#		w = self.__controlBLF(neighborInfo)
		w = self.wOrbit + self.gain * np.sign(dV[0] * math.cos(self.Theta) + dV[1] * math.sin(self.Theta))

		# Output cutoff
		if(w > self.wThres):
			w = self.wThres
			rospy.logwarn("ANGULAR THRESHOLD VIOLATED: %.2f !", w)
		elif(w < -self.wThres):
			w = -self.wThres
			rospy.logwarn("ANGULAR THRESHOLD VIOLATED: %.2f !", w)

		# Constant Heading Velocity
		self.angularVel = w
		return [self.vConst, self.angularVel]



	def computeVBLF(self):
		nBndLines = len(self.bBnd)
		tmpV = 0
		for j in range(nBndLines):
			tmpV += 1 // (self.bBnd[j] - self.ABnd[j][0] * self.VmX + self.ABnd[j][1] * self.VmY)
		V = tmpV * (math.pow(self.VmX - self.TargetX, 2) + math.pow(self.VmY - self.TargetY, 2)) / 2 
		return V

	def simpleController(self):
		w = self.wOrbit + self.gain * np.sign((self.VmX - self.TargetX) * math.cos(self.Theta) 
							+ (self.VmY - self.TargetY) * math.sin(self.Theta))
		return w

	# This method is only using simple Euler derivative, however can be improved by theoretical integration 
	# and update the algorithm
	def __controlBLF(self, neighborInfo):
		#rospy.logwarn([neighborInfo, np.array(neighborInfo[:,1]), sum(np.array(neighborInfo[:,1]))])
		# Some filter position here
		posFilterThres = 0
		dx = self.VmX - self.lastVmX
		dy = self.VmY - self.lastVmY
		if(np.array(neighborInfo).size == 0):
			rospy.logwarn("No neighbor detected ! Double check Adjacent list")
			return self.angularVel
		else:
			isValid = abs(dx) > posFilterThres and abs(dy) > posFilterThres
			if(isValid):
				dV_total = (self.dVBLF + sum(np.array(neighborInfo[:,1]))) # Add this term to avoid division to the small number in the following lines
				dVdzx = dV_total / np.double(dx)
				dVdzy = dV_total / np.double(dy)
				w = self.wOrbit + self.gain * np.sign(dVdzx * math.cos(self.Theta) + dVdzy * math.sin(self.Theta))
				#rospy.loginfo("%d dV: %.7f dx: %.7f dy: %.7f, sumAdj: %.3f", self.ID, dV_total, dx, dy, dV_Adjacent)
				return w
			else:
				return self.angularVel	# If nothing happened, just keep the old angular velocity
		

import numpy as np
import math
from AgentBase import AgentBase
from voronoi_custom import getConvexBndMatrix 
from controlAlgo import *

class UnicycleAgent(AgentBase):
    pose3 = np.array([0,0,0])
    lastPose3 = pose3
    vm2 = np.array([0,0])

    def __init__(self, pose3) -> None:
        self.pose3 = pose3
        self.vm2 = np.array([pose3[0],pose3[1]])
        print("Agent:", self.vm2)

    def getPose(self):
        return self.pose3, self.vm2

    def updateVM(self, radius):
		# Calculate the VM ourself so dont need to change the config in cpp file
		# radius = 200 # Using constant orbit Radius for now
        self.vm2[0] = self.pose3[0] -  radius* math.sin(self.pose3[2]) 
        self.vm2[1] = self.pose3[1] +  radius* math.cos(self.pose3[2])


class UnicycleCoverageAgent(UnicycleAgent):
    
    def __init__(self, pose3) -> None:
        super().__init__(pose3)
        
        self.ABnd = 0
        self.bBnd = 0
        # State Feedback
        self.VBLF = 0
        self.verticesList = 0
        # Target
        self.CVT = np.array([0,0])
        # Algorithm Variables
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

    def controlBLF(self, dV):	
        # Constant parameter =================================
        # This should be initalised at the beginning, however config here for easy tuning
        self.wThres = 1.5
        self.vConst = 16
        self.gain = np.double(1) 
        self.wOrbit = 0.5
        eps = 5
        # Control output ====================================
        w = self.wOrbit + self.gain * calcSigmoid(dV[0] * math.cos(self.pose3[2]) + dV[1] * math.sin(self.pose3[2]), eps)
        # Output cutoff
        if(w > self.wThres):
            w = self.wThres
            print("ANGULAR THRESHOLD VIOLATED")
        elif(w < -self.wThres):
            w = -self.wThres
            print("ANGULAR THRESHOLD VIOLATED")

        # Constant Heading Velocity
        self.vConst = 16
        self.angularVel = w
        return [self.vConst, self.angularVel]

    def updateLyapunov(self, newCVT_2d, V, dV):
        self.CVT[0] = newCVT_2d[0]
        self.CVT[1] = newCVT_2d[1]
        self.dVBLF = dV
        self.lastVBLF = V
        return 0

class SimUnicycleCoverageAgent(UnicycleCoverageAgent):
    dt = 0
    def __init__(self, dt, pose3) -> None:
        super().__init__(pose3)
        self.dt = dt


    def move(self, v, w, wOrbit):  
        self.lastPose3 = self.pose3
        self.pose3[0] = self.pose3[0] +  self.dt * (v * math.cos(self.pose3[2]))
        self.pose3[1] = self.pose3[1] +  self.dt * (v * math.sin(self.pose3[2]))
        self.pose3[2] = self.pose3[2] +  self.dt * w
        # Update the virtual mass
        self.updateVM(v//wOrbit)
        

def calcSigmoid(x, eps):
	return x / (abs(x) + eps)
import numpy as np
import math
from AgentSimulationBase import AgentSimulationBase
from BLFController import BLFController

class UnicycleAgent(AgentSimulationBase, BLFController):
    dt = 0
    pose3 = np.array([0,0,0])
    lastPose3 = pose3
    vm2 = np.array([0,0])
    
    def __init__(self, dt, pose3) -> None:
        self.dt = dt
        self.pose3 = pose3
        self.vm2 = np.array([pose3[0],pose3[1]])
        pass

    def move(self, v, w, wOrbit):
        
        self.lastPose3 = self.pose3

        self.pose3[0] = self.pose3[0] +  self.dt * (v * math.cos(self.pose3[2]))
        self.pose3[1] = self.pose3[1] +  self.dt * (v * math.sin(self.pose3[2]))
        self.pose3[2] = self.pose3[2] +  self.dt * w

        # Update the virtual mass
        self.vm2[0] = self.pose3[0] - (v//wOrbit) * math.sin(self.pose3[2])
        self.vm2[1] = self.pose3[1] + (v//wOrbit) * math.cos(self.pose3[2])

        print("Movement ", self.pose3, self.vm2, "with ", v, w, wOrbit)

    def getPose(self):
        return self.pose3, self.vm2


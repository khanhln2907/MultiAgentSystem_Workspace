import numpy as np
import math
from voronoi_custom import getConvexBndMatrix 
from controlAlgo import *
import time
from tip.msg import ControlMsg

class LoggingInfo:
    ID = 0
    Timestamp = 0
    pose3 = np.float32(np.array([0,0,0]))
    vm2 = np.array([0,0])
    CVT2 = np.array([0,0])
    w = 0
    Vi = 0

    def toString(self):
        str = "%d, %d, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f" %(self.ID, self.Timestamp, self.pose3[0], self.pose3[1], self.pose3[2], 
                                                                            self.vm2[0], self.vm2[1], self.CVT2[0], self.CVT2[1], self.w, self.Vi)
        return str

    def parse(self, str):
        tmp = str.split(',')
        self.ID = int(tmp[0])
        self.Timestamp = int(tmp[1])
        self.pose3 = np.array([float(tmp[2]), float(tmp[3]), float(tmp[4])])
        self.vm2 = np.array([float(tmp[5]), float(tmp[6])])
        self.CVT2 = np.array([float(tmp[7]), float(tmp[8])])
        self.w = float(tmp[9])
        self.Vi = float(tmp[10])
        return self.toString()

class AgentBase(object):
    ID = -1

    def __init__(self): #-> None:
        #self.ID = ID
        pass

    def move(self):
        raise NotImplementedError

    def getPose(self):
        raise NotImplementedError

    def getLog(self):
        raise NotImplementedError

class UnicycleAgent(AgentBase):
    pose3 = np.array([0,0,0])
    lastPose3 = pose3
    vm2 = np.array([0,0])

    def __init__(self, ID, pose3): # -> None:
        super(AgentBase, self).__init__()
        self.ID = ID
        self.pose3 = pose3
        self.vm2 = np.array([pose3[0],pose3[1]])

    def getPose(self):
        return self.pose3, self.vm2

    def updateVM(self, radius):
        self.vm2[0] = self.pose3[0] -  radius* math.sin(self.pose3[2]) 
        self.vm2[1] = self.pose3[1] +  radius* math.cos(self.pose3[2])


class UnicycleCoverageAgent(UnicycleAgent):
    
    def __init__(self, ID, pose3): # -> None:
        super(UnicycleAgent, self).__init__(ID, pose3)
        
        self.ABnd = 0
        self.bBnd = 0
        # State Feedback
        self.VBLF = 0
        self.verticesList = 0
        # Target
        self.CVT2 = np.array([0,0])
        # Algorithm Variables
        self.controlParam = 0

    def setParameter(self, param):
        self.controlParam = param

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

    
    # Obain the new Voronoi information and return the partial derivatives / or Lyapuno feedback for adjacent agents
    def updateVoronoiInfo(self, myCVT, neighborTelegraph, mVi, bndCoeff):
        # Update the CVT
        #self.dC = myCVT - self.CVT2
        self.CVT2 = myCVT 
        myZ = np.array([self.vm2[0], self.vm2[1]])

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
        [self.VBLF, self.dVidzi, dVidzj_Arr] = Voronoi2DCaldVdz(myAgent, dCi_dzj_list, bndCoeff, self.controlParam)
       
        dVidzjTelegraph = []
        for i in range(nNeighbor):
            tmp = NeighborLyapunovInfo()
            tmp.publisherID = self.ID
            tmp.neighborID = neighborTelegraph[i].neighborID
            tmp.dCidzj = dCi_dzj_list[i]
            tmp.dVidzj = dVidzj_Arr[i]
            dVidzjTelegraph.append(tmp)

        return self.VBLF, dVidzjTelegraph

    def calcControlInput(self, lyapunovTelegraph):
		# Save the last sampled record for debugging purpose
        self.lastLyapunovReport = lyapunovTelegraph

        sumdV = self.dVidzi		
        for msg in lyapunovTelegraph:
            sumdV += msg.dVidzj

        # Control output ====================================
        w = self.controlParam.wOrbit +\
            self.controlParam.gain * calcSigmoid(sumdV[0] * math.cos(self.pose3[2]) + sumdV[1] * math.sin(self.pose3[2]), self.controlParam.eps)

        # Output cutoff
        if(w > self.controlParam.wThres):
            print("CONTROL SATURATION VIOLATED")
            w = self.wThres
        elif(w < -self.controlParam.wThres):
            print("CONTROL SATURATION VIOLATED")
            w = -self.wThres

        self.angularVel = w
        return [self.controlParam.vConst, self.angularVel]

class ROSUnicycleCoverageAgent(UnicycleCoverageAgent):
    def __init__(self, ID, pose3): # -> None:
        super(UnicycleCoverageAgent, self).__init__(ID, pose3)

    def updatePose(self, pose3, radius):
        self.lastPose3 = self.pose3
        self.pose3[0] = pose3[0] 
        self.pose3[1] = pose3[1] 
        self.pose3[2] = pose3[2] 
        # Update the virtual mass
        self.updateVM(radius) # the theoretical radius is vConst // wOribt

    def command(self, v, w, publishHandle):  
        # Publish the message
        msg = ControlMsg()
        msg.ID = self.ID
        msg.translation = v
        msg.rotation = w			
        publishHandle.publish(msg)

    def getLogStr(self):
        report = LoggingInfo()
        report.ID = self.ID
        report.Timestamp = round(time.time()*1000)
        report.pose3 = self.pose3
        report.Vi = self.VBLF
        report.w = self.angularVel
        report.vm2 = self.vm2
        report.CVT2 = self.CVT2

        return report.toString()

class SimUnicycleCoverageAgent(UnicycleCoverageAgent):
    dt = 0
    def __init__(self, ID, dt, pose3): # -> None:
        super(UnicycleCoverageAgent, self).__init__(ID, pose3)
        self.dt = dt


    def move(self, v, w, wOrbit):  
        self.lastPose3 = self.pose3
        self.pose3[0] = self.pose3[0] +  self.dt * (v * math.cos(self.pose3[2]))
        self.pose3[1] = self.pose3[1] +  self.dt * (v * math.sin(self.pose3[2]))
        self.pose3[2] = self.pose3[2] +  self.dt * w
        # Update the virtual mass
        self.updateVM(v//wOrbit)

    def getLogStr(self):
        report = LoggingInfo()
        report.ID = self.ID
        report.Timestamp = time.time()
        report.pose3 = self.pose3
        report.Vi = self.VBLF
        report.w = self.angularVel
        report.vm2 = self.vm2
        report.CVT2 = self.CVT2

        return report.toString()


def calcSigmoid(x, eps):
	return x / (abs(x) + eps)
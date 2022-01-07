import numpy as np
import math
from voronoi_custom import getConvexBndMatrix 
from controlAlgo import *
import time

class LoggingInfo:
    ID = 0
    Timestamp = 0
    pose3 = np.array([0,0,0])
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
        print("Agent: ", ID,  self.vm2)

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
        # Output
        self.vConst = 0
        self.angularVel = 0
        self.wThres = 0
        self.wOrbit = 0
        
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
        # sOME ONETIME CONFIG
        controlParam = ControlParameter()
        controlParam.eps = 5
        controlParam.gain = 3
        controlParam.P = 3
        controlParam.Q_2x2 = 5 * np.identity(2)

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
        [self.VBLF, self.dVidzi, dVidzj_Arr] = Voronoi2DCaldVdz(myAgent, dCi_dzj_list, bndCoeff, controlParam)
       
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

        # Constant parameter =================================
        # This should be initalised at the beginning, however config here for easy tuning
        # self.wThres = 127
        # self.vConst = 16
        # self.gain = np.double(30) 
        # self.wOrbit = 30
        # eps = 5
        self.wThres = 1.5
        self.vConst = 16
        self.gain = np.double(1) 
        self.wOrbit = 0.5
        eps = 5
        # Control output ====================================
        w = self.wOrbit + self.gain * calcSigmoid(sumdV[0] * math.cos(self.pose3[2]) + sumdV[1] * math.sin(self.pose3[2]), eps)

        # Output cutoff
        if(w > self.wThres):
            w = self.wThres
        elif(w < -self.wThres):
            w = -self.wThres

        self.angularVel = w
        return [self.vConst, self.angularVel]

class ROSUnicycleCoverageAgent(UnicycleCoverageAgent):
    def __init__(self, ID, pose3): # -> None:
        super(UnicycleCoverageAgent, self).__init__(pose3)

    def updatePose(self, pose3, v, wOrbit):
        self.lastPose3 = self.pose3
        self.pose3[0] = pose3[0] 
        self.pose3[1] = pose3[1] 
        self.pose3[2] = pose3[2] 
        # Update the virtual mass
        self.updateVM(v//wOrbit)

    def command(self, v, w):  
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
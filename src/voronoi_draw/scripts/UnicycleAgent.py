import numpy as np
import math
from AgentBase import AgentBase
from voronoi_custom import getConvexBndMatrix 
from controlAlgo import *

class VoronoiPrivateData():
    C = np.array([0, 0])
    z = np.array([0, 0])
    dCi_dzi = grad_2d(np.array([[0, 0], [0, 0]]))
    

class ControlParameter:
    eps = 0
    P = 1
    Q_2x2 = np.array([[0, 0], [0, 0]])
    gain = 1

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

class UnicycleAgent(AgentBase):
    pose3 = np.array([0,0,0])
    lastPose3 = pose3
    vm2 = np.array([0,0])

    def __init__(self, ID, pose3) -> None:
        super().__init__(ID)
        self.pose3 = pose3
        self.vm2 = np.array([pose3[0],pose3[1]])
        print("Agent: ", ID,  self.vm2)

    def getPose(self):
        return self.pose3, self.vm2

    def updateVM(self, radius):
		# Calculate the VM ourself so dont need to change the config in cpp file
		# radius = 200 # Using constant orbit Radius for now
        self.vm2[0] = self.pose3[0] -  radius* math.sin(self.pose3[2]) 
        self.vm2[1] = self.pose3[1] +  radius* math.cos(self.pose3[2])


class UnicycleCoverageAgent(UnicycleAgent):
    
    def __init__(self, ID, pose3) -> None:
        super().__init__(ID, pose3)
        
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

        return V, dVidzjTelegraph

    def calcControlInput(self, lyapunovTelegraph):
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
        w = self.wOrbit + self.gain * calcSigmoid(sumdV[0] * math.cos(self.pose3[2]) + sumdV[1] * math.sin(self.pose3[2]), eps)

        # Output cutoff
        if(w > self.wThres):
            w = self.wThres
        elif(w < -self.wThres):
            w = -self.wThres

        self.angularVel = w
        return [self.vConst, self.angularVel]

class SimUnicycleCoverageAgent(UnicycleCoverageAgent):
    dt = 0
    def __init__(self, ID, dt, pose3) -> None:
        super().__init__(ID, pose3)
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
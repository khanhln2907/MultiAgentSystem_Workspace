import sys

from numpy.core.numeric import identity
sys.path.append('C:\\Users\\khanh\\Desktop\\Workspace\\MultiAgentSystem_Workspace\\voronoi_draw\\scripts')

import numpy as np
from CentralizedControllerBase import *
from UnicycleAgent import *

np.random.seed(0)

class SimParam:
    nAgent = 4
    dt = 0.01
    boundaries = np.array([[20,20], [20,2800], [4000,2800], [4000, 20]])
    wOrbit = 0.5
    vConst = 16
    P = 3
    EPS_SIGMOID = 5
    Q_2x2 = 5 * np.identity(2)

config = SimParam()

com = CentralizedControllerBase()
com.begin(4, config.boundaries)


agentList = []

rXY = 2000;      
nAgent = 4

for i in range(config.nAgent):
    pose3 = np.array([rXY * np.random.rand(), rXY * np.random.rand(), 0])
    agentList.append(UnicycleAgent(config.dt, pose3))

while 1:
    pntsArr = []
    for i in range(config.nAgent):
        _, vm2 = agentList[i].getPose()
        pntsArr.append(vm2)

    #print("Points Collected:", pntsArr) 
    controlInput, lyaRet = com.updateCoverage(pntsArr)

    totV = 0
    for report in lyaRet:
        totV += report[0][0]

    #print(totV, controlInput)
    
    for i in range(config.nAgent):
        agentList[i].move(config.vConst, controlInput[i], config.wOrbit)
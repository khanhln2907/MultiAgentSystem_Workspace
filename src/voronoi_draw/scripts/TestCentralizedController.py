import sys
import time
from numpy.core.numeric import identity
sys.path.append('C:\\Users\\khanh\\Desktop\\Workspace\\MultiAgentSystem_Workspace\\voronoi_draw\\scripts')

import numpy as np
from CentralizedControllerBase import CentralizedControllerBase
from Agent import SimUnicycleCoverageAgent, LoggingInfo

import logging
name = "Logging/LogSim%d.log" %(time.time())
logging.basicConfig(filename = name, encoding='utf-8', level=logging.DEBUG)


np.random.seed(6)

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




agentList = []

rXY = 2000;      
nAgent = 4

for i in range(config.nAgent):
    pose3 = np.array([rXY * np.random.rand(), rXY * np.random.rand(), np.random.rand()])
    agent = SimUnicycleCoverageAgent(i, config.dt, pose3)
    agent.begin(1, 2, 1, 2, config.boundaries)
    agentList.append(agent)

com = CentralizedControllerBase(agentList, config.boundaries)

while 1:
    pntsArr = []
    for i in range(config.nAgent):
        _, vm2 = agentList[i].getPose()
        pntsArr.append(vm2)

    if 0:
        controlInput, _ = com.updateCoverageDep(pntsArr)
    else:
        totV, controlInput = com.updateCoverage(pntsArr)
        print(totV)
    
    str = "Logging \n"
    for i in range(config.nAgent):
        str += agentList[i].getLogStr()
        str += "\n"
    
    logging.info(str)

    for i in range(config.nAgent):
        agentList[i].move(config.vConst, controlInput[i], config.wOrbit)

    
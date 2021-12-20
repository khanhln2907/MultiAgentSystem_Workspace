import sys
sys.path.append('C:\\Users\\khanh\\Desktop\\Workspace\\MultiAgentSystem_Workspace\\voronoi_draw\\scripts')

import numpy as np
from CentralizedControllerBase import *


com = CentralizedControllerBase()
boundaries = np.array([[20,20], [20,2800], [4000,2800], [4000, 20]])
com.begin(4, boundaries)


pntsArr = [[50,23], [234, 543], [232, 124], [227, 596]]

while 1:
    com.updateCoverage(pntsArr)

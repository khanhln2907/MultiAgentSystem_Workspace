import numpy as np

class grad_2d:
    dx_dx = 0
    dx_dy = 0 
    dy_dx = 0 
    dy_dy = 0

    def __init__(self, mat2x2):
        self.dx_dx = mat2x2[0,0]
        self.dx_dy = mat2x2[0,1]
        self.dy_dx = mat2x2[1,0]
        self.dy_dy = mat2x2[1,1]
        pass

   
    def npForm(self):
        return np.array([[self.dx_dx, self.dx_dy],
                        [self.dy_dx, self.dy_dy]])

    def __call__(self):
        return np.array([[self.dx_dx, self.dx_dy],
                        [self.dy_dx, self.dy_dy]])

    def show(self):
        print("dCx_dzx", self.dx_dx, "dCx_dzy", self.dx_dy, "dCy_dzx", self.dy_dx, "dCy_dzy", self.dy_dy)

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


class vorAgentData:
    C = np.array([0, 0])
    z = np.array([0, 0])

class vorPrivateData(vorAgentData):
    dCi_dzi = grad_2d(np.array([[0, 0], [0, 0]]))

class vorNeighborData(vorAgentData):
    dCj_dzi = grad_2d(np.array([[0, 0], [0, 0]]))
    

class controlParameter:
    eps = 0
    P = 1
    Q_2x2 = np.array([[0, 0], [0, 0]])
    gain = 1
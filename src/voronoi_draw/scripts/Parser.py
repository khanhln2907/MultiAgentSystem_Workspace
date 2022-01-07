import numpy as np
from Agent import LoggingInfo
import matplotlib.pyplot as plt

class TimeSeries: 
    def __init__(self):
        self.cnt = 0
        self.samples = [None for i in range(1000000)]
        self.ID = []
        self.time = []
        self.pose3 = []
        self.vm2 = []
        self.CVT2 = []
        self.V = 0
        pass

    def addSample(self, id, sample):
        self.ID = id
        self.samples[self.cnt] = sample
        self.cnt += 1

    def toArray(self):
        self.time = np.ndarray((self.cnt,), np.longlong)
        self.pose3 = np.zeros((self.cnt, 3))
        self.vm2 = np.zeros((self.cnt, 2))
        self.CVT2 = np.zeros((self.cnt, 2))
        self.V = np.ndarray((self.cnt,), float)
        for i in range(self.cnt):
            self.time[i] = self.samples[i].Timestamp
            self.pose3[i,:] = self.samples[i].pose3
            self.vm2[i,:] = self.samples[i].vm2
            self.V[i] = self.samples[i].Vi
            self.CVT2[i,:] = self.samples[i].CVT2

NAGENT = 4

boundaries = np.array([[20,20], [20,2800], [4000,2800], [4000, 20]])
logHandles = []

for i in range(NAGENT):
    h = TimeSeries()
    logHandles.append(h)

logHandles = list(logHandles)
file = "/home/qingchen/catkin_ws/src/voronoi_draw/scripts/Logging/" + "LogSim1641577278.log" 
REAL = 1

a0 = TimeSeries()
a1 = TimeSeries()
a2 = TimeSeries()
a3 = TimeSeries()


with open(file) as f:
    lines = f.readlines() # list containing lines of file
   
for line in lines:
    line = line.strip() # remove leading/trailing white spaces
    if (len(line) <20):
        pass
    else:
        tmp = LoggingInfo()
        tmp.parse(line)
        id = tmp.ID
        if(REAL):
            if(id == 20003):
                logHandles[0].addSample(id, tmp)
            elif(id == 20005):
                logHandles[1].addSample(id, tmp)
            elif(id == 20006):
                logHandles[2].addSample(id, tmp)
            elif(id == 20007):
                logHandles[3].addSample(id, tmp)
            else:
                print("UNDEFINED PAGENT ID", id)
        else:          
            logHandles[id].addSample(id, tmp)

sumV = 0
for handle in logHandles:
    handle.toArray()
    plt.plot(range(0, handle.cnt), handle.V)
    sumV = sumV + handle.V
plt.plot(range(0, handle.cnt), sumV)

fig1, ax1 = plt.subplots()
for handle in logHandles:
    ax1.plot(handle.vm2[:,0], handle.vm2[:,1], 'o', color = 'green')
    ax1.plot(handle.pose3[:,0], handle.pose3[:,1], 'x', color = 'blue')
    ax1.plot(handle.CVT2[:,0], handle.CVT2[:,1], 'o', color='red')
ax1.set_xlim([20, 4000])
ax1.set_ylim([20, 2800])
        
plt.show()     





          
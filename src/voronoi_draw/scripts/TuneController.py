#! /usr/bin/env python
import os
import time
import math
import random
import numpy as np
# from matplotlib import pyplot as plt
import cv2
import message_filters
import scipy.ndimage as ndimage
# import seaborn as sns
# import matplotlib.pyplot as plt
#from skimage.draw import circle
#from skimage.feature import peak_local_max
import rospy
from cv_bridge import CvBridge
from rospy.numpy_msg import numpy_msg
#from tip.msg import Vector
# from rospy_tutorials.msg import Floats

# some message type
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Image, CameraInfo
from platform import python_version

from tip.msg import UnicycleInfoMsg
from tip.msg import ControlMsg

import numpy as np
from CentralizedControllerBase import CentralizedControllerBase
from Agent import SimUnicycleCoverageAgent,ROSUnicycleCoverageAgent, LoggingInfo
import sys
sys.path.append('/home/qingchen/catkin_ws/src/voronoi_draw')

# These are logged by python Logging module, no ROS
import logging
logFileName = "/home/qingchen/catkin_ws/src/voronoi_draw/scripts/" + "Logging/LogSim%d.log" %(time.time())
logging.basicConfig(filename = logFileName, encoding='utf-8', level=logging.DEBUG)


def updateAgentInfo(data):
	global cntRegisteredROSAgent, agentList, config, readyFlag

	# Update all the subscribed topic from ROS
	# SENSORS - State Feedback
	# Obtain the new data from agents and assign into the controller lists
	# Find the registered Agent ID and assign the values
	isAssigned = False
	for agent in agentList:
		if ((agent.ID == -1) or (agent.ID == np.int32(data.packet.TransmitterID))):
			# Assign new agent
			if(agent.ID == -1):
				cntRegisteredROSAgent += 1
				# Update the state
				agent.ID = np.int32(data.packet.TransmitterID)
			
			# Update the new pose
			pose3 = np.array([np.float32(data.packet.AgentPosX), np.float32(data.packet.AgentPosY), np.float32(data.packet.AgentTheta)])
			agent.updatePose(pose3, config.vConst, config.wOrbit)
			# Check all agents are registered
			isAssigned = True
			break

	if(isAssigned == False):
		print("List is full ! New Agent detected")
	
# Initialize the simulation's parameters
np.random.seed(6)


def updateAgentInfo(data):
	global cntRegisteredROSAgent, agentList, config, readyFlag

	# Update all the subscribed topic from ROS
	# SENSORS - State Feedback
	# Obtain the new data from agents and assign into the controller lists
	# Find the registered Agent ID and assign the values
	isAssigned = False
	for agent in agentList:
		if ((agent.ID == -1) or (agent.ID == np.int32(data.packet.TransmitterID))):
			# Assign new agent
			if(agent.ID == -1):
				cntRegisteredROSAgent += 1
				# Update the state
				agent.ID = np.int32(data.packet.TransmitterID)
			
			# Update the new pose
			pose3 = np.array([np.float32(data.packet.AgentPosX), np.float32(data.packet.AgentPosY), np.float32(data.packet.AgentTheta)])
			agent.updatePose(pose3, config.vConst, config.wOrbit)
			# Check all agents are registered
			isAssigned = True
			break

	if(isAssigned == False):
		print("List is full ! New Agent detected")

class SimParam:
	nAgent = 4
	SIM = 0
	dt = 0.01
	boundaries = np.array([[20,20], [20,2800], [4000,2800], [4000, 20]])
	wOrbit = 0.5
	vConst = 16
	P = 3
	EPS_SIGMOID = 5
	Q_2x2 = 5 * np.identity(2)

# Initialize the simulation's parameters
np.random.seed(6)
config = SimParam()

cntRegisteredROSAgent = 0

agentList = []  
for i in range(config.nAgent):
	if(config.SIM):
		rXY = 2000;    
		pose3 = np.array([rXY * np.random.rand(), rXY * np.random.rand(), np.random.rand()])
		agent = SimUnicycleCoverageAgent(-1, config.dt, pose3)
		agent.begin(1, 2, 1, 2, config.boundaries)
		agentList.append(agent)
		pass
	else:
		pose3 = np.float32(np.array([0.0, 0.0, 0.0]))
		agent = ROSUnicycleCoverageAgent(-1, pose3)
		agent.begin(1, 2, 1, 2, config.boundaries)
		agentList.append(agent)

if __name__ == '__main__':
    rospy.init_node('stream_voronoi')
    rate = rospy.Rate(50)
    Info3 = rospy.Subscriber('/san/CoverageInfo', UnicycleInfoMsg, updateAgentInfo)
    Info5 = rospy.Subscriber('/wu/CoverageInfo', UnicycleInfoMsg, updateAgentInfo)
    Info6 = rospy.Subscriber('/liu/CoverageInfo', UnicycleInfoMsg, updateAgentInfo)
    Info7 = rospy.Subscriber('/qi/CoverageInfo', UnicycleInfoMsg, updateAgentInfo)
    Publisher = rospy.Publisher('centralNode/controlInput', ControlMsg, queue_size = 1)	

    # Reinitialize the logger	
    logger = logging.getLogger('__name__')  
    fh = logging.FileHandler(logFileName)  
    logger.setLevel(logging.DEBUG)  
    logger.addHandler(fh)  
    logger.info("TUNING CONTROLLER")

    while not rospy.is_shutdown():
        msg = ControlMsg()
        msg.ID = 3
        msg.translation = 10
        msg.rotation = 60			
        Publisher.publish(msg)


        # Logging routines
        str = "Logging \n"
        for i in range(config.nAgent):
            str += agentList[i].getLogStr()
            str += "\n"	
        logger.info(str)
        print(str)

        rate.sleep()







	
#! /usr/bin/env python

import os
import time
import math
import numpy as np
# from matplotlib import pyplot as plt
import cv2
import message_filters
import shapely
import scipy.ndimage as ndimage
# import seaborn as sns
# import matplotlib.pyplot as plt
from skimage.draw import circle
from skimage.feature import peak_local_max
import rospy
from cv_bridge import CvBridge
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Float32MultiArray
# from rospy_tutorials.msg import Floats

# some message type
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, CameraInfo
from platform import python_version
from shapely.geometry import Polygon




bridge = CvBridge()
print(python_version())
rospy.init_node('ggcnn_detection')

# Output publishers.
dynamic_painting_pub = rospy.Publisher('/img/paint', Image, queue_size=1)

# Execution Timing
class TimeIt:
    def __init__(self, s):
        self.s = s
        self.t0 = None
        self.t1 = None
        self.print_output = True

    def __enter__(self):
        self.t0 = time.time()

    def __exit__(self, t, value, traceback):
        self.t1 = time.time()
        if self.print_output:
            print('%s time: %s' % (self.s, self.t1 - self.t0))


def save_np_img(folder, name, data):
    np.save(os.path.join(folder, name), data)


def Cal_mid_point(a1, a2):
    mid_point = (a1 + a2)/2
    #print(mid_point[0])
    return mid_point
    

def findIntersection(line1, line2):
        x1 = line1[0]
        y1 = line1[1]
        x2 = line1[2]
        y2 = line1[3]

        x3 = line2[0]
        y3 = line2[1]
        x4 = line2[2]
        y4 = line2[3]

        denominator = (x1 - x2)*(y3 - y4) - (y1 - y2)*(x3 - x4)

        if denominator == 0:
	    return np.array([0,0])		

	xNominator = (x1*y2 - y1*x2)*(x3 - x4) - (x1 - x2)*(x3*y4 - y3*x4)
        yNominator = (x1*y2 - y1*x2)*(y3 - y4) - (y1 - y2)*(x3*y4 - y3*x4)

	px = xNominator / denominator
        py = yNominator / denominator

	if px >= x3 and px <= x4 and py >= y3 and py <= y4:
       	    return np.array([px,py])
	else:
	    return np.array([0,0])


def findIntersection1(line1, line2):
        x1 = line1[0]
        y1 = line1[1]
        x2 = line1[2]
        y2 = line1[3]

        x3 = line2[0]
        y3 = line2[1]
        x4 = line2[2]
        y4 = line2[3]

        denominator = (x1 - x2)*(y3 - y4) - (y1 - y2)*(x3 - x4)

        if denominator == 0:
	    return np.array([0,0])		

	xNominator = (x1*y2 - y1*x2)*(x3 - x4) - (x1 - x2)*(x3*y4 - y3*x4)
        yNominator = (x1*y2 - y1*x2)*(y3 - y4) - (y1 - y2)*(x3*y4 - y3*x4)

	px = xNominator / denominator
        py = yNominator / denominator

	return np.array([px,py])
   

def findPerpenticularVector(line1):
        x1 = line1[0]
        y1 = line1[1]
        x2 = line1[2]
        y2 = line1[3]

	return np.array([y1-y2, x2-x1])

def callback(rgb_data):
    with TimeIt('rendering'):
        #print(rgb_data.header.stamp)
        
    	rgb = bridge.imgmsg_to_cv2(rgb_data, 'rgb8')

##############################################coverage area##################################################

	rectangle_1 = np.array([30,20,600,20])

	rectangle_2 = np.array([600,20,600,460])

	rectangle_3 = np.array([30,460,600,460])

	rectangle_4 = np.array([30,20,30,460])  	

    	lineThickness = 2
    	cv2.rectangle(rgb, (30, 20), (600, 460), (0,0,255), lineThickness) 

##################################################3 agents###################################################

	agent_1 = np.array([80, 100])
	agent_2 = np.array([500,200])
	agent_3 = np.array([300,350])

    	cv2.circle(rgb, (80,  100), 8, (255,0,0), -1)
    	cv2.circle(rgb, (500, 200), 8, (0,255,0), -1)
	cv2.circle(rgb, (300, 350), 8, (0,0,255), -1)  
 

################################################Compute E1####################################################

	distance12 = math.sqrt(((agent_1[0] - agent_2[0])**2)+((agent_1[1] - agent_2[1])**2))
	distance13 = math.sqrt(((agent_1[0] - agent_3[0])**2)+((agent_1[1] - agent_3[1])**2))

	if distance12 <= distance13:
		connection_1 = np.concatenate((agent_1, agent_2), axis=None)
		connection_2 = np.concatenate((agent_1, agent_3), axis=None)
		connect_agent = agent_2
		disconnect_agent = agent_3
		disconnect_distance = distance13 
	else:
		connection_1 = np.concatenate((agent_1, agent_3), axis=None)
		connection_2 = np.concatenate((agent_1, agent_2), axis=None)
		connect_agent = agent_3
		disconnect_agent = agent_2
		disconnect_distance = distance12  	

	temp = findPerpenticularVector(connection_1)

	temp_line = np.concatenate((connect_agent, connect_agent+temp), axis=None)

	temp = findIntersection(temp_line, rectangle_1)

	end_point_count = 0

	if temp.all() != False:
		#cv2.circle(rgb, (temp[0],temp[1]), 8, (100,100,100), -1)
		end_point_1 = temp
		end_point_count = end_point_count + 1 

	temp = findIntersection(temp_line, rectangle_2)

	if temp.all() != False:
		#cv2.circle(rgb, (temp[0],temp[1]), 8, (100,100,100), -1)
		if end_point_count == 0:							
			end_point_1 = temp
			end_point_count = end_point_count + 1

		if end_point_count == 1:
			end_point_2 = temp
			end_point_count = end_point_count + 1

	temp = findIntersection(temp_line, rectangle_3)

	if temp.all() != False:
		#cv2.circle(rgb, (temp[0],temp[1]), 8, (100,100,100), -1)
		if end_point_count == 0:							
			end_point_1 = temp
			end_point_count = end_point_count + 1

		if end_point_count == 1:
			end_point_2 = temp
			end_point_count = end_point_count + 1

 	temp = findIntersection(temp_line, rectangle_4)

	if temp.all() != False:
		#cv2.circle(rgb, (temp[0],temp[1]), 8, (100,100,100), -1)
		if end_point_count == 0:							
			end_point_1 = temp
			end_point_count = end_point_count + 1

		if end_point_count == 1:
			end_point_2 = temp
			end_point_count = end_point_count + 1


	#cv2.line(rgb, (end_point_1[0],end_point_1[1]), (end_point_2[0],end_point_2[1]), (255,0,0), lineThickness)
	
	radius_1 = math.sqrt(((agent_1[0] - end_point_1[0])**2)+((agent_1[1] - end_point_1[1])**2))

	radius_2 = math.sqrt(((agent_1[0] - end_point_2[0])**2)+((agent_1[1] - end_point_2[1])**2))

	radius = max(radius_1, radius_2)

	if radius <= disconnect_distance:
	   #print('E1 close')
	   mid = Cal_mid_point(agent_1, connect_agent)
	   temp = findPerpenticularVector(connection_1)
           temp_line = np.concatenate((mid, mid+temp), axis=None)
	   
	   temp = findIntersection(temp_line, rectangle_1)
	   end_point_count = 0

	   if temp.all() == True:
		#cv2.circle(rgb, (temp[0],temp[1]), 8, (100,100,100), -1)
		end_point_1 = temp
		end_point_count = end_point_count + 1 

	   temp = findIntersection(temp_line, rectangle_2)
	   if temp.all() ==  True:
		#cv2.circle(rgb, (temp[0],temp[1]), 8, (100,100,100), -1)
		if end_point_count == 0:							
			end_point_1 = temp
			end_point_count = end_point_count + 1

		elif end_point_count == 1:
			end_point_2 = temp
			end_point_count = end_point_count + 1

	   temp = findIntersection(temp_line, rectangle_3)
	   if temp.all() == True:
		#cv2.circle(rgb, (temp[0],temp[1]), 8, (100,100,100), -1)
		if end_point_count == 0:							
			end_point_1 = temp
			end_point_count = end_point_count + 1

		elif end_point_count == 1:
			end_point_2 = temp
			end_point_count = end_point_count + 1

	   temp = findIntersection(temp_line, rectangle_4)
	   if temp.all() == True:
		#cv2.circle(rgb, (temp[0],temp[1]), 8, (100,100,100), -1)
		if end_point_count == 0:							
			end_point_1 = temp
			end_point_count = end_point_count + 1

		elif end_point_count == 1:
			end_point_2 = temp
			end_point_count = end_point_count + 1
                        print(end_point_count)

	   cv2.line(rgb, (end_point_1[0],end_point_1[1]), (end_point_2[0],end_point_2[1]), (255,0,0), lineThickness)	   	   
	
	else:
	   print('E1 continue')
	   mid = Cal_mid_point(agent_1, connect_agent)
	   temp = findPerpenticularVector(connection_1)
           temp_line = np.concatenate((mid, mid+temp), axis=None)
	   
	   temp = findIntersection(temp_line, rectangle_1)
	   end_point_count = 0

	   flag = 0
	   if temp.all() == True:
		#cv2.circle(rgb, (temp[0],temp[1]), 8, (100,100,100), -1)
		end_point_1 = temp
		end_point_count = end_point_count + 1
		flag = flag + 1; 

	   temp = findIntersection(temp_line, rectangle_2)
	   if temp.all() == True:
		#cv2.circle(rgb, (temp[0],temp[1]), 8, (100,100,100), -1)
		flag = flag + 1; 
		if end_point_count == 0:							
			end_point_1 = temp
			end_point_count = end_point_count + 1
		elif end_point_count == 1: 
			end_point_2 = temp
			end_point_count = end_point_count + 1

	   temp = findIntersection(temp_line, rectangle_3)
	   if temp.all() > 0:
		#cv2.circle(rgb, (temp[0],temp[1]), 8, (100,100,100), -1)
		flag = flag + 1; 
		if end_point_count == 0:							
			end_point_1 = temp
			end_point_count = end_point_count + 1

		elif end_point_count == 1:
			end_point_2 = temp
			end_point_count = end_point_count + 1

	   temp = findIntersection(temp_line, rectangle_4)
	   if temp.all() == True:
		#cv2.circle(rgb, (temp[0],temp[1]), 8, (100,100,100), -1)
		flag = flag + 1; 
		if end_point_count == 0:							
			end_point_1 = temp
			end_point_count = end_point_count + 1

		elif end_point_count == 1:
			end_point_2 = temp
			end_point_count = end_point_count + 1

	   #print(flag)
	   #cv2.line(rgb, (end_point_1[0],end_point_1[1]), (end_point_2[0],end_point_2[1]), (255,0,0), lineThickness)

	   mid = Cal_mid_point(agent_1, disconnect_agent)
	   temp = findPerpenticularVector(connection_2)
           temp_line = np.concatenate((mid, mid+temp), axis=None)
	   
	   temp = findIntersection(temp_line, rectangle_1)
	   end_point_count = 0
	   #print(temp.all)

	   if temp.all() > 0:
		#cv2.circle(rgb, (temp[0],temp[1]), 8, (100,100,100), -1)
		end_point_3 = temp
		end_point_count = end_point_count + 1 

	   temp = findIntersection(temp_line, rectangle_2)

	   if temp.all() == True:
		#cv2.circle(rgb, (temp[0],temp[1]), 8, (100,100,100), -1)
		if end_point_count == 0:							
			end_point_3 = temp
			end_point_count = end_point_count + 1

		elif end_point_count == 1:
			end_point_4 = temp
			end_point_count = end_point_count + 1

	   temp = findIntersection(temp_line, rectangle_3)

	   if temp.all() == True:
		#cv2.circle(rgb, (temp[0],temp[1]), 8, (100,100,100), -1)
		if end_point_count == 0:							
			end_point_3 = temp
			end_point_count = end_point_count + 1

		elif end_point_count == 1:
			end_point_4 = temp
			end_point_count = end_point_count + 1

	   temp = findIntersection(temp_line, rectangle_4)

	   if temp.all() == True:
		#cv2.circle(rgb, (temp[0],temp[1]), 8, (100,100,100), -1)
		if end_point_count == 0:							
			end_point_3 = temp
			end_point_count = end_point_count + 1

		elif end_point_count == 1:
			end_point_4 = temp
			end_point_count = end_point_count + 1
	
	   cv2.circle(rgb, (end_point_1[0],end_point_1[1]), 8, (255,0,0), -1)
	   cv2.circle(rgb, (end_point_2[0],end_point_2[1]), 8, (255,0,0), -1)
	   cv2.circle(rgb, (end_point_3[0],end_point_3[1]), 8, (0,255,0), -1)
	   cv2.circle(rgb, (end_point_4[0],end_point_4[1]), 8, (0,255,0), -1)
	   #print(end_point_count)
	   #print(end_point_4)
	   #cv2.line(rgb, (end_point_3[0],end_point_3[1]), (end_point_4[0],end_point_4[1]), (255,0,0), lineThickness)
	   
	   Intersection_point = findIntersection1(np.concatenate((end_point_1, end_point_2), axis=None), np.concatenate((end_point_3, end_point_4), axis=None))

	   a_1 = (end_point_1[1] - end_point_2[1])/(end_point_1[0] - end_point_2[0])
	   b_1 = end_point_1[1] - a_1*end_point_1[0]

	   a_2 = (end_point_3[1] - end_point_4[1])/(end_point_3[0] - end_point_4[0])
	   b_2 = end_point_3[1] - a_2*end_point_3[0]

	   if a_1*agent_1[0] + b_1 - agent_1[1] > 0:
		   if a_1*end_point_3[0] + b_1 - end_point_3[1] > 0:
			cv2.line(rgb, (end_point_3[0],end_point_3[1]), (Intersection_point[0],Intersection_point[1]), (255,0,0), lineThickness)

		   if a_1*end_point_4[0] + b_1 - end_point_4[1] > 0:
			cv2.line(rgb, (end_point_4[0],end_point_4[1]), (Intersection_point[0],Intersection_point[1]), (255,0,0), lineThickness)
  	   elif a_1*agent_1[0] + b_1 - agent_1[1] < 0:
		   if a_1*end_point_3[0] + b_1 - end_point_3[1] > 0:
			cv2.line(rgb, (end_point_3[0],end_point_3[1]), (Intersection_point[0],Intersection_point[1]), (255,0,0), lineThickness)

		   if a_1*end_point_4[0] + b_1 - end_point_4[1] > 0:
			cv2.line(rgb, (end_point_4[0],end_point_4[1]), (Intersection_point[0],Intersection_point[1]), (255,0,0), lineThickness)

	   if a_2*agent_1[0] + b_2 - agent_1[1] > 0:
		   if a_2*end_point_1[0] + b_2 - end_point_1[1] > 0:
			cv2.line(rgb, (end_point_1[0],end_point_1[1]), (Intersection_point[0],Intersection_point[1]), (255,0,0), lineThickness)

		   if a_2*end_point_2[0] + b_2 - end_point_2[1] > 0:
			cv2.line(rgb, (end_point_2[0],end_point_2[1]), (Intersection_point[0],Intersection_point[1]), (255,0,0), lineThickness)
	   if a_2*agent_1[0] + b_2 - agent_1[1] < 0:
		   if a_2*end_point_1[0] + b_2 - end_point_1[1] < 0:
			cv2.line(rgb, (end_point_1[0],end_point_1[1]), (Intersection_point[0],Intersection_point[1]), (255,0,0), lineThickness)

		   if a_2*end_point_2[0] + b_2 - end_point_2[1] < 0:
			cv2.line(rgb, (end_point_2[0],end_point_2[1]), (Intersection_point[0],Intersection_point[1]), (255,0,0), lineThickness)


	   #distance_a1_end_1 = math.sqrt(((agent_1[0] - end_point_1[0])**2)+((agent_1[1] - end_point_1[1])**2))
	   #distance_a1_end_2 = math.sqrt(((agent_1[0] - end_point_2[0])**2)+((agent_1[1] - end_point_2[1])**2))
	   #distance_a1_end_3 = math.sqrt(((agent_1[0] - end_point_3[0])**2)+((agent_1[1] - end_point_3[1])**2))
	   #distance_a1_end_4 = math.sqrt(((agent_1[0] - end_point_4[0])**2)+((agent_1[1] - end_point_4[1])**2))

	   #distance = (distance_a1_end_1, distance_a1_end_2, distance_a1_end_3, distance_a1_end_4)
	   #distance = sorted(distance)
	   	
	   #if distance_a1_end_1 == distance[0] or distance_a1_end_1 == distance[1]:
		#cv2.line(rgb, (end_point_1[0],end_point_1[1]), (Intersection_point[0],Intersection_point[1]), (255,0,0), lineThickness)

	   #if distance_a1_end_2 == distance[0] or distance_a1_end_2 == distance[1]:
		#cv2.line(rgb, (end_point_2[0],end_point_2[1]), (Intersection_point[0],Intersection_point[1]), (255,0,0), lineThickness)
	   	     	   
	   #if distance_a1_end_3 == distance[0] or distance_a1_end_3 == distance[1]:
		#cv2.line(rgb, (end_point_3[0],end_point_3[1]), (Intersection_point[0],Intersection_point[1]), (255,0,0), lineThickness)

	   #if distance_a1_end_4 == distance[0] or distance_a1_end_4 == distance[1]:
		#cv2.line(rgb, (end_point_4[0],end_point_4[1]), (Intersection_point[0],Intersection_point[1]), (255,0,0), lineThickness)

	polygon = Polygon([(0, 0), (1, 1), (1, 0)])
        polygon.area

        #cmd_msg = Float32MultiArray();
        #cmd-msg.data = [ ]

        rgb_addline = bridge.cv2_to_imgmsg(rgb, 'rgb8')
        rgb_addline.header = rgb_data.header
        dynamic_painting_pub.publish(rgb_addline)


rgb_sub = rospy.Subscriber('/image_raw', Image, callback, queue_size=1)

while not rospy.is_shutdown():
    rospy.spin()



























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
from tip.msg import Vector
# from rospy_tutorials.msg import Floats

# some message type
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, CameraInfo
from platform import python_version

import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull
from voronoi import voronoi 

vdatax = np.arange(1,8)
vdatay = np.arange(1,8)

a1_x = 0
a1_y = 0
a2_x = 0
a2_y = 0
a3_x = 0
a3_y = 0
a4_x = 0
a4_y = 0

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

#def callback_1(data):
#	rospy.loginfo(rospy.get_caller_id() + "I heard %f", data.data)
	

def callback(rgb_data):
#	with TimeIt('rendering'):
#		rgb = bridge.imgmsg_to_cv2(rgb_data, 'rgb8')
	rgb = np.full([564, 804, 3], 225, dtype=np.uint8)

##############################################coverage area##################################################
	rectangle_1 = np.array([4.0,  4.0, 800.0, 4.0])

	rectangle_2 = np.array([800.0, 4.0, 800.0, 560.0])

	rectangle_3 = np.array([4.0, 560.0, 800.0, 560.0])
 
	rectangle_4 = np.array([4.0, 4.0, 4.0, 560.0])  	

	lineThickness = 2

	cv2.rectangle(rgb, (4, 4), (800, 560), (255,0,0), lineThickness) 

	pins = 200*np.random.rand(3,2) + 30
	#pins = np.array([[a1_x,a1_y], [a2_x,a2_y], [a3_x,a3_y], [a4_x,a4_y]])
	print(pins)
		# boundary points in R^2
	#bnd = np.random.rand(30,2)
	bnd = np.array([[4,4], [4,560], [800,560], [800, 4]])

	# create the polytope bounded voronoi diagram
	pnts,vorn = voronoi(pins,bnd)
	#print(pnts)
	#print(vorn)

	# plot the result...
	pntsv = [ [] for row in pnts]
			#plt.plot(pnts[:,0],pnts[:,1],'o')

	for i, obj in enumerate(pnts):
		pntsv[i] =  np.array(vorn[i])
		if len(pntsv[i]) >= 3:
			vorhull = ConvexHull(pntsv[i])		
				#plt.plot(pntsv[i][:,0],pntsv[i][:,1],'x')
			for simplex in vorhull.simplices:
				#plt.plot(pntsv[i][simplex, 0],pntsv[i][simplex,1],'k-')
				temp_1 = pntsv[i][simplex, 0]
				temp_2 = pntsv[i][simplex, 1]
				#print(temp_1)
				#print(temp_2)
				#print('--')
				temp_x_1  = temp_1.item(0)
				temp_x_2  = temp_1.item(1)
				temp_y_1  = temp_2.item(0)
				temp_y_2  = temp_2.item(1)
				#print(temp_x_1)
				#print(temp_y_1)
				#print(temp_x_2)
				#print(temp_y_2)
				cv2.line(rgb, (int(temp_x_1), int(temp_y_1)), (int(temp_x_2), int(temp_y_2)), (255,0,0), lineThickness)
				#cv2.line(rgb, (10, 10), (2000, 2000), (255,0,0), lineThickness)	
				rgb_addline = bridge.cv2_to_imgmsg(rgb, 'rgb8')
				rgb_addline.header = rgb_data.header
				dynamic_painting_pub.publish(rgb_addline)
		#plt.axis('equal')
		#plt.show()

'''

	polyx = vdatax[vdatax!=0]/7
	polyy = vdatay[vdatay!=0]/7

	for i, obj in enumerate(polyx[0:-1]):
		cv2.line(rgb, (int(polyx[i]), int(polyy[i])), (int(polyx[i+1]), int(polyy[i+1])), (255,0,0), lineThickness)

		
	print "Can you hear me?"
'''
'''
def callback_3(data):
	a1_x = data.x;
	a1_y = data.y;

def callback_5(data):
	a1_x = data.x;
	a1_y = data.y;

def callback_6(data):
	a1_x = data.x;
	a1_y = data.y;

def callback_7(data):
	a1_x = data.x;
	a1_y = data.y;
'''


'''

def callback_v(data):
#	rgb = bridge.imgmsg_to_cv2(rgb_data, 'rgb8')
	vdatax[0] = data.x1
	vdatax[1] = data.x2
	vdatax[2] = data.x3
	vdatax[3] = data.x4
	vdatax[4] = data.x5
	vdatax[5] = data.x6
	vdatax[6] = data.x1

	vdatay[0] = data.y1
	vdatay[1] = data.y2
	vdatay[2] = data.y3
	vdatay[3] = data.y4
	vdatay[4] = data.y5
	vdatay[5] = data.y6
	vdatay[6] = data.y1
'''    

if __name__ == '__main__':

	rgb_sub = rospy.Subscriber('/image_raw', Image, callback, queue_size=1)
	#sub_1 = rospy.Subscriber('/wu/voronoi_vertices', callback_1)
#	voi = rospy.Subscriber('wu/voronoi', Vector, callback_v)
	'''
	vt_3 = rospy.Subscriber('/san/vt_ctr', geometry_msgs::Vector3, callback_3)
	vt_5 = rospy.Subscriber('/wu/vt_ctr', geometry_msgs::Vector3, callback_5)
	vt_6 = rospy.Subscriber('/liu/vt_ctr', geometry_msgs::Vector3, callback_6)
	vt_7 = rospy.Subscriber('/qi/vt_ctr', geometry_msgs::Vector3, callback_7)
	'''
	while not rospy.is_shutdown():
		rospy.spin()

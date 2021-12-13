#!/usr/bin/python
import numpy as np
import random, itertools, collections
import rospy
import math


from scipy.spatial import Delaunay
from scipy.spatial import ConvexHull

from interLine import interLine
from perpBisector2d import perpBisector2d
from inHull import inHull

def getConvexBndMatrix(bnd):
	#pnts = inHull(pins,bnd)
	#print np.shape(pout)[0]
	# linear equations for the boundary
	bndhull = ConvexHull(bnd)
	bndTmp = bndhull.equations
	bndMat = np.matrix(bndTmp)
	Abnd = bndMat[:,0:2]
	bbnd = bndMat[:,2]
	return [Abnd, bbnd]
	#rospy.loginfo([pins, pnts, Abnd, bbnd])

def voronoi(pins,Abnd, bbnd):
	pnts = pins
	# Delaunay triangulation
	tri = Delaunay(pnts)
	# find voronoi neighbors for each generater point
	neib = [ [] for row in pnts]
	for j, obj in enumerate(pnts):
		neib[j]=[]
		i = 0
		for row in tri.simplices:
		    i = i + 1
		    tmp = np.intersect1d(tri.simplices[i-1],[j])
		    if tmp.size !=0:
		        neib[j].append(np.setdiff1d(tri.simplices[i-1],j))
		neib[j] = np.unique(neib[j])

	# find linear equations for perpendicular bisectors

	mylistA = []
	mylistb = []

	for i, obj in enumerate(pnts):
		A = np.array([[0, 0]])
		b = np.array([0])
		for j in range(0,len(neib[i])):
		    Altmp, blt = perpBisector2d(pnts[i],pnts[neib[i][j]])
		    Al = np.array([Altmp])
		    bl = np.array([blt])
		    A = np.concatenate((A,Al),axis=0)
		    b = np.concatenate((b,bl),axis=0)
		Amat = np.matrix(A)
		bmat = np.matrix(b)
		mylistA.append(Amat[1:np.shape(Amat)[0]])
		mylistb.append(bmat[:,1:np.shape(bmat)[1]])

	# obtain voronoi vertices
	vertexes = [ [] for row in pnts]
	for j in range(0,len(mylistA)):
		Atmp = np.concatenate((mylistA[j],Abnd))
		btmp = np.concatenate((mylistb[j].transpose(),-bbnd))
		k =0
		for comb in list(itertools.combinations(range(1,np.shape(Atmp)[0]+1),2)):
			k = k+1
			if k <= len(list(itertools.combinations(range(1,np.shape(Atmp)[0]+1),2))):
				lineA = [Atmp[comb[0]-1].item(0,0),Atmp[comb[0]-1].item(0,1),btmp[comb[0]-1].item(0,0)]
				lineB = [Atmp[comb[1]-1].item(0,0),Atmp[comb[1]-1].item(0,1),btmp[comb[1]-1].item(0,0)]
				output = interLine(lineA,lineB)
				if type(output) != type(False):
					if (np.round(np.dot(Atmp,np.array([output.item(0,0),output.item(1,0)])),7)<=np.round(btmp.transpose(),7)).all():
						vertexes[j].append([output.item(0,0),output.item(1,0)])

	#rospy.loginfo(vertexes)
	centroid = []
	partitionMass = []
	for i in range(0,len(pnts)):
		x = np.array([])
		y = np.array([])
		thisVertex = np.array(vertexes[i])
		for j in range(0, len(thisVertex)):
			x = np.append(x, thisVertex[j][0])
			y = np.append(y, thisVertex[j][1])
		[Cx, Cy, miV] = VoronoiCenterMass(x, y)
		centroid.append([Cx, Cy])
		partitionMass.append(miV)

	return pnts, vertexes, centroid, partitionMass

def VoronoiCenterMass(xRaw, yRaw):
	x, y = sortPolygonVertexesClockwise(xRaw, yRaw)
	n = len(x)
	xShift = np.append(x[1:], x[0])
	yShift = np.append(y[1:], y[0])
	#rospy.loginfo([x, xShift, y, yShift])
	tmpSumX = 0
	tmpSumY = 0
	tmpSumA = 0
	for i in range(0,n):
		tmpSumX += np.double((x[i] + xShift[i]) * (x[i]*yShift[i] - xShift[i]*y[i]))
		tmpSumY += np.double((y[i] + yShift[i]) * (x[i]*yShift[i] - xShift[i]*y[i]))
		tmpSumA += np.double(x[i]*yShift[i] - xShift[i]*y[i])
	A = np.double(tmpSumA)/2 
	Cx = tmpSumX / A / 6
	Cy = tmpSumY / A / 6

	#rospy.loginfo([x, y])
	#rospy.loginfo("N: %d A: %f SX: %f SY: %f Cx: %f Cy: %f", n, tmpSumA, tmpSumX, tmpSumY, Cx, Cy)

	return [Cx, Cy, A]
	

def sortPolygonVertexesClockwise(x, y):
	PI = 3.14159265359
	# Compute the center of the polygon
	centerPnt = [sum(x)/len(x), sum(y)/len(y)]

	pntList =[]
	for i in range(0, len(x)):
		pntList.append([x[i], y[i]])

	angleList = []
	for pnt in pntList:
		dx = pnt[0] - centerPnt[0]
		dy = pnt[1] - centerPnt[1]

		angle = math.atan2(dy,dx)
		if(angle < 0):
			angle += 2 * PI
		angleList.append(angle);	

	sortedVertexes = [x for _, x in sorted(zip(angleList, pntList))]
	sortedX = []
	sortedY = []
	for vertexes in sortedVertexes:
		sortedX.append(vertexes[0])
		sortedY.append(vertexes[1])

	#rospy.loginfo([pntList, angleList, sortedVertexes])
	return sortedX, sortedY


def getAdjacentList(Vertices, centroidArr):
	nTotal = len(centroidArr)
	adjMat = np.zeros((nTotal, nTotal))
	vArr = [[None] * nTotal for i in range(nTotal)]
	tmpCentroidArr = centroidArr
	for thisAgent in range(nTotal):			# This agent
		for nextAgent in range(thisAgent + 1, nTotal):		# Aother agent
			
			commonVertexes = []
			isNeighBor = False
			cnt = 0
			# Set all of the current Agent
			for thisVertex in Vertices[thisAgent]:
				# Compare with all vertexes of the next agent
				for comparedVertex in Vertices[nextAgent]:
					# Some tolerant in numeical error
					if(abs(thisVertex[0]-comparedVertex[0])<0.05 and abs(thisVertex[1]-comparedVertex[1])<0.05):
						cnt += 1
						commonVertexes.append(comparedVertex)
			
			if(cnt == 2): # Perfect match
				isNeighBor = True
				adjMat[thisAgent][nextAgent] = 1
				adjMat[nextAgent][thisAgent] = 1
				vArr[thisAgent][nextAgent] = (commonVertexes)
				vArr[nextAgent][thisAgent] = (commonVertexes)
			elif (cnt == 0):
				pass
			else:
				rospy.logwarn("adjacentList Fault: id: %d vs %d: %d", thisAgent, nextAgent, cnt)
				rospy.logwarn(Vertices)
				
	return adjMat, vArr


#include "ros/ros.h"
#include "qualisys/Marker.h"
#include "qualisys/Subject.h"
#include <geometry_msgs/Vector3.h>
#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <math.h>
#include "stdio.h"
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/socket.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <time.h>



using std::string;
using namespace std;

#define PORT     50001 
#define MAXLINE 80
#define SERV_PORT 8888



//*******************************************************************Global variables******************************************************************
struct sockaddr_in servaddr,cliaddr;
int sockfd; 
char buffer[MAXLINE];
sockaddr_storage addrDest = {};

geometry_msgs::Vector3 n1_ctr;
geometry_msgs::Vector3 n2_ctr;
geometry_msgs::Vector3 vt_ctr;
geometry_msgs::Vector3 centroid;
double temp_x, temp_y;
double v_x, v_y, v; 
int k;

int x_vt_ctr;
int y_vt_ctr;

typedef struct Point
{
    double x;
    double y;
}Point;


//若点a大于点b,即点a在点b顺时针方向,返回true,否则返回false
bool PointCmp(const Point &a,const Point &b,const Point &center)
{
    if (a.x >= 0 && b.x < 0)
        return true;
    if (a.x == 0 && b.x == 0)
        return a.y > b.y;
    //向量OA和向量OB的叉积
    int det = (a.x - center.x) * (b.y - center.y) - (b.x - center.x) * (a.y - center.y);
    if (det < 0)
        return true;
    if (det > 0)
        return false;
    //向量OA和向量OB共线，以距离判断大小
    int d1 = (a.x - center.x) * (a.x - center.x) + (a.y - center.y) * (a.y - center.y);
    int d2 = (b.x - center.x) * (b.x - center.y) + (b.y - center.y) * (b.y - center.y);
    return d1 > d2;
}

void ClockwiseSortPoints(double Points[], int nPoints)
{
    //计算重心
    Point center;
    Point temp_1;
    Point temp_2;
    Point tmp;

    double x = 0,y = 0;

    for (int i = 0; i < nPoints; i++)
    {
        x += Points[2*i];
        y += Points[2*i+1];
    }

    center.x = x/nPoints;
    center.y = y/nPoints;

    //冒泡排序
    for(int i = 0; i < nPoints - 1; i++)
    {
        for (int j = 0; j < nPoints-i-1; j++)
        {

            temp_1.x = Points[2*j];
            temp_1.y = Points[2*j+1];
            temp_2.x = Points[2*(j+1)];
            temp_2.y = Points[2*(j+1)+1];

	    if (PointCmp(temp_1, temp_2, center))
            {
                tmp = temp_1;
                Points[2*j] = Points[2*(j+1)];
		Points[2*j+1] = Points[2*(j+1)+1];
                Points[2*(j+1)] = tmp.x;
                Points[2*(j+1)+1] = tmp.y;
            }
        }
    }
}

double *Cal_mid_point(double a1[], double a2[])
{
    static double mid_point[2];
    mid_point[0] = (a1[0] + a2[0])/2;
    mid_point[1] = (a1[1] + a2[1])/2;
    return mid_point;
}    

double *findIntersection(double line1[], double line2[], int x_limit_min, int x_limit_max, int y_limit_min, int y_limit_max)
{

    double x1,x2,y1,y2,x3,y3,x4,y4,denominator,xNominator,yNominator,px,py;
    static double intersection[2];
	
    x1 = line1[0];
    y1 = line1[1];
    x2 = line1[2];
    y2 = line1[3];

    x3 = line2[0];
    y3 = line2[1];
    x4 = line2[2];
    y4 = line2[3];

    denominator = (x1 - x2)*(y3 - y4) - (y1 - y2)*(x3 - x4);

    if (denominator == 0)
    {
	intersection[0] = 0;
        intersection[1] = 0;
	//printf("denominator = 0");
        return intersection;		
    }    

    xNominator = (x1*y2 - y1*x2)*(x3 - x4) - (x1 - x2)*(x3*y4 - y3*x4);
    yNominator = (x1*y2 - y1*x2)*(y3 - y4) - (y1 - y2)*(x3*y4 - y3*x4);

    px = xNominator/denominator;
    py = yNominator/denominator;

    //printf("px = %f, py = %f\n", px, py);


    x_limit_min = x_limit_min - 1;
    x_limit_max = x_limit_max + 1;
    y_limit_min = y_limit_min - 1;
    y_limit_max = y_limit_max + 1;

    //printf("x_limit_min = %d\n", x_limit_min);
    //printf("x_limit_max = %d\n", x_limit_max);
    //printf("y_limit_min = %d\n", y_limit_min);
    //printf("y_limit_max = %d\n", y_limit_min);

    //if (px >= x3 && px <= x4 && py >= y3 && py <= y4)
    if ((px > x_limit_min) && (px <= x_limit_max) && (py >= y_limit_min) && (py <= y_limit_max))
    {
	intersection[0] = px;
        intersection[1] = py;
	//printf("px = %f, py = %f\n", intersection[0], intersection[1]);
   	return intersection;
    }
    else
    { 	
	intersection[0] = 0;
        intersection[1] = 0;
	//printf("px = %f, py = %f\n", intersection[0], intersection[1]);
	return intersection;
    }

}


double *findIntersection1(double line1[], double line2[], double x_limit_min, double x_limit_max, double y_limit_min, double y_limit_max)
{
    double x1,x2,y1,y2,x3,y3,x4,y4,denominator,xNominator,yNominator,px,py;
    static double intersection[2];
    double shang;
    x1 = line1[0];
    y1 = line1[1];
    x2 = line1[2];
    y2 = line1[3];

    x3 = line2[0];
    y3 = line2[1];
    x4 = line2[2];
    y4 = line2[3];

    denominator = (x1 - x2)*(y3 - y4) - (y1 - y2)*(x3 - x4);

    if (denominator != 0)
    {
	 xNominator = (x1*y2 - y1*x2)*(x3 - x4) - (x1 - x2)*(x3*y4 - y3*x4);
         yNominator = (x1*y2 - y1*x2)*(y3 - y4) - (y1 - y2)*(x3*y4 - y3*x4);
         px = xNominator / denominator;
         py = yNominator / denominator;

	 if ((px >= x_limit_min) && (px <= x_limit_max) && (py > y_limit_min) && (py < y_limit_max))
	 {
	 	intersection[0] = px;
         	intersection[1] = py;

         	return intersection;
	 }
	 else
	 {
		intersection[0] = 0;
        	intersection[1] = 0;
        	return intersection;			
	}		
    }
    else
    {
	intersection[0] = 0;
        intersection[1] = 0;
	//printf("denominator = 0");
        return intersection;		
    }	
}

   

double *findPerpenticularVector(double line1[])
{
	double x1,x2,y1,y2;
    	static double pVector[2];

    	//srand( (unsigned)time( NULL ) );
    
    	x1 = line1[0];
    	y1 = line1[1];
    	x2 = line1[2];
    	y2 = line1[3];
	

    	pVector[0] = y1 - y2;
    	pVector[1] = x2 - x1;

    	return pVector;

}


double *computeCentroid(double Vertices_X[], double Vertices_Y[], int nVertices)
{

        static double Centroid_P[2];
	double centroidX = 0, centroidY = 0;
	double det = 0, tempDet = 0;
	unsigned int j = 0;
	

	for (unsigned int i = 0; i < nVertices; i++)
	{
		// closed polygon
		if (i + 1 == nVertices)
			j = 0;
		else
			j = i + 1;
		// compute the determinant
		tempDet = Vertices_X[i] * Vertices_Y[j] - Vertices_X[j]*Vertices_Y[i];
		//ROS_INFO("Vertices_X[%d] = %f",i,Vertices_X[i]);
		//ROS_INFO("Vertices_Y[%d] = %f",j,Vertices_Y[j]);
		//ROS_INFO("Vertices_X[%d] = %f",j,Vertices_X[j]);
		//ROS_INFO("Vertices_Y[%d] = %f",i,Vertices_Y[i]);
		//ROS_INFO("tempDet[%d] = %f",i,tempDet);
		det += tempDet;

		centroidX += (Vertices_X[i] + Vertices_X[j])*tempDet;
		centroidY += (Vertices_Y[i] + Vertices_Y[j])*tempDet;
	}

	// divide by the total mass of the polygon
	centroidX /= 3*det;
	centroidY /= 3*det;

	Centroid_P[0] = centroidX;
	Centroid_P[1] = centroidY;
        //ROS_INFO("Give you: %f\n", Centroid_P[0]);
	//ROS_INFO("centroid = %f, %f", Centroid_P[0], Centroid_P[1]);	
        
	return Centroid_P;
}


int sgn(double v) 
{

  if (v < 0) return -1;
  if (v > 0) return 1;
  return 0;

}

void N1call(const geometry_msgs::Vector3& n1)
{
    n1_ctr.x = n1.x;
    n1_ctr.y = n1.y;
}

void N2call(const geometry_msgs::Vector3& n2)
{
    n2_ctr.x = n2.x;
    n2_ctr.y = n2.y;
}

void Translate(const qualisys::Subject& center)
{

/*************************************************************************compute virtual center***************************************************************************/
    double alpha;
 	   
    alpha = 2*atan(center.orientation.z/center.orientation.w);

    if (alpha <0)
    { 
        alpha += 6.2831852;
    }


    double z_x, z_y, v_0, omega_0;

    z_x = center.position.x*1000 - 162/0.5416*sin(alpha+1.5707963);
    z_y = center.position.y*1000 + 162/0.5416*cos(alpha+1.5707963);
    v_0 = 16;
    omega_0 = 0.5416;

    //printf("Virtual center x.%f,Virtual center y.%f\n", z_x, z_y);

    vt_ctr.x = z_x;
    vt_ctr.y = z_y;


/********************************************************************compute voronoi area and its center*******************************************************************/



/****************************************************************************coverage area*********************************************************************************/
    		
    double rectangle_1[4] = {20,20,4000,20};   
    double rectangle_2[4] = {4000,20,4000,2800}; 
    double rectangle_3[4] = {20,2800,4000,2800}; 
    double rectangle_4[4] = {20,20,20,2800};
  
/******************************************************************************agents' states******************************************************************************/
    	
    double agent_1[2] = {z_x, z_y};
    double agent_2[2] = {n1_ctr.x,n1_ctr.y};
    double agent_3[2] = {n2_ctr.x,n2_ctr.y};
    	
    //double agent_3[2] = {3000,2000};
/********************************************************************************Compute V1********************************************************************************/

    double distance12, distance13;

    distance12 = sqrt((agent_1[0] - agent_2[0])*(agent_1[0] - agent_2[0]) + (agent_1[1] - agent_2[1])*(agent_1[1] - agent_2[1]));
    distance13 = sqrt((agent_1[0] - agent_3[0])*(agent_1[0] - agent_3[0]) + (agent_1[1] - agent_3[1])*(agent_1[1] - agent_3[1]));

    double connect_agent[2], disconnect_agent[2];
    double connection_1[4], connection_2[4];
    double disconnect_distance;

    if (distance12 <= distance13)
    {
	connection_1[0] = agent_1[0];     
	connection_1[1] = agent_1[1]; 
        connection_1[2] = agent_2[0];
        connection_1[3] = agent_2[1];  
        
        connection_2[0] = agent_1[0];     
	connection_2[1] = agent_1[1]; 
        connection_2[2] = agent_3[0];
        connection_2[3] = agent_3[1];	
        
	connect_agent[0] = agent_2[0];
	connect_agent[1] = agent_2[1];

	disconnect_agent[0] = agent_3[0];
	disconnect_agent[1] = agent_3[1];

	disconnect_distance = distance13; 
    }
    else
    {		
	connection_1[0] = agent_1[0]; 
	connection_1[1] = agent_1[1]; 
        connection_1[2] = agent_3[0];
        connection_1[3] = agent_3[1];

        connection_2[0] = agent_1[0];     
	connection_2[1] = agent_1[1]; 
        connection_2[2] = agent_2[0];
        connection_2[3] = agent_2[1];

	connect_agent[0] = agent_3[0];
        connect_agent[1] = agent_3[1];
   
	disconnect_agent[0] = agent_2[0];
	disconnect_agent[1] = agent_2[1];

	disconnect_distance = distance12;  	
    }


    double *temp;
	
    temp = findPerpenticularVector(connection_1);

    double temp_line[4];

    temp_line[0] = connect_agent[0]; //np.concatenate((connect_agent, connect_agent+temp), axis=None);
    temp_line[1] = connect_agent[1];
    temp_line[2] = connect_agent[0] + temp[0];
    temp_line[3] = connect_agent[1] + temp[1];

    //int j = 0;
    //for(j = 0; j < 2; j += 1)
    //   printf( "%f\n", temp[j]);
 
    temp = findIntersection(temp_line, rectangle_1, 20, 4000, 20, 2800);
    
    int end_point_count;
    end_point_count = 0;

    double end_point_1[2], end_point_2[2], end_point_3[2], end_point_4[2];

    if ((temp[0] != 0) && (temp[1] != 0))
    {
	end_point_1[0] = temp[0];
	end_point_1[1] = temp[1];
	end_point_count = end_point_count + 1; 
    }
	
    temp = findIntersection(temp_line, rectangle_2, 20, 4000, 20, 2800);

    if ((temp[0] != 0) && (temp[1] != 0))
    {		
	if (end_point_count == 0)
	{							
	    end_point_1[0] = temp[0];
	    end_point_1[1] = temp[1];
	    end_point_count = end_point_count + 1;
	}

	if (end_point_count == 1)
	{
	    end_point_2[0] = temp[0];
	    end_point_2[1] = temp[1];
	    end_point_count = end_point_count + 1;
 	}   
    }
	
    temp = findIntersection(temp_line, rectangle_3, 20, 4000, 20, 2800);

    if ((temp[0] != 0) && (temp[1] != 0))
    {
	if (end_point_count == 0)
	{							
	    end_point_1[0] = temp[0];
	    end_point_1[1] = temp[1];
	    end_point_count = end_point_count + 1;
	}
		
	if (end_point_count == 1)
	{
	    end_point_2[0] = temp[0];
	    end_point_2[1] = temp[1];
	    end_point_count = end_point_count + 1;
	}    
    }

    temp = findIntersection(temp_line, rectangle_4, 20, 4000, 20, 2800);

    if ((temp[0] != 0) && (temp[1] != 0))
    {
	if (end_point_count == 0)							
	{   
	   end_point_1[0] = temp[0];
	   end_point_1[1] = temp[1];
	   end_point_count = end_point_count + 1;
	}
		
	if (end_point_count == 1)
	{		
	   end_point_2[0] = temp[0];
	   end_point_2[1] = temp[1];
	   end_point_count = end_point_count + 1;
	}
    }

    //printf("End point count1 = %d\n", end_point_count);


    double radius_1, radius_2;	
	
    radius_1 = sqrt((agent_1[0] - end_point_1[0])*(agent_1[0] - end_point_1[0]) + (agent_1[1] - end_point_1[1])*(agent_1[1] - end_point_1[1]));
    radius_2 = sqrt((agent_1[0] - end_point_2[0])*(agent_1[0] - end_point_2[0]) + (agent_1[1] - end_point_2[1])*(agent_1[1] - end_point_2[1]));

    double radius;

    if(radius_1 >= radius_2)    
	radius = radius_1;
    else
	radius = radius_2;

    double *mid;	

    if (radius <= disconnect_distance)
    {   
	mid = Cal_mid_point(agent_1, connect_agent);
	temp = findPerpenticularVector(connection_1);
    	
	temp_line[0] = mid[0]; 
	temp_line[1] = mid[1];
        temp_line[2] = mid[0] + temp[0];
        temp_line[3] = mid[1] + temp[1];
	   
	temp = findIntersection(temp_line, rectangle_1, 20, 4000, 20, 2800);
	end_point_count = 0;

	if ((temp[0] != 0) && (temp[1] != 0))
	{
	    end_point_1[0] = temp[0];
	    end_point_1[1] = temp[1];
	    end_point_count = end_point_count + 1; 
	}
	   
	temp = findIntersection(temp_line, rectangle_2, 20, 4000, 20, 2800);
	   
	if ((temp[0] != 0) && (temp[1] != 0))
	{
	    if (end_point_count == 0)							
	    {   
	    	end_point_1[0] = temp[0];
	    	end_point_1[1] = temp[1];
		end_point_count = end_point_count + 1;
	    }
	    else if (end_point_count == 1)
	    {
	    	end_point_2[0] = temp[0];
	    	end_point_2[1] = temp[1];
		end_point_count = end_point_count + 1;
	    }
	}

	temp = findIntersection(temp_line, rectangle_3, 20, 4000, 20, 2800);

        if ((temp[0] != 0) && (temp[1] != 0))
	{
	    if (end_point_count == 0)							
	    {
	       end_point_1[0] = temp[0];
	       end_point_1[1] = temp[1];
	       end_point_count = end_point_count + 1;
	    }
	    else if (end_point_count == 1)
	    {
	       end_point_2[0] = temp[0];
	       end_point_2[1] = temp[1];
	       end_point_count = end_point_count + 1;
	    }	
	}
	   
	temp = findIntersection(temp_line, rectangle_4, 20, 4000, 20, 2800);
	   
	if ((temp[0] != 0) && (temp[1] != 0))
	{
		if (end_point_count == 0)
		{							
		   end_point_1[0] = temp[0];
		   end_point_1[1] = temp[1];
		   end_point_count = end_point_count + 1;
		}
		else if (end_point_count == 1)
		{
		   end_point_2[0] = temp[0];
		   end_point_2[1] = temp[1];
		   end_point_count = end_point_count + 1;
		}
	}
	//printf("End point count2 = %d\n", end_point_count);

     }	
     else
     {	   
	   mid = Cal_mid_point(agent_1, connect_agent);
	   temp = findPerpenticularVector(connection_1);
	   
	   temp_line[0] = mid[0]; 
	   temp_line[1] = mid[1];
           temp_line[2] = mid[0] + temp[0];
           temp_line[3] = mid[1] + temp[1];

	   temp = findIntersection(temp_line, rectangle_1, 20, 4000, 20, 2800);
	   end_point_count = 0;

	   int flag;
	   flag = 0;

	   if ((temp[0] != 0) && (temp[1] != 0))
	   {
		end_point_1[0] = temp[0];
		end_point_1[1] = temp[1];
		end_point_count = end_point_count + 1;
		flag = flag + 1; 
	   }

	   temp = findIntersection(temp_line, rectangle_2, 20, 4000, 20, 2800);

	   if ((temp[0] != 0) && (temp[1] != 0))
	   {	
		flag = flag + 1; 
		if (end_point_count == 0)
		{							
		    end_point_1[0] = temp[0];
		    end_point_1[1] = temp[1];
		    end_point_count = end_point_count + 1;
		}		
		else if (end_point_count == 1)
		{ 
		    end_point_2[0] = temp[0];
		    end_point_2[1] = temp[1];		
		    end_point_count = end_point_count + 1;
		}
	   }

	   temp = findIntersection(temp_line, rectangle_3, 20, 4000, 20, 2800);

	   if ((temp[0] != 0) && (temp[1] != 0))
	   {
		flag = flag + 1; 
		if (end_point_count == 0)
		{							
		    end_point_1[0] = temp[0];
		    end_point_1[1] = temp[1];
		    end_point_count = end_point_count + 1;
		}
		else if (end_point_count == 1)
		{
		    end_point_2[0] = temp[0];
		    end_point_2[1] = temp[1];
		    end_point_count = end_point_count + 1;
		}
	   }

	   temp = findIntersection(temp_line, rectangle_4, 20, 4000, 20, 2800);

	   if ((temp[0] != 0) && (temp[1] != 0))
	   {
		flag = flag + 1; 
		if (end_point_count == 0)
		{							
		    end_point_1[0] = temp[0];
		    end_point_1[1] = temp[1];		
		    end_point_count = end_point_count + 1;
		}
		else if (end_point_count == 1)
		{
		    end_point_2[0] = temp[0];
		    end_point_2[1] = temp[1];
		    end_point_count = end_point_count + 1;
		}
	  }
	  //printf("End point count3 = %d\n", end_point_count);
    }


    //printf( "end_point_1:%f,%f\n", end_point_1[0], end_point_1[1]);
    //printf( "end_point_2:%f,%f\n", end_point_2[0], end_point_2[1]);
	   

    mid = Cal_mid_point(agent_1, disconnect_agent);
    temp = findPerpenticularVector(connection_2);
           
    temp_line[0] = mid[0]; 
    temp_line[1] = mid[1];
    temp_line[2] = mid[0] + temp[0];
    temp_line[3] = mid[1] + temp[1];
	   
    temp = findIntersection(temp_line, rectangle_1, 20, 4000, 20, 2800);
    end_point_count = 0;

    if ((temp[0] != 0) && (temp[1] != 0))
    {
	end_point_3[0] = temp[0];
	end_point_3[1] = temp[1];
	end_point_count = end_point_count + 1; 
    }
	   
    temp = findIntersection(temp_line, rectangle_2, 20, 4000, 20, 2800);

    if ((temp[0] != 0) && (temp[1] != 0))
    {

	if (end_point_count == 0)							
	{	   
	   end_point_3[0] = temp[0];
	   end_point_3[1] = temp[1];
	   end_point_count = end_point_count + 1;
	}
	else if (end_point_count == 1)
	{
	   end_point_4[0] = temp[0];
	   end_point_4[1] = temp[1];
	   end_point_count = end_point_count + 1;
	}
    }

    temp = findIntersection(temp_line, rectangle_3, 20, 4000, 20, 2800);

    if ((temp[0] != 0) && (temp[1] != 0))
    {
       if (end_point_count == 0)							
       {				
           end_point_3[0] = temp[0];
           end_point_3[1] = temp[1];
	   end_point_count = end_point_count + 1;
       }
       else if (end_point_count == 1)
       {
	   end_point_4[0] = temp[0];
	   end_point_4[1] = temp[1];
	   end_point_count = end_point_count + 1;
       }
    }	   
       
    temp = findIntersection(temp_line, rectangle_4, 20, 4000, 20, 2800);

    if ((temp[0] != 0) && (temp[1] != 0))
    {	
	if (end_point_count == 0)
	{							
	    end_point_3[0] = temp[0];
	    end_point_3[1] = temp[1];
	    end_point_count = end_point_count + 1;
	}
	else if (end_point_count == 1)
	{
	    end_point_4[0] = temp[0];
	    end_point_4[1] = temp[1];
	    end_point_count = end_point_count + 1;
	}
    }

    //printf( "End point count4 = %d\n", end_point_count);	
    //printf( "End_point_1 = %f,%f\n", end_point_1[0], end_point_1[1]);
    //printf( "End_point_2 = %f,%f\n", end_point_2[0], end_point_2[1]);
    //printf( "End_point_3 = %f,%f\n", end_point_3[0], end_point_3[1]);
    //printf( "End_point_4 = %f,%f\n", end_point_4[0], end_point_4[1]);


    double *Intersection_point;
    double temp_line_1[4], temp_line_2[4];
	
    temp_line_1[0] = end_point_1[0];
    temp_line_1[1] = end_point_1[1];
    temp_line_1[2] = end_point_2[0];
    temp_line_1[3] = end_point_2[1];

    temp_line_2[0] = end_point_3[0];
    temp_line_2[1] = end_point_3[1];
    temp_line_2[2] = end_point_4[0];
    temp_line_2[3] = end_point_4[1];

    //ROS_INFO("1_0:%f", end_point_1[1]);	
    //ROS_INFO("2_0:%f", end_point_2[1]);	   

    Intersection_point = findIntersection1(temp_line_1, temp_line_2, 20, 4000, 20, 2800);

    int area_counter;
    area_counter = 0;
    double coverage_area_1[20] = {0};

    //ROS_INFO( "inter_point:%f,%f\n", Intersection_point[0], Intersection_point[1]);

    double a_1,b_1,a_2,b_2;

    double vertex_1[2] = {20,  20};   
    double vertex_2[2] = {4000, 20}; 
    double vertex_3[2] = {4000, 2800}; 
    double vertex_4[2] = {20,  2800};   

    a_1 = (end_point_1[1] - end_point_2[1])/(end_point_1[0] - end_point_2[0]);
    b_1 = end_point_1[1] - a_1*end_point_1[0];

    a_2 = (end_point_3[1] - end_point_4[1])/(end_point_3[0] - end_point_4[0]);
    b_2 = end_point_3[1] - a_2*end_point_3[0];


    if ((sgn(a_1*vertex_1[0] + b_1 - vertex_1[1]) == sgn(a_1*agent_1[0] + b_1 - agent_1[1])) && (sgn(a_2*vertex_1[0] + b_2 - vertex_1[1]) == sgn(a_2*agent_1[0] + b_2 - agent_1[1])))
    {
	coverage_area_1[2*area_counter] = vertex_1[0];
	coverage_area_1[2*area_counter + 1] = vertex_1[1];
	area_counter = area_counter + 1;
    }
 
    if ((sgn(a_1*vertex_2[0] + b_1 - vertex_2[1]) == sgn(a_1*agent_1[0] + b_1 - agent_1[1])) && (sgn(a_2*vertex_2[0] + b_2 - vertex_2[1]) == sgn(a_2*agent_1[0] + b_2 - agent_1[1])))
    {   
	coverage_area_1[2*area_counter] = vertex_2[0];
	coverage_area_1[2*area_counter + 1] = vertex_2[1];
	area_counter = area_counter + 1;	
    }

    if ((sgn(a_1*vertex_3[0] + b_1 - vertex_3[1]) == sgn(a_1*agent_1[0] + b_1 - agent_1[1])) && (sgn(a_2*vertex_3[0] + b_2 - vertex_3[1]) == sgn(a_2*agent_1[0] + b_2 - agent_1[1])))
    { 
	coverage_area_1[2*area_counter] = vertex_3[0];
	coverage_area_1[2*area_counter + 1] = vertex_3[1];
	area_counter = area_counter + 1;
    }

    if ((sgn(a_1*vertex_4[0] + b_1 - vertex_4[1]) == sgn(a_1*agent_1[0] + b_1 - agent_1[1])) && (sgn(a_2*vertex_4[0] + b_2 - vertex_4[1]) == sgn(a_2*agent_1[0] + b_2 - agent_1[1])))
    {
	coverage_area_1[2*area_counter] = vertex_4[0];
	coverage_area_1[2*area_counter + 1] = vertex_4[1];
	area_counter = area_counter + 1;
    }


    if (a_1*agent_1[0] + b_1 - agent_1[1] > 0)
    {
	if (a_1*end_point_3[0] + b_1 - end_point_3[1] > 0)
	{		
		//cv2.line(rgb, (end_point_3[0],end_point_3[1]), (Intersection_point[0],Intersection_point[1]), (255,0,0), lineThickness)
	    coverage_area_1[2*area_counter] = end_point_3[0];
	    coverage_area_1[2*area_counter + 1] = end_point_3[1];
	    area_counter = area_counter + 1;
		
	}

	if (a_1*end_point_4[0] + b_1 - end_point_4[1] > 0)
	{	//cv2.line(rgb, (end_point_4[0],end_point_4[1]), (Intersection_point[0],Intersection_point[1]), (255,0,0), lineThickness)
	    coverage_area_1[2*area_counter] = end_point_4[0];
	    coverage_area_1[2*area_counter + 1] = end_point_4[1];
	    area_counter = area_counter + 1;

	}
    }    
    else if (a_1*agent_1[0] + b_1 - agent_1[1] < 0)
    {
        if (a_1*end_point_3[0] + b_1 - end_point_3[1] < 0)
	{	//cv2.line(rgb, (end_point_3[0],end_point_3[1]), (Intersection_point[0],Intersection_point[1]), (255,0,0), lineThickness)
	    coverage_area_1[2*area_counter] = end_point_3[0];
	    coverage_area_1[2*area_counter + 1] = end_point_3[1];
	    area_counter = area_counter + 1;

	}
	if (a_1*end_point_4[0] + b_1 - end_point_4[1] < 0)
	{	//cv2.line(rgb, (end_point_4[0],end_point_4[1]), (Intersection_point[0],Intersection_point[1]), (255,0,0), lineThickness)
	    coverage_area_1[2*area_counter] = end_point_4[0];
	    coverage_area_1[2*area_counter + 1] = end_point_4[1];
	    area_counter = area_counter + 1;

	}
    }

    coverage_area_1[2*area_counter] = Intersection_point[0];
    coverage_area_1[2*area_counter + 1] = Intersection_point[1];
    area_counter = area_counter + 1;
	   

    if (a_2*agent_1[0] + b_2 - agent_1[1] > 0)
    {
       if (a_2*end_point_1[0] + b_2 - end_point_1[1] > 0)
       {	//cv2.line(rgb, (end_point_1[0],end_point_1[1]), (Intersection_point[0],Intersection_point[1]), (255,0,0), lineThickness)
	    coverage_area_1[2*area_counter] = end_point_1[0];
	    coverage_area_1[2*area_counter + 1] = end_point_1[1];
	    area_counter = area_counter + 1;

       }
       if (a_2*end_point_2[0] + b_2 - end_point_2[1] > 0)
       {	//cv2.line(rgb, (end_point_2[0],end_point_2[1]), (Intersection_point[0],Intersection_point[1]), (255,0,0), lineThickness)
	    coverage_area_1[2*area_counter] = end_point_2[0];
	    coverage_area_1[2*area_counter + 1] = end_point_2[1];
	    area_counter = area_counter + 1;

       }
    }	   
    else if (a_2*agent_1[0] + b_2 - agent_1[1] < 0)
    {
       if (a_2*end_point_1[0] + b_2 - end_point_1[1] < 0)
       {        //cv2.line(rgb, (end_point_1[0],end_point_1[1]), (Intersection_point[0],Intersection_point[1]), (255,0,0), lineThickness)
	    coverage_area_1[2*area_counter] = end_point_1[0];
	    coverage_area_1[2*area_counter + 1] = end_point_1[1];
	    area_counter = area_counter + 1;

       }
       if (a_2*end_point_2[0] + b_2 - end_point_2[1] < 0)
       {	//cv2.line(rgb, (end_point_2[0],end_point_2[1]), (Intersection_point[0],Intersection_point[1]), (255,0,0), lineThickness)
	    coverage_area_1[2*area_counter] = end_point_2[0];
	    coverage_area_1[2*area_counter + 1] = end_point_2[1];
	    area_counter = area_counter + 1;

       }
    }


    int i;
    double vertice[area_counter] = {0};
    double vertice_x[area_counter] = {0};
    double vertice_y[area_counter] = {0};

    for (i = 0; i < area_counter; i += 1)
    {
	vertice[2*i] = coverage_area_1[2*i];
	vertice[2*i+1] = coverage_area_1[2*i + 1];
	vertice_x[i] = coverage_area_1[2*i];
	vertice_y[i] = coverage_area_1[2*i + 1];
    }

    ClockwiseSortPoints(vertice, area_counter);

    for (i = 0; i < area_counter; i += 1)
    {
	vertice_x[i] = vertice[2*i];
	vertice_y[i] = vertice[2*i + 1];	
    }


    double *centroid_1;


    centroid_1 = computeCentroid(vertice_x, vertice_y, area_counter);

    centroid.x = centroid_1[0];
    centroid.y = centroid_1[1];


/**/

/********************************************************************************switching control*****************************************************************************/

    k = k + 1; //a temp counter

    if (k == 50)
    {
	v = sqrt((center.position.x - temp_x)*(center.position.x - temp_x)*1000000 + (center.position.y - temp_y)*(center.position.y - temp_y)*1000000)/0.5;
	temp_x = center.position.x;
    	temp_y = center.position.y;
	//printf("speed=%f\n", v);
	k = 0;
    }	 
   

    double u[2];

    u[0] = 16.0;
    //u[1] = 14;
    double temp_c;
    temp_c = (omega_0 + 0.00001*omega_0*((z_x - centroid_1[0])*162*cos(alpha+1.5707963) + (z_y - centroid_1[1])*162*sin(alpha+1.5707963)))/3.1415926*180;
    //temp_c = (omega_0 + 0.00001*omega_0*((z_x - x_vt_ctr)*162*cos(alpha+1.5707963) + (z_y - y_vt_ctr)*162*sin(alpha+1.5707963)))/3.1415926*180;
    //int8_t temp1;
    //temp1 = int8_t(temp);

    if(temp_c > 127)
 	temp_c = 127;

    if(temp_c < -127)
	temp_c = -127;	
 
    u[1] = (double)temp_c;

    //printf("alpha=%d.\n", u[1]);
 
    int n;
    socklen_t len;
    char mesg[MAXLINE];
    //char *hello = "Hello from server"; 

    sendto(sockfd, u, sizeof(u), MSG_CONFIRM, (const struct sockaddr *) &servaddr, sizeof(servaddr));
 

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "tip_node");
    ros::NodeHandle push;
    ros::NodeHandle pull;
    ros::NodeHandle cent;
    ros::NodeHandle neighbour1; 
    ros::NodeHandle neighbour2;
    ros::Publisher pub = push.advertise<geometry_msgs::Vector3>("/vehicle/vt_ctr",1000);
    ros::Publisher pubc = cent.advertise<geometry_msgs::Vector3>("/vehicle/centroid",1000);   
    ros::Rate rate(250);

    temp_x = 0;
    temp_y = 0;


    std:string ip_param;
    int port_param;
    


    ros::param::get("~ip_param", ip_param);
    ros::param::get("~port_param", port_param);
    ros::param::get("~x_vt_ctr",x_vt_ctr);
    ros::param::get("~y_vt_ctr",y_vt_ctr);

    // Creating socket file descriptor 
    if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) 
    { 
		perror("socket creation failed"); 
		exit(EXIT_FAILURE); 
    } 
	      
    memset(&servaddr, 0, sizeof(servaddr)); 
    memset(&cliaddr, 0, sizeof(cliaddr)); 
	      
    // Filling server information 
    servaddr.sin_family = AF_INET; // IPv4 
    servaddr.sin_addr.s_addr = INADDR_ANY; 
    servaddr.sin_port = htons(port_param); 
	      
    // Bind the socket with teh server address  
    if ( bind(sockfd, (const struct sockaddr *)&servaddr,  sizeof(servaddr)) < 0 ) 
    { 
		perror("bind failed"); 
		exit(EXIT_FAILURE); 
    } 


    bzero(&servaddr, sizeof(servaddr)); 
    servaddr.sin_addr.s_addr = inet_addr(ip_param.c_str()); 
    servaddr.sin_port = htons(PORT); 
    servaddr.sin_family = AF_INET; 


    vt_ctr.x = 0;
    vt_ctr.y = 0;
    vt_ctr.z = 0;

    n1_ctr.x = 0;
    n1_ctr.y = 0;
    n1_ctr.z = 0;

    n2_ctr.x = 0;
    n2_ctr.y = 0;
    n2_ctr.z = 0;
    
    centroid.x = 0;
    centroid.y = 0;
    centroid.z = 0;

    ros::Subscriber sub = pull.subscribe("/qualisys/Center", 1000, Translate);
    ros::Subscriber neib1 = neighbour1.subscribe("/jet1/vt_ctr", 1000, N1call);
    ros::Subscriber neib2 = neighbour2.subscribe("/jet2/vt_ctr", 1000, N2call);
    //double test[2];
    //test[0] = 16.0;
    //test[1] = 0.5416/3.1415926*180;

/********************************************************************compute voronoi area and its center*******************************************************************/

/****************************************************************************coverage area*********************************************************************************/
/*    		
    double rectangle_1[4] = {414,20,1454,20};   
    double rectangle_2[4] = {1454,20,1454,1577}; 
    double rectangle_3[4] = {414,1577,1454,1577}; 
    double rectangle_4[4] = {414,20,414,1577};
*/    
/******************************************************************************agents' states******************************************************************************/
/*    	
    double agent_1[2] = {906.076160, 738.840106};
    double agent_2[2] = {800,800};
    double agent_3[2] = {1100,1100};
*/    	
/********************************************************************************Compute V1********************************************************************************/
/*
    double distance12, distance13;

    distance12 = sqrt((agent_1[0] - agent_2[0])*(agent_1[0] - agent_2[0]) + (agent_1[1] - agent_2[1])*(agent_1[1] - agent_2[1]));
    distance13 = sqrt((agent_1[0] - agent_3[0])*(agent_1[0] - agent_3[0]) + (agent_1[1] - agent_3[1])*(agent_1[1] - agent_3[1]));

    double connect_agent[2], disconnect_agent[2];
    double connection_1[4], connection_2[4];
    double disconnect_distance;

    if (distance12 <= distance13)
    {
	connection_1[0] = agent_1[0];     
	connection_1[1] = agent_1[1]; 
        connection_1[2] = agent_2[0];
        connection_1[3] = agent_2[1];  
        
        connection_2[0] = agent_1[0];     
	connection_2[1] = agent_1[1]; 
        connection_2[2] = agent_3[0];
        connection_2[3] = agent_3[1];	
        
	connect_agent[0] = agent_2[0];
	connect_agent[1] = agent_2[1];

	disconnect_agent[0] = agent_3[0];
	disconnect_agent[1] = agent_3[1];

	disconnect_distance = distance13; 
    }
    else
    {		
	connection_1[0] = agent_1[0]; 
	connection_1[1] = agent_1[1]; 
        connection_1[2] = agent_3[0];
        connection_1[3] = agent_3[1];

        connection_2[0] = agent_1[0];     
	connection_2[1] = agent_1[1]; 
        connection_2[2] = agent_2[0];
        connection_2[3] = agent_2[1];

	connect_agent[0] = agent_3[0];
        connect_agent[1] = agent_3[1];
   
	disconnect_agent[0] = agent_2[0];
	disconnect_agent[1] = agent_2[1];

	disconnect_distance = distance12;  	
    }


    double *temp;
	
    temp = findPerpenticularVector(connection_1);

    double temp_line[4];

    temp_line[0] = connect_agent[0]; //np.concatenate((connect_agent, connect_agent+temp), axis=None);
    temp_line[1] = connect_agent[1];
    temp_line[2] = connect_agent[0] + temp[0];
    temp_line[3] = connect_agent[1] + temp[1];

    //int j = 0;
    //for(j = 0; j < 2; j += 1)
    //   printf( "%f\n", temp[j]);
 
    temp = findIntersection(temp_line, rectangle_1, 414, 1454, 20, 1577);
    
    int end_point_count;
    end_point_count = 0;

    double end_point_1[2], end_point_2[2], end_point_3[2], end_point_4[2];

    if (temp[0] != 0 && temp[1] != 0)
    {
	end_point_1[0] = temp[0];
	end_point_1[1] = temp[1];
	end_point_count = end_point_count + 1; 
    }
	
    temp = findIntersection(temp_line, rectangle_2, 414, 1454, 20, 1577);

    if (temp[0] != 0 && temp[1] != 0)
    {		
	if (end_point_count == 0)
	{							
	    end_point_1[0] = temp[0];
	    end_point_1[1] = temp[1];
	    end_point_count = end_point_count + 1;
	}

	if (end_point_count == 1)
	{
	    end_point_2[0] = temp[0];
	    end_point_2[1] = temp[1];
	    end_point_count = end_point_count + 1;
 	}   
    }
	
    temp = findIntersection(temp_line, rectangle_3, 414, 1454, 20, 1577);

    if (temp[0] != 0 && temp[1] != 0)
    {
	if (end_point_count == 0)
	{							
	    end_point_1[0] = temp[0];
	    end_point_1[1] = temp[1];
	    end_point_count = end_point_count + 1;
	}
		
	if (end_point_count == 1)
	{
	    end_point_2[0] = temp[0];
	    end_point_2[1] = temp[1];
	    end_point_count = end_point_count + 1;
	}    
    }

    temp = findIntersection(temp_line, rectangle_4, 414, 1454, 20, 1577);

    if (temp[0] != 0 && temp[1] != 0)
    {
	if (end_point_count == 0)							
	{   
	   end_point_1[0] = temp[0];
	   end_point_1[1] = temp[1];
	   end_point_count = end_point_count + 1;
	}
		
	if (end_point_count == 1)
	{		
	   end_point_2[0] = temp[0];
	   end_point_2[1] = temp[1];
	   end_point_count = end_point_count + 1;
	}
    }

    printf("End point count1 = %d\n", end_point_count);


    double radius_1, radius_2;	
	
    radius_1 = sqrt((agent_1[0] - end_point_1[0])*(agent_1[0] - end_point_1[0]) + (agent_1[1] - end_point_1[1])*(agent_1[1] - end_point_1[1]));
    radius_2 = sqrt((agent_1[0] - end_point_2[0])*(agent_1[0] - end_point_2[0]) + (agent_1[1] - end_point_2[1])*(agent_1[1] - end_point_2[1]));

    double radius;

    if(radius_1 >= radius_2)    
	radius = radius_1;
    else
	radius = radius_2;

    double *mid;	

    if (radius <= disconnect_distance)
    {   
	mid = Cal_mid_point(agent_1, connect_agent);
	temp = findPerpenticularVector(connection_1);
    	
	temp_line[0] = mid[0]; 
	temp_line[1] = mid[1];
        temp_line[2] = mid[0] + temp[0];
        temp_line[3] = mid[1] + temp[1];
	   
	temp = findIntersection(temp_line, rectangle_1, 414, 1454, 20, 1577);
	end_point_count = 0;

	if (temp[0] != 0 && temp[1] != 0)
	{
	    end_point_1[0] = temp[0];
	    end_point_1[1] = temp[1];
	    end_point_count = end_point_count + 1; 
	}
	   
	temp = findIntersection(temp_line, rectangle_2, 414, 1454, 20, 1577);
	   
	if (temp[0] != 0 && temp[1] != 0)
	{
	    if (end_point_count == 0)							
	    {   
	    	end_point_1[0] = temp[0];
	    	end_point_1[1] = temp[1];
		end_point_count = end_point_count + 1;
	    }
	    else if (end_point_count == 1)
	    {
	    	end_point_2[0] = temp[0];
	    	end_point_2[1] = temp[1];
		end_point_count = end_point_count + 1;
	    }
	}

	temp = findIntersection(temp_line, rectangle_3, 414, 1454, 20, 1577);

        if (temp[0] != 0 && temp[1] != 0)
	{
	    if (end_point_count == 0)							
	    {
	       end_point_1[0] = temp[0];
	       end_point_1[1] = temp[1];
	       end_point_count = end_point_count + 1;
	    }
	    else if (end_point_count == 1)
	    {
	       end_point_2[0] = temp[0];
	       end_point_2[1] = temp[1];
	       end_point_count = end_point_count + 1;
	    }	
	}
	   
	temp = findIntersection(temp_line, rectangle_4, 414, 1454, 20, 1577);
	   
	if (temp[0] != 0 && temp[1] != 0)
	{
		if (end_point_count == 0)
		{							
		   end_point_1[0] = temp[0];
		   end_point_1[1] = temp[1];
		   end_point_count = end_point_count + 1;
		}
		else if (end_point_count == 1)
		{
		   end_point_2[0] = temp[0];
		   end_point_2[1] = temp[1];
		   end_point_count = end_point_count + 1;
		}
	}
	printf("End point count2 = %d\n", end_point_count);

     }	
     else
     {	   
	   mid = Cal_mid_point(agent_1, connect_agent);
	   temp = findPerpenticularVector(connection_1);
	   
	   temp_line[0] = mid[0]; 
	   temp_line[1] = mid[1];
           temp_line[2] = mid[0] + temp[0];
           temp_line[3] = mid[1] + temp[1];

	   temp = findIntersection(temp_line, rectangle_1, 414, 1454, 20, 1577);
	   end_point_count = 0;

	   int flag;
	   flag = 0;

	   if (temp[0] != 0 && temp[1] != 0)
	   {
		end_point_1[0] = temp[0];
		end_point_1[1] = temp[1];
		end_point_count = end_point_count + 1;
		flag = flag + 1; 
	   }

	   temp = findIntersection(temp_line, rectangle_2, 414, 1454, 20, 1577);

	   if (temp[0] != 0 && temp[1] != 0)
	   {	
		flag = flag + 1; 
		if (end_point_count == 0)
		{							
		    end_point_1[0] = temp[0];
		    end_point_1[1] = temp[1];
		    end_point_count = end_point_count + 1;
		}		
		else if (end_point_count == 1)
		{ 
		    end_point_2[0] = temp[0];
		    end_point_2[1] = temp[1];		
		    end_point_count = end_point_count + 1;
		}
	   }

	   temp = findIntersection(temp_line, rectangle_3, 414, 1454, 20, 1577);

	   if (temp[0] != 0 && temp[1] != 0)
	   {
		flag = flag + 1; 
		if (end_point_count == 0)
		{							
		    end_point_1[0] = temp[0];
		    end_point_1[1] = temp[1];
		    end_point_count = end_point_count + 1;
		}
		else if (end_point_count == 1)
		{
		    end_point_2[0] = temp[0];
		    end_point_2[1] = temp[1];
		    end_point_count = end_point_count + 1;
		}
	   }

	   temp = findIntersection(temp_line, rectangle_4, 414, 1454, 20, 1577);

	   if (temp[0] != 0 && temp[1] != 0)
	   {
		flag = flag + 1; 
		if (end_point_count == 0)
		{							
		    end_point_1[0] = temp[0];
		    end_point_1[1] = temp[1];		
		    end_point_count = end_point_count + 1;
		}
		else if (end_point_count == 1)
		{
		    end_point_2[0] = temp[0];
		    end_point_2[1] = temp[1];
		    end_point_count = end_point_count + 1;
		}
	  }
	  printf("End point count3 = %d\n", end_point_count);
    }


    //printf( "end_point_1:%f,%f\n", end_point_1[0], end_point_1[1]);
    //printf( "end_point_2:%f,%f\n", end_point_2[0], end_point_2[1]);
	   

    mid = Cal_mid_point(agent_1, disconnect_agent);
    temp = findPerpenticularVector(connection_2);
           
    temp_line[0] = mid[0]; 
    temp_line[1] = mid[1];
    temp_line[2] = mid[0] + temp[0];
    temp_line[3] = mid[1] + temp[1];
	   
    temp = findIntersection(temp_line, rectangle_1, 414, 1454, 20, 1577);
    end_point_count = 0;

    if (temp[0] != 0 && temp[1] != 0)
    {
	end_point_3[0] = temp[0];
	end_point_3[1] = temp[1];
	end_point_count = end_point_count + 1; 
    }
	   
    temp = findIntersection(temp_line, rectangle_2, 414, 1454, 20, 1577);

    if (temp[0] != 0 && temp[1] != 0)
    {

	if (end_point_count == 0)							
	{	   
	   end_point_3[0] = temp[0];
	   end_point_3[1] = temp[1];
	   end_point_count = end_point_count + 1;
	}
	else if (end_point_count == 1)
	{
	   end_point_4[0] = temp[0];
	   end_point_4[1] = temp[1];
	   end_point_count = end_point_count + 1;
	}
    }

    temp = findIntersection(temp_line, rectangle_3, 414, 1454, 20, 1577);

    if (temp[0] != 0 && temp[1] != 0)
    {
       if (end_point_count == 0)							
       {				
           end_point_3[0] = temp[0];
           end_point_3[1] = temp[1];
	   end_point_count = end_point_count + 1;
       }
       else if (end_point_count == 1)
       {
	   end_point_4[0] = temp[0];
	   end_point_4[1] = temp[1];
	   end_point_count = end_point_count + 1;
       }
    }	   
       
    temp = findIntersection(temp_line, rectangle_4, 414, 1454, 20, 1577);

    if (temp[0] != 0 && temp[1] != 0)
    {	
	if (end_point_count == 0)
	{							
	    end_point_3[0] = temp[0];
	    end_point_3[1] = temp[1];
	    end_point_count = end_point_count + 1;
	}
	else if (end_point_count == 1)
	{
	    end_point_4[0] = temp[0];
	    end_point_4[1] = temp[1];
	    end_point_count = end_point_count + 1;
	}
    }

    printf( "End point count4 = %d\n", end_point_count);	
    printf( "End_point_1 = %f,%f\n", end_point_1[0], end_point_1[1]);
    printf( "End_point_2 = %f,%f\n", end_point_2[0], end_point_2[1]);
    printf( "End_point_3 = %f,%f\n", end_point_3[0], end_point_3[1]);
    printf( "End_point_4 = %f,%f\n", end_point_4[0], end_point_4[1]);


    double *Intersection_point;
    double temp_line_1[4], temp_line_2[4];
	
    temp_line_1[0] = end_point_1[0];
    temp_line_1[1] = end_point_1[1];
    temp_line_1[2] = end_point_2[0];
    temp_line_1[3] = end_point_2[1];

    temp_line_2[0] = end_point_3[0];
    temp_line_2[1] = end_point_3[1];
    temp_line_2[2] = end_point_4[0];
    temp_line_2[3] = end_point_4[1];	   

    Intersection_point = findIntersection1(temp_line_1, temp_line_2, 414, 1454, 20, 1577);

    int area_counter;
    area_counter = 0;
    double coverage_area_1[20] = {0};



    printf( "inter_point:%f,%f\n", Intersection_point[0], Intersection_point[1]);

    double a_1,b_1,a_2,b_2;

    double vertex_1[2] = {414,  20};   
    double vertex_2[2] = {1454, 20}; 
    double vertex_3[2] = {1396, 1577}; 
    double vertex_4[2] = {383,  1534};   

    a_1 = (end_point_1[1] - end_point_2[1])/(end_point_1[0] - end_point_2[0]);
    b_1 = end_point_1[1] - a_1*end_point_1[0];

    a_2 = (end_point_3[1] - end_point_4[1])/(end_point_3[0] - end_point_4[0]);
    b_2 = end_point_3[1] - a_2*end_point_3[0];


    if ((sgn(a_1*vertex_1[0] + b_1 - vertex_1[1]) == sgn(a_1*agent_1[0] + b_1 - agent_1[1])) && (sgn(a_2*vertex_1[0] + b_2 - vertex_1[1]) == sgn(a_2*agent_1[0] + b_2 - agent_1[1])))
    {
	coverage_area_1[2*area_counter] = vertex_1[0];
	coverage_area_1[2*area_counter + 1] = vertex_1[1];
	area_counter = area_counter + 1;
    }
 
    if ((sgn(a_1*vertex_2[0] + b_1 - vertex_2[1]) == sgn(a_1*agent_1[0] + b_1 - agent_1[1])) && (sgn(a_2*vertex_2[0] + b_2 - vertex_2[1]) == sgn(a_2*agent_1[0] + b_2 - agent_1[1])))
    {   
	coverage_area_1[2*area_counter] = vertex_2[0];
	coverage_area_1[2*area_counter + 1] = vertex_2[1];
	area_counter = area_counter + 1;	
    }

    if ((sgn(a_1*vertex_3[0] + b_1 - vertex_3[1]) == sgn(a_1*agent_1[0] + b_1 - agent_1[1])) && (sgn(a_2*vertex_3[0] + b_2 - vertex_3[1]) == sgn(a_2*agent_1[0] + b_2 - agent_1[1])))
    { 
	coverage_area_1[2*area_counter] = vertex_3[0];
	coverage_area_1[2*area_counter + 1] = vertex_3[1];
	area_counter = area_counter + 1;
    }

    if ((sgn(a_1*vertex_4[0] + b_1 - vertex_4[1]) == sgn(a_1*agent_1[0] + b_1 - agent_1[1])) && (sgn(a_2*vertex_4[0] + b_2 - vertex_4[1]) == sgn(a_2*agent_1[0] + b_2 - agent_1[1])))
    {
	coverage_area_1[2*area_counter] = vertex_4[0];
	coverage_area_1[2*area_counter + 1] = vertex_4[1];
	area_counter = area_counter + 1;
    }


    if (a_1*agent_1[0] + b_1 - agent_1[1] > 0)
    {
	if (a_1*end_point_3[0] + b_1 - end_point_3[1] > 0)
	{		
		//cv2.line(rgb, (end_point_3[0],end_point_3[1]), (Intersection_point[0],Intersection_point[1]), (255,0,0), lineThickness)
	    coverage_area_1[2*area_counter] = end_point_3[0];
	    coverage_area_1[2*area_counter + 1] = end_point_3[1];
	    area_counter = area_counter + 1;
		
	}

	if (a_1*end_point_4[0] + b_1 - end_point_4[1] > 0)
	{	//cv2.line(rgb, (end_point_4[0],end_point_4[1]), (Intersection_point[0],Intersection_point[1]), (255,0,0), lineThickness)
	    coverage_area_1[2*area_counter] = end_point_4[0];
	    coverage_area_1[2*area_counter + 1] = end_point_4[1];
	    area_counter = area_counter + 1;

	}
    }    
    else if (a_1*agent_1[0] + b_1 - agent_1[1] < 0)
    {
        if (a_1*end_point_3[0] + b_1 - end_point_3[1] < 0)
	{	//cv2.line(rgb, (end_point_3[0],end_point_3[1]), (Intersection_point[0],Intersection_point[1]), (255,0,0), lineThickness)
	    coverage_area_1[2*area_counter] = end_point_3[0];
	    coverage_area_1[2*area_counter + 1] = end_point_3[1];
	    area_counter = area_counter + 1;

	}
	if (a_1*end_point_4[0] + b_1 - end_point_4[1] < 0)
	{	//cv2.line(rgb, (end_point_4[0],end_point_4[1]), (Intersection_point[0],Intersection_point[1]), (255,0,0), lineThickness)
	    coverage_area_1[2*area_counter] = end_point_4[0];
	    coverage_area_1[2*area_counter + 1] = end_point_4[1];
	    area_counter = area_counter + 1;

	}
    }

    coverage_area_1[2*area_counter] = Intersection_point[0];
    coverage_area_1[2*area_counter + 1] = Intersection_point[1];
    area_counter = area_counter + 1;
	   

    if (a_2*agent_1[0] + b_2 - agent_1[1] > 0)
    {
       if (a_2*end_point_1[0] + b_2 - end_point_1[1] > 0)
       {	//cv2.line(rgb, (end_point_1[0],end_point_1[1]), (Intersection_point[0],Intersection_point[1]), (255,0,0), lineThickness)
	    coverage_area_1[2*area_counter] = end_point_1[0];
	    coverage_area_1[2*area_counter + 1] = end_point_1[1];
	    area_counter = area_counter + 1;

       }
       if (a_2*end_point_2[0] + b_2 - end_point_2[1] > 0)
       {	//cv2.line(rgb, (end_point_2[0],end_point_2[1]), (Intersection_point[0],Intersection_point[1]), (255,0,0), lineThickness)
	    coverage_area_1[2*area_counter] = end_point_2[0];
	    coverage_area_1[2*area_counter + 1] = end_point_2[1];
	    area_counter = area_counter + 1;

       }
    }	   
    else if (a_2*agent_1[0] + b_2 - agent_1[1] < 0)
    {
       if (a_2*end_point_1[0] + b_2 - end_point_1[1] < 0)
       {        //cv2.line(rgb, (end_point_1[0],end_point_1[1]), (Intersection_point[0],Intersection_point[1]), (255,0,0), lineThickness)
	    coverage_area_1[2*area_counter] = end_point_1[0];
	    coverage_area_1[2*area_counter + 1] = end_point_1[1];
	    area_counter = area_counter + 1;

       }
       if (a_2*end_point_2[0] + b_2 - end_point_2[1] < 0)
       {	//cv2.line(rgb, (end_point_2[0],end_point_2[1]), (Intersection_point[0],Intersection_point[1]), (255,0,0), lineThickness)
	    coverage_area_1[2*area_counter] = end_point_2[0];
	    coverage_area_1[2*area_counter + 1] = end_point_2[1];
	    area_counter = area_counter + 1;

       }
    }


    int i;
    double vertice[area_counter] = {0};
    double vertice_x[area_counter] = {0};
    double vertice_y[area_counter] = {0};

    for (i = 0; i < area_counter; i += 1)
    {
	vertice[2*i] = coverage_area_1[2*i];
	vertice[2*i+1] = coverage_area_1[2*i + 1];
	vertice_x[i] = coverage_area_1[2*i];
	vertice_y[i] = coverage_area_1[2*i + 1];
    }

    ClockwiseSortPoints(vertice, area_counter);

    for (i = 0; i < area_counter; i += 1)
    {
	printf("%f, %f\n", vertice[2*i], vertice[2*i+1]);
    }


    double *centroid_1;
    centroid_1 = computeCentroid(vertice_x, vertice_y, area_counter);

    printf("%f, %f\n", centroid_1[0], centroid_1[1]);

*/

    while(ros::ok())
    {
	if(!((vt_ctr.x==0)&&(vt_ctr.y==0)&&(vt_ctr.z==0)))
	{
		pub.publish(vt_ctr);
	}
	
	if(!((centroid.x==0)&&(centroid.y==0)&&(centroid.z==0)))
	{
		pubc.publish(centroid);
	}

	ros::spinOnce();
	rate.sleep();
    }
    
    return 0;


}

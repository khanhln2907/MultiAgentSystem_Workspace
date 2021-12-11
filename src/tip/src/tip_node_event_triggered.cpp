#include "ros/ros.h"
#include "qualisys/Marker.h"
#include "qualisys/Subject.h"
#include <geometry_msgs/Vector3.h>
#include "tip/Vector.h"
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
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>

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
geometry_msgs::Vector3 n3_ctr;
geometry_msgs::Vector3 vt_ctr;
geometry_msgs::Vector3 centroid;
geometry_msgs::Vector3 event;

tip::Vector voronoi_out;


double temp_c_t;    //control input at events
double v_x, v_y, v; 
unsigned long k;    //A global counter

int x_vt_ctr;
int y_vt_ctr;

double u[2]; //global 
//int flag; //to test the stop button 

double time_init;


typedef struct Point
{
    long x;
    long y;
}Point;


typedef struct Line
{
    long x1;
    long y1;
    long x2;
    long y2;
}Line;

typedef struct Vector2
{
    long x;
    long y;
}Vector2;

Point vertice_1;
Point vertice_2;
Point vertice_3;
Point vertice_4;

Line rectangle_1; // = {20,20,4000,20};   
Line rectangle_2; // = {4000,20,4000,2800}; 
Line rectangle_3; // = {20,2800,4000,2800}; 
Line rectangle_4; // = {20,20,20,2800};

Point centroid_1;

Point agent_0;
Point agent_1;
Point agent_2;
Point agent_3;

void N1call(const geometry_msgs::Vector3& n1)
{
    agent_1.x = (long)n1.x;
    agent_1.y = (long)n1.y;
}

void N2call(const geometry_msgs::Vector3& n2)
{
    agent_2.x = (long)n2.x;
    agent_2.y = (long)n2.y;
}

void N3call(const geometry_msgs::Vector3& n3)
{
    agent_3.x = (long)n3.x;
    agent_3.y = (long)n3.y;
}

int sgn(double v) 
{

    if (v < 0) return -1;
    if (v > 0) return 1;
    return 0;

}

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

void ClockwiseSortPoints(long Points[], int nPoints)
{
    //计算重心
    Point center;
    Point temp_1;
    Point temp_2;
    Point tmp;

    long x = 0,y = 0;

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

Point Cal_mid_point(Point a1, Point a2)
{
    Point mid_point;

    mid_point.x = (a1.x + a2.x)/2;
    mid_point.y = (a1.y + a2.y)/2;

    return mid_point;
}    

Point findIntersection(Line line1, Line line2, int x_limit_min, int x_limit_max, int y_limit_min, int y_limit_max)
{

    long x1,x2,y1,y2,x3,y3,x4,y4,denominator,xNominator,yNominator,px,py;
    Point intersection;
    intersection.x = -1;
    intersection.y = -1;
	
    x1 = line1.x1;
    y1 = line1.y1;
    x2 = line1.x2;
    y2 = line1.y2;

    x3 = line2.x1;
    y3 = line2.y1;
    x4 = line2.x2;
    y4 = line2.y2;

    denominator = (x1 - x2)*(y3 - y4) - (y1 - y2)*(x3 - x4);

    if (denominator == 0)
    {
	intersection.x = 0;
        intersection.y = 0;
        return intersection;		
    }    

    xNominator = (x1*y2 - y1*x2)*(x3 - x4) - (x1 - x2)*(x3*y4 - y3*x4);
    yNominator = (x1*y2 - y1*x2)*(y3 - y4) - (y1 - y2)*(x3*y4 - y3*x4);

    px = xNominator/denominator;
    py = yNominator/denominator;

    x_limit_min = x_limit_min - 1;
    x_limit_max = x_limit_max + 1;
    y_limit_min = y_limit_min - 1;
    y_limit_max = y_limit_max + 1;


    if ((px > x_limit_min) && (px <= x_limit_max) && (py >= y_limit_min) && (py <= y_limit_max))
    {
	intersection.x = px;
        intersection.y = py;
    }
    else
    { 	
	intersection.x = 0;
        intersection.y = 0;
    }

    return intersection;
}


Point findIntersection1(Line line1, Line line2, int x_limit_min, int x_limit_max, int y_limit_min, int y_limit_max)
{
    long x1,x2,y1,y2,x3,y3,x4,y4,denominator,xNominator,yNominator,px,py;
    Point intersection;
    intersection.x = -1;
    intersection.y = -1;

    x1 = line1.x1;
    y1 = line1.y1;
    x2 = line1.x2;
    y2 = line1.y2;

    x3 = line2.x1;
    y3 = line2.y1;
    x4 = line2.x2;
    y4 = line2.y2;

    denominator = (x1 - x2)*(y3 - y4) - (y1 - y2)*(x3 - x4);

    if (denominator != 0)
    {
	xNominator = (x1*y2 - y1*x2)*(x3 - x4) - (x1 - x2)*(x3*y4 - y3*x4);
        yNominator = (x1*y2 - y1*x2)*(y3 - y4) - (y1 - y2)*(x3*y4 - y3*x4);
        px = xNominator / denominator;
        py = yNominator / denominator;

	 	if ((px > x_limit_min) && (px < x_limit_max) && (py > y_limit_min) && (py < y_limit_max))
	 	{
	 		intersection.x = px;
         		intersection.y = py;
	 	}
	 	else
	 	{
			intersection.x = 0;
        		intersection.y = 0;
		}		
    }
    else
    {
	intersection.x = 0;
        intersection.y = 0;
    }
	
    return intersection;
}


Vector2 findPerpenticularVector(Line line)
{
    long x1,x2,y1,y2;
    Vector2 pVector;
    
    x1 = line.x1;
    y1 = line.y1;
    x2 = line.x2;
    y2 = line.y2;
	
    pVector.x = y1 - y2;
    pVector.y = x2 - x1;

    return pVector;

}


Point computeCentroid(long Vertices_X[], long Vertices_Y[], int nVertices)
{

    Point Centroid_P;
    long centroidX = 0, centroidY = 0;
    long det = 0, tempDet = 0;
    unsigned int j = 0;

    Centroid_P.x = -1;
    Centroid_P.y = -1;
	
    for (unsigned int i = 0; i < nVertices; i++)
    {
	// closed polygon
	if (i + 1 == nVertices)
	    j = 0;
	else
	    j = i + 1;
	// compute the determinant
	tempDet = Vertices_X[i] * Vertices_Y[j] - Vertices_X[j]*Vertices_Y[i];

	det += tempDet;

	centroidX += (Vertices_X[i] + Vertices_X[j])*tempDet;
	centroidY += (Vertices_Y[i] + Vertices_Y[j])*tempDet;
    }

    // divide by the total mass of the polygon
    centroidX /= 3*det;
    centroidY /= 3*det;

    Centroid_P.x = centroidX;
    Centroid_P.y = centroidY;
    
    //ROS_INFO("Give you: %f\n", Centroid_P[0]);
    //ROS_INFO("centroid = %f, %f", Centroid_P[0], Centroid_P[1]);	
        
    return Centroid_P;
}


Line findIntersection_rectangle(Line line)
{

    Line line_end;
    line_end.x1 = -1;
    line_end.y1 = -1;
    line_end.x2 = -1; 
    line_end.y2 = -1;    

    int end_point_count = 0;

    Point temp;

    temp.x = -1;
    temp.y = -1;

    temp = findIntersection(line, rectangle_1, 20, 4000, 20, 2800);

    //ROS_INFO("temp1:%ld,%ld", temp.x, temp.y);

    if ((temp.x != 0) && (temp.y != 0))
    {
		line_end.x1 = temp.x;
		line_end.y1 = temp.y;
		end_point_count = end_point_count + 1;
    }
	
    temp = findIntersection(line, rectangle_2, 20, 4000, 20, 2800);

	//ROS_INFO("temp2:%ld,%ld", temp.x, temp.y);

    if ((temp.x != 0) && (temp.y != 0))
    {		
		if (end_point_count == 0)
		{							
	    	line_end.x1 = temp.x;
	    	line_end.y1 = temp.y;
	    	end_point_count = end_point_count + 1;
		}

		else if (end_point_count == 1)
		{
	    	line_end.x2 = temp.x;
	    	line_end.y2 = temp.y;
	    	end_point_count = end_point_count + 1;
 		}   
    }
	
    temp = findIntersection(line, rectangle_3, 20, 4000, 20, 2800);

    //ROS_INFO("temp3:%ld,%ld", temp.x, temp.y);

    if ((temp.x != 0) && (temp.y != 0))
    {
		if (end_point_count == 0)
		{							
	    	line_end.x1 = temp.x;
	    	line_end.y1 = temp.y;
	    	end_point_count = end_point_count + 1;
		}
		
		else if (end_point_count == 1)
		{
	    	line_end.x2 = temp.x;
	    	line_end.y2 = temp.y;
	    	end_point_count = end_point_count + 1;
		}    
    }

    temp = findIntersection(line, rectangle_4, 20, 4000, 20, 2800);

    //ROS_INFO("temp4:%ld,%ld", temp.x, temp.y);

    if ((temp.x != 0) && (temp.y != 0))
    {
		if (end_point_count == 0)							
		{   
	    	line_end.x1 = temp.x;
	    	line_end.y1 = temp.y;
	   		end_point_count = end_point_count + 1;
		}
		
		else if (end_point_count == 1)
		{		
	    	line_end.x2 = temp.x;
	    	line_end.y2 = temp.y;
	   		end_point_count = end_point_count + 1;
		}
    }


    return line_end;

}


void voronoi_area_compute(Point agent_0, Point agent_1, Point agent_2, Point agent_3)
{

    double distance[3] = {-1};
    double distance_1 = -1;
    double distance_2 = -1;
    double distance_3 = -1;

    Point connect_agent_1;
    Point connect_agent_2;
    Point connect_agent_3;

    int E1_vertice_counter = 0;
    long E1_vertices[20] = {0};

    Vector2 direction_vector_1;
    Vector2 direction_vector_2;
    Vector2 direction_vector_3;

    Line E1_segment_1;
    Line E1_segment_2;

    Line E1_segment_1_intersection;
    Line E1_segment_2_intersection;

    long E1_segment_1_a = 0;            /*line E1_segment_1 parameter a*/
    long E1_segment_1_b = 0;		/*line E1_segment_1 parameter b*/
    long E1_segment_1_c = -1;		/*line E1_segment_1 parameter c*/

    long E1_segment_2_a = 0;            /*line E1_segment_2 parameter a*/
    long E1_segment_2_b = 0;		/*line E1_segment_2 parameter b*/
    long E1_segment_2_c = -1;		/*line E1_segment_2 parameter c*/

    Point mid_of_connection_1;
    Point mid_of_connection_2;
    Point mid_of_connection_3;

    Point s1_s2_inter_point;
    Point V1_s1_s2_inter_point;
    Point V1_s2_s3_inter_point;
    Point V1_s1_s3_inter_point;

    int V1_vertice_counter = 0;
    long V1_vertices[20] = {0};
    Line V1_segment_1;
    Line V1_segment_1_intersection;
    Line V1_segment_2;
    Line V1_segment_2_intersection;
    Line V1_segment_3;
    Line V1_segment_3_intersection;

    int sgn_1;
    int sgn_2;
    int sgn_3;

    int	sgn_1_s_3_1; 
    int sgn_1_s_3_2; 
    int sgn_2_s_3_1; 
    int sgn_2_s_3_2;

    int sgn_1_s_2_1;
    int sgn_1_s_2_2;
    int sgn_3_s_2_1;
    int sgn_3_s_2_2;

    int sgn_2_s_1_1;
    int sgn_2_s_1_2;
    int sgn_3_s_1_1;
    int sgn_3_s_1_2;

    int sgn_1_23_intersection; 
    int sgn_2_13_intersection;	    
    int sgn_3_12_intersection;

    double V1_segment_1_a = 0;            /*line V1_segment_1 parameter a*/
    double V1_segment_1_b = 0;		  /*line V1_segment_1 parameter b*/
    double V1_segment_1_c = -1;		  /*line V1_segment_1 parameter c*/

    double V1_segment_2_a = 0;            /*line V1_segment_2 parameter a*/
    double V1_segment_2_b = 0;		  /*line V1_segment_2 parameter b*/
    double V1_segment_2_c = -1;		  /*line V1_segment_2 parameter c*/

    double V1_segment_3_a = 0;            /*line V1_segment_3 parameter a*/
    double V1_segment_3_b = 0;		  /*line V1_segment_3 parameter b*/
    double V1_segment_3_c = -1;		  /*line V1_segment_3 parameter c*/

    E1_segment_1.x1 = -1;
    E1_segment_1.y1 = -1;
    E1_segment_1.x2 = -1;
    E1_segment_1.y2 = -1;

    E1_segment_2.x1 = -1;
    E1_segment_2.y1 = -1;
    E1_segment_2.x2 = -1;
    E1_segment_2.y2 = -1;

    E1_segment_1_intersection.x1 = -1;
    E1_segment_1_intersection.y1 = -1;
    E1_segment_1_intersection.x2 = -1;
    E1_segment_1_intersection.y2 = -1;

    E1_segment_2_intersection.x1 = -1;
    E1_segment_2_intersection.y1 = -1;
    E1_segment_2_intersection.x2 = -1;
    E1_segment_2_intersection.y2 = -1;

    V1_segment_1.x1 = -1;
    V1_segment_1.y1 = -1;
    V1_segment_1.x2 = -1;
    V1_segment_1.y2 = -1;

    V1_segment_2.x1 = -1;
    V1_segment_2.y1 = -1;
    V1_segment_2.x2 = -1;
    V1_segment_2.y2 = -1;

    V1_segment_3.x1 = -1;
    V1_segment_3.y1 = -1;
    V1_segment_3.x2 = -1;
    V1_segment_3.y2 = -1;

    V1_segment_1_intersection.x1 = -1;
    V1_segment_1_intersection.y1 = -1;
    V1_segment_1_intersection.x2 = -1;
    V1_segment_1_intersection.y2 = -1;

    V1_segment_2_intersection.x1 = -1;
    V1_segment_2_intersection.y1 = -1;
    V1_segment_2_intersection.x2 = -1;
    V1_segment_2_intersection.y2 = -1;

    V1_segment_3_intersection.x1 = -1;
    V1_segment_3_intersection.y1 = -1;
    V1_segment_3_intersection.x2 = -1;
    V1_segment_3_intersection.y2 = -1;

    connect_agent_1.x = -1;
    connect_agent_1.y = -1;

    connect_agent_2.x = -1;
    connect_agent_2.y = -1;

    connect_agent_3.x = -1;
    connect_agent_3.y = -1;

    Line connection_1;
    Line connection_2;
    Line connection_3;

    connection_1.x1 = -1;
    connection_1.y1 = -1;
    connection_1.x2 = -1;
    connection_1.y2 = -1;

    connection_2.x1 = -1;
    connection_2.y1 = -1;
    connection_2.x2 = -1;
    connection_2.y2 = -1;

    connection_3.x1 = -1;
    connection_3.y1 = -1;
    connection_3.x2 = -1;
    connection_3.y2 = -1;

    s1_s2_inter_point.x = -1;
    s1_s2_inter_point.y = -1;

    V1_s1_s2_inter_point.x = -1;
    V1_s1_s2_inter_point.y = -1;
    V1_s2_s3_inter_point.x = -1;
    V1_s2_s3_inter_point.y = -1;
    V1_s1_s3_inter_point.x = -1;
    V1_s1_s3_inter_point.y = -1;

    sgn_1 = -2;
    sgn_2 = -2;
    sgn_3 = -2;

    sgn_1_s_3_1 = -2; 
    sgn_1_s_3_2 = -2; 
    sgn_2_s_3_1 = -2; 
    sgn_2_s_3_2 = -2;

    sgn_1_s_2_1 = -2;
    sgn_1_s_2_2 = -2;
    sgn_3_s_2_1 = -2;
    sgn_3_s_2_2 = -2;

    sgn_2_s_1_1 = -2;
    sgn_2_s_1_2 = -2;
    sgn_3_s_1_1 = -2;
    sgn_3_s_1_2 = -2;

    /**************Sort the neighbours according to distance***************/ 
    distance[0] = sqrt((agent_0.x - agent_1.x)*(agent_0.x - agent_1.x) + (agent_0.y - agent_1.y)*(agent_0.y - agent_1.y));   //distance between agent 0 and 1
    distance[1] = sqrt((agent_0.x - agent_2.x)*(agent_0.x - agent_2.x) + (agent_0.y - agent_2.y)*(agent_0.y - agent_2.y));   //distance between agent 0 and 2
    distance[2] = sqrt((agent_0.x - agent_3.x)*(agent_0.x - agent_3.x) + (agent_0.y - agent_3.y)*(agent_0.y - agent_3.y));   //distance between agent 0 and 3

    if ( (distance[0] <= distance[1]) && (distance[1] <= distance[2]) )
    {
	connection_1.x1 = agent_0.x;     
	connection_1.y1 = agent_0.y; 
        connection_1.x2 = agent_1.x;
        connection_1.y2 = agent_1.y;  
        
        connection_2.x1 = agent_0.x;     
	connection_2.y1 = agent_0.y; 
        connection_2.x2 = agent_2.x;
        connection_2.y2 = agent_2.y;

        connection_3.x1 = agent_0.x;     
	connection_3.y1 = agent_0.y; 
        connection_3.x2 = agent_3.x;
        connection_3.y2 = agent_3.y;	
        
	connect_agent_1.x = agent_1.x;
	connect_agent_1.y = agent_1.x;

	connect_agent_2.x = agent_2.x;
	connect_agent_2.y = agent_2.y;

	connect_agent_3.x = agent_3.x;
	connect_agent_3.y = agent_3.y;

        distance_1 = distance[0];
        distance_2 = distance[1];
        distance_3 = distance[2];
    }

    if ( (distance[0] <= distance[2]) && (distance[2] <= distance[1]) )
    {
	connection_1.x1 = agent_0.x;     
	connection_1.y1 = agent_0.y; 
        connection_1.x2 = agent_1.x;
        connection_1.y2 = agent_1.y;  
        
        connection_2.x1 = agent_0.x;     
	connection_2.y1 = agent_0.y; 
        connection_2.x2 = agent_3.x;
        connection_2.y2 = agent_3.y;

        connection_3.x1 = agent_0.x;     
	connection_3.y1 = agent_0.y; 
        connection_3.x2 = agent_2.x;
        connection_3.y2 = agent_2.y;	
        
	connect_agent_1.x = agent_1.x;
	connect_agent_1.y = agent_1.y;

	connect_agent_2.x = agent_3.x;
	connect_agent_2.y = agent_3.y;

	connect_agent_3.x = agent_2.x;
	connect_agent_3.y = agent_2.y;

        distance_1 = distance[0];
        distance_2 = distance[2];
        distance_3 = distance[1];
    }

    if ((distance[1] <= distance[0]) && (distance[0] <= distance[2]))
    {
	connection_1.x1 = agent_0.x;     
	connection_1.y1 = agent_0.y; 
        connection_1.x2 = agent_2.x;
        connection_1.y2 = agent_2.y;  
        
        connection_2.x1 = agent_0.x;     
	connection_2.y1 = agent_0.y; 
        connection_2.x2 = agent_1.x;
        connection_2.y2 = agent_1.y;

        connection_3.x1 = agent_0.x;     
	connection_3.y1 = agent_0.y; 
        connection_3.x2 = agent_3.x;
        connection_3.y2 = agent_3.y;	
        
	connect_agent_1.x = agent_2.x;
	connect_agent_1.y = agent_2.y;

	connect_agent_2.x = agent_1.x;
	connect_agent_2.y = agent_1.y;

	connect_agent_3.x = agent_3.x;
	connect_agent_3.y = agent_3.y;

        distance_1 = distance[1];
        distance_2 = distance[0];
        distance_3 = distance[2];
    }

    if ((distance[1] <= distance[2]) && (distance[2] <= distance[0]))
    {
	connection_1.x1 = agent_0.x;     
	connection_1.y1 = agent_0.y; 
        connection_1.x2 = agent_2.x;
        connection_1.y2 = agent_2.y;  
        
        connection_2.x1 = agent_0.x;     
	connection_2.y1 = agent_0.y; 
        connection_2.x2 = agent_3.x;
        connection_2.y2 = agent_3.y;

        connection_3.x1 = agent_0.x;     
	connection_3.y1 = agent_0.y; 
        connection_3.x2 = agent_1.x;
        connection_3.y2 = agent_1.y;	
        
	connect_agent_1.x = agent_2.x;
	connect_agent_1.y = agent_2.y;

	connect_agent_2.x = agent_3.x;
	connect_agent_2.y = agent_3.y;

	connect_agent_3.x = agent_1.x;
	connect_agent_3.y = agent_1.y;

        distance_1 = distance[1];
        distance_2 = distance[2];
        distance_3 = distance[0];

    }

    if ((distance[2] <= distance[1]) && (distance[1] <= distance[0]))
    {
	connection_1.x1 = agent_0.x;     
	connection_1.y1 = agent_0.y; 
        connection_1.x2 = agent_3.x;
        connection_1.y2 = agent_3.y;  
        
        connection_2.x1 = agent_0.x;     
	connection_2.y1 = agent_0.y; 
        connection_2.x2 = agent_2.x;
        connection_2.y2 = agent_2.y;

        connection_3.x1 = agent_0.x;     
	connection_3.y1 = agent_0.y; 
        connection_3.x2 = agent_1.x;
        connection_3.y2 = agent_1.y;	
        
	connect_agent_1.x = agent_3.x;
	connect_agent_1.y = agent_3.y;

	connect_agent_2.x = agent_2.x;
	connect_agent_2.y = agent_2.y;

	connect_agent_3.x = agent_1.x;
	connect_agent_3.y = agent_1.y;

        distance_1 = distance[2];
        distance_2 = distance[1];
        distance_3 = distance[0];
    }

    if ((distance[2] <= distance[0]) && (distance[0] <= distance[1]))
    {
	connection_1.x1 = agent_0.x;     
	connection_1.y1 = agent_0.y; 
        connection_1.x2 = agent_3.x;
        connection_1.y2 = agent_3.y;  
        
        connection_2.x1 = agent_0.x;     
	connection_2.y1 = agent_0.y; 
        connection_2.x2 = agent_1.x;
        connection_2.y2 = agent_1.y;

        connection_3.x1 = agent_0.x;     
	connection_3.y1 = agent_0.y; 
        connection_3.x2 = agent_2.x;
        connection_3.y2 = agent_2.y;	
        
	connect_agent_1.x = agent_3.x;
	connect_agent_1.y = agent_3.y;

	connect_agent_2.x = agent_1.x;
	connect_agent_2.y = agent_1.y;

	connect_agent_3.x = agent_2.x;
	connect_agent_3.y = agent_2.y;

        distance_1 = distance[2];
        distance_2 = distance[0];
        distance_3 = distance[1];	
    }

    /*********************compute the enclosure region E1**********************/
	
    direction_vector_1 = findPerpenticularVector(connection_1);
    direction_vector_2 = findPerpenticularVector(connection_2);

    E1_segment_1.x1 = connect_agent_1.x;
    E1_segment_1.y1 = connect_agent_1.y;
    E1_segment_1.x2 = connect_agent_1.x + direction_vector_1.x;
    E1_segment_1.y2 = connect_agent_1.y + direction_vector_1.y;

    //ROS_INFO("E1_segment_1: %ld, %ld, %ld, %ld", E1_segment_1.x1, E1_segment_1.y1, E1_segment_1.x2, E1_segment_1.y2);

    E1_segment_1_intersection = findIntersection_rectangle(E1_segment_1);   /***This is 4-dimensional***/

    //ROS_INFO("E1_segment_1_intersection: %ld,%ld,%ld,%ld", E1_segment_1_intersection.x1, E1_segment_1_intersection.y1, E1_segment_1_intersection.x2, E1_segment_1_intersection.y2);

    E1_vertices[0] = E1_segment_1_intersection.x1;
    E1_vertices[1] = E1_segment_1_intersection.y1;
    E1_vertices[2] = E1_segment_1_intersection.x2;
    E1_vertices[3] = E1_segment_1_intersection.y2;

    E1_vertice_counter = 2;

    if (E1_segment_1.x1 != E1_segment_1.x2)
    { 
    	E1_segment_1_a = (E1_segment_1.y1 - E1_segment_1.y2)/(E1_segment_1.x1 - E1_segment_1.x2);
    	E1_segment_1_b = E1_segment_1.y1 - E1_segment_1_a*E1_segment_1.x1;
    }
    else
    {
    	E1_segment_1_a = 1;
    	E1_segment_1_b = 0 - E1_segment_1.x1;
	E1_segment_1_c = 0;
    }

    if ( sgn(E1_segment_1_a*vertice_1.x + E1_segment_1_b - vertice_1.y) == sgn(E1_segment_1_a*agent_0.x + E1_segment_1_b - agent_0.y) )
    {
	E1_vertices[E1_vertice_counter*2] = vertice_1.x;
	E1_vertices[E1_vertice_counter*2+1] = vertice_1.y;
	E1_vertice_counter = E1_vertice_counter + 1;
    }
 
    if ( sgn(E1_segment_1_a*vertice_2.x + E1_segment_1_b - vertice_2.y) == sgn(E1_segment_1_a*agent_0.x + E1_segment_1_b - agent_0.y) )
    {   
	E1_vertices[E1_vertice_counter*2] = vertice_2.x;
	E1_vertices[E1_vertice_counter*2+1] = vertice_2.y;
	E1_vertice_counter = E1_vertice_counter + 1;	
    }

    if ( sgn(E1_segment_1_a*vertice_3.x + E1_segment_1_b - vertice_3.y) == sgn(E1_segment_1_a*agent_0.x + E1_segment_1_b - agent_0.y) )
    { 
	E1_vertices[E1_vertice_counter*2] = vertice_3.x;
	E1_vertices[E1_vertice_counter*2+1] = vertice_3.y;
	E1_vertice_counter = E1_vertice_counter + 1;
    }

    if ( sgn(E1_segment_1_a*vertice_4.x + E1_segment_1_b - vertice_4.y) == sgn(E1_segment_1_a*agent_0.x + E1_segment_1_b - agent_0.y) )
    {
	E1_vertices[E1_vertice_counter*2] = vertice_4.x;
	E1_vertices[E1_vertice_counter*2+1] = vertice_4.y;
	E1_vertice_counter = E1_vertice_counter + 1;
    }

    /*************************compute the disk radius****************************/
    double radius[E1_vertice_counter] = {0};	
    double max_radius = -1;
	
    for (unsigned int i = 0; i < E1_vertice_counter; i++)
    {    
	radius[i] = sqrt((agent_0.x - E1_vertices[2*i])*(agent_0.x - E1_vertices[2*i]) + (agent_0.y - E1_vertices[2*i+1])*(agent_0.y - E1_vertices[2*i+1]));
        
        if (radius[i] > max_radius)
		max_radius = radius[i];
    }


    if (max_radius <= distance_2)   /*Agent 1 is the isolated neighbour of agent 0*/
    {
	//ROS_INFO("E1 found");
	mid_of_connection_1 = Cal_mid_point(agent_0, connect_agent_1);

        V1_segment_1.x1 = mid_of_connection_1.x;
        V1_segment_1.y1 = mid_of_connection_1.y;
        V1_segment_1.x2 = mid_of_connection_1.x + direction_vector_1.x;
        V1_segment_1.y2 = mid_of_connection_1.y + direction_vector_1.y;

        V1_segment_1_intersection = findIntersection_rectangle(V1_segment_1);   /*temp_1 is 4-dimensional*/

        V1_vertices[0] = V1_segment_1_intersection.x1;
        V1_vertices[1] = V1_segment_1_intersection.y1;
        V1_vertices[2] = V1_segment_1_intersection.x2;
        V1_vertices[3] = V1_segment_1_intersection.y2;

    	V1_vertice_counter = 2;

        if (V1_segment_1.x1 != V1_segment_1.x2)
    	{ 
    	    V1_segment_1_a = (V1_segment_1.y1 - V1_segment_1.y2)/(V1_segment_1.x1 - V1_segment_1.x2);
    	    V1_segment_1_b = V1_segment_1.y1 - V1_segment_1_a*V1_segment_1.x1;
    	}
    	else
    	{
    	    V1_segment_1_a = 1;
    	    V1_segment_1_b = 0 - V1_segment_1.x1;
	    	V1_segment_1_c = 0;
    	}

    	if ( sgn(V1_segment_1_a*vertice_1.x + V1_segment_1_b - vertice_1.y) == sgn(V1_segment_1_a*agent_0.x + V1_segment_1_b - agent_0.y) )
    	{
	     	V1_vertices[V1_vertice_counter*2] = vertice_1.x;
	     	V1_vertices[V1_vertice_counter*2+1] = vertice_1.y;
	     	V1_vertice_counter = V1_vertice_counter + 1;
    	}
 
        if ( sgn(V1_segment_1_a*vertice_2.x + V1_segment_1_b - vertice_2.y) == sgn(V1_segment_1_a*agent_0.x + V1_segment_1_b - agent_0.y) )
    	{   
	     	V1_vertices[V1_vertice_counter*2] = vertice_2.x;
	     	V1_vertices[V1_vertice_counter*2+1] = vertice_2.y;
	     	V1_vertice_counter = V1_vertice_counter + 1;	
    	}

    	if ( sgn(V1_segment_1_a*vertice_3.x + V1_segment_1_b - vertice_3.y) == sgn(V1_segment_1_a*agent_0.x + V1_segment_1_b - agent_0.y) )
    	{ 
	     	V1_vertices[V1_vertice_counter*2] = vertice_3.x;
	     	V1_vertices[V1_vertice_counter*2+1] = vertice_3.y;
	     	V1_vertice_counter = V1_vertice_counter + 1;
    	}

    	if ( sgn(V1_segment_1_a*vertice_4.x + V1_segment_1_b - vertice_4.y) == sgn(V1_segment_1_a*agent_0.x + V1_segment_1_b - agent_0.y) )
    	{
	     	V1_vertices[V1_vertice_counter*2] = vertice_4.x;
	     	V1_vertices[V1_vertice_counter*2+1] = vertice_4.y;
	     	V1_vertice_counter = V1_vertice_counter + 1;
    	}
    	
	
    }
    else if ( (max_radius > distance_2) && (max_radius <= distance_3) ) 
    {
	//ROS_INFO("E1 continue search");
	
	E1_vertices[20] = {0};
	E1_vertice_counter = 0;	

	direction_vector_2 = findPerpenticularVector(connection_2);

	E1_segment_2.x1 = connect_agent_2.x;
        E1_segment_2.y1 = connect_agent_2.y;
        E1_segment_2.x2 = connect_agent_2.x + direction_vector_2.x;
        E1_segment_2.y2 = connect_agent_2.y + direction_vector_2.y;

    	E1_segment_2_intersection = findIntersection_rectangle(E1_segment_2);       /*temp_2 is 4-dimensional*/

    	//ROS_INFO("E1_segment_2_intersection: %ld,%ld,%ld,%ld", E1_segment_2_intersection.x1, E1_segment_2_intersection.y1, E1_segment_2_intersection.x2, E1_segment_2_intersection.y2);

    	if (E1_segment_2.x1 != E1_segment_2.x2)
    	{ 
    	     E1_segment_2_a = (E1_segment_2.y1 - E1_segment_2.y2)/(E1_segment_2.x1 - E1_segment_2.x2);
    	     E1_segment_2_b = E1_segment_2.y1 - E1_segment_2_a*E1_segment_2.x1;
    	}
    	else
    	{
    	     E1_segment_2_a = 1;
             E1_segment_2_b = 0 - E1_segment_2.x1;
	     	 E1_segment_2_c = 0;
    	}

	s1_s2_inter_point = findIntersection1(E1_segment_1, E1_segment_2, 20, 4000, 20, 2800);

	if ((s1_s2_inter_point.x > 0) && (s1_s2_inter_point.y > 0))
	{
	    	E1_vertices[2*E1_vertice_counter] = s1_s2_inter_point.x;
	    	E1_vertices[2*E1_vertice_counter + 1] = s1_s2_inter_point.y;
	    	E1_vertice_counter = E1_vertice_counter + 1;
	}		

	if ((sgn(E1_segment_1_a*vertice_1.x + E1_segment_1_b - vertice_1.y) == sgn(E1_segment_1_a*agent_0.x + E1_segment_1_b - agent_0.y)) && (sgn(E1_segment_2_a*vertice_1.x + E1_segment_2_b - vertice_1.y) == sgn(E1_segment_2_a*agent_0.x + E1_segment_2_b - agent_0.y)))
    	{
	    	E1_vertices[2*E1_vertice_counter] = vertice_1.x;
	    	E1_vertices[2*E1_vertice_counter + 1] = vertice_1.y;
	    	E1_vertice_counter = E1_vertice_counter + 1;
    	}
 
	if ((sgn(E1_segment_1_a*vertice_2.x + E1_segment_1_b - vertice_2.y) == sgn(E1_segment_1_a*agent_0.x + E1_segment_1_b - agent_0.y)) && (sgn(E1_segment_2_a*vertice_2.x + E1_segment_2_b - vertice_2.y) == sgn(E1_segment_2_a*agent_0.x + E1_segment_2_b - agent_0.y)))
    	{   
	    	E1_vertices[2*E1_vertice_counter] = vertice_2.x;
	    	E1_vertices[2*E1_vertice_counter + 1] = vertice_2.y;
	    	E1_vertice_counter = E1_vertice_counter + 1;
    	}

	if ((sgn(E1_segment_1_a*vertice_3.x + E1_segment_1_b - vertice_3.y) == sgn(E1_segment_1_a*agent_0.x + E1_segment_1_b - agent_0.y)) && (sgn(E1_segment_2_a*vertice_3.x + E1_segment_2_b - vertice_3.y) == sgn(E1_segment_2_a*agent_0.x + E1_segment_2_b - agent_0.y)))
     	{ 
	    	E1_vertices[2*E1_vertice_counter] = vertice_3.x;
	    	E1_vertices[2*E1_vertice_counter + 1] = vertice_3.y;
	    	E1_vertice_counter = E1_vertice_counter + 1;
    	}

	if ((sgn(E1_segment_1_a*vertice_4.x + E1_segment_1_b - vertice_4.y) == sgn(E1_segment_1_a*agent_0.x + E1_segment_1_b - agent_0.y)) && (sgn(E1_segment_2_a*vertice_4.x + E1_segment_2_b - vertice_4.y) == sgn(E1_segment_2_a*agent_0.x + E1_segment_2_b - agent_0.y)))
    	{
	    	E1_vertices[2*E1_vertice_counter] = vertice_4.x;
	    	E1_vertices[2*E1_vertice_counter + 1] = vertice_4.y;
	    	E1_vertice_counter = E1_vertice_counter + 1;
    	}


    	if (E1_segment_1_a*agent_0.x + E1_segment_1_b - agent_0.y > 0)
    	{
	    	if (E1_segment_1_a*E1_segment_2_intersection.x1 + E1_segment_1_b - E1_segment_2_intersection.y1 > 0)
	    	{				
	    		E1_vertices[2*E1_vertice_counter] = E1_segment_2_intersection.x1;
	    		E1_vertices[2*E1_vertice_counter + 1] = E1_segment_2_intersection.y1;
	    		E1_vertice_counter = E1_vertice_counter + 1;
	    	}

	    	if (E1_segment_1_a*E1_segment_2_intersection.x2 + E1_segment_1_b - E1_segment_2_intersection.y2 > 0)
	    	{	
	    		E1_vertices[2*E1_vertice_counter] = E1_segment_2_intersection.x2;
	    		E1_vertices[2*E1_vertice_counter + 1] = E1_segment_2_intersection.y2;
	    		E1_vertice_counter = E1_vertice_counter + 1;
	    	}
    	}    
    	else if (E1_segment_1_a*agent_0.x + E1_segment_1_b - agent_0.y < 0)
    	{
            if (E1_segment_1_a*E1_segment_2_intersection.x1 + E1_segment_1_b - E1_segment_2_intersection.y1 < 0)
	    	{	
	    		E1_vertices[2*E1_vertice_counter] = E1_segment_2_intersection.x1;
	    		E1_vertices[2*E1_vertice_counter + 1] = E1_segment_2_intersection.y1;
	    		E1_vertice_counter = E1_vertice_counter + 1;
	    	}
	    	if (E1_segment_1_a*E1_segment_2_intersection.x2 + E1_segment_1_b - E1_segment_2_intersection.y2 < 0)
	    	{	
	    		E1_vertices[2*E1_vertice_counter] = E1_segment_2_intersection.x2;
	    		E1_vertices[2*E1_vertice_counter + 1] = E1_segment_2_intersection.y2;
	    		E1_vertice_counter = E1_vertice_counter + 1;
	    	}
    	}
   

    	if (E1_segment_2_a*agent_0.x + E1_segment_2_b - agent_0.y > 0)
    	{
       	    if (E1_segment_2_a*E1_segment_1_intersection.x1 + E1_segment_2_b - E1_segment_2_intersection.y1 > 0)
       	    {	
	    		E1_vertices[2*E1_vertice_counter] = E1_segment_1_intersection.x1;
	    		E1_vertices[2*E1_vertice_counter + 1] = E1_segment_1_intersection.y1;
	    		E1_vertice_counter = E1_vertice_counter + 1;

       	    }
       	    if (E1_segment_2_a*E1_segment_1_intersection.x2 + E1_segment_2_b - E1_segment_1_intersection.y2 > 0)
       	    {	
	    		E1_vertices[2*E1_vertice_counter] = E1_segment_1_intersection.x2;
	    		E1_vertices[2*E1_vertice_counter + 1] = E1_segment_1_intersection.y2;
	    		E1_vertice_counter = E1_vertice_counter + 1;
       	    }
    	}	   
    	else if (E1_segment_2_a*agent_0.x + E1_segment_2_b - agent_0.y < 0)
    	{
       	    if (E1_segment_2_a*E1_segment_1_intersection.x1 + E1_segment_2_b - E1_segment_1_intersection.y1 < 0)
       	    {     
	    		E1_vertices[2*E1_vertice_counter] = E1_segment_1_intersection.x1;
	    		E1_vertices[2*E1_vertice_counter + 1] = E1_segment_1_intersection.y1;
	    		E1_vertice_counter = E1_vertice_counter + 1;
       	    }
       	    if (E1_segment_2_a*E1_segment_1_intersection.x2 + E1_segment_2_b - E1_segment_1_intersection.y2 < 0)
       	    {
	    		E1_vertices[2*E1_vertice_counter] = E1_segment_1_intersection.x2;
	    		E1_vertices[2*E1_vertice_counter + 1] = E1_segment_1_intersection.y2;
	    		E1_vertice_counter = E1_vertice_counter + 1;
	     	}
    	}

    	/*
    	for (unsigned int temp_E_i = 0; temp_E_i < E1_vertice_counter; temp_E_i++)
    	{
    		ROS_INFO("E1_vertices[%d] coordinates: %ld, %ld", temp_E_i, E1_vertices[2*temp_E_i], E1_vertices[2*temp_E_i + 1]);
    	}
		*/
    	/*************************compute the disk radius again****************************/
    	double radius_temp[E1_vertice_counter] = {0};	
    	double max_radius_temp = -1;
	
    	for (unsigned int i = 0; i < E1_vertice_counter; i++)
    	{    
	    radius_temp[i] = sqrt((agent_0.x - E1_vertices[2*i])*(agent_0.x - E1_vertices[2*i]) + (agent_0.y - E1_vertices[2*i+1])*(agent_0.y - E1_vertices[2*i+1]));
            if (radius_temp[i] > max_radius_temp)
		max_radius_temp = radius_temp[i];
    	}

        if ( (max_radius_temp > distance_2) && (max_radius <= distance_3) )
	{

            V1_vertices[20] = {0};
    	    V1_vertice_counter = 0;

	    mid_of_connection_1 = Cal_mid_point(agent_0, connect_agent_1);
	    mid_of_connection_2 = Cal_mid_point(agent_0, connect_agent_2);

            V1_segment_1.x1 = mid_of_connection_1.x;
            V1_segment_1.y1 = mid_of_connection_1.y;
            V1_segment_1.x2 = mid_of_connection_1.x + direction_vector_1.x;
            V1_segment_1.y2 = mid_of_connection_1.y + direction_vector_1.y;

            V1_segment_2.x1 = mid_of_connection_2.x;
            V1_segment_2.y1 = mid_of_connection_2.y;
            V1_segment_2.x2 = mid_of_connection_2.x + direction_vector_2.x;
            V1_segment_2.y2 = mid_of_connection_2.y + direction_vector_2.y;

	    V1_segment_1_intersection = findIntersection_rectangle(V1_segment_1);   /*V1_segment_1_intersection is 4-dimensional*/

	    V1_segment_2_intersection = findIntersection_rectangle(V1_segment_2);   /*V1_segment_1_intersection is 4-dimensional*/

	    if (V1_segment_1.x1 != V1_segment_1.x2)
	    { 
	    	V1_segment_1_a = (V1_segment_1.y1 - V1_segment_1.y2)/(V1_segment_1.x1 - V1_segment_1.x2);
	    	V1_segment_1_b = V1_segment_1.y1 - V1_segment_1_a*V1_segment_1.x1;
	    }
	    else
	    {
	    	V1_segment_1_a = 1;
	    	V1_segment_1_b = 0 - V1_segment_2.x1;
		V1_segment_1_c = 0;
	    }

	    if (V1_segment_2.x1 != V1_segment_2.x2)
	    { 
	    	V1_segment_2_a = (V1_segment_2.y1 - V1_segment_2.y2)/(V1_segment_2.x1 - V1_segment_2.x2);
	    	V1_segment_2_b = V1_segment_2.y1 - V1_segment_2_a*V1_segment_2.x1;
	    }
	    else
	    {
	    	V1_segment_2_a = 1;
	    	V1_segment_2_b = 0 - V1_segment_2.x1;
		V1_segment_2_c = 0;
	    }

	    if ( (sgn(V1_segment_1_a*vertice_1.x + V1_segment_1_b - vertice_1.y) == sgn(V1_segment_1_a*agent_0.x + V1_segment_1_b - agent_0.y) ) && (sgn(V1_segment_2_a*vertice_1.x + V1_segment_2_b - vertice_1.y) == sgn(V1_segment_2_a*agent_0.x + V1_segment_2_b - agent_0.y)) )
	    {
		V1_vertices[V1_vertice_counter*2] = vertice_1.x;
		V1_vertices[V1_vertice_counter*2+1] = vertice_1.y;
		V1_vertice_counter = V1_vertice_counter + 1;
	    }
	 
	    if ( (sgn(V1_segment_1_a*vertice_2.x + V1_segment_1_b - vertice_2.y) == sgn(V1_segment_1_a*agent_0.x + V1_segment_1_b - agent_0.y) ) && (sgn(V1_segment_2_a*vertice_2.x + V1_segment_2_b - vertice_2.y) == sgn(V1_segment_2_a*agent_0.x + V1_segment_2_b - agent_0.y)) )
	    {   
		V1_vertices[V1_vertice_counter*2] = vertice_2.x;
		V1_vertices[V1_vertice_counter*2+1] = vertice_2.y;
		V1_vertice_counter = V1_vertice_counter + 1;	
	    }

	    if ( (sgn(V1_segment_1_a*vertice_3.x + V1_segment_1_b - vertice_3.y) == sgn(V1_segment_1_a*agent_0.x + V1_segment_1_b - agent_0.y) ) && (sgn(V1_segment_2_a*vertice_3.x + V1_segment_2_b - vertice_3.y) == sgn(V1_segment_2_a*agent_0.x + V1_segment_2_b - agent_0.y)) )
	    { 
		V1_vertices[V1_vertice_counter*2] = vertice_3.x;
		V1_vertices[V1_vertice_counter*2+1] = vertice_3.y;
		V1_vertice_counter = V1_vertice_counter + 1;
	    }

	    if ( (sgn(V1_segment_1_a*vertice_4.x + V1_segment_1_b - vertice_4.y) == sgn(V1_segment_1_a*agent_0.x + V1_segment_1_b - agent_0.y) ) && (sgn(V1_segment_2_a*vertice_4.x + V1_segment_2_b - vertice_4.y) == sgn(V1_segment_2_a*agent_0.x + V1_segment_2_b - agent_0.y)) )
	    {
		V1_vertices[V1_vertice_counter*2] = vertice_4.x;
		V1_vertices[V1_vertice_counter*2+1] = vertice_4.y;
		V1_vertice_counter = V1_vertice_counter + 1;
	    }

    	    if (V1_segment_1_a*agent_0.x + V1_segment_1_b - agent_0.y > 0)
    	    {
	    	if (V1_segment_1_a*V1_segment_2_intersection.x1 + V1_segment_1_b - V1_segment_2_intersection.y1 > 0)
	    	{				
	    	    V1_vertices[2*V1_vertice_counter] = V1_segment_2_intersection.x1;
	    	    V1_vertices[2*V1_vertice_counter + 1] = V1_segment_2_intersection.y1;
	    	    V1_vertice_counter = V1_vertice_counter + 1;
	    	}

	    	if (V1_segment_1_a*V1_segment_2_intersection.x2 + V1_segment_1_b - V1_segment_2_intersection.y2 > 0)
	        {	
	    	    V1_vertices[2*V1_vertice_counter] = V1_segment_2_intersection.x2;
	    	    V1_vertices[2*V1_vertice_counter + 1] = V1_segment_2_intersection.y2;
	    	    V1_vertice_counter = V1_vertice_counter + 1;
	    	}
    	    }    
    	    else if (V1_segment_1_a*agent_0.x + V1_segment_1_b - agent_0.y < 0)
    	    {
            	if (V1_segment_1_a*V1_segment_2_intersection.x1 + V1_segment_1_b - V1_segment_2_intersection.y1 < 0)
	    	{	
	    	    V1_vertices[2*V1_vertice_counter] = V1_segment_2_intersection.x1;
	    	    V1_vertices[2*V1_vertice_counter + 1] = V1_segment_2_intersection.y1;
	    	    V1_vertice_counter = V1_vertice_counter + 1;
	    	}
	    	if (V1_segment_1_a*V1_segment_2_intersection.x2 + V1_segment_1_b - V1_segment_2_intersection.y2 < 0)
	    	{	
	    	    V1_vertices[2*V1_vertice_counter] = V1_segment_2_intersection.x2;
	    	    V1_vertices[2*E1_vertice_counter + 1] = V1_segment_2_intersection.y2;
	    	    V1_vertice_counter = V1_vertice_counter + 1;
	    	}
    	    }
   

    	    if (V1_segment_2_a*agent_0.x + V1_segment_2_b - agent_0.y > 0)
    	    {
       	    	if (V1_segment_2_a*V1_segment_1_intersection.x1 + V1_segment_2_b - V1_segment_2_intersection.y1 > 0)
       	    	{	
	    	    V1_vertices[2*V1_vertice_counter] = V1_segment_1_intersection.x1;
	    	    V1_vertices[2*V1_vertice_counter + 1] = V1_segment_1_intersection.y1;
	    	    V1_vertice_counter = V1_vertice_counter + 1;
       	    	}
       	   	if (V1_segment_2_a*V1_segment_1_intersection.x2 + V1_segment_2_b - V1_segment_1_intersection.y2 > 0)
       	    	{	
	    	    V1_vertices[2*V1_vertice_counter] = V1_segment_1_intersection.x2;
	    	    V1_vertices[2*V1_vertice_counter + 1] = V1_segment_1_intersection.y2;
	    	    V1_vertice_counter = V1_vertice_counter + 1;
       	    	}
    	    }	   
    	    else if (V1_segment_2_a*agent_0.x + V1_segment_2_b - agent_0.y < 0)
    	    {
       	    	if (V1_segment_2_a*V1_segment_1_intersection.x1 + V1_segment_2_b - V1_segment_1_intersection.y1 < 0)
       	    	{     
	    	    V1_vertices[2*V1_vertice_counter] = V1_segment_1_intersection.x1;
	    	    V1_vertices[2*V1_vertice_counter + 1] = V1_segment_1_intersection.y1;
	    	    V1_vertice_counter = V1_vertice_counter + 1;
       	    	}
       	    	if (V1_segment_2_a*V1_segment_1_intersection.x2 + V1_segment_2_b - V1_segment_1_intersection.y2 < 0)
       	    	{
	    	    V1_vertices[2*V1_vertice_counter] = V1_segment_1_intersection.x2;
	    	    V1_vertices[2*V1_vertice_counter + 1] = V1_segment_1_intersection.y2;
	            V1_vertice_counter = V1_vertice_counter + 1;
	     	}
    	    }

    	    V1_s1_s2_inter_point = findIntersection1(V1_segment_1, V1_segment_2, 20, 4000, 20, 2800);

    	    if ((V1_s1_s2_inter_point.x != 0) && (V1_s1_s2_inter_point.y != 0))
    	    {
		V1_vertices[2*V1_vertice_counter] = V1_s1_s2_inter_point.x;				    	    	
		V1_vertices[2*V1_vertice_counter + 1] = V1_s1_s2_inter_point.y;
		V1_vertice_counter = V1_vertice_counter + 1;
    	    }
    	}

	}
	else if ( max_radius > distance_3 )    /*compute voronoi area with three neighbours*/
	{

		//ROS_INFO("complete graph");
    		V1_vertices[20] = {0};
    		V1_vertice_counter = 0;

		direction_vector_3 = findPerpenticularVector(connection_3);

	    	mid_of_connection_1 = Cal_mid_point(agent_0, connect_agent_1);
	    	mid_of_connection_2 = Cal_mid_point(agent_0, connect_agent_2);
	    	mid_of_connection_3 = Cal_mid_point(agent_0, connect_agent_3);

        	V1_segment_1.x1 = mid_of_connection_1.x;
        	V1_segment_1.y1 = mid_of_connection_1.y;
        	V1_segment_1.x2 = mid_of_connection_1.x + direction_vector_1.x;
        	V1_segment_1.y2 = mid_of_connection_1.y + direction_vector_1.y;

        	//ROS_INFO("V1_segment_1: %ld, %ld, %ld,%ld", V1_segment_1.x1, V1_segment_1.y1, V1_segment_1.x2, V1_segment_1.y2);

        	V1_segment_2.x1 = mid_of_connection_2.x;
        	V1_segment_2.y1 = mid_of_connection_2.y;
        	V1_segment_2.x2 = mid_of_connection_2.x + direction_vector_2.x;
        	V1_segment_2.y2 = mid_of_connection_2.y + direction_vector_2.y;

        	//ROS_INFO("V1_segment_2: %ld, %ld, %ld,%ld", V1_segment_2.x1, V1_segment_2.y1, V1_segment_2.x2, V1_segment_2.y2);

        	V1_segment_3.x1 = mid_of_connection_3.x;
        	V1_segment_3.y1 = mid_of_connection_3.y;
        	V1_segment_3.x2 = mid_of_connection_3.x + direction_vector_3.x;
        	V1_segment_3.y2 = mid_of_connection_3.y + direction_vector_3.y;

        	//ROS_INFO("V1_segment_3: %ld, %ld, %ld,%ld", V1_segment_3.x1, V1_segment_3.y1, V1_segment_3.x2, V1_segment_3.y2);

	    	if (V1_segment_1.x1 != V1_segment_1.x2)
	    	{ 
	    		V1_segment_1_a = (double)(V1_segment_1.y1 - V1_segment_1.y2)/(V1_segment_1.x1 - V1_segment_1.x2);
	    		V1_segment_1_b = (double)(V1_segment_1.y1 - V1_segment_1_a*V1_segment_1.x1);
	    	}
	    	else
	    	{
	    		V1_segment_1_a = 1;
	    		V1_segment_1_b = 0 - V1_segment_2.x1;
			V1_segment_1_c = 0;
	    	}

	    	if (V1_segment_2.x1 != V1_segment_2.x2)
	    	{ 
	    		V1_segment_2_a = (double)(V1_segment_2.y1 - V1_segment_2.y2)/(V1_segment_2.x1 - V1_segment_2.x2);
	    		V1_segment_2_b = (double)(V1_segment_2.y1 - V1_segment_2_a*V1_segment_2.x1);
	    	}
	    	else
	    	{
	    		V1_segment_2_a = 1;
	    		V1_segment_2_b = 0 - V1_segment_2.x1;
			V1_segment_2_c = 0;
	    	}

	    	//ROS_INFO("V1_segment_2_a: %f", V1_segment_2_a);

	    	if (V1_segment_3.x1 != V1_segment_3.x2)
	    	{ 
	    		V1_segment_3_a = (double)(V1_segment_3.y1 - V1_segment_3.y2)/(V1_segment_3.x1 - V1_segment_3.x2);
	    		V1_segment_3_b = (double)(V1_segment_3.y1 - V1_segment_3_a*V1_segment_3.x1);
	    	}
	    	else
	    	{
	    		V1_segment_3_a = 1;
	    		V1_segment_3_b = 0 - V1_segment_3.x1;
			V1_segment_3_c = 0;
	    	}

	    	V1_segment_1_intersection = findIntersection_rectangle(V1_segment_1);   /*V1_segment_1_intersection is 4-dimensional*/

	    	V1_segment_2_intersection = findIntersection_rectangle(V1_segment_2);   /*V1_segment_2_intersection is 4-dimensional*/

	    	V1_segment_3_intersection = findIntersection_rectangle(V1_segment_3);   /*V1_segment_2_intersection is 4-dimensional*/

	    //ROS_INFO("V1_segment_1_intersection: %ld, %ld, %ld, %ld", V1_segment_1_intersection.x1, V1_segment_1_intersection.y1, V1_segment_1_intersection.x2, V1_segment_1_intersection.y2);
	    //ROS_INFO("V1_segment_2_intersection: %ld, %ld, %ld, %ld", V1_segment_2_intersection.x1, V1_segment_2_intersection.y1, V1_segment_2_intersection.x2, V1_segment_2_intersection.y2);
	    //ROS_INFO("V1_segment_3_intersection: %ld, %ld, %ld, %ld", V1_segment_3_intersection.x1, V1_segment_3_intersection.y1, V1_segment_3_intersection.x2, V1_segment_3_intersection.y2);

	    if ( (sgn(V1_segment_1_a*vertice_1.x + V1_segment_1_b - vertice_1.y) == sgn(V1_segment_1_a*agent_0.x + V1_segment_1_b - agent_0.y) ) && (sgn(V1_segment_2_a*vertice_1.x + V1_segment_2_b - vertice_1.y) == sgn(V1_segment_2_a*agent_0.x + V1_segment_2_b - agent_0.y)) && (sgn(V1_segment_3_a*vertice_1.x + V1_segment_3_b - vertice_1.y) == sgn(V1_segment_3_a*agent_0.x + V1_segment_3_b - agent_0.y) ))
	    {
		V1_vertices[V1_vertice_counter*2] = vertice_1.x;
		V1_vertices[V1_vertice_counter*2+1] = vertice_1.y;
		V1_vertice_counter = V1_vertice_counter + 1;
	    }
	 
	    if ( (sgn(V1_segment_1_a*vertice_2.x + V1_segment_1_b - vertice_2.y) == sgn(V1_segment_1_a*agent_0.x + V1_segment_1_b - agent_0.y) ) && (sgn(V1_segment_2_a*vertice_2.x + V1_segment_2_b - vertice_2.y) == sgn(V1_segment_2_a*agent_0.x + V1_segment_2_b - agent_0.y)) && (sgn(V1_segment_3_a*vertice_2.x + V1_segment_3_b - vertice_2.y) == sgn(V1_segment_3_a*agent_0.x + V1_segment_3_b - agent_0.y) ))
	    {   
		V1_vertices[V1_vertice_counter*2] = vertice_2.x;
		V1_vertices[V1_vertice_counter*2+1] = vertice_2.y;
		V1_vertice_counter = V1_vertice_counter + 1;	
	    }

	    if ( (sgn(V1_segment_1_a*vertice_3.x + V1_segment_1_b - vertice_3.y) == sgn(V1_segment_1_a*agent_0.x + V1_segment_1_b - agent_0.y) ) && (sgn(V1_segment_2_a*vertice_3.x + V1_segment_2_b - vertice_3.y) == sgn(V1_segment_2_a*agent_0.x + V1_segment_2_b - agent_0.y)) && (sgn(V1_segment_3_a*vertice_3.x + V1_segment_3_b - vertice_3.y) == sgn(V1_segment_3_a*agent_0.x + V1_segment_3_b - agent_0.y) ) )
	    { 
		V1_vertices[V1_vertice_counter*2] = vertice_3.x;
		V1_vertices[V1_vertice_counter*2+1] = vertice_3.y;
		V1_vertice_counter = V1_vertice_counter + 1;
	    }

	    if ( (sgn(V1_segment_1_a*vertice_4.x + V1_segment_1_b - vertice_4.y) == sgn(V1_segment_1_a*agent_0.x + V1_segment_1_b - agent_0.y) ) && (sgn(V1_segment_2_a*vertice_4.x + V1_segment_2_b - vertice_4.y) == sgn(V1_segment_2_a*agent_0.x + V1_segment_2_b - agent_0.y)) && (sgn(V1_segment_3_a*vertice_4.x + V1_segment_3_b - vertice_4.y) == sgn(V1_segment_3_a*agent_0.x + V1_segment_3_b - agent_0.y) ))
	    {
		V1_vertices[V1_vertice_counter*2] = vertice_4.x;
		V1_vertices[V1_vertice_counter*2+1] = vertice_4.y;
		V1_vertice_counter = V1_vertice_counter + 1;
	    }

	    	
	    sgn_1 = sgn(V1_segment_1_a*agent_0.x + V1_segment_1_b - agent_0.y);
	    sgn_2 = sgn(V1_segment_2_a*agent_0.x + V1_segment_2_b - agent_0.y);
	    sgn_3 = sgn(V1_segment_3_a*agent_0.x + V1_segment_3_b - agent_0.y);

	    sgn_1_s_3_1 = sgn(V1_segment_1_a*V1_segment_3_intersection.x1 + V1_segment_1_b - V1_segment_3_intersection.y1); 
	    sgn_1_s_3_2 = sgn(V1_segment_1_a*V1_segment_3_intersection.x2 + V1_segment_1_b - V1_segment_3_intersection.y2); 
	    sgn_2_s_3_1 = sgn(V1_segment_2_a*V1_segment_3_intersection.x1 + V1_segment_2_b - V1_segment_3_intersection.y1); 
	    sgn_2_s_3_2 = sgn(V1_segment_2_a*V1_segment_3_intersection.x2 + V1_segment_2_b - V1_segment_3_intersection.y2);

	    sgn_1_s_2_1 = sgn(V1_segment_1_a*V1_segment_2_intersection.x1 + V1_segment_1_b - V1_segment_2_intersection.y1);
	    sgn_1_s_2_2 = sgn(V1_segment_1_a*V1_segment_2_intersection.x2 + V1_segment_1_b - V1_segment_2_intersection.y2);
	    sgn_3_s_2_1 = sgn(V1_segment_3_a*V1_segment_2_intersection.x1 + V1_segment_3_b - V1_segment_2_intersection.y1);
	    sgn_3_s_2_2 = sgn(V1_segment_3_a*V1_segment_2_intersection.x2 + V1_segment_3_b - V1_segment_2_intersection.y2);

	    sgn_2_s_1_1 = sgn(V1_segment_2_a*V1_segment_1_intersection.x1 + V1_segment_2_b - V1_segment_1_intersection.y1);
	    sgn_2_s_1_2 = sgn(V1_segment_2_a*V1_segment_1_intersection.x2 + V1_segment_2_b - V1_segment_1_intersection.y2);
	    sgn_3_s_1_1 = sgn(V1_segment_3_a*V1_segment_1_intersection.x1 + V1_segment_3_b - V1_segment_1_intersection.y1);
	    sgn_3_s_1_2 = sgn(V1_segment_3_a*V1_segment_1_intersection.x2 + V1_segment_3_b - V1_segment_1_intersection.y2);
		

	    /*
	    ROS_INFO("sgn_1:%d", sgn_1);
	    ROS_INFO("sgn_2:%d", sgn_2);
	    ROS_INFO("sgn_3:%d", sgn_3);

	    ROS_INFO("sgn_1_s_3_1:%d", sgn_1_s_3_1);
	    ROS_INFO("sgn_1_s_3_2:%d", sgn_1_s_3_2);
	    ROS_INFO("sgn_2_s_3_1:%d", sgn_2_s_3_1);
	    ROS_INFO("sgn_2_s_3_2:%d", sgn_2_s_3_2);

	    ROS_INFO("sgn_1_s_2_1:%d", sgn_1_s_2_1);
	    ROS_INFO("sgn_1_s_2_2:%d", sgn_1_s_2_2);
	    ROS_INFO("sgn_3_s_2_1:%d", sgn_3_s_2_1);	    
	    ROS_INFO("sgn_3_s_2_2:%d", sgn_3_s_2_2);

	    ROS_INFO("sgn_2_s_1_1:%d", sgn_2_s_1_1);
	    ROS_INFO("sgn_2_s_1_2:%d", sgn_2_s_1_2);
	    ROS_INFO("sgn_3_s_1_1:%d", sgn_3_s_1_1);
	    ROS_INFO("sgn_3_s_1_2:%d", sgn_3_s_1_2);
		*/

	    if ( (sgn_1 == sgn_1_s_3_1) && (sgn_2 == sgn_2_s_3_1) )
    	    {
	    	V1_vertices[2*V1_vertice_counter] = V1_segment_3_intersection.x1;
	    	V1_vertices[2*V1_vertice_counter + 1] = V1_segment_3_intersection.y1;
	    	V1_vertice_counter = V1_vertice_counter + 1;
    	    }
   
	    if ( (sgn_1 == sgn_1_s_3_2) && (sgn_2 == sgn_2_s_3_2) )
    	    {
	    	V1_vertices[2*V1_vertice_counter] = V1_segment_3_intersection.x2;
	    	V1_vertices[2*V1_vertice_counter + 1] = V1_segment_3_intersection.y2;
	    	V1_vertice_counter = V1_vertice_counter + 1;
    	    }

    	    if ( (sgn_1 == sgn_1_s_2_1) && (sgn_3 == sgn_3_s_2_1) )
    	    {
	    	V1_vertices[2*V1_vertice_counter] = V1_segment_2_intersection.x1;
	    	V1_vertices[2*V1_vertice_counter + 1] = V1_segment_2_intersection.y1;
	    	V1_vertice_counter = V1_vertice_counter + 1;
    	    }
   
	    if ( (sgn_1 == sgn_1_s_2_2) && (sgn_3 == sgn_3_s_2_2) )
    	    {
	    	V1_vertices[2*V1_vertice_counter] = V1_segment_2_intersection.x2;
	    	V1_vertices[2*V1_vertice_counter + 1] = V1_segment_2_intersection.y2;
	    	V1_vertice_counter = V1_vertice_counter + 1;
    	    }

    	    if ( (sgn_2 == sgn_2_s_1_1) && (sgn_3 == sgn_3_s_1_1) )
    	    {
	    	V1_vertices[2*V1_vertice_counter] = V1_segment_1_intersection.x1;
	    	V1_vertices[2*V1_vertice_counter + 1] = V1_segment_1_intersection.y1;
	    	V1_vertice_counter = V1_vertice_counter + 1;
    	    }
   
	    if ( (sgn_2 == sgn_2_s_1_2) && (sgn_3 == sgn_3_s_1_2) )
    	    {
	    	V1_vertices[2*V1_vertice_counter] = V1_segment_1_intersection.x2;
	    	V1_vertices[2*V1_vertice_counter + 1] = V1_segment_1_intersection.y2;
	    	V1_vertice_counter = V1_vertice_counter + 1;
    	    }

    	    V1_s1_s2_inter_point = findIntersection1(V1_segment_1, V1_segment_2, 20, 4000, 20, 2800);
    	    V1_s2_s3_inter_point = findIntersection1(V1_segment_2, V1_segment_3, 20, 4000, 20, 2800);
    	    V1_s1_s3_inter_point = findIntersection1(V1_segment_1, V1_segment_3, 20, 4000, 20, 2800);

    	    //ROS_INFO("V1_s1_s2_inter_point: %ld, %ld", V1_s1_s2_inter_point.x, V1_s1_s2_inter_point.y);
    	    //ROS_INFO("V1_s2_s3_inter_point: %ld, %ld", V1_s2_s3_inter_point.x, V1_s2_s3_inter_point.y);
    	    //ROS_INFO("V1_s1_s3_inter_point: %ld, %ld", V1_s1_s3_inter_point.x, V1_s1_s3_inter_point.y);

	    sgn_1_23_intersection = sgn(V1_segment_1_a*V1_s2_s3_inter_point.x + V1_segment_1_b - V1_s2_s3_inter_point.y); 
	    sgn_2_13_intersection = sgn(V1_segment_2_a*V1_s1_s3_inter_point.x + V1_segment_2_b - V1_s1_s3_inter_point.y);
	    sgn_3_12_intersection = sgn(V1_segment_3_a*V1_s1_s2_inter_point.x + V1_segment_3_b - V1_s1_s2_inter_point.y);

	    if ( sgn_1 == sgn_1_23_intersection )
	    {
	    	V1_vertices[2*V1_vertice_counter] = V1_s2_s3_inter_point.x;
	    	V1_vertices[2*V1_vertice_counter + 1] = V1_s2_s3_inter_point.y;
	    	V1_vertice_counter = V1_vertice_counter + 1;	    	
	    }
   	
	    if ( sgn_2 == sgn_2_13_intersection )
	    {
	    	V1_vertices[2*V1_vertice_counter] = V1_s1_s3_inter_point.x;
	    	V1_vertices[2*V1_vertice_counter + 1] = V1_s1_s3_inter_point.y;
	    	V1_vertice_counter = V1_vertice_counter + 1;	    	
	    }

	    if ( sgn_3 == sgn_3_12_intersection )
	    {
	    	V1_vertices[2*V1_vertice_counter] = V1_s1_s2_inter_point.x;
	    	V1_vertices[2*V1_vertice_counter + 1] = V1_s1_s2_inter_point.y;
	    	V1_vertice_counter = V1_vertice_counter + 1;	    	
	    }	    


	} 

    	ClockwiseSortPoints(V1_vertices, V1_vertice_counter);

    	long V1_vertice_x[V1_vertice_counter] = {0};
    	long V1_vertice_y[V1_vertice_counter] = {0};

    	for (unsigned int i = 0; i < V1_vertice_counter; i += 1)
    	{
		V1_vertice_x[i] = V1_vertices[2*i];
		V1_vertice_y[i] = V1_vertices[2*i + 1];	
    	}

    	centroid_1 = computeCentroid(V1_vertice_x, V1_vertice_y, V1_vertice_counter);
	
    //ROS_INFO("Centroid_1: %ld, %ld", centroid_1.x, centroid_1.y);
    /*
    for (unsigned int temp_i = 0; temp_i < V1_vertice_counter; temp_i++)
    {
    	ROS_INFO("V1_vertices[%d] coordinates: %ld, %ld", temp_i, V1_vertices[2*temp_i], V1_vertices[2*temp_i+1]);
    }
	
    voronoi_out.x1 = V1_vertices[0];
    voronoi_out.y1 = V1_vertices[1];
    voronoi_out.x2 = V1_vertices[2];
    voronoi_out.y2 = V1_vertices[3];
    voronoi_out.x3 = V1_vertices[4];
    voronoi_out.y3 = V1_vertices[5];
    voronoi_out.x4 = V1_vertices[6];
    voronoi_out.y4 = V1_vertices[7];
    voronoi_out.x5 = V1_vertices[8];
    voronoi_out.y5 = V1_vertices[9];
    voronoi_out.x6 = V1_vertices[10];
    voronoi_out.y6 = V1_vertices[11];
    */
	
}

void JoyCallback(const geometry_msgs::Twist& input)
{
   int plus;
   int minus;
   float temp1;
   float temp2;
   //ROS_INFO("%.2f, %.2f\r\n", input.linear.x, input.angular.z);
   //ROS_INFO("I heard:");

   temp1 = (float)input.linear.x;
   temp2 = (float)input.angular.z;
  
   if(( temp1 == 2.00 ) && ( u[0] < 20.0))
   {
	u[0] = u[0] + 1.0;	
   }
   if((temp1== -2.00) && (u[0] > 10.0))
   {
 	u[0] = u[0] - 1.0; 
   }
   
   if((temp1 == 2.00) && ( u[0] == 20.0))
   {
   	u[0] = 20.00; 
   }
   if((temp1 == -2.00) && (u[0] == 10.0))
   {
 	u[0] = 10.00; 
   }

   if((temp2 == 2.00)) 
   {
 	u[0] = 0.00; 
	ROS_INFO("Stop");
   }
	
}

void Translate(const qualisys::Subject& center)
{

/************************************************************************Obtain the system time******************************************************************************/
    double secs, time_t;

    secs = ros::Time::now().toSec(); 

    if (k==0)
    {
	time_init = secs;
	//ROS_INFO("a");
	//printf("time_t:%f\n", time_init);
    }

    time_t = secs - time_init;

    //printf("time_t:%f\n", time_t);

/*************************************************************************compute virtual center*****************************************************************************/
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

    agent_0.x = (long)z_x;
    agent_0.y = (long)z_y;

/************************************************************************compute voronoi area*********************************************************************************/

    voronoi_area_compute(agent_0, agent_1, agent_2, agent_3);

/********************************************************************************switching control*****************************************************************************/

/*
    k = k + 1; //a temp counter

    if (k == 50)
    {
	v = sqrt((center.position.x - temp_x)*(center.position.x - temp_x)*1000000 + (center.position.y - temp_y)*(center.position.y - temp_y)*1000000)/0.5;
	temp_x = center.position.x;
    	temp_y = center.position.y;
	//printf("speed=%f\n", v);
	k = 0;
    }	 
*/   

    double temp_c;
    temp_c = (omega_0 + 0.00001*omega_0*((z_x - (double)centroid_1.x)*162*cos(alpha+1.5707963) + (z_y - (double)centroid_1.y)*162*sin(alpha+1.5707963)))/3.1415926*180;
    //temp_c = (omega_0 + 0.00001*omega_0*((z_x - 2000.0)*162*cos(alpha+1.5707963) + (z_y - 2000.0)*162*sin(alpha+1.5707963)))/3.1415926*180;


    if (k == 0)
	temp_c_t = temp_c;

    double error, threshold;

    error = abs(temp_c - temp_c_t);

    threshold = abs(0.8*(0.00001*omega_0*((z_x - (double)centroid_1.x)*162*cos(alpha+1.5707963) + (z_y - (double)centroid_1.y)*162*sin(alpha+1.5707963)))/3.1415926*180) + 0.00001*omega_0*57.29578*exp(-0.001*time_t);

    if(error > threshold)
    {	
        temp_c_t = temp_c;
	//event.z = time_t; 
    }	

    event.x = error;
    event.y = threshold;

    if(temp_c_t > 127)
 	temp_c_t = 127;

    if(temp_c_t < -127)
	temp_c_t = -127;	

 
    u[1] = (double)temp_c_t;


    //printf("alpha=%d.\n", u[1]);
 
    int n;
    socklen_t len;
    char mesg[MAXLINE];
    //char *hello = "Hello from server"; 

    centroid.x = (double)centroid_1.x;
    centroid.y = (double)centroid_1.y;

    sendto(sockfd, u, sizeof(u), MSG_CONFIRM, (const struct sockaddr *) &servaddr, sizeof(servaddr));
 
    k = k + 1;   //counter+1

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "tip_node");
    ros::NodeHandle push;
    ros::NodeHandle pull;
    ros::NodeHandle cent;
    ros::NodeHandle vect;
    ros::NodeHandle neighbour1; 
    ros::NodeHandle neighbour2;
    ros::NodeHandle neighbour3;
    ros::NodeHandle n;	    

    ros::Publisher pub  = push.advertise<geometry_msgs::Vector3>("/vehicle/vt_ctr",1000);
    ros::Publisher pubc = cent.advertise<geometry_msgs::Vector3>("/vehicle/centroid",1000);
    ros::Publisher pube = vect.advertise<geometry_msgs::Vector3>("/vehicle/event",1000);
    
    ros::Rate rate(250);

    //temp_x = 0;
    //temp_y = 0;

    //flag = 0; // to control the stop buttom joy left
    u[0] = 16;
    u[1] = 14;

    k = 0;     //global counter initialization

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

    vertice_1.x = 20;
    vertice_1.y = 20;
    vertice_2.x = 20;
    vertice_2.y = 2800;
    vertice_3.x = 4000;
    vertice_3.y = 2800;
    vertice_4.x = 4000;
    vertice_4.y = 20;

    rectangle_1.x1 = 20;
    rectangle_1.y1 = 20;
    rectangle_1.x2 = 4000;
    rectangle_1.y2 = 20;

    rectangle_2.x1 = 4000;
    rectangle_2.y1 = 20;
    rectangle_2.x2 = 4000;
    rectangle_2.y2 = 2800;

    rectangle_3.x1 = 20;
    rectangle_3.y1 = 2800;
    rectangle_3.x2 = 4000;
    rectangle_3.y2 = 2800;

    rectangle_4.x1 = 20;
    rectangle_4.y1 = 20;
    rectangle_4.x2 = 20;
    rectangle_4.y2 = 2800;

    vt_ctr.x = 0;
    vt_ctr.y = 0;
    vt_ctr.z = 0;

    n1_ctr.x = 0;
    n1_ctr.y = 0;
    n1_ctr.z = 0;

    n2_ctr.x = 0;
    n2_ctr.y = 0;
    n2_ctr.z = 0;

    n3_ctr.x = 0;
    n3_ctr.y = 0;
    n3_ctr.z = 0;
    
    centroid.x = 0;
    centroid.y = 0;
    centroid.z = 0;

    event.x = 0;
    event.y = 0;
    event.z = 0;	

    ros::Subscriber sub = pull.subscribe("/qualisys/Center", 1000, Translate);
    ros::Subscriber neib1 = neighbour1.subscribe("/jet1/vt_ctr", 1000, N1call);
    ros::Subscriber neib2 = neighbour2.subscribe("/jet2/vt_ctr", 1000, N2call);
    ros::Subscriber neib3 = neighbour3.subscribe("/jet3/vt_ctr", 1000, N3call);

    ros::Subscriber joycar = n.subscribe("/turtle1/cmd_vel", 10, JoyCallback); /*to get info from joy */
    /*
    	Point agent_0;
    	Point agent_1;
    	Point agent_2;
    	Point agent_3;
    */ 
    agent_0.x = 700;
    agent_0.y = 1500;
    agent_1.x = 1000;
    agent_1.y = 500;
    agent_2.x = 300;
    agent_2.y = 700;
    agent_3.x = 2300;
    agent_3.y = 1600;

    voronoi_area_compute(agent_0, agent_1, agent_2, agent_3);

    while(ros::ok()) 
    {
	if(!((vt_ctr.x==0)&&(vt_ctr.y==0)&&(vt_ctr.z==0)))
	{
		pub.publish(vt_ctr);
	}
	
	if(!((centroid.x==0)&&(centroid.y==0)&&(centroid.z==0)))
	{
		pubc.publish(centroid);
		//ROS_INFO("Can you see me? Can you see me?!!");
	}

	if(!((event.x==0)&&(event.y==0)&&(event.z==0)))
	{
		pube.publish(event);
		//ROS_INFO("Can you see me? Can you see me?!!");
	}
	
	/*	
	if(!(voronoi_out.x1==0))
	{
		pubv.publish(voronoi_out);
		//ROS_INFO("Can you see me? Can you see me?!!");
	}
	*/
	ros::spinOnce();
	rate.sleep();
    }
    
    return 0;

}

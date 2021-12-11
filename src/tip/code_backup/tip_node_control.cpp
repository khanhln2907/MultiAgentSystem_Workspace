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

// geometry_msgs::Vector3 n1_ctr;
// geometry_msgs::Vector3 n2_ctr;
// geometry_msgs::Vector3 n3_ctr;
// geometry_msgs::Vector3 vt_ctr;
// geometry_msgs::Vector3 centroid;
// tip::Vector voronoi_out;


double temp_x, temp_y;
double v_x, v_y, v; 
int k;

int x_vt_ctr;
int y_vt_ctr;


// void Translate(const qualisys::Subject& center)
// {

// /*************************************************************************compute virtual center*****************************************************************************/
//     double alpha;
 	   
//     alpha = 2*atan(center.orientation.z/center.orientation.w);

//     if (alpha <0)
//     { 
//         alpha += 6.2831852;
//     }

//     double z_x, z_y, v_0, omega_0;

//     z_x = center.position.x*1000 - 162/0.5416*sin(alpha+1.5707963);
//     z_y = center.position.y*1000 + 162/0.5416*cos(alpha+1.5707963);
//     v_0 = 16;
//     omega_0 = 0.5416;

//     //printf("Virtual center x.%f,Virtual center y.%f\n", z_x, z_y);

//     vt_ctr.x = z_x;
//     vt_ctr.y = z_y;

//     agent_0.x = (long)z_x;
//     agent_0.y = (long)z_y;

// /************************************************************************compute voronoi area*********************************************************************************/

// 	voronoi_area_compute(agent_0, agent_1, agent_2, agent_3);


// /********************************************************************************switching control*****************************************************************************/

//     k = k + 1; //a temp counter

//     if (k == 50)
//     {
// 	v = sqrt((center.position.x - temp_x)*(center.position.x - temp_x)*1000000 + (center.position.y - temp_y)*(center.position.y - temp_y)*1000000)/0.5;
// 	temp_x = center.position.x;
//     	temp_y = center.position.y;
// 	//printf("speed=%f\n", v);
// 	k = 0;
//     }	 
   

//     double u[2];

//     u[0] = 16.0;
//     //u[1] = 14;
//     double temp_c;
//     temp_c = (omega_0 + 0.00001*omega_0*((z_x - (double)centroid_1.x)*162*cos(alpha+1.5707963) + (z_y - (double)centroid_1.y)*162*sin(alpha+1.5707963)))/3.1415926*180;
//     //temp_c = (omega_0 + 0.00001*omega_0*((z_x - x_vt_ctr)*162*cos(alpha+1.5707963) + (z_y - y_vt_ctr)*162*sin(alpha+1.5707963)))/3.1415926*180;
//     //int8_t temp1;
//     //temp1 = int8_t(temp);

//     if(temp_c > 127)
//  		temp_c = 127;

//     if(temp_c < -127)
// 		temp_c = -127;	
 
//     u[1] = (double)temp_c;

//     //printf("alpha=%d.\n", u[1]);
 
//     int n;
//     socklen_t len;
//     char mesg[MAXLINE];
//     //char *hello = "Hello from server"; 

//     centroid.x = (double)centroid_1.x;
//     centroid.y = (double)centroid_1.y;

//     sendto(sockfd, u, sizeof(u), MSG_CONFIRM, (const struct sockaddr *) &servaddr, sizeof(servaddr));
 

// }



int main(int argc, char **argv)
{
    ros::init(argc, argv, "tip_node");
    
    //ros::Publisher pubv = vect.advertise<tip::Vector>("/vehicle/voronoi",1000);
    ros::Rate rate(250);

    temp_x = 0;
    temp_y = 0;


    // std:string ip_param;
    // int port_param;   

    // ros::param::get("~ip_param", ip_param);
    // ros::param::get("~port_param", port_param);
    // ros::param::get("~x_vt_ctr",x_vt_ctr);
    // ros::param::get("~y_vt_ctr",y_vt_ctr);

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

    double u[2];

    u[0] = 16.0;
    //u[1] = 14;
    double temp_c;
    temp_c = (omega_0 + 0.00001*omega_0*((z_x - (double)centroid_1.x)*162*cos(alpha+1.5707963) + (z_y - (double)centroid_1.y)*162*sin(alpha+1.5707963)))/3.1415926*180;
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

    centroid.x = (double)centroid_1.x;
    centroid.y = (double)centroid_1.y;

    sendto(sockfd, u, sizeof(u), MSG_CONFIRM, (const struct sockaddr *) &servaddr, sizeof(servaddr));

    while(ros::ok()) 
    {
		// if(!((vt_ctr.x==0)&&(vt_ctr.y==0)&&(vt_ctr.z==0)))
		// {
		// 	pub.publish(vt_ctr);
		// }
	
		// if(!((centroid.x==0)&&(centroid.y==0)&&(centroid.z==0)))
		// {
		// 	pubc.publish(centroid);
		// 	//ROS_INFO("Can you see me? Can you see me?!!");
		// }
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

#include "ros/ros.h"
#include "qualisys/Marker.h"
#include "qualisys/Subject.h"

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

qualisys::Subject wrist;
double temp_x, temp_y;
double v_x, v_y, v; 
int k;




void Translate(const qualisys::Subject& center)
{
    wrist.name = "wrist";
    wrist.occluded = false;
    wrist.position.x = center.position.x;
    wrist.position.y = center.position.y;
    wrist.position.z = center.position.z;
    wrist.orientation.w = center.orientation.w;
    wrist.orientation.x = center.orientation.x;
    wrist.orientation.y = center.orientation.y;
    wrist.orientation.z = center.orientation.z; 

    k = k + 1;

   
    //v = sqrt((wrist.position.x - temp_x)*(wrist.position.x - temp_x)*1000000 + (wrist.position.y - temp_y)*(wrist.position.y - temp_y)*1000000)/0.5;

    if (k == 50)
    {
        //v_x = (wrist.position.x - temp_x)*1000/0.5;
        //v_y = (wrist.position.y - temp_y)*1000/0.5;
	v = sqrt((wrist.position.x - temp_x)*(wrist.position.x - temp_x)*1000000 + (wrist.position.y - temp_y)*(wrist.position.y - temp_y)*1000000)/0.5;
	temp_x = wrist.position.x;
    	temp_y = wrist.position.y;
	//printf("speed=%f\n", v);
	k = 0;
    }	 

   double alpha;
 

    //if (wrist.orientation.z >= 0)
        //alpha = 2*asin(double(wrist.orientation.z));

    //if (wrist.orientation.z < 0)
	//alpha = 2*asin(double(wrist.orientation.z))+6.2831852;
 	   
    alpha = 2*atan(wrist.orientation.z/wrist.orientation.w);
    if (alpha <0)
    { 
        alpha += 6.2831852;
    }

    //printf("sum=%f", wrist.orientation.w);

    double z_x, z_y, v_0, omega_0;

    z_x = wrist.position.x*1000 - 162/0.5416*sin(alpha+1.5707963);
    z_y = wrist.position.y*1000 + 162/0.5416*cos(alpha+1.5707963);
    v_0 = 16;
    omega_0 = 0.5416;

    //printf("position x.%f, y.%f\n", wrist.position.x*1000, wrist.position.y*1000);
    
    //printf("Orientation %f\n", alpha/2/3.1415926*360);
    printf("Virtual center x.%f,Virtual center y.%f\n", z_x, z_y);
    //printf("Virtual center y.%f\n", z_y);

    double u[2];

    u[0] = 16.0;
    //u[1] = 14;
    double temp;
    temp = (omega_0 + 0.00001*omega_0*((z_x - 500)*162*cos(alpha+1.5707963) + (z_y - 500)*162*sin(alpha+1.5707963)))/3.1415926*180;

    //int8_t temp1;
    //temp1 = int8_t(temp);

    if(temp > 127)
 	temp = 127;

    if(temp < -127)
	temp = -127;	
 
    u[1] = temp;

    //printf("alpha=%d.\n", u[1]);
 
    int n;
    socklen_t len;
    char mesg[MAXLINE];
    char *hello = "Hello from server"; 

    //sendto(sockfd, (const char *)hello, strlen(hello), MSG_CONFIRM, (const struct sockaddr *) &servaddr, sizeof(servaddr));
    sendto(sockfd, u, sizeof(u), MSG_CONFIRM, (const struct sockaddr *) &servaddr, sizeof(servaddr));
 
    //printf("Hello message sent.\n");

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tip_node");
    ros::NodeHandle push;
    ros::NodeHandle pull;
    ros::Publisher pub = push.advertise<qualisys::Subject>("/qualisys/wrist",1000);   
    ros::Rate rate(250);

    temp_x = 0;
    temp_y = 0;
    
    // Creating socket file descriptor 
    if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) { 
        perror("socket creation failed"); 
        exit(EXIT_FAILURE); 
    } 
      
    memset(&servaddr, 0, sizeof(servaddr)); 
    memset(&cliaddr, 0, sizeof(cliaddr)); 
      
    // Filling server information 
    servaddr.sin_family = AF_INET; // IPv4 
    servaddr.sin_addr.s_addr = INADDR_ANY; 
    servaddr.sin_port = htons(PORT); 
      
    // Bind teh socket with teh server address 
    if ( bind(sockfd, (const struct sockaddr *)&servaddr,  
            sizeof(servaddr)) < 0 ) 
    { 
        perror("bind failed"); 
        exit(EXIT_FAILURE); 
    } 

    bzero(&servaddr, sizeof(servaddr)); 
    servaddr.sin_addr.s_addr = inet_addr("192.168.1.117"); 
    servaddr.sin_port = htons(PORT); 
    servaddr.sin_family = AF_INET; 


    wrist.orientation.x = 0;
    wrist.orientation.y = 0;
    wrist.orientation.z = 0;
    wrist.orientation.w = 0;
    ros::Subscriber sub = pull.subscribe("/qualisys/Center", 1000, Translate);

    //double test[2];
    //test[0] = 16.0;
    //test[1] = 0.5416/3.1415926*180;

    while(ros::ok())
    {
	if(!((wrist.orientation.x==0)&&(wrist.orientation.y==0)&&(wrist.orientation.z==0)&&(wrist.orientation.w==0)))
	{pub.publish(wrist);}
	
	//sendto(sockfd, test, sizeof(test), MSG_CONFIRM, (const struct sockaddr *) &servaddr, sizeof(servaddr));

	ros::spinOnce();
	rate.sleep();
    }
    
    return 0;


}

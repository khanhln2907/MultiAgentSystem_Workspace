//Includes all the headers necessary to use the most common public pieces of the ROS system.
#include <ros/ros.h>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"

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

#define MAXLINE 80
#define SERV_PORT 8888

ros::Publisher udppub;
std_msgs::Int32MultiArray udp;
int udp_data[3];
struct sockaddr_in servaddr,cliaddr;
//cereal::CerealPort device;


void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
   ROS_INFO("I heard: [%s]", msg->data.c_str());
}

void do_echo(int sockfd,struct sockaddr *pcliaddr,socklen_t clilen)
{
	int n;
	socklen_t len;
	char mesg[MAXLINE];//="$UWB,100.0,100.0,100.0";
	std::string tem_str = "$UWB,100.0,100.0,100.0";
	//struct sockaddr_in client;
	//socklen_t addrlen;


	char reply[8];
	 

	for(;;)
	{
		len = clilen;
		/* waiting for receive data */
		n = recvfrom(sockfd,mesg,MAXLINE,0,pcliaddr,&len);
		//n =recvfrom(sockfd,mesg,MAXLINE,0,(struct sockaddr*)&client,&addrlen);    
		printf("message received from client, length=%d\n",n);
		printf("You got a message (%s%) from client.\nIt's ip is%s, port is %d.\n",mesg,inet_ntoa(cliaddr.sin_addr),htons(cliaddr.sin_port)); 
		//printf("datalength:%d",sizeof(mesg));
		/* sent data back to client */
		//pcliaddr=client;
		//sendto(sockfd,mesg,n,0,pcliaddr,len);
   		/*int a = n - 41;
		printf("a=%d\n",a);
		char temp[a];
		temp[0] = '$';
		temp[1] = 'U';
		temp[2] = 'W';
		temp[3] = 'B';
		temp[4] = ',';
		//udp.data.clear();
		for (int i = 5; i <= a; i++)
		{ 
			temp[i] = mesg[i+40];
		}*/

		//device.write(temp);
		//printf("Serial Printf:%s\n",temp);
		udp.data.clear();
		//printf("datalength:%s",mesg);	
		//printf("datalength:%d",tem_str.length());		
 
		ROS_INFO("UDP data published");

	}
     
}


int main(int argc, char **argv)
{
        ros::init(argc, argv, "udp_listener");
        ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);


	int sockfd;
	//int n;
	//socklen_t len;
	//char mesg[MAXLINE];
	sockfd = socket(AF_INET,SOCK_DGRAM,0); /* create a socket */

	/* init servaddr */
	bzero(&servaddr,sizeof(servaddr));
	servaddr.sin_family = AF_INET;
	servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
	//servaddr.sin_port = htons(SERV_PORT);
	servaddr.sin_port = htons(22222);

	if(bind(sockfd,(struct sockaddr *)&servaddr,sizeof(servaddr)) == -1)
	{
		perror("bind error");
		exit(1);
	}
	printf("luxifu ready to receive message from port %d \n",6000);//SERV_PORT
	/**************************************************************************************/	
	/*sockaddr_in addrClient;
	socklen_t len = sizeof(servaddr);
	char recvBuf[100];
	int n;
	n = recvfrom(sockfd,recvBuf,100,0,(sockaddr*)&addrClient,&len);
	printf("%s\n",recvBuf);*/
	/**************************************************************************************/

	do_echo(sockfd,(struct sockaddr *)&cliaddr,sizeof(cliaddr));   
	ros::spin();
	ROS_INFO("tutorialROSOpenCV::main.cpp::No error.");
}

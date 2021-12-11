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
#define MAXLINE 80
#define SERV_PORT 8888

ros::Publisher udppub;
std_msgs::Int32MultiArray udp;
int udp_data[3];
struct sockaddr_in servaddr,cliaddr;
void do_echo(int sockfd,struct sockaddr *pcliaddr,socklen_t clilen)
{
int n;
socklen_t len;
char mesg[MAXLINE];
//       struct sockaddr_in client;
//       socklen_t addrlen;
	for(;;)
	{
		len = clilen;
		/* waiting for receive data */
		n = recvfrom(sockfd,mesg,MAXLINE,0,pcliaddr,&len);
		//n =recvfrom(sockfd,mesg,MAXLINE,0,(struct sockaddr*)&client,&addrlen);    
		//printf("message received from client, length=%d\n",n);
		printf("You got a message (%s%) from client.\nIt's ip is%s, port is %d.\n",mesg,inet_ntoa(cliaddr.sin_addr),htons(cliaddr.sin_port)); 
		/* sent data back to client */
		//pcliaddr=client;
		sendto(sockfd,mesg,n,0,pcliaddr,len);
		     
		    udp.data.clear();
		    for (int i = 0; i < 3; i++)
		     { 
		       // 清空缓冲区
		//     udp_data[0]=mesg[？];

		     ////////////////////////装入数据////////////////////
		//     udp.data.push_back(udp_data[i]);
		     }
		      // Plulish UDP 数据
		//    udppub.publish(udp);

		ROS_INFO("UDP data published");

	}
     
}


int main(int argc, char **argv)
{
        ros::init(argc, argv, "udp_server");
        ros::NodeHandle nh;
	int sockfd;
	//int n;
	//socklen_t len;
	//char mesg[MAXLINE];
	sockfd = socket(AF_INET,SOCK_DGRAM,0); /* create a socket */

	/* init servaddr */
	bzero(&servaddr,sizeof(servaddr));
	servaddr.sin_family = AF_INET;
	servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
	servaddr.sin_port = htons(SERV_PORT);

	if(bind(sockfd,(struct sockaddr *)&servaddr,sizeof(servaddr)) == -1)
	{
		perror("bind error");
		exit(1);
	}
	
	printf("ready to receive message from port %d \n", SERV_PORT  );

        do_echo(sockfd,(struct sockaddr *)&cliaddr,sizeof(cliaddr));   

        ros::spin();

	ROS_INFO("tutorialROSOpenCV::main.cpp::No error.");
}

#include "ros/ros.h"
#include <iostream>
#include <vector>
#include <string>
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
// User's type definition
#include "commonType.h"
#include "Unicycle_BLF_Agent.h"
#include "CentralizedMonitor.h"

// Custom Message for ROS PS Service
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <turtlesim/Pose.h>
#include "qualisys/Marker.h"
#include "qualisys/Subject.h"
#include <geometry_msgs/Vector3.h>
#include "tip/UnicycleInfoStruct.h"
#include "tip/UnicycleInfoMsg.h"
#include "tip/ControlMsg.h"
#include "tip/Vector.h"

/// IP Settings ***********************************
#define PORT     50001 
#define MAXLINE 80
#define SERV_PORT 8888
struct sockaddr_in servaddr,cliaddr;
int sockfd; 
char buffer[MAXLINE];
sockaddr_storage addrDest = {};

void setupService(std::string ip_param, int port_param);

/// End         ***********************************


/// Global Variable Settings **********************
Unicycle_BLF_Agent thisAgent;

using std::string;
using namespace std;

/// Messages handler for ROS Topics
tip::UnicycleInfoMsg dataTobeTransmitted;
tip::Vector voronoi_out;

double time_init;
double u[2]; //global 
double temp_c_t;    //control input at events
double v_x, v_y, v; 
unsigned long k;    //A global counter
int x_vt_ctr;
int y_vt_ctr;


bool EMERGENCY_STOP = false;

void initGlobalVar();
/// End         ***********************************

/// ROS call back function
void rosTopicCoverageListen(const tip::UnicycleInfoMsg newCoverageInfo);
void rosTopicQualisysListen(const qualisys::Subject& center);
void rosJoyCallback(const geometry_msgs::Twist& input);
tip::UnicycleInfoMsg convertPublishCoverageMessage(BLF_CoverageMessage newInfo);
void rosTopicCentralizedNodeListen(const tip::ControlMsg controlInput);
/// End         ***********************************


int main(int argc, char **argv) {
    /// ROS PLATFORM SETUP *************************************************************
    ros::init(argc, argv, "tip_node");
    ros::NodeHandle topicPublisher;
    ros::NodeHandle agentListerner1;
    ros::NodeHandle agentListerner2;
    ros::NodeHandle agentListerner3;
    ros::NodeHandle qualisysListener;
    ros::NodeHandle centralizeNodeListener;
    ros::NodeHandle HardwareListener;	    

    // Users can modify the message for any desired information
    ros::Publisher coverageTopicPublisher = topicPublisher.advertise<tip::UnicycleInfoMsg>("/vehicle/CoverageInfo",1000);   
    
    // Setup IP servers
    std:string ip_param;
    int32_t port_param;   
    ros::param::get("~ip_param", ip_param);
    ros::param::get("~port_param", port_param);
    setupService(ip_param, port_param);
    ros::param::get("~x_vt_ctr",x_vt_ctr); // Do we need this ?
    ros::param::get("~y_vt_ctr",y_vt_ctr); 

    // Setup subscribing routine
    ros::Subscriber infoQualisys = qualisysListener.subscribe("/qualisys/Center", 1000, rosTopicQualisysListen);
    // Subscribe to the General Info of the neighbor agents
    // ros::Subscriber infoAgent1 = agentListerner1.subscribe("/jet1/CoverageInfo", 1000, rosTopicCoverageListen);
    // ros::Subscriber infoAgent2 = agentListerner2.subscribe("/jet2/CoverageInfo", 1000, rosTopicCoverageListen);
    // ros::Subscriber infoAgent3 = agentListerner3.subscribe("/jet3/CoverageInfo", 1000, rosTopicCoverageListen);

// Subscribe to the centralized controller
    ros::Subscriber cmdCentralNode = centralizeNodeListener.subscribe("/centralNode/controlInput", 5000, rosTopicCentralizedNodeListen);

    // Hardware Interface to monitor real agents    
    ros::Subscriber joycar = HardwareListener.subscribe("/turtle1/cmd_vel", 10, rosJoyCallback); /*to get info from joy */
    
    // Update rate at 250 Hz
    ros::Rate rate(250);

    /// ALGORITHM / CONTROLLER SETUP **************************************************
    // Initialize the agent with a fix ID (use the port_paramter as uniqueID)
#define TOTAL_AGENT 6
#define MAX_HEADING_VEL 10
#define MAX_ANGULAR_VEL 2
#define CONST_HEADING_VEL 16
#define ORBITAL_ANGULAR_VEL 0.5
#define CONTROL_GAIN 1

    BLF_CoverageAgentConfig_t tmpAllInConfig;
    tmpAllInConfig.nTotalAgents = TOTAL_AGENT;
    tmpAllInConfig.initPose = {0,0,0};
    tmpAllInConfig.myID = port_param; // Use the port parameter as unique ID
        BLF_ControlConfig_t tmpControlConfig;
        tmpControlConfig.constTranslationAndOrbitVelocity = {CONST_HEADING_VEL, ORBITAL_ANGULAR_VEL};
        tmpControlConfig.controlGain = CONTROL_GAIN;
        tmpControlConfig.inputMax = {MAX_HEADING_VEL, MAX_ANGULAR_VEL};
    tmpAllInConfig.controlParameter = tmpControlConfig;
   
    thisAgent.begin(tmpAllInConfig);
    initGlobalVar();
    while(ros::ok()) 
    {
        // Get the summary report of the agent and publish it to another node
        BLF_CoverageMessage infoStruct = thisAgent.getPublishedInfo();
        dataTobeTransmitted = convertPublishCoverageMessage(infoStruct);
        //ROS_INFO("ID: %d| x: %.2f| y: %.2f| the: %.1f| VMx: %.2f| VMy: %.2f",
        //infoStruct.TransmitterID, infoStruct.AgentPose.x, infoStruct.AgentPose.y, infoStruct.AgentPose.theta, infoStruct.VirtualCenter.x, infoStruct.VirtualCenter.y);


        coverageTopicPublisher.publish(dataTobeTransmitted);
        //ROS_INFO("Info published");


        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}

void setupService(std::string ip_param, int port_param){
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
}

void initGlobalVar(){
    u[0] = 16;
    u[1] = 14;

    k = 0;     //global counter initialization
}

void rosTopicCoverageListen(const tip::UnicycleInfoMsg newCoverageInfo){
    // Since we are updating the topic for the Coverage problem, the topic structure is
    // defined in class Unicycle_BLF_Agent.h.
    // Note that changing this protocol requires edit in the ROS Message (src/tip/msg)
    BLF_CoverageMessage info;
    info.TransmitterID = (int32_t) newCoverageInfo.packet.TransmitterID;
    info.AgentPose.x = (float) newCoverageInfo.packet.AgentPosX;
    info.AgentPose.y = (float) newCoverageInfo.packet.AgentPosY;
    info.AgentPose.theta = (float) newCoverageInfo.packet.AgentTheta;
    info.VirtualCenter.x = (float) newCoverageInfo.packet.VirtualCenterX;
    info.VirtualCenter.y = (float) newCoverageInfo.packet.VirtualCenterY;
    info.V_BLF = (float) newCoverageInfo.packet.V_BLF;
    //ROS_INFO("ID: %d| x: %.2f| y: %.2f| the: %.1f| VMx: %.2f| VMy: %.2f",
    //info.TransmitterID, info.AgentPose.x, info.AgentPose.y, info.AgentPose.theta, info.VirtualCenter.x, info.VirtualCenter.y);
    //ROS_INFO("ID: %d| x: %.2f| y: %.2f| the: %.1f| VMx: %.2f| VMy: %.2f",
    //newCoverageInfo.packet.TransmitterID, newCoverageInfo.packet.AgentPosX, newCoverageInfo.packet.AgentPosY,
    //newCoverageInfo.packet.AgentTheta, newCoverageInfo.packet.VirtualCenterX, newCoverageInfo.packet.VirtualCenterY);

    // Update the topic from this method. Class Agent will sort the neighbor into
    // their internal array automatically
    thisAgent.updateNeighbor(info);
    //ROS_INFO("Info published");
}

tip::UnicycleInfoMsg convertPublishCoverageMessage(BLF_CoverageMessage newInfo){
    tip::UnicycleInfoMsg tmp;
    tmp.packet.TransmitterID = newInfo.TransmitterID;
    tmp.packet.AgentPosX = newInfo.AgentPose.x;
    tmp.packet.AgentPosY = newInfo.AgentPose.y;
    tmp.packet.AgentTheta = newInfo.AgentPose.theta;
    tmp.packet.VirtualCenterX = newInfo.VirtualCenter.x;
    tmp.packet.VirtualCenterY = newInfo.VirtualCenter.y;
    tmp.packet.V_BLF = newInfo.V_BLF;
    return tmp;
}

void rosTopicCentralizedNodeListen(const tip::ControlMsg cmd){
    int32_t rxID = cmd.ID;
    float tranVel = cmd.translation;
    float angVel = cmd.rotation;

    if(EMERGENCY_STOP){
        u[0] = 0;
        u[1] = 0;
        sendto(sockfd, u, sizeof(u), MSG_CONFIRM, (const struct sockaddr *) &servaddr, sizeof(servaddr));

    }
    else if(thisAgent._myID == rxID){
        // Do something if the command is for us
        if(angVel > 127)
        angVel = 127;

        if(angVel < -127)
        angVel = -127;

        if(angVel > 127)
        angVel = 127;

        if(angVel < -127)
        angVel = -127;	

        u[0] = tranVel;
        u[1] = angVel;
        ROS_INFO("ID: %d V: %.3f W: %.3f, %d", rxID, tranVel, angVel, EMERGENCY_STOP);

        int n;
        socklen_t len;
        char mesg[MAXLINE];
        //char *hello = "Hello from server"; 
        sendto(sockfd, u, sizeof(u), MSG_CONFIRM, (const struct sockaddr *) &servaddr, sizeof(servaddr));
    }
}

// Activate emergency stop if we press the left joint
void rosJoyCallback(const geometry_msgs::Twist& input)
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
    EMERGENCY_STOP = true;
	ROS_INFO("EMERGENCY STOP ACTIVATED \n");
   }
	
}

void rosTopicQualisysListen(const qualisys::Subject& center){
    // Get the system time
    double secs, time_t;
    secs = ros::Time::now().toSec(); 
    //ROS_INFO(" NEW INFO QUALISYS");

    // Compute the heading orientation
    float theta;
#define ORIENTATION_OFFSET PI/2
    theta = 2*atan(center.orientation.z/center.orientation.w) + ORIENTATION_OFFSET;
    if (theta <0)
    { 
        theta += 6.2831852;
    }

    // Update the new pose to agent, which is used to compute the virtual center internally
    UnicycleState newPose;
    newPose.x = (float) center.position.x * 1000; // Convert to mm  
    newPose.y = (float) center.position.y * 1000;
    newPose.theta = (float) theta;
    thisAgent.updateState(newPose);
}
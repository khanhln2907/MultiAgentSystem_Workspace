#include "ros/ros.h"
#include "std_msgs/String.h"
//#include "qualisys/Marker.h"
//#include "qualisys/Subject.h"
#include <geometry_msgs/Vector3.h>
//#include "tip/Vector.h"
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
#include <std_msgs/Float64.h>
#include <turtlesim/Pose.h>


// Users object instances
#include "CentralizedMonitor.h"

CentralizedMonitor monitor;
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void updateUnicycleState(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
#define N_TOTAL_AGENT  (uint16_t) 4
    monitor.begin(N_TOTAL_AGENT);
    // Monitor just listen for the derived data
    //monitor.begin();

  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
    ros::init(argc, argv, "CentralizedNode");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
    ros::NodeHandle agent1;
    ros::NodeHandle agent2;
    ros::NodeHandle agent3;
    ros::NodeHandle agent4;
  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub1 = agent1.subscribe("/san/info", 1000, updateUnicycleState);
  ros::Subscriber sub2 = agent2.subscribe("/wu/info", 1000, updateUnicycleState);
  ros::Subscriber sub3 = agent3.subscribe("/liu/info", 1000, updateUnicycleState);
  ros::Subscriber sub4 = agent4.subscribe("/qi/info", 1000, updateUnicycleState);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  //ros::spin();

    while(ros::ok()) 
    {
        // Bumping the listener in the loop
	    ros::spinOnce();
    }
    
    return 0;

  return 0;
}

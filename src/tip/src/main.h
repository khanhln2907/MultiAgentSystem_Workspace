#pragma once
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
#include <std_msgs/Float64.h>
#include <turtlesim/Pose.h>

// User's type definition
#include "tip_type.h"

using std::string;
using namespace std;

#define PORT     50001 
#define MAXLINE 80
#define SERV_PORT 8888
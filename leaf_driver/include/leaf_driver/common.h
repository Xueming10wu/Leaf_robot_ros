#ifndef COMMON_H
#define COMMON_H

//所以平时需要用到的头文件都放在这里
#include <iostream>
#include <stdio.h>
#include <iomanip>
#include <unistd.h>
#include <typeinfo>
#include <inttypes.h>
#include <cstdlib>
#include <ctime>


#include <cmath>
#include <cstring>
#include <string.h>
#include <signal.h>
#include <vector>
#include "serial/serial.h"


//ros头文件
#include "ros/ros.h"
#include "std_msgs/UInt16.h"
#include "sensor_msgs/JointState.h"
#include "robot_state_publisher/robot_state_publisher.h"

//动作编程
#include "actionlib/server/action_server.h"
#include "actionlib/server/simple_action_server.h"
#include "actionlib/server/server_goal_handle.h" 
#include "control_msgs/FollowJointTrajectoryAction.h"

using namespace std;
using namespace serial;

//常量定义
const double PI = 3.1415926;


#endif // __COMMON__
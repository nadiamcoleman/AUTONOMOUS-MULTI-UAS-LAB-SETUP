///////////////////////////////////////////////////////////////////////////////
//
// Copyright (C) OMG Plc 2009.
// All rights reserved.  This software is protected by copyright
// law and international treaties.  No part of this software / document
// may be reproduced or distributed in any form or by any means,
// whether transiently or incidentally to some other use of this software,
// without the written permission of the copyright owner.
//
///////////////////////////////////////////////////////////////////////////////

#include "Client.h"
#include <iostream>
#include <string>
#include <time.h>

#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int16.h>

int main( int argc, char* argv[] )
{
    std::cout << "START1" << std::endl;
    // ************* ROS ADDITIONS *************
    ros::init(argc, argv, "test");
    std::cout << "START2" << std::endl;

    ros::Time time_keeper = ros::Time();
    std::cout << "START3" << std::endl;

    ros::NodeHandle nh;
    ros::Publisher test_pub = nh.advertise<std_msgs::Int16>("test",10);
    std_msgs::Int16Ptr msg(new std_msgs::Int16);
    msg->data = 0;

    ros::Rate rate(2); // 10 hz
    int count = 0;
    while(ros::ok()) {
        msg->data = count;
        test_pub.publish(msg);
        count++;
        ros::spinOnce();
        rate.sleep();
    }    
        
    return 0;
}

/* 
 * File:   GestureReceiver.h
 * Author: haikalpribadi
 *
 * Created on 05 March 2014, 19:56
 */

#ifndef GESTURERECEIVER_H
#define	GESTURERECEIVER_H

#include <cstdlib>
#include <fstream>
#include <ros/ros.h>
#include <vector>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/String.h>

class GestureReceiver {
public:
    GestureReceiver();
    
private:
    ros::NodeHandle node;
    ros::Subscriber gestureVector_sub;
    ros::Subscriber gestureMessage_sub;
    
    void gestureMessageCallback(const std_msgs::String::ConstPtr& msg);
    void gestureVectorCallback(const std_msgs::Int32MultiArray::ConstPtr& msg);
};

#endif	/* GESTURERECEIVER_H */


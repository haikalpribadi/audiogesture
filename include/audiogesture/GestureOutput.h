/* 
 * File:   GestureOutput.h
 * Author: haikalpribadi
 *
 * Created on 29 March 2014, 20:51
 */

#ifndef GESTUREOUTPUT_H
#define	GESTUREOUTPUT_H

#include <algorithm>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <vector>

using namespace std;

class GestureOutput {
public:
    GestureOutput();
private:
    ros::NodeHandle node;
    ros::Subscriber gestureOutput_sub;
    ros::Publisher oscSend_pub;
    
    int range;
    double max;
    double min;
    
    void gestureOutputCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);

};

#endif	/* GESTUREOUTPUT_H */


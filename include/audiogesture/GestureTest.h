/* 
 * File:   GestureTest.h
 * Author: haikalpribadi
 *
 * Created on 06 March 2014, 14:44
 */

#ifndef GESTURETEST_H
#define	GESTURETEST_H

#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>

class GestureTest {
public:
    GestureTest();
    void sendData(std_msgs::Int32MultiArray &msg);
    
private:
    ros::NodeHandle node;
    ros::Publisher gesture_pub;
    
    
};

#endif	/* GESTURETEST_H */


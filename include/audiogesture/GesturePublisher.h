/* 
 * File:   SerialDataPublisher.h
 * Author: haikalpribadi
 *
 * Created on 10 March 2014, 22:25
 */

#ifndef GESTUREPUBLISHER_H
#define	GESTUREPUBLISHER_H

#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <queue>
#include <vector>

#include "audiogesture/GestureVector.h"

using namespace std;

class GesturePublisher {
public:
    GesturePublisher();
    void publishGestureVector();
    
private:
    ros::NodeHandle node;
    ros::Publisher gestureVector_pub;
    ros::Subscriber gestureVector0_sub;
    ros::Subscriber gestureVector1_sub;
    
    queue<vector<int> > gesture0queue;
    queue<vector<int> > gesture1queue;
    
    void gestureVector0Callback(const std_msgs::Int32MultiArray::ConstPtr& msg);
    void gestureVector1Callback(const std_msgs::Int32MultiArray::ConstPtr& msg);
    
    
};

#endif	/* GESTUREPUBLISHER_H */


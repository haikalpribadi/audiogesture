/* 
 * File:   FunctionExtractor.h
 * Author: haikalpribadi
 *
 * Created on 02 March 2014, 20:46
 */

#ifndef FUNCTIONEXTRACTOR_H
#define	FUNCTIONEXTRACTOR_H

#include <cstdio>
#include <cstdlib>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

using namespace std;

class FunctionExtractor {
public:
    FunctionExtractor();
    
private:
    ros::NodeHandle node;
    ros::Subscriber subscriber;
    
    string feature;
    
    void extractorCallback(const std_msgs::Float32::ConstPtr& msg);
};

#endif	/* FUNCTIONEXTRACTOR_H */


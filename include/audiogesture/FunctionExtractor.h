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
#include <sstream>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>

#include "audiogesture/FeatureVector.h"

using namespace std;

class FunctionExtractor {
public:
    FunctionExtractor();
    
private:
    ros::NodeHandle node;
    ros::Subscriber extractor_sub;
    
    //string feature;
    
    void extractorCallback(const audiogesture::FeatureVector::ConstPtr& msg);
};

#endif	/* FUNCTIONEXTRACTOR_H */


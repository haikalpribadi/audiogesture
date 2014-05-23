/* 
 * File:   DataTransformer.h
 * Author: haikalpribadi
 *
 * Created on 23 May 2014, 19:36
 */

#ifndef DATATRANSFORMER_H
#define	DATATRANSFORMER_H

#include <fstream>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include "audiogesture/GestureVector.h"

using namespace std;

class DataTransformer {
public:
    DataTransformer();
    
private:
    ros::NodeHandle node;
    ros::Publisher data_pub;
    ros::Subscriber file_sub;
    
    string data_dir;
    string filename;
    double scale;
    
    void fileCallback(const std_msgs::String::ConstPtr& msg);
};

#endif	/* DATATRANSFORMER_H */


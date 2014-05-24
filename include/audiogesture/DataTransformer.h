/* 
 * File:   DataTransformer.h
 * Author: haikalpribadi
 *
 * Created on 23 May 2014, 19:36
 */

#ifndef DATATRANSFORMER_H
#define	DATATRANSFORMER_H

#include <dirent.h>
#include <fstream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "audiogesture/GestureVector.h"

using namespace std;

struct stat sb;

class DataTransformer {
public:
    DataTransformer();
    
private:
    ros::NodeHandle node;
    ros::Publisher data_pub;
    ros::Subscriber file_sub;
    ros::Rate rate;
    
    string data_dir;
    double scale;
    
    
    void fileCallback(const std_msgs::String::ConstPtr& msg);
    void transformFile(string filename);
};

#endif	/* DATATRANSFORMER_H */


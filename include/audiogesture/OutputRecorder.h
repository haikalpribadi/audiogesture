/* 
 * File:   OutputRecorder.h
 * Author: haikalpribadi
 *
 * Created on 09 June 2014, 05:11
 */

#ifndef OUTPUTRECORDER_H
#define	OUTPUTRECORDER_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <dirent.h>
#include <fstream>
#include <iostream>
#include <std_msgs/String.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <vector>

#include "audiogesture/OutputRecord.h"
#include "audiogesture/MagnitudeRecord.h"

using namespace std;

struct stat sb;

class OutputRecorder {
public:
    OutputRecorder();
    
private:
    ros::NodeHandle node;
    ros::Subscriber output_sub;
    ros::Subscriber status_sub;
    ros::Subscriber magnitude_sub;
    
    bool record;
    int recordCounter;
    string output_dir;
    string pca_dir;
    ofstream featureFile;
    ofstream gestureFile;
    ofstream featureMagFile;
    ofstream gestureMagFile;
    vector<vector<float> > featureMagnitudes;
    vector<vector<float> > gestureMagnitudes;
    
    void recordCallback(const audiogesture::OutputRecord::ConstPtr& msg);
    void statusCallback(const std_msgs::String::ConstPtr& msg);
    void magnitudeCallback(const audiogesture::MagnitudeRecord::ConstPtr& msg);
};

#endif	/* OUTPUTRECORDER_H */


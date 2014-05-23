/* 
 * File:   DataVisualizer.h
 * Author: haikalpribadi
 *
 * Created on 23 May 2014, 18:10
 */

#ifndef DATAVISUALIZER_H
#define	DATAVISUALIZER_H

#include <fstream>
#include <ros/ros.h>

#include "audiogesture/DataFile.h"
#include "audiogesture/GestureVector.h"

using namespace std;

class DataVisualizer {
public:
    DataVisualizer();
    
private:
    ros::NodeHandle node;
    ros::Publisher data_pub;
    ros::Subscriber file_sub;
    
    string data_dir;
    string filename;
    
    void fileCallback(const audiogesture::DataFile::ConstPtr& msg);
};


#endif	/* DATAVISUALIZER_H */


/* 
 * File:   DataVisualizer.h
 * Author: haikalpribadi
 *
 * Created on 23 May 2014, 18:10
 */

#ifndef DATAVISUALIZER_H
#define	DATAVISUALIZER_H

#include <dirent.h>
#include <fstream>
#include <ros/ros.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "audiogesture/DataFile.h"
#include "audiogesture/GestureVector.h"

using namespace std;

struct stat sb;

class DataVisualizer {
public:
    DataVisualizer();
    
private:
    ros::NodeHandle node;
    ros::Publisher data_pub;
    ros::Subscriber file_sub;
    
    double scale;
    double amp;
    int rate;
    string data_dir;
    string filename;
    
    void fileCallback(const audiogesture::DataFile::ConstPtr& msg);
    void visualizeFile(string filename, int height, int width);
};


#endif	/* DATAVISUALIZER_H */


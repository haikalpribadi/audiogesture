/* 
 * File:   MusicExtractor.h
 * Author: haikalpribadi
 *
 * Created on 14 February 2014, 19:22
 */

#ifndef MUSICEXTRACTOR_H
#define	MUSICEXTRACTOR_H

#include <cstdlib>
#include <marsyas/MarSystem.h>
#include <ros/ros.h>
#include <sstream>
#include <std_msgs/String.h>

#include "audiogesture/ExtractorStatus.h"
#include "audiogesture/FeatureVector.h"
#include "Bextract.h"

using namespace std;

class MusicExtractor {
public:
    MusicExtractor();
    
private:
    string music_dir;
    
    ros::NodeHandle node;
    ros::Subscriber extractor_sub;
    ros::Publisher extractorStatus_pub;
    
    void extractorCallback(const std_msgs::String::ConstPtr& msg);
};

#endif	/* MUSICEXTRACTOR_H */


/* 
 * File:   MusicExtractorBuffer.h
 * Author: haikalpribadi
 *
 * Created on 05 March 2014, 12:01
 */

#ifndef MUSICEXTRACTORBUFFER_H
#define	MUSICEXTRACTORBUFFER_H

#include <cstdlib>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <queue>

#include "audiogesture/ExtractorStatus.h"

using namespace std;

class MusicExtractorBuffer {
public:
    MusicExtractorBuffer();
    void processBuffer();
    
private:
    ros::NodeHandle node;
    ros::Publisher buffer_pub;
    ros::Subscriber buffer_sub;
    ros::Subscriber status_sub;
    
    bool hold;
    queue<std_msgs::String> messages;
    
    void bufferCallback(const std_msgs::String::ConstPtr& msg);
    void statusCallback(const audiogesture::ExtractorStatus::ConstPtr& msg);
};

#endif	/* MUSICEXTRACTORBUFFER_H */


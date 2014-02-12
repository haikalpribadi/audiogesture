/* 
 * File:   extractor.cpp
 * Author: haikalpribadi
 *
 * Created on 29 December 2013, 16:08
 */

#include <cstdlib>
#include <iostream>
#include <vector>
#include <marsyas/MarSystem.h>
#include <ros/ros.h>

#include "audiogesture/Strings.h"
#include "audiogesture/MusicExtractor.h"
#include "bextract.h"


using namespace std;


bool bextract(audiogesture::MusicExtractor::Request &req,
               audiogesture::MusicExtractor::Response &res)
{
    bextractor(req.args);
    ROS_INFO("BEXTRACTOR CALLED");
    return true;
}
/*
 * 
 */
int main(int argc, char** argv) 
{
    
    ros::init(argc, argv, "MusicFeatureExtractor");
    ros::NodeHandle node;
    
    ros::ServiceServer music_extractor_srv_ = node.advertiseService("music_extractor", bextract);
    
    ROS_INFO("Music Feature Extractor");
    
    
    string directory;
    if (node.getParam("music_dir", directory)){
        ROS_INFO("MusicFeatureExtractor using music_dir: %s", directory.c_str());
        chdir(directory.c_str());
    }
    else
    {
        ROS_ERROR("Please set the music_directory (file) parameter for extractor");
        ros::requestShutdown();
    }

    ros::spin();
    return 0;
}

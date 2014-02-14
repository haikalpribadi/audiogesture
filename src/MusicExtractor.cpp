/* 
 * File:   extractor.cpp
 * Author: haikalpribadi
 *
 * Created on 29 December 2013, 16:08
 */

#include <cstdlib>
#include <iostream>
#include <vector>
#include <sstream>
#include <marsyas/MarSystem.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include "Bextract.h"


using namespace std;


/*
 bool bextract(audiogesture::MusicExtractor::Request &req,
               audiogesture::MusicExtractor::Response &res)
{
    bextractor(req.args);
    ROS_INFO("BEXTRACTOR CALLED");
    return true;
}
 */
void extractorCallback(const std_msgs::String::ConstPtr msg){
    istringstream iss(msg->data.c_str());
    vector<string> tokens;
    copy(istream_iterator<string>(iss),
        istream_iterator<string>(),
        back_inserter<vector<string> >(tokens));
    bextractor(tokens);
    
    ROS_INFO("BEXTRACTOR CALLED");
}

int main(int argc, char** argv){
    
    ros::init(argc, argv, "MusicFeatureExtractor");
    ros::NodeHandle node;
    //ros::ServiceServer musicExtractor_srv = node.advertiseService("music_extractor", bextract);
    ros::Subscriber musicExtractor_sub = node.subscribe("music_extractor", 1000, extractorCallback);
    
    ROS_INFO("Music Feature Extractor");
    
    
    string directory;
    if (node.getParam("music_dir", directory)){
        ROS_INFO("MusicFeatureExtractor using music_dir: %s", directory.c_str());
        chdir(directory.c_str());
    }
    else{
        ROS_ERROR("Please set the music_directory (file) parameter for extractor");
        ros::requestShutdown();
    }

    ros::spin();
    return 0;
}

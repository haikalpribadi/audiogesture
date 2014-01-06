/* 
 * File:   main.cpp
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
#include "audiogesture/Bextractor.h"
#include "bextract.h"


using namespace std;


bool bextract(audiogesture::Bextractor::Request &req,
               audiogesture::Bextractor::Response &res)
{
    bextractor(req.args);
    ROS_INFO("BEXTRACTOR CALLED");
    return true;
}
/*
 * 
 */
int main(int argc, char** argv) {
    
    ros::init(argc, argv, "Bextractor");
    ros::NodeHandle node;
    
    ros::ServiceServer service = node.advertiseService("bextractor", bextract);
    
    ROS_INFO("Bextractor: Marsyas Audio Feature Extractor");
    ros::spin();
    
    /* 
    vector<string> args;
    for(int i=1; i<argc; i++)
        args.push_back(argv[i]);
    bextractor(args);
    
     */
    return 0;
}

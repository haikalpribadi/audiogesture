/* 
 * File:   AudioGestureTrainer.h
 * Author: haikalpribadi
 *
 * Created on 06 March 2014, 16:21
 */

#ifndef AUDIOGESTURETRAINER_H
#define	AUDIOGESTURETRAINER_H

#include <algorithm>
#include <iostream>
#include <math.h>
#include <ros/ros.h>
#include <stdio.h>
#include <std_msgs/Empty.h>
#include <vector>

#include "audiogesture/ExtractorStatus.h"
#include "audiogesture/GetFile.h"
#include "audiogesture/GetSamples.h"
#include "audiogesture/PlayerCommand.h"
#include "audiogesture/TrainerStatus.h"
#include "audiogesture/TrainerLogStatus.h"
#include "CompareNatural.h"

using namespace std;

class AudioGestureTrainer {
public:
    AudioGestureTrainer();
    
    void run();
    
private:
    ros::NodeHandle node;
    ros::Publisher kinectCalibrate_pub;
    ros::Publisher playerCommand_pub;
    ros::Publisher trainerStatus_pub;
    ros::Publisher trainerLogStatus_pub;
    ros::ServiceClient deleteLastGestureFile_cl;
    ros::ServiceClient getLastGestureFile_cl;
    ros::ServiceClient getSampleFile_cl;
    ros::ServiceClient getSamples_cl;
    
    
    string music_dir;
    vector<string> samples;
    
    void deleteLastGestureFile(string sample);
    void publishToPlay(string sample, string file);
    void publishToRecord(string sample, string file);
    void publishToStop(string sample, string file);
    bool trainSample(string sample);
    
    void printSamples(const vector<string>& samples);
    bool isNumber(const std::string& s);
};

#endif	/* AUDIOGESTURETRAINER_H */


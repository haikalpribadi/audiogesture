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
#include <vector>

#include "audiogesture/ExtractorStatus.h"
#include "audiogesture/GetSampleFile.h"
#include "audiogesture/GetSamples.h"
#include "audiogesture/TrainerStatus.h"
#include "CompareNatural.h"

using namespace std;

class AudioGestureTrainer {
public:
    AudioGestureTrainer();
    
    ros::Publisher trainerStatus_pub;
    ros::ServiceClient getSampleFile_cl;
    ros::ServiceClient getSamples_cl;
    
    void run();
    
private:
    ros::NodeHandle node;
    
    
    string music_dir;
    vector<string> samples;
    
    void printSamples(const vector<string>& samples);
};

#endif	/* AUDIOGESTURETRAINER_H */


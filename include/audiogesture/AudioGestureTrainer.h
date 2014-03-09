/* 
 * File:   AudioGestureTrainer.h
 * Author: haikalpribadi
 *
 * Created on 06 March 2014, 16:21
 */

#ifndef AUDIOGESTURETRAINER_H
#define	AUDIOGESTURETRAINER_H

#include <ros/ros.h>
#include <vector>

#include "audiogesture/ExtractorStatus.h"
#include "audiogesture/TrainerStatus.h"

using namespace std;

class AudioGestureTrainer {
public:
    AudioGestureTrainer();
    
    ros::Publisher trainerStatus_pub;
    
    
private:
    ros::NodeHandle node;
    ros::Subscriber extractorStatus_sub;

    string music_dir;
    vector<string> samples;
    
    void extractorStatusCallback(const audiogesture::ExtractorStatus::ConstPtr& msg);
    
    
};

#endif	/* AUDIOGESTURETRAINER_H */


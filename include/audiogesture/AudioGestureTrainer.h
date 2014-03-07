/* 
 * File:   AudioGestureTrainer.h
 * Author: haikalpribadi
 *
 * Created on 06 March 2014, 16:21
 */

#ifndef AUDIOGESTURETRAINER_H
#define	AUDIOGESTURETRAINER_H

#include <ros/ros.h>

#include "audiogesture/TrainerStatus.h"

using namespace std;

class AudioGestureTrainer {
public:
    AudioGestureTrainer();
    
    ros::Publisher trainerStatus_pub;
    
private:
    ros::NodeHandle node;
    

    string music_dir;
    
    
};

#endif	/* AUDIOGESTURETRAINER_H */

